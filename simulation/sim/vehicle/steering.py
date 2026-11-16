"""
Steering actuator

On the car the steering command goes to a Kraken motor on the steering column, which moves along a trapezoidal
motion profile (capped velocity and acceleration) and stops at the column's end stops. This models that instead of
setting the wheels to the commanded angle instantly. Numbers are drivebrain's kraken_comms config, in column degrees,
and the column to wheel ratio is the one drivebrain_app.cpp uses to turn the wheel command into a column angle.
"""

import math
from typing import NamedTuple

class SteeringParams(NamedTuple):
    column_to_wheel: float = 1.0 / 0.2428  # DrivebrainApp::_actuate: set_angle(desired_steering_deg * (1.0/0.2428))
    min_column_deg: float = -82.0          # kraken_comms/min_angle_deg
    max_column_deg: float = 90.0           # kraken_comms/max_angle_deg
    max_vel_column_deg_s: float = 720.0    # kraken_comms/mm_cruise_velocity_deg_s
    max_acc_column_deg_s2: float = 2880.0  # kraken_comms/mm_acceleration_deg_s2

class SteeringActuator:
    """Column angle following the commanded wheel angle along a trapezoidal profile (port of KrakenComms::TrapezoidalProfile)"""

    def __init__(self, params: SteeringParams = SteeringParams()) -> None:
        self._p = params
        self._pos_deg: float = 0.0  # column
        self._vel_deg_s: float = 0.0

    def update(self, wheel_cmd_rad: float, dt: float) -> float:
        """Steps the column towards the command; returns the actual wheel angle (rad)"""
        p = self._p
        goal = min(max(math.degrees(wheel_cmd_rad) * p.column_to_wheel, p.min_column_deg), p.max_column_deg)
        a = p.max_acc_column_deg_s2

        # Travel towards the goal from wherever the column would stop if it braked now, in a frame where that is positive
        stop_offset = self._vel_deg_s * abs(self._vel_deg_s) / (2.0 * a)
        direction = 1.0 if goal - self._pos_deg - stop_offset >= 0.0 else -1.0
        dist = direction * (goal - self._pos_deg)
        v0 = direction * self._vel_deg_s

        # Accelerate (or slow) to the peak speed, cruise, then brake to land on the goal
        peak = min(p.max_vel_column_deg_s, math.sqrt(max(0.0, (2.0 * a * dist + v0 * v0) / 2.0)))
        a1 = a if peak >= v0 else -a
        t1 = abs(peak - v0) / a
        d1 = (peak * peak - v0 * v0) / (2.0 * a1)
        t3 = peak / a
        d3 = peak * peak / (2.0 * a)
        d2 = max(0.0, dist - d1 - d3)
        t2 = d2 / peak if peak > 1e-6 else 0.0

        if dt >= t1 + t2 + t3:
            self._pos_deg, self._vel_deg_s = goal, 0.0
        else:
            if dt <= t1:
                moved, vel = v0 * dt + 0.5 * a1 * dt * dt, v0 + a1 * dt
            elif dt <= t1 + t2:
                moved, vel = d1 + peak * (dt - t1), peak
            else:
                t = dt - t1 - t2
                moved, vel = d1 + d2 + peak * t - 0.5 * a * t * t, peak - a * t
            self._pos_deg += direction * moved
            self._vel_deg_s = direction * vel

        return math.radians(self._pos_deg / p.column_to_wheel)
