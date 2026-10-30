"""
Onboard inverter speed controller

Drivebrain doesn't command torque directly: it sends a motor speed setpoint and a
torque limit per wheel, and the inverters close the speed loop themselves. This
mimics that with a per-wheel PI controller whose output is clamped to +/- the limit.
"""

import numpy as np
from typing import NamedTuple

RPM_TO_RAD_S: float = 2.0 * np.pi / 60.0

class SpeedControllerParams(NamedTuple):
    # Motor side gains, Nm per rad/s of motor speed error. kp must satisfy
    # kp * dt / J < 2 with J the motor-reflected wheel inertia (I_w / gear_ratio^2,
    # ~6.7e-4 kg m^2) or the loop goes unstable whenever a tire breaks traction
    kp: float = 0.15
    ki: float = 1.5
    # motor peak torque. commanded limits are capped to this so a garbage limit
    # can't spin a wheel up to inf and NaN the whole state
    max_torque_nm: float = 21.0

class SpeedController:
    def __init__(self, gear_ratio: float, dt: float,
                 params: SpeedControllerParams = SpeedControllerParams()) -> None:
        self._gear_ratio = gear_ratio
        self._dt = dt
        self._p = params
        self._integral: np.ndarray = np.zeros(4)

    def update(self, wheel_omega: np.ndarray, speed_rpm: np.ndarray, torque_lim: np.ndarray) -> np.ndarray:
        """Motor torques (Nm) from wheel speeds (rad/s), motor speed setpoints (rpm) and torque limits (Nm)"""
        lim: np.ndarray = np.minimum(np.abs(torque_lim), self._p.max_torque_nm)
        err: np.ndarray = speed_rpm * RPM_TO_RAD_S - wheel_omega * self._gear_ratio

        # clamp the integrator to the limit so it can't wind up while saturated,
        # and so a zeroed limit (timeout) also resets it
        self._integral = np.clip(self._integral + self._p.ki * err * self._dt, -lim, lim)
        return np.clip(self._p.kp * err + self._integral, -lim, lim)
