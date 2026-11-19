"""
Steering actuator

On the car the steering command goes to a Kraken motor on the steering column, which moves along a trapezoidal
motion profile (capped velocity and acceleration) and stops at the column's end stops. This models that instead of
setting the wheels to the commanded angle instantly. Numbers are drivebrain's kraken_comms config, in column degrees,
and the column to wheel ratio is the one drivebrain_app.cpp uses to turn the wheel command into a column angle.
"""

import jax
import jax.numpy as jnp
from typing import NamedTuple

class SteeringParams(NamedTuple):
    column_to_wheel: float = 1.0 / 0.2428  # DrivebrainApp::_actuate: set_angle(desired_steering_deg * (1.0/0.2428))
    min_column_deg: float = -82.0          # kraken_comms/min_angle_deg
    max_column_deg: float = 90.0           # kraken_comms/max_angle_deg
    max_vel_column_deg_s: float = 720.0    # kraken_comms/mm_cruise_velocity_deg_s
    max_acc_column_deg_s2: float = 2880.0  # kraken_comms/mm_acceleration_deg_s2

class SteeringState(NamedTuple):
    pos_deg: jax.Array    # column
    vel_deg_s: jax.Array

def init() -> SteeringState:
    """Column centred and at rest"""
    return SteeringState(jnp.array(0.0), jnp.array(0.0))

@jax.jit
def step(
    s: SteeringState,
    wheel_cmd_rad: jax.Array,
    dt: float,
    params: SteeringParams = SteeringParams(),
) -> tuple[jax.Array, SteeringState]:
    """Steps the column towards the command along a trapezoidal profile (port of KrakenComms::TrapezoidalProfile).
    Returns (actual wheel angle (rad), new column state)"""
    p: SteeringParams = params
    goal: jax.Array = jnp.clip(jnp.degrees(wheel_cmd_rad) * p.column_to_wheel, p.min_column_deg, p.max_column_deg)
    a: float = p.max_acc_column_deg_s2

    # Travel towards the goal from wherever the column would stop if it braked now, in a frame where that is positive
    stop_offset: jax.Array = s.vel_deg_s * jnp.abs(s.vel_deg_s) / (2.0 * a)
    direction: jax.Array = jnp.where(goal - s.pos_deg - stop_offset >= 0.0, 1.0, -1.0)
    dist: jax.Array = direction * (goal - s.pos_deg)
    v0: jax.Array = direction * s.vel_deg_s

    # Accelerate (or slow) to the peak speed, cruise, then brake to land on the goal
    peak: jax.Array = jnp.minimum(p.max_vel_column_deg_s, jnp.sqrt(jnp.maximum(0.0, (2.0 * a * dist + v0 * v0) / 2.0)))
    a1: jax.Array = jnp.where(peak >= v0, a, -a)
    t1: jax.Array = jnp.abs(peak - v0) / a
    d1: jax.Array = (peak * peak - v0 * v0) / (2.0 * a1)
    t3: jax.Array = peak / a
    d3: jax.Array = peak * peak / (2.0 * a)
    d2: jax.Array = jnp.maximum(0.0, dist - d1 - d3)
    t2: jax.Array = jnp.where(peak > 1e-6, d2 / jnp.maximum(peak, 1e-6), 0.0)

    # Position and velocity along the profile at dt, for each phase
    t: jax.Array = dt - t1 - t2
    moved: jax.Array = jnp.where(dt <= t1, v0 * dt + 0.5 * a1 * dt * dt,
                       jnp.where(dt <= t1 + t2, d1 + peak * (dt - t1),
                                 d1 + d2 + peak * t - 0.5 * a * t * t))
    vel: jax.Array = jnp.where(dt <= t1, v0 + a1 * dt,
                     jnp.where(dt <= t1 + t2, peak, peak - a * t))

    done: jax.Array = dt >= t1 + t2 + t3
    pos_deg: jax.Array = jnp.where(done, goal, s.pos_deg + direction * moved)
    vel_deg_s: jax.Array = jnp.where(done, 0.0, direction * vel)

    return jnp.radians(pos_deg / p.column_to_wheel), SteeringState(pos_deg, vel_deg_s)
