"""
Onboard inverter speed controller

Drivebrain doesn't command torque directly: it sends a motor speed setpoint and a
torque limit per wheel, and the inverters close the speed loop themselves. This
mimics that with a per-wheel PI controller whose output is clamped to +/- the limit.

Each motor follows the AMK torque-speed curve (peak torque, then constant power up to max speed), and the car's
total power is capped like the MCU does.
"""

import jax
import jax.numpy as jnp
from typing import NamedTuple

RPM_TO_RAD_S: float = 2.0 * jnp.pi / 60.0

class SpeedControllerParams(NamedTuple):
    # Motor side gains, Nm per rad/s of motor speed error. kp must satisfy
    # kp * dt / J < 2 with J the motor-reflected wheel inertia (I_w / gear_ratio^2,
    # ~6.7e-4 kg m^2) or the loop goes unstable whenever a tire breaks traction
    kp: float = 0.15
    ki: float = 1.5
    # motor peak torque. commanded limits are capped to this so a garbage limit
    # can't spin a wheel up to inf and NaN the whole state
    max_torque_nm: float = 20.6
    max_motor_rpm: float = 20000.0
    motor_power_kw: float = 26.0  # estimate: inverter rated power
    max_power_kw: float = 63.0    # whole car

def init() -> jax.Array:
    """Initial integrator state, one per wheel"""
    return jnp.zeros(4)

@jax.jit
def step(
    integral: jax.Array,
    wheel_omega: jax.Array,
    speed_rpm: jax.Array,
    torque_lim: jax.Array,
    gear_ratio: float,
    dt: float,
    params: SpeedControllerParams = SpeedControllerParams(),
) -> tuple[jax.Array, jax.Array]:
    """Motor torques (Nm) from wheel speeds (rad/s), motor speed setpoints (rpm) and torque limits (Nm).
    Returns (torques, new integrator state)"""
    p: SpeedControllerParams = params
    motor_omega: jax.Array = wheel_omega * gear_ratio
    err: jax.Array = speed_rpm * RPM_TO_RAD_S - motor_omega

    curve_lim: jax.Array = jnp.minimum(p.max_torque_nm, p.motor_power_kw * 1000.0 / jnp.maximum(jnp.abs(motor_omega), 1e-3))
    lim: jax.Array = jnp.minimum(jnp.abs(torque_lim), curve_lim)

    # clamp the integrator to the limit so it can't wind up while saturated,
    # and so a zeroed limit (timeout) also resets it
    integral = jnp.clip(integral + p.ki * err * dt, -lim, lim)
    tau: jax.Array = jnp.clip(p.kp * err + integral, -lim, lim)

    past_max_speed: jax.Array = jnp.abs(motor_omega) >= p.max_motor_rpm * RPM_TO_RAD_S
    tau = jnp.where(past_max_speed & (tau * motor_omega > 0.0), 0.0, tau)

    # Scale every wheel down together when over the car's power cap (scale = 1 when under it)
    power_w: jax.Array = jnp.sum(jnp.abs(tau * motor_omega))
    scale: jax.Array = jnp.minimum(1.0, p.max_power_kw * 1000.0 / jnp.maximum(power_w, 1e-6))
    tau = tau * scale
    integral = jnp.clip(integral, -lim * scale, lim * scale)
    return tau, integral
