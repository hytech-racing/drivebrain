"""
JAX Two-Track Vehicle Dynamics Model

Reimplementation of the OpenModelica PlanarMechanics two-track model
with SlipBasedWheelJoint tire model.

State vector (10 x 1):
[x, y, psi, vx, vy, omega, omega_fl, omega_fr, omega_rl, omega_rr]

x, y: CG position in world frame (m)
psi: Heading angle (rad)
vx, vy: CG velocity in world frame (m/s)
omega: Yaw rate (rad/s)
omega_fl..rr: Wheel angular velocities (rad/s)

Input vector (5 x 1):
[tau_fl, tau_fr, tau_rl, tau_rr, delta]

tau_fl..rr: Motor torques (Nm)
delta: Front wheel steering angle (rad)
"""

import jax
import jax.numpy as jnp
from typing import NamedTuple

class VehicleParams(NamedTuple):
    m: float = 250.0
    I_zz: float = 120.0
    l_f: float = 0.765
    l_r: float = 0.765
    w_f: float = 0.6
    w_r: float = 0.6
    r_w: float = 0.2
    I_w: float = 0.094
    gear_ratio: float = 11.83
    f_n: float = 600.0
    mu_a: float = 1.8
    mu_s: float = 1.5
    s_adh: float = 0.117
    s_sld: float = 0.35
    # Low-speed slip regularization. Must stay >~1 m/s at dt=0.004: smaller makes the tire
    # too stiff for RK4 near standstill and the car limit-cycles instead of stopping
    v_adh_min: float = 1.0
    v_sld_min: float = 2.0
    c_rr: float = 0.015
    w_rr_eps: float = 0.1
    rho: float = 1.225
    cd_a: float = 1.2

X, Y, PSI, VX, VY, OMEGA, W_FL, W_FR, W_RL, W_RR = range(10)
"""Numerical indicies for the vehicle state JAX array"""

TAU_FL, TAU_FR, TAU_RL, TAU_RR, DELTA = range(5)
"""Numerical indicies for the vehicle input JAX array"""

def _tire_friction(
    v_slip: jax.Array,
    v_adh: jax.Array,
    v_sld: jax.Array,
    mu_a: float,
    mu_s: float,
    f_n: float,
) -> jax.Array:
    """Friction force magnitude from slip velocity magnitude"""
    eps: float = 1e-8
    f_adhesion = mu_a * f_n * v_slip / jnp.maximum(v_adh, eps)
    t = jnp.clip((v_slip - v_adh) / jnp.maximum(v_sld - v_adh, eps), 0.0, 1.0)
    mu_t = mu_a - (mu_a - mu_s) * 0.5 * (1.0 - jnp.cos(jnp.pi * t))
    f_transition = mu_t * f_n
    f_sliding = mu_s * f_n
    return jnp.where(v_slip < v_adh, f_adhesion,
                     jnp.where(v_slip < v_sld, f_transition, f_sliding))

def state(
    x: float = 0., y: float = 0., psi: float = 0.,
    vx: float = 0., vy: float = 0., omega: float = 0.,
    w_fl: float = 0., w_fr: float = 0.,
    w_rl: float = 0., w_rr: float = 0.,
) -> jax.Array:
    """Returns a JAX array of a vehicle state"""
    return jnp.array([x, y, psi, vx, vy, omega, w_fl, w_fr, w_rl, w_rr])

def input(
    tau_fl: float = 0., tau_fr: float = 0.,
    tau_rl: float = 0., tau_rr: float = 0.,
    steer: float = 0.,
) -> jax.Array:
    """Returns a JAX array of a vehicle input"""
    return jnp.array([tau_fl, tau_fr, tau_rl, tau_rr, steer])

@jax.jit
def xdot(state: jax.Array, u: jax.Array, params: VehicleParams) -> jax.Array:
    """State derivative x_dot = f(x, u)."""
    p: VehicleParams = params
    eps: float = 1e-10

    psi: jax.Array = state[PSI]
    vx: jax.Array = state[VX]
    vy: jax.Array = state[VY]
    omega: jax.Array = state[OMEGA]
    w: jax.Array = state[6:10]

    # Negative torque only brakes: fade it out as the wheel stops so it can't spin it backwards.
    # Ramp instead of a hard cutoff so RK4 doesn't chatter around w = 0
    tau_cmd: jax.Array = u[:4]
    brake_scale: jax.Array = jnp.clip(w / p.w_rr_eps, 0.0, 1.0)
    tau: jax.Array = jnp.where(tau_cmd < 0.0, tau_cmd * brake_scale, tau_cmd)
    tau_drive: jax.Array = tau * p.gear_ratio
    delta: jax.Array = u[DELTA]

    c_psi: jax.Array = jnp.cos(psi)
    s_psi: jax.Array = jnp.sin(psi)

    r_body: jax.Array = jnp.array([
        [ p.l_f,  p.w_f],
        [ p.l_f, -p.w_f],
        [-p.l_r,  p.w_r],
        [-p.l_r, -p.w_r],
    ])

    r_wx: jax.Array = c_psi * r_body[:, 0] - s_psi * r_body[:, 1]
    r_wy: jax.Array = s_psi * r_body[:, 0] + c_psi * r_body[:, 1]

    v_cx: jax.Array = vx - omega * r_wy
    v_cy: jax.Array = vy + omega * r_wx

    steer: jax.Array = jnp.array([delta, delta, 0.0, 0.0])
    heading: jax.Array = psi + steer
    c_h: jax.Array = jnp.cos(heading)
    s_h: jax.Array = jnp.sin(heading)

    v_long: jax.Array = v_cx * c_h + v_cy * s_h
    v_lat: jax.Array = -v_cx * s_h + v_cy * c_h

    v_slip_long: jax.Array = v_long - w * p.r_w
    v_slip_lat: jax.Array = v_lat
    v_slip_mag: jax.Array = jnp.sqrt(v_slip_long**2 + v_slip_lat**2 + eps)

    v_adh: jax.Array = jnp.maximum(p.s_adh * jnp.abs(v_long), p.v_adh_min)
    v_sld: jax.Array = jnp.maximum(p.s_sld * jnp.abs(v_long), p.v_sld_min)

    f_mag: jax.Array = _tire_friction(v_slip_mag, v_adh, v_sld, p.mu_a, p.mu_s, p.f_n)
    f_long: jax.Array = -f_mag * v_slip_long / v_slip_mag
    f_lat: jax.Array = -f_mag * v_slip_lat / v_slip_mag

    Fx: jax.Array = f_long * c_h - f_lat * s_h
    Fy: jax.Array = f_long * s_h + f_lat * c_h

    ax: jax.Array = jnp.sum(Fx) / p.m
    ay: jax.Array = jnp.sum(Fy) / p.m

    # Aero drag at the CG, opposing velocity. eps keeps the sqrt gradient finite at standstill
    v: jax.Array = jnp.sqrt(vx**2 + vy**2 + eps)
    k_drag: jax.Array = 0.5 * p.rho * p.cd_a * v / p.m
    ax = ax - k_drag * vx
    ay = ay - k_drag * vy
    alpha: jax.Array = jnp.sum(r_wx * Fy - r_wy * Fx) / p.I_zz

    # Rolling resistance, opposing wheel rotation. tanh instead of sign RK4 agrees at standstill
    tau_rr: jax.Array = -jnp.tanh(w / p.w_rr_eps) * p.c_rr * p.f_n * p.r_w

    # I_w * w_dot = tau_drive + tau_rr - f_long * r_w
    w_dot: jax.Array = (tau_drive + tau_rr - f_long * p.r_w) / p.I_w

    return jnp.array([
        vx, vy, omega,
        ax, ay, alpha,
        w_dot[0], w_dot[1],
        w_dot[2], w_dot[3],
    ])

@jax.jit
def step_rk4(state: jax.Array, u: jax.Array, params: VehicleParams, dt: float) -> jax.Array:
    """Single RK4 step with zero-order hold on input"""
    k1: jax.Array = xdot(state, u, params)
    k2: jax.Array = xdot(state + 0.5 * dt * k1, u, params)
    k3: jax.Array = xdot(state + 0.5 * dt * k2, u, params)
    k4: jax.Array = xdot(state + dt * k3, u, params)
    return state + (dt / 6.0) * (k1 + 2.0*k2 + 2.0*k3 + k4)


def slip_angle(s: jax.Array) -> jax.Array:
    psi: jax.Array = s[..., PSI]
    c: jax.Array = jnp.cos(psi)
    sin: jax.Array = jnp.sin(psi)
    vx_b: jax.Array = c * s[..., VX] + sin * s[..., VY]
    vy_b: jax.Array = -sin * s[..., VX] + c * s[..., VY]
    return jnp.arctan2(vy_b, jnp.maximum(vx_b, 1e-3))

