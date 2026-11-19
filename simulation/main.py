import argparse
import math
import signal
import sys
import time
import jax
import jax.numpy as jnp
import visualize
import numpy as np
import rerun as rr

from types import FrameType
from sim.vehicle import dynamics
from sim.vehicle.dynamics import (
    VehicleParams, step_rk4,
    X, Y, PSI, VX, VY, OMEGA, W_FL, W_RR, slip_angle,
)
from sim.vehicle import inverter, steering
from sim.comms import SimComms
from sim.scene import load_scene, scene_from_dict, LIDAR_X_M, LIDAR_Y_M, LIDAR_YAW, SENSOR_Z
from sim.lidar import LidarSensor
from sim.race import Race
from sim.trackgen import generate
from sim import render
from sim.proto import load_can, load_proto
from sim.sensors import GroundSpeedSensor, GssConfig

LIDAR_HZ: float = 10.0
GSS_HZ: float = 250.0
GSS_POS_X_M: float = -1.108  # sensor mount behind base_link, matches drivebrain's Odometry/oms_position_base_x_m
VIS_HZ: float = 60.0
CONES_HZ: float = 1.0  # static ground truth map
CAN_TAG: int = 268  # must match the HT_CAN release drivebrain is built against

def parse_args():
    p: argparse.ArgumentParser = argparse.ArgumentParser()
    p.add_argument("--track", default="FSG", choices=["FSG", "FSI", "skidpad", "acceleration", "thin", "random"])
    p.add_argument("--seed", type=int, default=None, help="Seed for --track random (default: a new one each run, printed)")
    p.add_argument("--logless", action="store_true", help="Disable Rerun (headless)")
    p.add_argument("--grip", type=float, default=1.0, help="Scales tyre friction (1.0 = peak mu 1.3, about 1 g of cornering)")
    p.add_argument("--laps", type=int, default=0, help="Stop after this many timed laps, exit 1 if stopped before (0 = run forever)")
    return p.parse_args()

def main():
    args = parse_args()
    defaults: VehicleParams = VehicleParams()
    params: VehicleParams = defaults._replace(mu_a=defaults.mu_a * args.grip, mu_s=defaults.mu_s * args.grip)
    dt: float = 0.004

    vis_rate: int = max(1, round(1.0 / (VIS_HZ * dt)))
    lidar_rate: int = max(1, round(1.0 / (LIDAR_HZ * dt)))
    cones_rate: int = max(1, round(1.0 / (CONES_HZ * dt)))
    gss_rate: int = max(1, round(1.0 / (GSS_HZ * dt)))

    s0 = dynamics.state()
    u0 = dynamics.input()
    step_rk4(s0, u0, params, dt).block_until_ready()

    if args.track == "random":
        seed: int = args.seed if args.seed is not None else int(np.random.default_rng().integers(1 << 31))
        print(f"random track, seed {seed}")
        scene = scene_from_dict(generate(seed))
    else:
        scene = load_scene(args.track)
    lidar: LidarSensor = LidarSensor(scene)
    gss: GroundSpeedSensor = GroundSpeedSensor(GssConfig(rate_hz=GSS_HZ))
    proto = load_proto()
    can = load_can(CAN_TAG)

    cone_xy: np.ndarray = np.asarray(scene.cone_xy)
    cone_type: np.ndarray = np.asarray(scene.cone_type)

    x0, y0, psi0 = [float(v) for v in scene.start_pose]
    state = dynamics.state(x=x0, y=y0, psi=psi0)
    inverter_state: jax.Array = inverter.init()
    steering_state: steering.SteeringState = steering.init()
    race: Race = Race(scene, np.array([
        [params.l_f, params.w_f], [params.l_f, -params.w_f],
        [-params.l_r, params.w_r], [-params.l_r, -params.w_r],
    ]))

    if not args.logless:
        rr.init("vehicle_sim", spawn=True)
        render.setup_static(scene)
        visualize.setup_series()
        rr.send_blueprint(visualize.blueprint())

    comms: SimComms = SimComms(proto, can)
    running: bool = True

    def stop(_sig: int, _frame: FrameType | None) -> None:
        nonlocal running
        running = False

    signal.signal(signal.SIGINT, stop)
    signal.signal(signal.SIGTERM, stop)

    sim_t: float = 0.0
    step_i: int = 0
    wall_start: float = time.perf_counter()
    latest_pts: np.ndarray = np.empty((0, 3), dtype=np.float32)

    while running:
        comms.drain_commands()
        cmd = comms.current_input()

        tau, inverter_state = inverter.step(
            inverter_state, state[W_FL:W_RR + 1],
            jnp.asarray(cmd.speed_rpm), jnp.asarray(cmd.torque_lim),
            params.gear_ratio, dt,
        )
        steer_rad, steering_state = steering.step(steering_state, jnp.asarray(cmd.wheel_steer_rad), dt)

        u: jax.Array = dynamics.input(
            tau_fl=tau[0], tau_fr=tau[1],
            tau_rl=tau[2], tau_rr=tau[3],
            steer=steer_rad,
        )

        state: jax.Array = step_rk4(state, u, params, dt)
        sim_t += dt
        step_i += 1

        x_i: float = float(state[X])
        y_i: float = float(state[Y])
        psi_i: float = float(state[PSI])

        if race.update(sim_t, x_i, y_i, psi_i, float(state[VX]), float(state[VY])):
            lidar.up = jnp.asarray(race.up)
            lidar.off_xy = jnp.asarray(race.off_xy)
            if not args.logless:
                rr.set_time("sim_time", duration=sim_t)
                render.log_cone_poses(scene, race.up, race.off_xy)
        if args.laps and len(race.laps) >= args.laps:
            running = False

        if step_i % lidar_rate == 0:
            latest_pts = lidar.scan((x_i, y_i, psi_i))
            comms.send_pointcloud(latest_pts)
            comms.send_pose(x_i, y_i, psi_i)
            c, s = math.cos(psi_i), math.sin(psi_i)
            comms.send_transform(
                x_i + c * LIDAR_X_M - s * LIDAR_Y_M,
                y_i + s * LIDAR_X_M + c * LIDAR_Y_M,
                SENSOR_Z,
                psi_i + LIDAR_YAW,
            )
            if not args.logless:
                rr.set_time("sim_time", duration=sim_t)
                render.log_points(latest_pts)

        if step_i % cones_rate == 0:
            comms.send_cones(cone_xy, cone_type)

        if step_i % vis_rate == 0 and not args.logless:
            rr.set_time("sim_time", duration=sim_t)
            render.log_car(x_i, y_i, psi_i, comms.timed_out())
            visualize.log_scalars(
                speed=math.hypot(float(state[VX]), float(state[VY])),
                yaw_rate=float(state[OMEGA]),
                slip=float(slip_angle(state)),
                torque_fl=float(tau[0]), torque_fr=float(tau[1]),
                torque_rl=float(tau[2]), torque_rr=float(tau[3]),
                steering=float(steer_rad),
            )

        if step_i % gss_rate == 0:
            c, s = math.cos(psi_i), math.sin(psi_i)
            vx_w, vy_w, omega = float(state[VX]), float(state[VY]), float(state[OMEGA])
            # velocity at the sensor mount: v_sensor = v_base + w x r
            meas = gss.measure(c * vx_w + s * vy_w, -s * vx_w + c * vy_w + omega * GSS_POS_X_M)
            if meas is not None:
                comms.send_oms(*meas, omega)

        drift: float = (wall_start + sim_t) - time.perf_counter()
        if drift > 0:
            time.sleep(drift)

    comms.close()
    print(race.summary())
    if args.laps and len(race.laps) < args.laps:
        print("DNF")
        sys.exit(1)

if __name__ == "__main__":
    main()
