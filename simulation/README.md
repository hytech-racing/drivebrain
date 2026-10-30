# Simulation

A simulation environment for the FS Driverless competition, built on JAX and visualized with [Rerun](https://rerun.io). 
Thanks to [AMZ's simulator](https://github.com/AMZ-Racing/fssim) for the track and cone models!

## Drivebrain communication

The sim binds three ZMQ sockets and drivebrain (`drivebrain_comms/src/SimComms.cpp`) connects to them:

| Port | Direction | Contents |
|------|-----------|----------|
| 5940 | drivebrain → sim | CAN commands (speed setpoints, torque limits, steering) |
| 6767 | sim → drivebrain | pose, ground speed, cone ground truth |
| 1155 | sim → drivebrain | lidar point clouds |

- The sim binds to `SIM_BIND_HOST` (default `127.0.0.1`), and drivebrain connects to `SIM_HOST`.
- If no command arrives for 1 s, the sim treats drivebrain as disconnected.
- Messages use HT_proto and HT_CAN. On first run, `sim/proto.py` downloads them from GitHub and caches them under `~/.cache/`. HT_CAN is pinned by `CAN_TAG` in `main.py` and must match the release drivebrain was built against.

## Running

Requires Python ≥ 3.14 and [uv](https://docs.astral.sh/uv/).

```bash
uv sync
uv run main.py --track FSG            # opens a Rerun viewer
uv run main.py --track FSG --logless  # headless, no Rerun
```

### Sim on your machine, drivebrain in the container

This is the default setup: `docker-compose.yml` already sets `SIM_HOST=host.docker.internal`. Run the sim on the host as above, then build and run drivebrain in the container (see the top-level README, `--local` build flag).

## Layout

```
main.py            sim loop: commands in → inverters → dynamics → sensors out
visualize.py       Rerun time-series panels and layout
sim/comms.py       ZMQ sockets and message packing
sim/proto.py       HT_proto / HT_CAN download and caching
sim/scene.py       track loading, sensor mount transforms
sim/render.py      Rerun 3D scene (cones, car, lidar points)
sim/lidar.py       lidar model
sim/sensors.py     ground speed sensor model
sim/vehicle/       dynamics, inverter controller, input state
tracks/            track definitions
```
