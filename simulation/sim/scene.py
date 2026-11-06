"""Track geometry loaded from a track YAML"""
import math
from pathlib import Path
from typing import NamedTuple

import numpy as np
import yaml

TRACKS_DIR: Path = Path(__file__).parent.parent / "tracks/"

CAR_Z: float = 0.4 # car body center height
# lidar static transform (base_link -> lidar), matches drivebrain's static_transforms
LIDAR_X_M: float = 1.244
LIDAR_Y_M: float = 0.0
LIDAR_Z_M: float = 0.107
LIDAR_YAW: float = math.radians(90.0)
SENSOR_Z: float = CAR_Z + LIDAR_Z_M

CONE_BLUE: int = 1
CONE_YELLOW: int = 2
CONE_ORANGE: int = 3
CONE_ORANGE_BIG: int = 4

SMALL_R, SMALL_H = 0.114, 0.325 # blue/yellow, 228 mm base
BIG_R, BIG_H = 0.1425, 0.505 # big orange, 285 mm base

_RGB = {CONE_BLUE: (30, 100, 255), CONE_YELLOW: (255, 210, 0),
        CONE_ORANGE: (255, 90, 0), CONE_ORANGE_BIG: (255, 90, 0)}

class Scene(NamedTuple):
    cone_xy: np.ndarray     # (M,2) f32 base centre
    cone_r: np.ndarray      # (M,)  f32 base radius
    cone_h: np.ndarray      # (M,)  f32 height
    cone_type: np.ndarray   # (M,)  i32 CONE_*
    start_pose: np.ndarray  # (3,)  f32 [x, y, psi]


def load_scene(track: str) -> Scene:
    d = yaml.safe_load(open(TRACKS_DIR / f"{track}.yaml"))
    xy: list = []; r: list = []; h: list = []; typ: list = []
    for key, tc, rr, hh in [
        ("cones_left", CONE_BLUE, SMALL_R, SMALL_H),
        ("cones_right", CONE_YELLOW, SMALL_R, SMALL_H),
        ("cones_orange", CONE_ORANGE, SMALL_R, SMALL_H),
        ("cones_orange_big", CONE_ORANGE_BIG, BIG_R, BIG_H),
    ]:
        for c in (d.get(key) or []):
            xy.append([float(c[0]), float(c[1])]); r.append(rr); h.append(hh); typ.append(tc)

    pose = d.get("starting_pose_front_wing", [0.0, 0.0, 0.0])
    return Scene(
        cone_xy=np.asarray(xy, np.float32).reshape(-1, 2),
        cone_r=np.asarray(r, np.float32),
        cone_h=np.asarray(h, np.float32),
        cone_type=np.asarray(typ, np.int32),
        start_pose=np.asarray([float(pose[0]), float(pose[1]), float(pose[2])], np.float32),
    )
