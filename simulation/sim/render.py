"""Rerun rendering for the JAX scene. Draws cones as ACTUAL cones (instanced
meshes) and sets up a transform hierarchy so lidar points logged in the sensor
frame land in world space automatically. Replaces the MuJoCo viewer + the
sphere-cones in visualize.setup_static."""
import math

import numpy as np
import rerun as rr

from .scene import Scene, CAR_Z, LIDAR_X_M, LIDAR_Y_M, LIDAR_Z_M, LIDAR_YAW, _RGB


def setup_static(scene: Scene) -> None:
    """Log the parts of the scene that never move: ground, cones, sensor mount."""
    log_ground()
    log_cones(scene)
    hyaw = 0.5 * LIDAR_YAW
    rr.log("world/car/lidar", rr.Transform3D(translation=[LIDAR_X_M, LIDAR_Y_M, LIDAR_Z_M],
           quaternion=rr.Quaternion(xyzw=[0.0, 0.0, math.sin(hyaw), math.cos(hyaw)])), static=True)


def cone_mesh(radius: float, height: float, n_seg: int = 24):
    """Closed cone mesh (base fan + side walls) with smooth per-vertex normals."""
    ang = np.linspace(0, 2 * np.pi, n_seg, endpoint=False)
    ring = np.stack([radius * np.cos(ang), radius * np.sin(ang), np.zeros(n_seg)], 1)
    pos = np.concatenate([[[0, 0, 0]], ring, [[0, 0, height]]], 0).astype("f4")
    apex = n_seg + 1
    tris = []
    for i in range(n_seg):
        a, b = 1 + i, 1 + (i + 1) % n_seg
        tris.append([0, b, a]); tris.append([a, b, apex])
    tris = np.asarray(tris, np.uint32)
    nrm = np.zeros_like(pos)
    for i0, i1, i2 in tris:
        n = np.cross(pos[i1] - pos[i0], pos[i2] - pos[i0])
        nrm[i0] += n; nrm[i1] += n; nrm[i2] += n
    nrm /= np.clip(np.linalg.norm(nrm, axis=1, keepdims=True), 1e-9, None)
    return pos, tris, nrm.astype("f4")


def log_cones(scene: Scene) -> None:
    """One instanced Mesh3D per (type, size) group, positioned by InstancePoses3D."""
    xy = np.asarray(scene.cone_xy)
    r = np.asarray(scene.cone_r); h = np.asarray(scene.cone_h)
    typ = np.asarray(scene.cone_type)
    groups = np.unique(np.stack([typ, np.round(r, 4), np.round(h, 4)], 1), axis=0)
    for t, rr_, hh in groups:
        m = (typ == int(t)) & (np.round(r, 4) == rr_) & (np.round(h, 4) == hh)
        pos, tris, nrm = cone_mesh(float(rr_), float(hh))
        color = np.array(_RGB[int(t)], np.uint8)
        rr.log(f"world/cones/{int(t)}_{rr_:.3f}",
               rr.Mesh3D(vertex_positions=pos, triangle_indices=tris, vertex_normals=nrm,
                         vertex_colors=np.tile(color, (pos.shape[0], 1))),
               rr.InstancePoses3D(translations=np.column_stack([xy[m], np.zeros(m.sum())]).astype("f4")),
               static=True)


def log_ground(size: float = 200.0) -> None:
    q = np.array([[-size, -size, 0], [size, -size, 0], [size, size, 0], [-size, size, 0]], "f4")
    rr.log("world/ground", rr.Mesh3D(vertex_positions=q, triangle_indices=np.array([[0, 1, 2], [0, 2, 3]], np.uint32),
                                     vertex_colors=np.tile(np.array([55, 55, 60], np.uint8), (4, 1))), static=True)


def log_car(x: float, y: float, psi: float, timed_out: bool = False) -> None:
    """Car frame + body box; the lidar frame + points hang off world/car."""
    hpsi = 0.5 * psi
    rr.log("world/car", rr.Transform3D(translation=[x, y, CAR_Z],
           quaternion=rr.Quaternion(xyzw=[0.0, 0.0, math.sin(hpsi), math.cos(hpsi)])))
    rr.log("world/car/body", rr.Boxes3D(half_sizes=[[1.2, 0.6, 0.2]], centers=[[0, 0, 0]],
           colors=[[100, 100, 100] if timed_out else [220, 50, 50]]))


def log_points(pts_local: np.ndarray) -> None:
    """Sensor-frame points; the world/car/lidar transform places them in world."""
    rr.log("world/car/lidar/points", rr.Points3D(pts_local, radii=0.03, colors=[200, 200, 200]))
