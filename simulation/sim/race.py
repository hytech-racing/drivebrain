"""Competition scoring: cone hits (DOO), off-course excursions and splits at the timekeeping lines. Also owns which
cones are knocked over, which the lidar and renderer read back"""
import math

import numpy as np

from .scene import Scene, CAR_HALF_LENGTH, CAR_HALF_WIDTH, CONE_BLUE, CONE_YELLOW

DOO_PENALTY_S: float = 2.0  # per cone down or out
OC_PENALTY_S: float = 10.0  # per off-course excursion
OC_REENTRY_S: float = 0.5   # back on track this long before another excursion counts, so skimming the line counts once

def _inside(poly: np.ndarray, pts: np.ndarray) -> np.ndarray:
    """Even-odd point in polygon. poly (N,2) closed implicitly, pts (K,2) -> (K,) bool"""
    a = poly[None, :, :]
    b = np.roll(poly, -1, axis=0)[None, :, :]
    px, py = pts[:, None, 0], pts[:, None, 1]
    straddles = (a[..., 1] > py) != (b[..., 1] > py)
    x_cross = a[..., 0] + (py - a[..., 1]) * (b[..., 0] - a[..., 0]) / np.where(straddles, b[..., 1] - a[..., 1], 1.0)
    return np.sum(straddles & (px < x_cross), axis=1) % 2 == 1

def _closed_chain(chain: np.ndarray) -> bool:
    """A cone chain is a loop if its last cone is about one cone gap from its first. Not true of skidpad or accel"""
    if len(chain) < 3:
        return False
    gaps = np.linalg.norm(np.diff(chain, axis=0), axis=1)
    return np.linalg.norm(chain[0] - chain[-1]) <= 1.5 * np.median(gaps)

class Race:
    def __init__(self, scene: Scene, wheel_xy: np.ndarray) -> None:
        """wheel_xy (4,2): wheel contact points in the body frame, for the off-course check"""
        self._cone_xy = np.asarray(scene.cone_xy, np.float64)
        self._cone_r = np.asarray(scene.cone_r, np.float64)
        self._wheel_xy = wheel_xy
        m = len(self._cone_xy)
        self.up: np.ndarray = np.ones(m, np.float32)
        self.off_xy: np.ndarray = np.zeros((m, 2), np.float32)

        # off course needs a boundary on both sides, so only closed tracks get it
        left = self._cone_xy[scene.cone_type == CONE_BLUE]
        right = self._cone_xy[scene.cone_type == CONE_YELLOW]
        self._boundary: tuple[np.ndarray, np.ndarray] | None = \
            (left, right) if _closed_chain(left) and _closed_chain(right) else None
        self._off_course: bool = False
        self._on_track_since: float = 0.0

        # gate normals point away from the start, so only forward crossings count
        self._gates = np.asarray(scene.timing_gates, np.float64)
        start = np.asarray(scene.start_pose[:2], np.float64)
        self._gate_n = np.empty((len(self._gates), 2))
        for i, (a, b) in enumerate(self._gates):
            n = np.array([-(b - a)[1], (b - a)[0]])
            self._gate_n[i] = n if np.dot(start - a, n) < 0.0 else -n
        self._prev_xy: np.ndarray | None = None
        self._crossings: list[float] = []

        self.laps: list[float] = []
        self.doo: int = 0
        self.oc: int = 0

    def update(self, t: float, x: float, y: float, psi: float, vx: float, vy: float) -> bool:
        """Advance one step with the car's pose and world-frame velocity. True if a cone was knocked over"""
        xy = np.array([x, y])
        c, s = math.cos(psi), math.sin(psi)
        knocked = self._contact_cones(xy, c, s, vx, vy, t)
        self._check_off_course(xy, c, s, t)
        if self._prev_xy is not None:
            self._check_gates(self._prev_xy, xy, t)
        self._prev_xy = xy
        return knocked

    def _contact_cones(self, xy: np.ndarray, c: float, s: float, vx: float, vy: float, t: float) -> bool:
        # cone centres in the body frame, then circle vs footprint rectangle
        d = self._cone_xy + self.off_xy - xy
        lx = c * d[:, 0] + s * d[:, 1]
        ly = -s * d[:, 0] + c * d[:, 1]
        qx = np.maximum(np.abs(lx) - CAR_HALF_LENGTH, 0.0)
        qy = np.maximum(np.abs(ly) - CAR_HALF_WIDTH, 0.0)
        hit = (qx * qx + qy * qy < self._cone_r ** 2) & (self.up > 0.5)
        if not hit.any():
            return False

        # knocked cones fall over and get shoved along the car's direction of travel
        speed = math.hypot(vx, vy)
        push = np.array([vx, vy]) / speed if speed > 0.1 else np.array([c, s])
        self.up[hit] = 0.0
        self.off_xy[hit] += push * min(max(0.15 * speed, 0.3), 2.0)
        for i in np.flatnonzero(hit):
            self.doo += 1
            print(f"[{t:7.2f}s] cone hit at ({self._cone_xy[i, 0]:.1f}, {self._cone_xy[i, 1]:.1f}), {self.doo} DOO")
        return True

    def _check_off_course(self, xy: np.ndarray, c: float, s: float, t: float) -> None:
        if self._boundary is None:
            return
        wx = xy[0] + c * self._wheel_xy[:, 0] - s * self._wheel_xy[:, 1]
        wy = xy[1] + s * self._wheel_xy[:, 0] + c * self._wheel_xy[:, 1]
        wheels = np.stack([wx, wy], axis=1)
        left, right = self._boundary
        # on track = inside exactly one of the two loops, whichever is the outer one
        on_track = _inside(left, wheels) != _inside(right, wheels)
        if on_track.any():
            if self._off_course and t - self._on_track_since >= OC_REENTRY_S:
                self._off_course = False
        else:
            if not self._off_course:
                self.oc += 1
                print(f"[{t:7.2f}s] off course, {self.oc} OC")
            self._off_course = True
            self._on_track_since = t

    def _check_gates(self, p0: np.ndarray, p1: np.ndarray, t: float) -> None:
        step = p1 - p0
        for (a, b), n in zip(self._gates, self._gate_n):
            # forward across the gate's line, half-open so a sample exactly on it counts once
            if not np.dot(p0 - a, n) < 0.0 <= np.dot(p1 - a, n):
                continue
            # and between its two ends
            side_a = step[0] * (a[1] - p0[1]) - step[1] * (a[0] - p0[0])
            side_b = step[0] * (b[1] - p0[1]) - step[1] * (b[0] - p0[0])
            if side_a * side_b > 0.0:
                continue
            if self._crossings:
                self.laps.append(t - self._crossings[-1])
                print(f"[{t:7.2f}s] lap {len(self.laps)}: {self.laps[-1]:.3f}s")
            else:
                print(f"[{t:7.2f}s] timing started")
            self._crossings.append(t)

    def summary(self) -> str:
        lines = [f"laps: {len(self.laps)}"]
        lines += [f"  {i + 1}: {lap:.3f}s" for i, lap in enumerate(self.laps)]
        if self.laps:
            lines.append(f"best: {min(self.laps):.3f}s  total: {sum(self.laps):.3f}s")
        oc = f"{self.oc}" if self._boundary is not None else "n/a (open track)"
        lines.append(f"DOO: {self.doo}  OC: {oc}")
        lines.append(f"penalties: {self.doo * DOO_PENALTY_S + self.oc * OC_PENALTY_S:.1f}s")
        return "\n".join(lines)
