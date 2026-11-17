"""Random closed tracks, so the planner gets tested on more than the fixed YAMLs. Built to the Trackdrive layout in the
FS Rules 2026 (D 8.1, D 8.3.2) and the FS Driverless Specification 2026 (DS 1.1). Same seed, same track"""
import math

import numpy as np

from .vehicle.dynamics import VehicleParams

# FS Rules 2026 D 8.1
TRACK_LEN_M: float = 350.0  # one lap, middle of the 200 m to 500 m allowed
MIN_WIDTH_M: float = 3.0
MAX_STRAIGHT_M: float = 80.0
TURN_DIAMETER_M: float = 9.0  # tightest turn needs at most this turning diameter (D 1.1.10): car on the outside edge
STRAIGHT_RADIUS_M: float = 100.0  # gentler than this counts as straight
# DS 1.1: at most 5 m between cones in driving direction, closer in corners
CONE_GAP_M: tuple[float, float] = (2.0, 5.0)  # tightest corner, straight
# D 8.3.2: front wheels staged 6 m before the start line
START_LINE_M: float = 6.0 + VehicleParams().l_f  # from base_link

WIDTH_M: tuple[float, float] = (MIN_WIDTH_M, 4.5)  # the rules only set the minimum, FSG is about 4 m
MIN_INNER_RADIUS_M: float = 1.0  # keeps the inside edge of the tightest hairpin from folding over
CLEARANCE_M: float = 3.0      # between the boundaries of separate parts of the track
START_WINDOW_M: tuple[float, float] = (-5.0, 15.0)  # around the start pose, has to be near straight
START_MAX_CURV: float = 1.0 / 30.0
MIN_TURNING: float = 2.0  # total turning over a lap, in full circles, so it isn't just one big loop. FSG is about 2.5
CONE_GAP_PER_RADIUS: float = 0.15  # gap grows with the boundary's radius until it hits the straight gap
DS: float = 0.1  # centreline sample spacing (m)

def _loop(rng: np.random.Generator) -> tuple[np.ndarray, np.ndarray]:
    """Random loop resampled every DS m -> points (N,2) and signed curvature (N,), left turns positive. A closed cubic
    B-spline through control points spread around a circle at random radii: points pulled in give hairpins and chicanes,
    points in line give straights"""
    n = int(rng.integers(16, 26))
    ang = (np.arange(n) + rng.uniform(-0.4, 0.4, n)) * 2.0 * math.pi / n  # evenly spread, so they don't bunch into kinks
    rad = rng.uniform(0.15, 1.0, n)
    ctrl = np.stack([rad * np.cos(ang), rad * np.sin(ang)], axis=1)

    # each span uses four neighbouring control points
    t = np.linspace(0.0, 1.0, 200, endpoint=False)[:, None]
    basis = np.hstack([(1 - t) ** 3, 3 * t ** 3 - 6 * t ** 2 + 4, -3 * t ** 3 + 3 * t ** 2 + 3 * t + 1, t ** 3]) / 6.0
    d1 = np.hstack([-(1 - t) ** 2, 3 * t ** 2 - 4 * t, -3 * t ** 2 + 2 * t + 1, t ** 2]) / 2.0
    d2 = np.hstack([1 - t, 3 * t - 2, 1 - 3 * t, t])
    spans = np.stack([np.roll(ctrl, -k, axis=0) for k in range(4)], axis=1)  # (n,4,2)
    pts = np.einsum("tk,nkd->ntd", basis, spans).reshape(-1, 2)
    v = np.einsum("tk,nkd->ntd", d1, spans).reshape(-1, 2)
    a = np.einsum("tk,nkd->ntd", d2, spans).reshape(-1, 2)
    curv = (v[:, 0] * a[:, 1] - v[:, 1] * a[:, 0]) / np.linalg.norm(v, axis=1) ** 3

    # scale to the lap length, then resample evenly by arc length
    closed = np.vstack([pts, pts[:1]])
    s = np.concatenate([[0.0], np.cumsum(np.linalg.norm(np.diff(closed, axis=0), axis=1))])
    scale = TRACK_LEN_M / s[-1]
    s, closed, curv = s * scale, closed * scale, np.append(curv, curv[0]) / scale
    t = np.arange(0.0, s[-1], DS)
    pts = np.stack([np.interp(t, s, closed[:, 0]), np.interp(t, s, closed[:, 1])], axis=1)
    return pts, np.interp(t, s, curv)

def _overlaps(pts: np.ndarray, width: float) -> bool:
    """True if two parts of the track that aren't neighbours along it come closer than the width plus clearance"""
    p = pts[::10]  # every 1 m is plenty
    n = len(p)
    i = np.arange(n)
    gap = np.abs(i[:, None] - i[None, :])
    along = np.minimum(gap, n - gap) * 10 * DS
    close = np.linalg.norm(p[:, None] - p[None, :], axis=2) < width + CLEARANCE_M
    return bool(np.any(close & (along > 1.6 * (width + CLEARANCE_M))))

def _longest_straight(curv: np.ndarray) -> float:
    """Longest run of the loop gentler than STRAIGHT_RADIUS_M (m)"""
    straight = np.abs(curv) < 1.0 / STRAIGHT_RADIUS_M
    if straight.all():
        return math.inf
    straight = np.roll(straight, -int(np.argmin(straight)))  # start on a bend so no run wraps around the end
    edges = np.flatnonzero(np.diff(np.concatenate([[0], straight.astype(int), [0]])))
    return float(np.max(edges[1::2] - edges[::2], initial=0)) * DS

def _cones(pts: np.ndarray, normal: np.ndarray, curv: np.ndarray, offset: float) -> list[list[float]]:
    """Cones along the boundary `offset` m left of the centreline, closer together in corners"""
    edge = pts + offset * normal
    stretch = 1.0 - offset * curv  # boundary length per centreline length
    radius = np.abs(stretch / np.where(np.abs(curv) < 1e-6, 1e-6, curv))
    gap = np.clip(CONE_GAP_PER_RADIUS * radius, *CONE_GAP_M)
    # start the chain where the gaps are tightest for a couple of metres either side, so the closing gap is no bigger
    # than the rest
    near = np.stack([np.roll(gap, k) for k in range(-25, 26)]).max(axis=0)
    order = np.roll(np.arange(len(edge)), -int(np.argmin(near)))
    u = np.concatenate([[0.0], np.cumsum(stretch[order] * DS / gap[order])])
    n = max(3, math.ceil(u[-1]))  # round up so no gap is longer than asked
    idx = np.interp(np.arange(n) * u[-1] / n, u, np.arange(len(order) + 1))
    e = np.vstack([edge[order], edge[order[:1]]])
    xy = np.stack([np.interp(idx, np.arange(len(e)), e[:, 0]), np.interp(idx, np.arange(len(e)), e[:, 1])], axis=1)
    return xy.round(3).tolist()

def generate(seed: int, max_tries: int = 1000) -> dict:
    """A random track in the track YAML's layout"""
    rng = np.random.default_rng(seed)
    for _ in range(max_tries):
        pts, curv = _loop(rng)
        if rng.random() < 0.5:  # either direction
            pts, curv = pts[::-1].copy(), -curv[::-1]
        width = rng.uniform(*WIDTH_M)
        min_radius = max(TURN_DIAMETER_M / 2.0 - width / 2.0, width / 2.0 + MIN_INNER_RADIUS_M)
        if np.max(np.abs(curv)) > 1.0 / min_radius or _longest_straight(curv) > MAX_STRAIGHT_M \
                or np.sum(np.abs(curv)) * DS < MIN_TURNING * 2.0 * math.pi or _overlaps(pts, width):
            continue

        # start where the track around it is straightest
        lo, hi = round(START_WINDOW_M[0] / DS), round(START_WINDOW_M[1] / DS)
        window = np.stack([np.roll(np.abs(curv), -k) for k in range(lo, hi + 1)]).max(axis=0)
        start = int(np.argmin(window))
        if window[start] > START_MAX_CURV:
            continue

        # start pose at the origin facing +x
        pts, curv = np.roll(pts, -start, axis=0), np.roll(curv, -start)
        tangent = (np.roll(pts, -1, axis=0) - np.roll(pts, 1, axis=0)) / (2.0 * DS)
        tangent /= np.linalg.norm(tangent, axis=1, keepdims=True)
        c, s = tangent[0]
        rot = np.array([[c, s], [-s, c]])
        pts, tangent = (pts - pts[0]) @ rot.T, tangent @ rot.T
        normal = np.stack([-tangent[:, 1], tangent[:, 0]], axis=1)

        # big orange cones before and after the start/finish line (DS 1.1)
        def beside(dist_m: float, offset: float) -> list[float]:
            i = round(dist_m / DS)
            return (pts[i] + offset * normal[i]).round(3).tolist()
        half = width / 2.0
        return {
            "cones_left": _cones(pts, normal, curv, half),
            "cones_right": _cones(pts, normal, curv, -half),
            "cones_orange": [],
            "cones_orange_big": [beside(START_LINE_M + d, o) for d in (-1.3, 1.3) for o in (half + 0.5, -half - 0.5)],
            "starting_pose_front_wing": [0.0, 0.0, 0.0],
            "tk_device": [beside(START_LINE_M, half + 1.0), beside(START_LINE_M, -half - 1.0)],
        }
    raise RuntimeError(f"no usable track after {max_tries} tries (seed {seed})")
