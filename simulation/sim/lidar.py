"""Ouster OS1-128 lidar: analytic ray/cone + ray/ground caster with a distance-based noise model"""
import jax
import jax.numpy as jnp
import numpy as np

from .scene import Scene, LIDAR_X_M, LIDAR_Y_M, LIDAR_YAW, SENSOR_Z

N_AZIM: int = 512
N_ELEV: int = 64
ELEV_MIN: float = np.deg2rad(-22.0)
ELEV_MAX: float = 0.0 # We'll never actually hit anything above this angle on the real track. No point in simulating this part
MIN_RANGE: float = 0.5
MAX_RANGE: float = 50.0

_EPS: float = 1e-7

_elev, _azim = jnp.meshgrid(
    jnp.linspace(ELEV_MIN, ELEV_MAX, N_ELEV),
    jnp.linspace(-jnp.pi, jnp.pi, N_AZIM, endpoint=False),
    indexing="ij",
)
EL_FLAT: jax.Array = _elev.ravel().astype(jnp.float32)
AZ_FLAT: jax.Array = _azim.ravel().astype(jnp.float32)
RAYS_LOCAL: jax.Array = jnp.stack(
    [jnp.cos(EL_FLAT) * jnp.cos(AZ_FLAT),
     jnp.cos(EL_FLAT) * jnp.sin(AZ_FLAT),
     jnp.sin(EL_FLAT)], axis=1).astype(jnp.float32)

def _cone_dist(origin, dirs, cone):
    """Ray/cone distance per beam. origin (3,), dirs (N,3), cone=[cx,cy,r,h].
    Side surface (x-cx)^2+(y-cy)^2 = (r/h)^2 (h-z)^2 -> quadratic A t^2+B t+C=0"""
    cx, cy, r, h = cone[0], cone[1], cone[2], cone[3]
    ox, oy, oz = origin[0] - cx, origin[1] - cy, origin[2]
    dx, dy, dz = dirs[:, 0], dirs[:, 1], dirs[:, 2]
    k2 = (r / h) ** 2
    g = h - oz
    A = dx * dx + dy * dy - k2 * dz * dz
    B = 2.0 * (ox * dx + oy * dy + k2 * g * dz)
    C = ox * ox + oy * oy - k2 * g * g
    disc = B * B - 4.0 * A * C
    sqrt_disc = jnp.sqrt(jnp.maximum(disc, 0.0))

    A_safe = jnp.where(jnp.abs(A) < _EPS, _EPS, A)
    t0 = (-B - sqrt_disc) / (2.0 * A_safe)
    t1 = (-B + sqrt_disc) / (2.0 * A_safe)
    t_lin = -C / jnp.where(jnp.abs(B) < _EPS, _EPS, B) # A~0: linear fallback
    INF = jnp.float32(jnp.inf)

    def on_real_cone(t, usable):
        z = oz + t * dz
        return usable & (t > MIN_RANGE) & (t < MAX_RANGE) & (z >= 0.0) & (z <= h)

    is_quad = jnp.abs(A) >= _EPS
    has_roots = disc >= 0.0
    c0 = jnp.where(on_real_cone(t0, is_quad & has_roots), t0, INF)
    c1 = jnp.where(on_real_cone(t1, is_quad & has_roots), t1, INF)
    cl = jnp.where(on_real_cone(t_lin, ~is_quad & (jnp.abs(B) >= _EPS)), t_lin, INF)
    t_side = jnp.minimum(jnp.minimum(c0, c1), cl)

    dz_safe = jnp.where(jnp.abs(dz) < _EPS, _EPS, dz)
    t_base = -oz / dz_safe # base disk at z=0
    bx, by = ox + t_base * dx, oy + t_base * dy
    base_ok = (dz < -_EPS) & (t_base > MIN_RANGE) & (t_base < MAX_RANGE) & (bx * bx + by * by <= r * r)
    t_base = jnp.where(base_ok, t_base, INF)

    t_best = jnp.minimum(t_side, t_base)
    return t_best, jnp.isfinite(t_best)


BUMP_AMP: float = 0.01 # ground height variation 0 -> flat plane

def _ground_height(x, y):
    """Smooth world-fixed surface unevenness"""
    return BUMP_AMP * (jnp.sin(1.3 * x + 0.5 * y) + 0.6 * jnp.sin(2.9 * y - 1.1 * x) + 0.4 * jnp.sin(4.7 * x))

def _ground_dist(origin, dirs):
    """Ray vs bumpy ground z=bump(x,y); only downward beams hit. 2 fixed-point iters."""
    dz = dirs[:, 2]
    dz_safe = jnp.where(jnp.abs(dz) < _EPS, _EPS, dz)
    t = -origin[2] / dz_safe # flat z=0 start
    for _ in range(2): # solve o_z + t dz = bump(hit_xy)
        hx, hy = origin[0] + t * dirs[:, 0], origin[1] + t * dirs[:, 1]
        t = (_ground_height(hx, hy) - origin[2]) / dz_safe
    return t, (dz < -_EPS) & (t > MIN_RANGE) & (t < MAX_RANGE)

def _raycast(origin, R, geoms):
    """Nearest hit across all cones + ground, per beam. R is sensor->world."""
    dirs = RAYS_LOCAL @ R.T
    n = RAYS_LOCAL.shape[0]

    def keep_closest(best_t, cone):
        t, hit = _cone_dist(origin, dirs, cone)
        return jnp.where(hit & (t < best_t), t, best_t), None

    t_cone, _ = jax.lax.scan(keep_closest, jnp.full((n,), jnp.inf, jnp.float32), geoms)
    t_g, hit_g = _ground_dist(origin, dirs)
    t = jnp.minimum(t_cone, jnp.where(hit_g, t_g, jnp.inf))
    hit = jnp.isfinite(t)
    return jnp.where(hit, t, 0.0).astype(jnp.float32), hit


SIGMA0: float = 0.005 # range 1-sigma
SIGMA_SLOPE: float = 1.5e-4
DROP_RANGE: float = 35.0 # p_keep = 1 - r/DROP_RANGE
ANG_SIGMA: float = np.deg2rad(0.01)
FALSE_POS_RATE: float = 1.0e-4

def range_sigma(ranges):
    return SIGMA0 + SIGMA_SLOPE * jnp.clip(ranges, 0.0, MAX_RANGE)

def detection_prob(ranges):
    return jnp.clip(1.0 - ranges / DROP_RANGE, 0.0, 1.0)

def _apply_noise(ranges, key, hit):
    """-> (noisy_range, keep, d_az, d_el), all (N,)."""
    n = ranges.shape[0]
    k_prec, k_drop, k_fp, k_ang = jax.random.split(key, 4)

    noisy = ranges + range_sigma(ranges) * jax.random.normal(k_prec, (n,))
    keep = hit & (jax.random.uniform(k_drop, (n,)) < detection_prob(ranges))

    miss = ~hit
    fired = miss & (jax.random.uniform(k_fp, (n,)) < FALSE_POS_RATE)
    fp_range = jax.random.uniform(jax.random.fold_in(k_fp, 1), (n,), minval=1.0, maxval=MAX_RANGE)
    noisy = jnp.clip(jnp.where(fired, fp_range, noisy), 0.0, MAX_RANGE)

    k_az, k_el = jax.random.split(k_ang)
    d_az = ANG_SIGMA * jax.random.normal(k_az, (n,))
    d_el = ANG_SIGMA * jax.random.normal(k_el, (n,))
    return noisy, keep | fired, d_az, d_el


def _sensor_pose(pose):
    """(x,y,psi) -> (origin (3,), R (3,3) sensor->world). Mount is the base_link -> lidar static transform."""
    x, y, psi = pose[0], pose[1], pose[2]
    c, s = jnp.cos(psi), jnp.sin(psi)
    cl, sl = jnp.cos(psi + LIDAR_YAW), jnp.sin(psi + LIDAR_YAW)
    R = jnp.array([[cl, -sl, 0.0], [sl, cl, 0.0], [0.0, 0.0, 1.0]], jnp.float32)
    origin = jnp.array([x + c * LIDAR_X_M - s * LIDAR_Y_M, y + s * LIDAR_X_M + c * LIDAR_Y_M, SENSOR_Z], jnp.float32)
    return origin, R


@jax.jit
def _scan_dense(pose, key, cone_xy, cone_r, cone_h, up, off_xy):
    origin, R = _sensor_pose(pose)
    eff_xy = cone_xy + off_xy
    eff_h = jnp.where(up > 0.5, cone_h, 0.02)            # downed cone -> ~flat
    geoms = jnp.stack([eff_xy[:, 0], eff_xy[:, 1], cone_r, eff_h], axis=1)

    ranges, hit = _raycast(origin, R, geoms)
    ranges, keep, d_az, d_el = _apply_noise(ranges, key, hit)

    el, az = EL_FLAT + d_el, AZ_FLAT + d_az
    ce = jnp.cos(el)
    dirs = jnp.stack([ce * jnp.cos(az), ce * jnp.sin(az), jnp.sin(el)], axis=1)
    return (ranges[:, None] * dirs).astype(jnp.float32), keep


class LidarSensor:
    def __init__(self, scene: Scene, seed: int = 0) -> None:
        self.cone_xy = jnp.asarray(scene.cone_xy, jnp.float32)
        self.cone_r = jnp.asarray(scene.cone_r, jnp.float32)
        self.cone_h = jnp.asarray(scene.cone_h, jnp.float32)
        m = scene.cone_xy.shape[0]
        self.up = jnp.ones(m, jnp.float32)
        self.off_xy = jnp.zeros((m, 2), jnp.float32)
        self._key = jax.random.PRNGKey(seed)

    def scan(self, pose) -> np.ndarray:
        """One revolution from pose=(x,y,psi) -> surviving points in sensor frame (N,3)."""
        self._key, sub = jax.random.split(self._key)
        pts, keep = _scan_dense(jnp.asarray(pose, jnp.float32), sub,
                                self.cone_xy, self.cone_r, self.cone_h, self.up, self.off_xy)
        keep = np.asarray(keep)
        return np.asarray(pts)[keep].astype(np.float32)
