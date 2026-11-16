import math
from collections import deque
from dataclasses import dataclass
import numpy as np

class GaussMarkov:
    """First-order Gauss-Markov process: bounded, time-correlated noise with steady-state std `sigma`"""
    def __init__(self, sigma: float, tau: float, dt: float, rng: np.random.Generator) -> None:
        self._phi = math.exp(-dt / tau)
        self._q = sigma * math.sqrt(1.0 - self._phi * self._phi)
        self._rng = rng
        self.x = rng.normal(0.0, sigma)

    def step(self) -> float:
        self.x = self._phi * self.x + self._q * self._rng.normal()
        return self.x

@dataclass(frozen=True)
class GssConfig:
    """Placeholder values, replace with the datasheet's"""
    rate_hz: float = 250.0
    latency_s: float = 0.010
    min_speed: float = 0.15                         # m/s, optics lose tracking below this
    scale_sigma: float = 0.002                      # fraction, per-run calibration error
    mount_yaw_sigma: float = math.radians(0.25)     # rad, per-run misalignment
    v_bias_sigma: float = 0.02                      # m/s
    v_noise_floor: float = 0.02                     # m/s
    v_noise_frac: float = 0.001                     # fraction of speed
    ang_bias_sigma: float = math.radians(0.05)      # rad
    ang_noise_sigma: float = math.radians(0.1)      # rad
    bias_tau: float = 30.0                          # s
    filter_tau: float = 0.02                        # s, onboard low-pass time constant

class GroundSpeedSensor:
    """Fused optical + IMU ground speed sensor. Noise is applied in (speed, slip angle) space,
    since that's what the optics actually measure"""
    def __init__(self, cfg: GssConfig = GssConfig(), seed: int = 0) -> None:
        dt: float = 1.0 / cfg.rate_hz
        rng: np.random.Generator = np.random.default_rng(seed)
        self._cfg = cfg
        self._scale: float = rng.normal(0.0, cfg.scale_sigma)
        self._mount_yaw: float = rng.normal(0.0, cfg.mount_yaw_sigma)
        self._v_bias = GaussMarkov(cfg.v_bias_sigma, cfg.bias_tau, dt, rng)
        self._ang_bias = GaussMarkov(cfg.ang_bias_sigma, cfg.bias_tau, dt, rng)
        self._v_noise = GaussMarkov(1.0, cfg.filter_tau, dt, rng)  # unit std, scaled by speed in measure()
        self._ang_noise = GaussMarkov(cfg.ang_noise_sigma, cfg.filter_tau, dt, rng)
        self._delay: deque[tuple[float, float]] = deque(maxlen=round(cfg.latency_s * cfg.rate_hz) + 1)

    def measure(self, vx_b: float, vy_b: float) -> tuple[float, float] | None:
        """True body-frame (FLU) velocity in, delayed noisy (vx, vy) out. None below min speed.
        Call once per sample at cfg.rate_hz so the noise correlation times hold"""
        self._delay.append((vx_b, vy_b))
        vx_b, vy_b = self._delay[0]

        # step every process each sample, even when invalid, so they stay on the sample clock
        v_bias: float = self._v_bias.step()
        a_bias: float = self._ang_bias.step()
        v_n: float = self._v_noise.step()
        a_n: float = self._ang_noise.step()

        v: float = math.hypot(vx_b, vy_b)
        if v < self._cfg.min_speed:
            return (0.0, 0.0)
        beta: float = math.atan2(vy_b, vx_b)

        sigma_v: float = self._cfg.v_noise_floor + self._cfg.v_noise_frac * v
        v_m: float = v * (1.0 + self._scale) + v_bias + sigma_v * v_n
        b_m: float = beta + self._mount_yaw + a_bias + a_n
        return v_m * math.cos(b_m), v_m * math.sin(b_m)
