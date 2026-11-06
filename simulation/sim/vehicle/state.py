from dataclasses import dataclass

import numpy as np

@dataclass
class VehicleInput:
    speed_rpm_fl: float = 0.0
    speed_rpm_fr: float = 0.0
    speed_rpm_rl: float = 0.0
    speed_rpm_rr: float = 0.0
    torque_lim_fl: float = 0.0
    torque_lim_fr: float = 0.0
    torque_lim_rl: float = 0.0
    torque_lim_rr: float = 0.0
    wheel_steer_rad: float = 0.0

    @property
    def speed_rpm(self) -> np.ndarray:
        return np.array([self.speed_rpm_fl, self.speed_rpm_fr, self.speed_rpm_rl, self.speed_rpm_rr])

    @property
    def torque_lim(self) -> np.ndarray:
        return np.array([self.torque_lim_fl, self.torque_lim_fr, self.torque_lim_rl, self.torque_lim_rr])
