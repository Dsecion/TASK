from __future__ import annotations

from dataclasses import dataclass
from typing import Dict
import numpy as np


@dataclass
class SensorNoise:
    gyro_std: float = np.deg2rad(0.05)     # rad/s
    gyro_bias_walk_std: float = np.deg2rad(0.005)  # rad/s per sqrt(s)
    angle_std: float = np.deg2rad(0.1)     # rad
    baro_std: float = 0.02                 # m
    vel_std: float = 0.02                  # m/s
    seed: int = 42


class Sensors:
    """
    Simple sensor models: gyro (with random walk bias), 'attitude' (truth + noise),
    barometer (z + noise), vertical velocity (vz + noise).
    """

    def __init__(self, noise: SensorNoise) -> None:
        self.noise = noise
        self.rng = np.random.default_rng(noise.seed)
        self._gyro_bias = np.zeros(3)

    def reset(self) -> None:
        self._gyro_bias[:] = 0.0

    def measure(
        self,
        truth: Dict[str, np.ndarray],
        dt: float,
    ) -> Dict[str, float | np.ndarray]:
        # Unpack
        angles = np.asarray(truth["angles"], dtype=float).reshape(3)
        omega = np.asarray(truth["omega"], dtype=float).reshape(3)
        pos = np.asarray(truth["pos"], dtype=float).reshape(3)
        vel = np.asarray(truth["vel"], dtype=float).reshape(3)

        # Gyro bias random walk
        self._gyro_bias += self.rng.normal(0.0, self.noise.gyro_bias_walk_std * np.sqrt(max(dt, 1e-6)), size=3)
        gyro_meas = omega + self._gyro_bias + self.rng.normal(0.0, self.noise.gyro_std, size=3)

        angles_meas = angles + self.rng.normal(0.0, self.noise.angle_std, size=3)
        z_meas = float(pos[2] + self.rng.normal(0.0, self.noise.baro_std))
        vz_meas = float(vel[2] + self.rng.normal(0.0, self.noise.vel_std))

        return {
            "angles": angles_meas,
            "omega": gyro_meas,
            "z": z_meas,
            "vz": vz_meas,
        }


