# [ZAV 변경] 기존 PX4 body IMU를 유지하고, 별도 장착 원점의 specific force를 계산합니다.
"""Rigidly mounted ROS-frame IMU, independent of the PX4 body IMU.

This is an ideal rigid-body model with optional white noise, not a calibrated
model of the physical Livox IMU. Mount coordinates use body FLU, meters, ZYX deg.
"""
import numpy as np
from scipy.spatial.transform import Rotation

from pegasus.simulator.logic.sensors.sensor import Sensor


class MountedIMU(Sensor):
    def __init__(self, config):
        rate = float(config.get("update_rate", 250.0))
        if not np.isfinite(rate) or rate <= 0:
            raise ValueError("Mounted IMU update_rate must be positive")
        super().__init__(sensor_type="MountedIMU", update_rate=rate)
        self.config = dict(config)
        self._position = np.asarray(config["position"], dtype=float)
        self._sensor_to_body = Rotation.from_euler("ZYX", config["orientation"], degrees=True)
        self._body_to_sensor = self._sensor_to_body.inv()
        self._gyro_noise = float(config.get("gyro_noise_density", 0.0))
        self._accel_noise = float(config.get("accel_noise_density", 0.0))
        if min(self._gyro_noise, self._accel_noise) < 0:
            raise ValueError("IMU noise densities must be nonnegative")
        self.reset()

    def reset(self):
        self._previous_omega = None
        self._elapsed = 0.0
        self._state = None

    def start(self):
        self.reset()

    @property
    def state(self):
        return self._state

    def update(self, state, dt):
        # Differentiate on every physics tick, even when output is decimated.
        if dt <= 0:
            return None
        omega = np.asarray(state.angular_velocity, dtype=float)
        alpha = np.zeros(3) if self._previous_omega is None else (omega - self._previous_omega) / dt
        self._previous_omega = omega.copy()
        self._elapsed += dt
        if self._elapsed + 1e-10 < self._update_period:
            return None
        sample_dt = self._elapsed
        self._elapsed %= self._update_period
        # Suppress floating-point residues close to a complete sample period.
        if self._update_period - self._elapsed < 1e-10:
            self._elapsed = 0.0
        world_to_body = Rotation.from_quat(state.attitude).inv()
        specific_force = world_to_body.apply(
            np.asarray(state.linear_acceleration) - np.array([0.0, 0.0, -9.80665])
        )
        specific_force += np.cross(alpha, self._position)
        specific_force += np.cross(omega, np.cross(omega, self._position))
        gyro = self._body_to_sensor.apply(omega)
        accel = self._body_to_sensor.apply(specific_force)
        gyro_variance = self._gyro_noise ** 2 / sample_dt
        accel_variance = self._accel_noise ** 2 / sample_dt
        if gyro_variance:
            gyro += np.random.normal(0.0, np.sqrt(gyro_variance), 3)
        if accel_variance:
            accel += np.random.normal(0.0, np.sqrt(accel_variance), 3)
        self._state = {
            "angular_velocity": gyro,
            "linear_acceleration": accel,
            "gyro_variance": gyro_variance,
            "accel_variance": accel_variance,
            "config": self.config,
        }
        return self._state
