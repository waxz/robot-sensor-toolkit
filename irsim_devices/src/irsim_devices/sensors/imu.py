"""Simulated IMU sensor with IEEE 517-style and Gaussian noise models.

Implements two noise models:

* ``"ieee517"`` (default) — white noise (angle/velocity random walk) and
  bias random walk on both gyroscope and accelerometer axes, consistent with
  the IEEE 517 Inertial Sensor Terminology Standard.
* ``"gaussian"`` — simple per-axis Gaussian noise without bias drift;
  useful for quick experimentation or when bias dynamics are not needed.

Both models produce 3-DOF outputs:

* ``angular_velocity`` — np.ndarray shape (3,): [ωx, ωy, ωz] (rad/s).
* ``linear_acceleration`` — np.ndarray shape (3,): [ax, ay, az] (m/s²),
  where az includes the static gravity component (+g ≈ 9.807 m/s² for a
  level ground robot).

For a 2-D ground-plane robot the true ωx = ωy = 0 and az = +g, so the
out-of-plane axes carry only sensor noise; a 2-D pose estimator only needs
the relevant sub-components (ωz and [ax, ay]).
"""

from __future__ import annotations

from typing import TYPE_CHECKING, ClassVar, Literal

import numpy as np

from irsim_devices.core.random_utils import _generator as _rng_generator

if TYPE_CHECKING:
    from irsim_devices.core.world_model import GeometryObject2D as ObjectBase

_G: float = 9.80665  # standard gravity (m/s²)


class IMU:
    """Simulated 3-D IMU sensor (gyroscope + accelerometer).

    Derives ground-truth angular velocity and linear acceleration from the
    parent object's state history using finite differences, then corrupts
    the signal with a configurable noise model.

    For a 2-D planar robot the true angular-velocity vector is
    [0, 0, ωz] and the true accelerometer vector is [ax_body, ay_body, +g].
    Out-of-plane gyro axes (ωx, ωy) are zero plus noise; the gravity axis
    (az) measures the static +g with additive sensor noise.

    Default parameters match a consumer-grade MEMS IMU (MPU-6050).

    Args:
        state (np.ndarray): Initial [x, y, theta] state of the parent object.
        obj_id (int): ID of the associated object.
        gyro_noise_std (float): Gyroscope white-noise density (rad/s/√Hz).
        accel_noise_std (float): Accelerometer white-noise density (m/s²/√Hz).
        gyro_bias_walk_std (float): Gyroscope bias random-walk rate (rad/s/√s).
            Ignored when ``noise_model="gaussian"``.
        accel_bias_walk_std (float): Accelerometer bias random-walk rate
            (m/s²/√s).  Ignored when ``noise_model="gaussian"``.
        step_time (float): Simulation step time in seconds.
        noise (bool): Enable noise and bias. Set False for ground-truth output.
        noise_model (str): ``"ieee517"`` (white noise + bias random walk) or
            ``"gaussian"`` (simple per-axis Gaussian, no drift).
        profile (str | None): Named sensor profile from :attr:`PROFILES`.
            Overrides explicit noise parameters when provided.
        shock_prob (float): Per-step probability of an impulsive shock event
            (0.0 = none).  Models wheel bumps on uneven terrain or collisions.
        shock_accel_std (float): Standard deviation of the impulsive
            acceleration spike (m/s²) when a shock event fires.
        shock_gyro_std (float): Standard deviation of the impulsive
            angular-rate spike (rad/s) when a shock event fires.
        gravity (float): Gravity constant (m/s²) added to the az measurement.
            Default 9.80665 m/s².
        **kwargs: Ignored extra keyword arguments passed by SensorFactory.

    Attr:
        sensor_type (str): ``"imu"``.
        angular_velocity (np.ndarray): Latest gyroscope measurement,
            shape (3,): [ωx, ωy, ωz] (rad/s).
        linear_acceleration (np.ndarray): Latest accelerometer measurement,
            shape (3,): [ax, ay, az] (m/s²), body frame, az includes gravity.
        gyro_bias (np.ndarray): Current gyroscope bias, shape (3,) (rad/s).
        accel_bias (np.ndarray): Current accelerometer bias, shape (3,) (m/s²).
        parent (ObjectBase | None): Owning simulation object; set externally.
    """

    # ── Named IMU profiles (noise spectral densities & bias walk rates) ──────
    #
    # All values in SI:  N_g  rad/s/√Hz,  N_a  m/s²/√Hz,
    #                    K_g  rad/s/√s,   K_a  m/s²/√s
    #
    # Sources: product datasheets; µg converted via g=9.80665 m/s²
    #
    #  mpu6050     InvenSense MPU-6050  (budget mobile-robot/drone)
    #              N_g = 0.005 °/s/√Hz  = 8.73e-5 rad/s/√Hz
    #              N_a = 400 µg/√Hz     = 3.92e-3 m/s²/√Hz
    #  bmi088      Bosch BMI-088  (Pixhawk, mid-range robots)
    #              N_g = 0.014 °/s/√Hz  = 2.44e-4 rad/s/√Hz
    #              N_a = 230 µg/√Hz     = 2.26e-3 m/s²/√Hz
    #  icm42688    TDK ICM-42688-P  (high-performance mobile robots)
    #              N_g = 0.0028 °/s/√Hz = 4.89e-5 rad/s/√Hz
    #              N_a = 70 µg/√Hz      = 6.87e-4 m/s²/√Hz
    #  adis16448   Analog Devices ADIS-16448  (navigation grade)
    #              N_g = 0.066 °/s/√Hz  = 1.15e-3 rad/s/√Hz
    #              N_a = 0.158 mg/√Hz   = 1.55e-3 m/s²/√Hz
    PROFILES: ClassVar[dict[str, dict[str, float]]] = {
        "mpu6050": {
            "gyro_noise_std": 8.73e-5,
            "accel_noise_std": 3.92e-3,
            "gyro_bias_walk_std": 1.75e-4,
            "accel_bias_walk_std": 1.96e-4,
        },
        "bmi088": {
            "gyro_noise_std": 2.44e-4,
            "accel_noise_std": 2.26e-3,
            "gyro_bias_walk_std": 3.49e-5,
            "accel_bias_walk_std": 1.96e-4,
        },
        "icm42688": {
            "gyro_noise_std": 4.89e-5,
            "accel_noise_std": 6.87e-4,
            "gyro_bias_walk_std": 8.73e-6,
            "accel_bias_walk_std": 9.81e-5,
        },
        "adis16448": {
            "gyro_noise_std": 1.15e-3,
            "accel_noise_std": 1.55e-3,
            "gyro_bias_walk_std": 2.91e-5,
            "accel_bias_walk_std": 9.81e-5,
        },
    }

    def __init__(
        self,
        state: np.ndarray | None = None,
        obj_id: int = 0,
        gyro_noise_std: float = 8.73e-5,
        accel_noise_std: float = 3.92e-3,
        gyro_bias_walk_std: float = 1.75e-4,
        accel_bias_walk_std: float = 1.96e-4,
        step_time: float = 0.001,
        imu_rate: int = 0,
        noise: bool = True,
        noise_model: Literal["ieee517", "gaussian"] = "ieee517",
        profile: str | None = None,
        shock_prob: float = 0.0,
        shock_accel_std: float = 5.0,
        shock_gyro_std: float = 0.5,
        gravity: float = _G,
        **kwargs,
    ) -> None:
        self.sensor_type = "imu"
        self.obj_id = obj_id
        self.noise = noise
        self.step_time = step_time
        self.gravity = float(gravity)

        # Sub-stepping: how many IMU ticks fit inside one sim step.
        # imu_rate=0 (default) keeps the original single-step behaviour.
        # imu_rate=1000 with step_time=0.05 gives n_sub=50 IMU ticks per sim tick.
        imu_rate = int(imu_rate)
        if imu_rate > 0:
            self._n_sub: int = max(1, round(step_time * imu_rate))
            self._imu_dt: float = step_time / self._n_sub
        else:
            self._n_sub = 1
            self._imu_dt = step_time

        if noise_model not in ("ieee517", "gaussian"):
            raise ValueError(
                f"Unknown noise_model '{noise_model}'. Use 'ieee517' or 'gaussian'."
            )
        self.noise_model: str = noise_model

        # Apply named profile if given (overrides explicit params)
        if profile is not None:
            if profile not in self.PROFILES:
                raise ValueError(
                    f"Unknown IMU profile '{profile}'. Available: {list(self.PROFILES)}"
                )
            p = self.PROFILES[profile]
            gyro_noise_std = p["gyro_noise_std"]
            accel_noise_std = p["accel_noise_std"]
            gyro_bias_walk_std = p["gyro_bias_walk_std"]
            accel_bias_walk_std = p["accel_bias_walk_std"]

        # Noise spectral densities
        self._N_g = gyro_noise_std
        self._N_a = accel_noise_std
        # Bias random-walk rate (used by ieee517 model only)
        self._K_g = gyro_bias_walk_std
        self._K_a = accel_bias_walk_std
        # Impulsive shock model (mobile robot bumping on uneven terrain)
        self._shock_prob = float(shock_prob)
        self._shock_accel_std = float(shock_accel_std)
        self._shock_gyro_std = float(shock_gyro_std)

        # Running bias state — 3-D (one per axis)
        self.gyro_bias: np.ndarray = np.zeros(3)
        self.accel_bias: np.ndarray = np.zeros(3)

        # Previous-step state for finite differences
        if state is not None:
            s = np.asarray(state).ravel()
            self._prev_pos: np.ndarray = s[:2].copy()
            self._prev_theta: float = float(s[2])
        else:
            self._prev_pos = np.zeros(2)
            self._prev_theta = 0.0
        self._prev_vel_world: np.ndarray = np.zeros(2)

        # Outputs — 3-D: [ωx, ωy, ωz] and [ax, ay, az]
        self.angular_velocity: np.ndarray = np.zeros(3)
        self.linear_acceleration: np.ndarray = np.array([0.0, 0.0, self.gravity])
        # Batch outputs (n_sub, 3) — populated when n_sub > 1; None otherwise.
        self.angular_velocity_batch: np.ndarray | None = None
        self.linear_acceleration_batch: np.ndarray | None = None

        # Visualisation / compatibility stubs
        self.parent: ObjectBase | None = None
        self.plot_patch_list: list = []
        self.plot_line_list: list = []
        self.plot_text_list: list = []

    # ------------------------------------------------------------------
    # Main interface
    # ------------------------------------------------------------------

    def step(self, state: np.ndarray) -> None:
        """Update IMU measurements from the current parent state.

        When ``imu_rate > 0`` was given at construction the step generates
        ``n_sub`` independent noisy measurements by distributing the
        ground-truth signal uniformly across all sub-steps and drawing
        independent noise at the IMU rate.  The full batch is stored in
        ``angular_velocity_batch`` / ``linear_acceleration_batch`` (shape
        ``(n_sub, 3)``), and the scalar ``angular_velocity`` /
        ``linear_acceleration`` attributes hold the last sub-step for
        backward compatibility.

        Args:
            state (np.ndarray): Current [x, y, theta] from the parent object.
        """
        s = np.asarray(state).ravel()
        pos = s[:2]
        theta = float(s[2])
        sim_dt = self.step_time
        imu_dt = self._imu_dt
        _rng = _rng_generator()  # bypass proxy on hot path

        # ── Ground-truth signals (constant across sub-steps for linear interp) ──
        omega_z_true = (theta - self._prev_theta) / sim_dt
        vel_world = (pos - self._prev_pos) / sim_dt
        dv_world = vel_world - self._prev_vel_world
        accel_world_2d = dv_world / sim_dt

        c, s_theta = np.cos(theta), np.sin(theta)
        R_inv = np.array([[c, s_theta], [-s_theta, c]])
        accel_body_2d = R_inv @ accel_world_2d

        omega_true = np.array([0.0, 0.0, omega_z_true])
        accel_true = np.array([accel_body_2d[0], accel_body_2d[1], self.gravity])

        n = self._n_sub

        if not self.noise:
            if n == 1:
                omega_meas = omega_true.copy()
                accel_meas = accel_true.copy()
                self.angular_velocity_batch = None
                self.linear_acceleration_batch = None
            else:
                omega_batch = np.broadcast_to(omega_true, (n, 3)).copy()
                accel_batch = np.broadcast_to(accel_true, (n, 3)).copy()
                self.angular_velocity_batch = omega_batch
                self.linear_acceleration_batch = accel_batch
                omega_meas = omega_batch[-1]
                accel_meas = accel_batch[-1]
        else:
            sigma_g = self._N_g / np.sqrt(imu_dt)
            sigma_a = self._N_a / np.sqrt(imu_dt)

            if self.noise_model == "ieee517":
                # Vectorised bias random walk: n increments at the IMU rate
                dB_g = self._K_g * np.sqrt(imu_dt) * _rng.standard_normal((n, 3))
                dB_a = self._K_a * np.sqrt(imu_dt) * _rng.standard_normal((n, 3))
                # Bias at each sub-step: start from current stored bias
                bias_g = self.gyro_bias + np.cumsum(dB_g, axis=0)
                bias_a = self.accel_bias + np.cumsum(dB_a, axis=0)
                # Advance stored bias to final state
                self.gyro_bias = bias_g[-1].copy()
                self.accel_bias = bias_a[-1].copy()

                omega_batch = (
                    omega_true + bias_g + sigma_g * _rng.standard_normal((n, 3))
                )
                accel_batch = (
                    accel_true + bias_a + sigma_a * _rng.standard_normal((n, 3))
                )
            else:  # gaussian
                omega_batch = omega_true + sigma_g * _rng.standard_normal((n, 3))
                accel_batch = accel_true + sigma_a * _rng.standard_normal((n, 3))

            # Impulsive shock — applied to a random sub-step
            if self._shock_prob > 0.0:
                shock_mask = _rng.random(n) < self._shock_prob
                if shock_mask.any():
                    omega_batch[shock_mask] += (
                        self._shock_gyro_std
                        * _rng.standard_normal((shock_mask.sum(), 3))
                    )
                    accel_batch[shock_mask] += (
                        self._shock_accel_std
                        * _rng.standard_normal((shock_mask.sum(), 3))
                    )

            if n == 1:
                omega_meas = omega_batch[0]
                accel_meas = accel_batch[0]
                self.angular_velocity_batch = None
                self.linear_acceleration_batch = None
            else:
                self.angular_velocity_batch = omega_batch
                self.linear_acceleration_batch = accel_batch
                omega_meas = omega_batch[-1]
                accel_meas = accel_batch[-1]

        # Store outputs (scalar — backward compat)
        self.angular_velocity = omega_meas
        self.linear_acceleration = accel_meas

        # Advance history
        self._prev_pos = pos.copy()
        self._prev_theta = theta
        self._prev_vel_world = vel_world.copy()

    def get_measurement(self) -> dict:
        """Return the most recent IMU reading as a dictionary.

        Returns:
            dict: Keys ``angular_velocity`` (np.ndarray shape (3,), rad/s) and
            ``linear_acceleration`` (np.ndarray shape (3,), m/s², body frame,
            az includes gravity).
        """
        return {
            "angular_velocity": self.angular_velocity.copy(),
            "linear_acceleration": self.linear_acceleration.copy(),
        }
