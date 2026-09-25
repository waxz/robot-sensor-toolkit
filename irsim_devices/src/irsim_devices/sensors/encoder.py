"""Wheel encoder sensor that reads state from a parent chassis object.

Pairs naturally with :class:`~irsim_devices.actuators.MotorDiffChassis`,
which already exposes an ``encoder_readings`` property, but works with any
parent object exposing the same duck-typed interface — no dependency on
ir-sim.

.. code-block:: python

    from irsim_devices.actuators import MotorDiffChassis
    from irsim_devices.sensors import Encoder

    chassis = MotorDiffChassis(motor_profile="dynamixel_xl430")
    encoder = Encoder(profile="dynamixel_xl430")
    encoder.parent = chassis

    for _ in range(1000):
        chassis.step([0.6, 0.6], dt=0.01)
        encoder.step(chassis.state)
        readings = encoder.get_measurement()  # {"left": {...}, "right": {...}}
"""

from __future__ import annotations

from typing import Any, ClassVar


class Encoder:
    """Wheel encoder sensor with named motor+encoder profiles.

    Reads per-wheel encoder data from the parent object's ``encoder_readings``
    property each :meth:`step` — no attachment or setup required beyond
    assigning :attr:`parent`.

    Named profiles document the expected CPR for a matching
    :class:`~irsim_devices.actuators.motor.Motor` preset of the same name
    (see ``Motor.PROFILES``):

    =================== ============================== ============
    Profile             Motor                          CPR
    =================== ============================== ============
    ``small_dc``        46:1 brushed gearmotor         2048
    ``agv_hub_motor``   BLDC hub motor (direct drive)  1024
    ``forklift_drive``  Heavy brush motor, 20:1 chain  500
    ``dynamixel_xl430`` ROBOTIS XL430-W250-T (46.13:1) 4096
    ``pololu_37d_50``   Pololu 37D 50:1 gearmotor       3200
    ``maxon_ec45_43``   Maxon EC45 + GP42C 43:1         4096
    =================== ============================== ============

    Args:
        state: Initial [x, y, theta] state (unused; kept for factory API parity).
        obj_id: ID of the associated object.
        profile: Named encoder+motor preset.  ``None`` means use explicit params.
        motor: Motor preset name this encoder is paired with (informational).
            Ignored when ``profile`` is set.
        encoder_cpr: Encoder counts per revolution (informational).
            Ignored when ``profile`` is set.
        **kwargs: Ignored extra keyword arguments passed by SensorFactory.

    Attr:
        sensor_type (str): ``"encoder"``.
        profile (str | None): Active profile name.
        motor (str): Motor preset name this encoder is paired with.
        encoder_cpr (int): Encoder counts per revolution.
        parent (Any): Object exposing ``encoder_readings``, e.g. a
            :class:`~irsim_devices.actuators.MotorDiffChassis`.
        data (dict): Latest encoder readings keyed by wheel name.
    """

    sensor_type: str = "encoder"

    # Named motor + encoder presets — mirrors Motor.PROFILES cpr values.
    PROFILES: ClassVar[dict[str, dict[str, Any]]] = {
        # 46:1 brushed planetary gearmotor; 48-64 CPR motor shaft x 46 = ~2200 CPR wheel
        "small_dc": {"motor": "small_dc", "encoder_cpr": 2048},
        # BLDC hub motor, direct drive (FOC); 512-4096 pulse/rev magnetic encoder
        "agv_hub_motor": {"motor": "agv_hub_motor", "encoder_cpr": 1024},
        # Heavy brush motor, 20:1 chain reduction; industrial resolver or disk encoder
        "forklift_drive": {"motor": "forklift_drive", "encoder_cpr": 500},
        # ROBOTIS XL430-W250-T; 46.13:1 planetary; 12-bit absolute encoder = 4096 CPR
        "dynamixel_xl430": {"motor": "dynamixel_xl430", "encoder_cpr": 4096},
        # Pololu 37D 50:1 gearmotor; 64 CPR motor x 50 = 3200 CPR at wheel
        "pololu_37d_50": {"motor": "pololu_37d_50", "encoder_cpr": 3200},
        # Maxon EC45 flat + GP42C 43:1; 2048 CPR motor-shaft encoder
        "maxon_ec45_43": {"motor": "maxon_ec45_43", "encoder_cpr": 4096},
    }

    def __init__(
        self,
        state=None,
        obj_id: int = 0,
        profile: str | None = None,
        motor: str = "small_dc",
        encoder_cpr: int = 0,
        **kwargs: Any,
    ) -> None:
        self.obj_id = obj_id
        self.parent: Any | None = None
        self.data: dict[str, dict[str, Any]] = {}

        if profile is not None:
            if profile not in self.PROFILES:
                raise ValueError(
                    f"Unknown encoder profile {profile!r}. "
                    f"Available: {list(self.PROFILES)}"
                )
            p = self.PROFILES[profile]
            self.motor: str = p["motor"]
            self.encoder_cpr: int = p["encoder_cpr"]
        else:
            self.motor = motor
            self.encoder_cpr = int(encoder_cpr)

        self.profile: str | None = profile

    def step(self, state) -> None:
        """Read the latest encoder data from ``parent.encoder_readings``.

        Args:
            state: Current [x, y, theta] state of the parent (unused here;
                kept for factory API parity with the other sensors).
        """
        if self.parent is None:
            return
        readings = getattr(self.parent, "encoder_readings", None)
        if readings is not None:
            self.data = readings

    def get_measurement(self) -> dict[str, dict[str, Any]]:
        """Return the latest encoder readings.

        Returns:
            Dict keyed by wheel name (e.g. ``"left"``/``"right"``).  The
            per-wheel contents mirror whatever ``parent.encoder_readings``
            reports — for :class:`~irsim_devices.actuators.MotorDiffChassis`
            that is ``{"ticks", "theta_output", "omega_output", "current",
            "omega_motor", "tick_delta", "velocity_estimate"}``.
        """
        return dict(self.data)
