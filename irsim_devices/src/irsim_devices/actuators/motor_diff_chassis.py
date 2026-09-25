"""Motor-driven differential drive chassis.

A standalone chassis model that operates at the actuator level: each wheel is
driven by an independent :class:`~irsim_devices.actuators.motor.Motor` instance.
The chassis integrates motor physics, encoder state, and 2-D pose kinematics
in one call without relying on ObjectBase or KinematicsFactory.

Typical usage
-------------
::

    from irsim_devices.actuators import Motor, MotorDiffChassis

    chassis = MotorDiffChassis(
        wheel_radius=0.05,         # 5 cm wheels
        wheel_base=0.30,           # 30 cm between wheels
        motor_profile="small_dc",  # both wheels use the same profile
        initial_state=[0, 0, 0],
    )

    # Simulation loop -- command = [left_duty, right_duty] in [-1, 1]
    for _ in range(steps):
        state  = chassis.step([0.6, 0.6], dt=0.05)   # drive straight
        enc    = chassis.encoder_readings              # wheel encoder data
        motors = chassis.motor_state                   # current, omega, …

The command units follow each motor's ``command_mode`` (``"pwm"`` by default
for ``small_dc``).  For ``"velocity"`` mode, commands are in rad/s at the
**output shaft** (wheel shaft).

Sign convention
---------------
Both ``left_cmd`` and ``right_cmd`` are **positive for forward motion**.
If a motor spins in the wrong direction physically, pass
``left_reversed=True`` or ``right_reversed=True`` at construction; the
chassis negates that motor's command and encoder readings transparently.
"""

from __future__ import annotations

import math
from typing import Any

import numpy as np

from irsim_devices.actuators.motor import Motor

_TWO_PI = 2.0 * math.pi


class MotorDiffChassis:
    """Differential drive chassis driven by two independent DC motors.

    Accepts per-wheel motor commands, integrates motor dynamics, updates
    encoder state, and propagates the result to the 2-D chassis pose via
    midpoint (2nd-order) integration.

    Args:
        wheel_radius (float): Wheel radius in metres.
        wheel_base (float): Distance between the two wheel contact points (m).
        left_motor (Motor | None): Pre-built left Motor instance.  When
            ``None`` a Motor is created from ``motor_profile`` / ``left_motor_kwargs``.
        right_motor (Motor | None): Pre-built right Motor instance.
        motor_profile (str | None): Named Motor profile applied to both wheels
            when no explicit Motor instances are given.
        left_motor_kwargs (dict | None): Extra keyword arguments forwarded to
            the left Motor constructor (override profile values).
        right_motor_kwargs (dict | None): Same for the right Motor.
        left_reversed (bool): Negate left motor command and encoder sign.
        right_reversed (bool): Negate right motor command and encoder sign.
        initial_state (array-like): Starting pose ``[x, y, theta]`` (m, m, rad).

    Attributes:
        left_motor (Motor): Left-wheel motor.
        right_motor (Motor): Right-wheel motor.
        wheel_radius (float): Wheel radius (m).
        wheel_base (float): Wheelbase (m).
        state (np.ndarray): Current pose ``[x, y, theta]``, shape (3,).
        linear_velocity (float): Chassis forward velocity (m/s).
        angular_velocity (float): Chassis yaw rate (rad/s).
    """

    def __init__(
        self,
        wheel_radius: float = 0.05,
        wheel_base: float = 0.30,
        left_motor: Motor | None = None,
        right_motor: Motor | None = None,
        motor_profile: str | None = "small_dc",
        left_motor_kwargs: dict[str, Any] | None = None,
        right_motor_kwargs: dict[str, Any] | None = None,
        left_reversed: bool = False,
        right_reversed: bool = False,
        initial_state: list | np.ndarray | None = None,
    ) -> None:
        self.wheel_radius: float = float(wheel_radius)
        self.wheel_base: float = float(wheel_base)
        self._left_sign: float = -1.0 if left_reversed else 1.0
        self._right_sign: float = -1.0 if right_reversed else 1.0

        # Build motors
        def _make(explicit: Motor | None, extra: dict | None) -> Motor:
            if explicit is not None:
                return explicit
            kw: dict[str, Any] = {"profile": motor_profile} if motor_profile else {}
            kw.update(extra or {})
            return Motor(**kw)

        self.left_motor: Motor = _make(left_motor, left_motor_kwargs)
        self.right_motor: Motor = _make(right_motor, right_motor_kwargs)

        # Chassis pose and velocity
        s0 = (
            np.asarray(initial_state, dtype=float).ravel()
            if initial_state is not None
            else np.zeros(3)
        )
        self.state: np.ndarray = np.zeros(3)
        self.state[: min(3, len(s0))] = s0[:3]
        self.linear_velocity: float = 0.0
        self.angular_velocity: float = 0.0

    # ── Main interface ─────────────────────────────────────────────────────────

    def step(
        self,
        command: list | np.ndarray,
        dt: float,
    ) -> np.ndarray:
        """Advance the chassis by one timestep.

        Args:
            command: ``[left_cmd, right_cmd]``.  Units match each motor's
                ``command_mode`` (PWM duty cycle, voltage, or rad/s).
            dt: Simulation timestep (s).  Must be > 0.

        Returns:
            Updated pose ``[x, y, theta]``, shape (3,).
        """
        cmd = np.asarray(command, dtype=float).ravel()
        left_cmd = self._left_sign * float(cmd[0])
        right_cmd = self._right_sign * float(cmd[1])

        # ── Motor dynamics ────────────────────────────────────────────────────
        omega_l = self.left_motor.step(left_cmd, dt) * self._left_sign
        omega_r = self.right_motor.step(right_cmd, dt) * self._right_sign

        # ── Wheel linear velocities (m/s) ─────────────────────────────────────
        v_l = omega_l * self.wheel_radius
        v_r = omega_r * self.wheel_radius

        # ── Differential kinematics ───────────────────────────────────────────
        v = (v_l + v_r) * 0.5
        omega_chassis = (v_r - v_l) / self.wheel_base

        # ── Midpoint integration (2nd-order) ──────────────────────────────────
        theta = float(self.state[2])
        theta_mid = theta + 0.5 * omega_chassis * dt
        theta_new = theta + omega_chassis * dt
        theta_new = (theta_new + math.pi) % _TWO_PI - math.pi

        self.state[0] += v * math.cos(theta_mid) * dt
        self.state[1] += v * math.sin(theta_mid) * dt
        self.state[2] = theta_new

        self.linear_velocity = v
        self.angular_velocity = omega_chassis
        return self.state.copy()

    # ── Accessors ──────────────────────────────────────────────────────────────

    @property
    def encoder_readings(self) -> dict[str, dict[str, float | int]]:
        """Encoder data for both wheels.

        Returns:
            dict with keys ``"left"`` and ``"right"``, each containing:

            * ``ticks``       — cumulative integer encoder ticks.
            * ``theta_output``— cumulative output-shaft angle (rad).
            * ``omega_output``— output-shaft angular velocity (rad/s).
            * ``current``     — armature current (A).
            * ``omega_motor`` — motor-shaft angular velocity (rad/s).
        """
        left = self.left_motor.get_encoder()
        right = self.right_motor.get_encoder()
        # Apply sign correction for reversed motors so positive = forward
        left["omega_output"] *= self._left_sign
        left["theta_output"] *= self._left_sign
        left["ticks"] = round(left["ticks"] * self._left_sign)
        right["omega_output"] *= self._right_sign
        right["theta_output"] *= self._right_sign
        right["ticks"] = round(right["ticks"] * self._right_sign)
        return {"left": left, "right": right}

    @property
    def motor_state(self) -> dict[str, dict[str, float]]:
        """Raw motor-shaft state for both wheels (before gear reduction).

        Returns:
            dict with keys ``"left"`` and ``"right"``, each containing:

            * ``omega``   — motor-shaft angular velocity (rad/s).
            * ``current`` — armature current (A).
            * ``voltage`` — last applied armature voltage (V); estimated as
              ``Ra·i + Ke·ω`` (ignores La drop).
        """

        def _state(m: Motor, sign: float) -> dict[str, float]:
            V_est = m.Ra * m.current + m.Ke * m.omega
            return {
                "omega": m.omega * sign,
                "current": m.current,
                "voltage_est": V_est * sign,
                "omega_output": m.omega_output * sign,
            }

        return {
            "left": _state(self.left_motor, self._left_sign),
            "right": _state(self.right_motor, self._right_sign),
        }

    def reset(self, state: list | np.ndarray | None = None) -> None:
        """Reset motors and chassis pose to rest.

        Args:
            state: Optional new initial pose ``[x, y, theta]``.  Defaults to
                all-zero.
        """
        self.left_motor.reset()
        self.right_motor.reset()
        if state is not None:
            s = np.asarray(state, dtype=float).ravel()
            self.state = np.zeros(3)
            self.state[: min(3, len(s))] = s[:3]
        else:
            self.state = np.zeros(3)
        self.linear_velocity = 0.0
        self.angular_velocity = 0.0
