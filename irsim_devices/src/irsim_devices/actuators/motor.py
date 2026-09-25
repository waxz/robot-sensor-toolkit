"""Simulated DC motor with built-in PID controller and quadrature encoder.

Models the electrical and mechanical dynamics of a brushed DC motor:

    Electrical:  La * di/dt = V - Ra*i - Ke*omega
    Mechanical:  J  * domega/dt = Kt*i - b*omega

When ``La`` is negligible (the default) the electrical equation collapses to
an instantaneous current: ``i = (V - Ke*omega) / Ra``, which is numerically
stiff-free and accurate for most small/medium DC motors.

Command modes
-------------
* ``"voltage"``  -- raw armature voltage V in [-V_max, V_max].
* ``"pwm"``      -- normalised duty cycle d in [-1, 1]; applied as V = d*V_max.
* ``"velocity"`` — target output-shaft angular velocity (rad/s); a PID
  controller closes the loop and outputs voltage.
* ``"position"`` — target output-shaft angle (rad); an outer position loop
  feeds a velocity PID.

Encoder
-------
The encoder tracks the **output-shaft** angle (post-gearbox) and reports
integer tick counts at the configured counts-per-revolution (``cpr``).
"""

from __future__ import annotations

import math
from typing import Any, ClassVar

import numpy as np

# ─────────────────────────────────────────────────────────────────────────────
# Internal PID controller
# ─────────────────────────────────────────────────────────────────────────────


class _PID:
    """Discrete PID with integral anti-windup (back-calculation clamping)."""

    __slots__ = ("Kd", "Ki", "Kp", "_integral", "_prev_error", "out_max", "out_min")

    def __init__(
        self,
        Kp: float,
        Ki: float,
        Kd: float,
        out_min: float,
        out_max: float,
    ) -> None:
        self.Kp, self.Ki, self.Kd = Kp, Ki, Kd
        self.out_min, self.out_max = out_min, out_max
        self._integral = 0.0
        self._prev_error = 0.0

    def compute(self, error: float, dt: float) -> float:
        self._integral += error * dt
        d_term = (error - self._prev_error) / dt if dt > 1e-12 else 0.0
        self._prev_error = error
        out = self.Kp * error + self.Ki * self._integral + self.Kd * d_term
        clamped = float(np.clip(out, self.out_min, self.out_max))
        if out != clamped:
            # Anti-windup: undo the last integral contribution
            self._integral -= error * dt
        return clamped

    def reset(self) -> None:
        self._integral = 0.0
        self._prev_error = 0.0


# ─────────────────────────────────────────────────────────────────────────────
# Motor
# ─────────────────────────────────────────────────────────────────────────────

_TWO_PI = 2.0 * math.pi


class Motor:
    """Simulated DC motor with integrated PID controller and quadrature encoder.

    All physical quantities are at the **motor shaft** (pre-gearbox) unless
    noted.  The output shaft (post-gearbox) is what drives the wheel.

    Args:
        profile (str | None): Named motor preset from :attr:`PROFILES`.
            Overrides all explicit noise/dynamics parameters when provided.
        Ra (float): Armature resistance (Ω).
        La (float): Armature inductance (H).  Set to ``0`` to use the
            simplified (no-inductance) model.
        Ke (float): Back-EMF constant (V·s/rad), motor-shaft.
        Kt (float): Torque constant (N·m/A), motor-shaft.
        J (float): Rotor inertia (kg·m²), motor-shaft.
        b (float): Viscous friction coefficient (N·m·s/rad), motor-shaft.
        V_max (float): Maximum supply voltage (V).
        I_max (float): Current saturation limit (A).
        gear_ratio (float): Motor-to-output-shaft speed reduction
            (``ω_motor / gear_ratio = ω_output``).  Must be > 0.
        cpr (int): Encoder counts per revolution of the **output shaft**.
        command_mode (str): One of ``"voltage"``, ``"pwm"``, ``"velocity"``,
            ``"position"``.
        pid_Kp (float): Velocity-PID proportional gain.
        pid_Ki (float): Velocity-PID integral gain.
        pid_Kd (float): Velocity-PID derivative gain.
        pos_Kp (float): Outer position-PID proportional gain (position mode).
        **kwargs: Absorbed; allows Motor to receive unused factory kwargs.

    Attributes:
        omega (float): Current motor-shaft angular velocity (rad/s).
        current (float): Current armature current (A).
        command_mode (str): Active command mode; change via :meth:`set_mode`.
        omega_output (float): Current output-shaft angular velocity (rad/s).
        theta_output (float): Cumulative output-shaft angle (rad).
        encoder_ticks (int): Cumulative encoder tick count.
        velocity_estimate (float): Tick-delta velocity estimate (rad/s, output shaft).
            Quantised to ``2*pi / (cpr * dt)`` rad/s per tick -- mirrors real
            encoder hardware that counts ticks between sample periods.
    """

    # ── Named motor presets ────────────────────────────────────────────────────
    #
    # Physical parameters are for the **motor shaft** (pre-gearbox).
    # cpr is defined at the **output shaft** (post-gearbox).
    #
    # Sources: manufacturer datasheets; values approximated for simulation.
    # Profile names match irsim_devices.sensors.Encoder.PROFILES.
    #
    PROFILES: ClassVar[dict[str, dict[str, Any]]] = {
        # ── small_dc ────────────────────────────────────────────────────────────
        # Generic 12 V brushed gearmotor (e.g., GA25-370, ~46:1).
        # Motor shaft: ~6800 RPM no-load → omega_nl ~= 712 rad/s.
        # Output shaft: ~148 RPM, stall torque ~0.8 N·m.
        # Velocity-mode PID (dt=1 ms): tau_m=71 ms >> dt; Kp_max ~= 129.
        # Kp=10 targets z_cl ~= 0.83 (~15 ms settling).
        "small_dc": {
            "Ra": 4.8,
            "La": 4e-4,
            "Ke": 0.017,
            "Kt": 0.017,
            "J": 5e-6,
            "b": 1e-5,
            "V_max": 12.0,
            "I_max": 2.5,
            "gear_ratio": 46.0,
            "cpr": 2048,
            "command_mode": "pwm",
            "pid_Kp": 10.0,
            "pid_Ki": 2.0,
            "pid_Kd": 0.05,
            "pos_Kp": 5.0,
        },
        # ── agv_hub_motor ───────────────────────────────────────────────────────
        # BLDC hub motor, 24 V, direct drive (gear_ratio = 1).
        # Output shaft: ~300 RPM no-load, rated torque ~7 N·m.
        #
        # Velocity-mode PID (dt=1 ms): tau_m ~= 0.26 ms < dt; pure-gain plant
        # G ~= 1.315 rad/s/V.  Stability requires Kp < 1/G ~= 0.76; use 0.3.
        "agv_hub_motor": {
            "Ra": 0.3,
            "La": 2e-3,
            "Ke": 0.76,
            "Kt": 0.76,
            "J": 5e-4,
            "b": 1e-3,
            "V_max": 24.0,
            "I_max": 20.0,
            "gear_ratio": 1.0,
            "cpr": 1024,
            "command_mode": "velocity",
            "pid_Kp": 0.3,
            "pid_Ki": 0.1,
            "pid_Kd": 0.0,
            "pos_Kp": 10.0,
        },
        # ── forklift_drive ──────────────────────────────────────────────────────
        # Heavy 48 V brushed motor + 20:1 chain drive.
        # Output shaft: ~150 RPM, stall torque ~50 N·m.
        # Velocity-mode PID (dt=1 ms): tau_m=32 ms >> dt; Kp_max ~= 200.
        # Kp=15 targets z_cl ~= 0.66 (~10 ms settling).
        "forklift_drive": {
            "Ra": 0.15,
            "La": 5e-4,
            "Ke": 0.15,
            "Kt": 0.15,
            "J": 5e-3,
            "b": 5e-3,
            "V_max": 48.0,
            "I_max": 50.0,
            "gear_ratio": 20.0,
            "cpr": 500,
            "command_mode": "pwm",
            "pid_Kp": 15.0,
            "pid_Ki": 2.0,
            "pid_Kd": 0.1,
            "pos_Kp": 5.0,
        },
        # ── dynamixel_xl430 ─────────────────────────────────────────────────────
        # ROBOTIS XL430-W250-T, 12 V, 46.13:1 planetary gear.
        # Output shaft: ~61 RPM no-load, stall torque 1.5 N·m.
        # Velocity-mode PID (dt=1 ms): tau_m=1.82 ms ~= dt; Kp_max ~= 7.2.
        "dynamixel_xl430": {
            "Ra": 3.5,
            "La": 2e-4,
            "Ke": 0.041,
            "Kt": 0.023,
            "J": 5e-7,
            "b": 5e-6,
            "V_max": 12.0,
            "I_max": 1.4,
            "gear_ratio": 46.13,
            "cpr": 4096,
            "command_mode": "velocity",
            "pid_Kp": 3.0,
            "pid_Ki": 0.5,
            "pid_Kd": 0.0,
            "pos_Kp": 3.0,
        },
        # ── pololu_37d_50 ───────────────────────────────────────────────────────
        # Pololu 37D metal gearmotor, 12 V, 50:1.
        # Output shaft: ~130 RPM, 64 CPR motor x 50 = 3200 CPR wheel.
        # Velocity-mode PID (dt=1 ms): tau_m=25 ms >> dt; Kp_max ~= 40.
        # Kp=8 targets z_cl ~= 0.65 (~10 ms settling).
        "pololu_37d_50": {
            "Ra": 2.4,
            "La": 4e-4,
            "Ke": 0.012,
            "Kt": 0.012,
            "J": 2e-6,
            "b": 2e-5,
            "V_max": 12.0,
            "I_max": 5.0,
            "gear_ratio": 50.4,
            "cpr": 3200,
            "command_mode": "pwm",
            "pid_Kp": 8.0,
            "pid_Ki": 1.5,
            "pid_Kd": 0.0,
            "pos_Kp": 5.0,
        },
        # ── maxon_ec45_43 ───────────────────────────────────────────────────────
        # Maxon EC 45 flat (brushless) + GP42C 43:1 gearhead, 24 V.
        # Motor shaft: ~7800 RPM; output ~181 RPM; 2048 CPR motor → ~4096 effective.
        # Velocity-mode PID (dt=1 ms): tau_m=1.36 ms ~= dt; Kp_max ~= 2.17.
        # Kp=0.7 targets z_cl ~= 0.0 (deadbeat, ~2 ms settling).
        "maxon_ec45_43": {
            "Ra": 0.316,
            "La": 4e-5,
            "Ke": 0.0177,
            "Kt": 0.0177,
            "J": 1.35e-6,
            "b": 5e-7,
            "V_max": 24.0,
            "I_max": 15.0,
            "gear_ratio": 43.0,
            "cpr": 4096,
            "command_mode": "voltage",
            "pid_Kp": 0.7,
            "pid_Ki": 0.1,
            "pid_Kd": 0.0,
            "pos_Kp": 8.0,
        },
    }

    def __init__(
        self,
        profile: str | None = None,
        Ra: float = 4.8,
        La: float = 4e-4,
        Ke: float = 0.017,
        Kt: float = 0.017,
        J: float = 5e-6,
        b: float = 1e-5,
        V_max: float = 12.0,
        I_max: float = 2.5,
        gear_ratio: float = 46.0,
        cpr: int = 2048,
        command_mode: str = "pwm",
        pid_Kp: float = 1.5,
        pid_Ki: float = 0.4,
        pid_Kd: float = 0.02,
        pos_Kp: float = 3.0,
        **kwargs: Any,
    ) -> None:
        if profile is not None:
            if profile not in self.PROFILES:
                raise ValueError(
                    f"Unknown motor profile {profile!r}. "
                    f"Available: {list(self.PROFILES)}"
                )
            p = self.PROFILES[profile]
            Ra = p.get("Ra", Ra)
            La = p.get("La", La)
            Ke = p.get("Ke", Ke)
            Kt = p.get("Kt", Kt)
            J = p.get("J", J)
            b = p.get("b", b)
            V_max = p.get("V_max", V_max)
            I_max = p.get("I_max", I_max)
            gear_ratio = p.get("gear_ratio", gear_ratio)
            cpr = p.get("cpr", cpr)
            command_mode = p.get("command_mode", command_mode)
            pid_Kp = p.get("pid_Kp", pid_Kp)
            pid_Ki = p.get("pid_Ki", pid_Ki)
            pid_Kd = p.get("pid_Kd", pid_Kd)
            pos_Kp = p.get("pos_Kp", pos_Kp)

        if command_mode not in ("voltage", "pwm", "velocity", "position"):
            raise ValueError(
                f"Unknown command_mode {command_mode!r}. "
                "Use 'voltage', 'pwm', 'velocity', or 'position'."
            )
        if gear_ratio <= 0:
            raise ValueError(f"gear_ratio must be > 0, got {gear_ratio}")

        # Motor parameters
        self.Ra: float = float(Ra)
        self.La: float = float(La)
        self.Ke: float = float(Ke)
        self.Kt: float = float(Kt)
        self.J: float = float(J)
        self.b: float = float(b)
        self.V_max: float = float(V_max)
        self.I_max: float = float(I_max)
        self.gear_ratio: float = float(gear_ratio)
        self.cpr: int = int(cpr)
        self.command_mode: str = command_mode

        # State
        self.current: float = 0.0  # armature current (A)
        self.omega: float = 0.0  # motor-shaft angular velocity (rad/s)
        self._theta_motor: float = 0.0  # cumulative motor-shaft angle (rad)

        # PID for velocity/position modes
        self._vel_pid: _PID = _PID(pid_Kp, pid_Ki, pid_Kd, -V_max, V_max)
        self._pos_Kp: float = float(pos_Kp)

        # Derived (updated each step)
        self._omega_output_prev: float = 0.0

        # Encoder velocity estimation (tick-delta method)
        self._prev_ticks: int = 0
        self._tick_delta: int = 0
        self.velocity_estimate: float = 0.0  # output-shaft rad/s, from tick delta

    # ── Properties ────────────────────────────────────────────────────────────

    @property
    def omega_output(self) -> float:
        """Output-shaft angular velocity (rad/s), post-gearbox."""
        return self.omega / self.gear_ratio

    @property
    def theta_output(self) -> float:
        """Cumulative output-shaft angle (rad), post-gearbox."""
        return self._theta_motor / self.gear_ratio

    @property
    def encoder_ticks(self) -> int:
        """Cumulative quadrature encoder tick count (output shaft)."""
        return round(self.theta_output * self.cpr / _TWO_PI)

    # ── Main interface ─────────────────────────────────────────────────────────

    def step(self, command: float, dt: float) -> float:
        """Advance motor physics by ``dt`` seconds.

        Args:
            command: Meaning depends on :attr:`command_mode`:
                ``"voltage"``  → armature voltage (V),
                ``"pwm"``      → duty cycle in [-1, 1],
                ``"velocity"`` → target output-shaft speed (rad/s),
                ``"position"`` → target output-shaft angle (rad).
            dt: Integration timestep (s).  Must be > 0.

        Returns:
            Current output-shaft angular velocity (rad/s).
        """
        V = self._command_to_voltage(float(command), dt)
        V = float(np.clip(V, -self.V_max, self.V_max))
        self._integrate(V, dt)

        # Tick-delta velocity estimation (matches real encoder hardware)
        new_ticks = self.encoder_ticks
        self._tick_delta = new_ticks - self._prev_ticks
        self.velocity_estimate = self._tick_delta * _TWO_PI / (self.cpr * dt)
        self._prev_ticks = new_ticks

        return self.omega_output

    def get_encoder(self) -> dict[str, float | int]:
        """Return the latest encoder and motor state.

        Returns:
            dict with keys:

            * ``ticks``            — cumulative integer tick count.
            * ``theta_output``     — cumulative output-shaft angle (rad).
            * ``omega_output``     — output-shaft angular velocity (rad/s).
            * ``current``          — armature current (A).
            * ``omega_motor``      — motor-shaft angular velocity (rad/s).
            * ``tick_delta``       — tick increment in the last step.
            * ``velocity_estimate``— velocity from tick-delta method (rad/s).
        """
        return {
            "ticks": self.encoder_ticks,
            "theta_output": self.theta_output,
            "omega_output": self.omega_output,
            "current": self.current,
            "omega_motor": self.omega,
            "tick_delta": self._tick_delta,
            "velocity_estimate": self.velocity_estimate,
        }

    def reset(self) -> None:
        """Reset motor state to rest (zero current, velocity, angle)."""
        self.current = 0.0
        self.omega = 0.0
        self._theta_motor = 0.0
        self._vel_pid.reset()
        self._prev_ticks = 0
        self._tick_delta = 0
        self.velocity_estimate = 0.0

    def set_mode(self, mode: str) -> None:
        """Switch command mode at runtime and reset controller state.

        All motors support all four modes regardless of their profile default.
        Switching resets the velocity PID integrator so the new mode starts
        from a clean state.

        Args:
            mode: One of ``"voltage"``, ``"pwm"``, ``"velocity"``,
                ``"position"``.

        Raises:
            ValueError: If ``mode`` is not a recognised command mode.
        """
        if mode not in ("voltage", "pwm", "velocity", "position"):
            raise ValueError(
                f"Unknown command_mode {mode!r}. "
                "Use 'voltage', 'pwm', 'velocity', or 'position'."
            )
        self.command_mode = mode
        self._vel_pid.reset()
        self._omega_output_prev = 0.0

    # ── Internal helpers ───────────────────────────────────────────────────────

    def _command_to_voltage(self, command: float, dt: float) -> float:
        if self.command_mode == "voltage":
            return command
        if self.command_mode == "pwm":
            return command * self.V_max
        if self.command_mode == "velocity":
            # Back-EMF feedforward + PID correction.
            # Feedforward covers the dominant back-EMF term so the PID only
            # corrects residual error (load torque, Ra·i drop), giving fast
            # convergence without integral windup.
            V_ff = self.Ke * command * self.gear_ratio
            error = command - self.omega_output
            return V_ff + self._vel_pid.compute(error, dt)
        # position: outer P loop → velocity setpoint → inner velocity PID
        vel_target = self._pos_Kp * (command - self.theta_output)
        error = vel_target - self.omega_output
        V_ff = self.Ke * vel_target * self.gear_ratio
        return V_ff + self._vel_pid.compute(error, dt)

    def _integrate(self, V: float, dt: float) -> None:
        """Integrate motor state by one timestep using exact ODE solutions.

        For typical simulation timesteps (dt ~ 1-50 ms) the electrical and
        mechanical time constants of real motors are *smaller* than dt, so
        Euler integration is numerically unstable.  Both ODEs are instead
        solved exactly as first-order linear systems (matrix-exponential
        approach, decoupled by neglecting La):

        **Unclamped** (|i_ss| <= I_max):
            J*domega/dt = Kt*V/Ra - (Kt*Ke/Ra + b)*omega
            alpha = (Kt*Ke/Ra + b) / J
            omega_ss = Kt*V / (Kt*Ke + Ra*b)
            omega(dt) = omega_ss + (omega - omega_ss)*exp(-alpha*dt)

        **Current-clamped** (|i_ss| > I_max):
            J*domega/dt = Kt*I_clamp - b*omega
            alpha_c = b / J
            omega_ss_c = Kt*I_clamp / b
            omega(dt) = omega_ss_c + (omega - omega_ss_c)*exp(-alpha_c*dt)

        In both cases the solution is unconditionally stable for any dt > 0.
        La is stored as a reference parameter but is not used in integration
        (La/Ra << 1/alpha for virtually all DC motors in the catalogue).
        """
        # Steady-state current without saturation
        i_free = (V - self.Ke * self.omega) / self.Ra

        if abs(i_free) <= self.I_max:
            # Unclamped: exact closed-loop mechanical solution
            alpha = (self.Kt * self.Ke / self.Ra + self.b) / self.J
            omega_ss = (
                (self.Kt * V / self.Ra) / (alpha * self.J)
                if alpha * self.J > 1e-15
                else 0.0
            )
            # Guard against very small alpha (b≈0 edge case)
            if alpha * dt > 1e-10:
                decay = math.exp(-alpha * dt)
                self.omega = omega_ss + (self.omega - omega_ss) * decay
            else:
                self.omega += (self.Kt * i_free - self.b * self.omega) / self.J * dt
            self.current = float(
                np.clip((V - self.Ke * self.omega) / self.Ra, -self.I_max, self.I_max)
            )
        else:
            # Current saturated: constant torque from clamped current
            i_clamp = math.copysign(self.I_max, i_free)
            self.current = i_clamp
            if self.b > 1e-15:
                omega_ss_c = self.Kt * i_clamp / self.b
                alpha_c = self.b / self.J
                decay_c = math.exp(-alpha_c * dt)
                self.omega = omega_ss_c + (self.omega - omega_ss_c) * decay_c
            else:
                self.omega += (self.Kt * i_clamp) / self.J * dt

        self._theta_motor += self.omega * dt
