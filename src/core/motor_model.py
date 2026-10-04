"""
FusionBot DC motor model (step 1).

The model that actually drives the wheels runs SIMULATOR-side in Lua at
500 Hz (sim/motor_script.py generates it from the same constants). This
Python class is the reference implementation of exactly the same equations:
it documents the model, is unit-tested, and is used by the offline simulator.

Signal chain for one wheel, every 2 ms physics step:

    command u in [-1, 1]            (from the wheel PI, held for 10 ms)
      -> PWM deadzone               u_dz = 0 if |u| <= d, else sign(u)(|u|-d)/(1-d)
      -> first-order lag            u_eff += (1 - exp(-dt/tau)) * (u_dz - u_eff)
      -> torque-speed line          tau_m = tau_stall * (u_eff - omega / omega_0)
      -> clamp to +-tau_stall
      -> friction                   tau = tau_m - c*sign(omega) - b*omega
      -> clamp to +-tau_stall

omega is the TRUE wheel velocity: correct for a physical motor model. It is
never used as speed feedback (that comes from the encoder, step 2).
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from core.robot_config import (
    COULOMB_FRICTION_NM,
    MOTOR_LAG_TAU_S,
    MOTOR_NO_LOAD_SPEED_RAD_S,
    MOTOR_STALL_TORQUE_NM,
    PWM_DEADZONE,
    VISCOUS_FRICTION_NM_S,
)


def clamp(x: float, lo: float, hi: float) -> float:
    return lo if x < lo else hi if x > hi else x


def sign(x: float, eps: float = 1e-12) -> float:
    return 1.0 if x > eps else -1.0 if x < -eps else 0.0


def apply_deadzone(u: float, deadzone: float = PWM_DEADZONE) -> float:
    """Symmetric deadzone that keeps the full output range [-1, 1]."""
    u = clamp(u, -1.0, 1.0)
    if abs(u) <= deadzone:
        return 0.0
    return math.copysign((abs(u) - deadzone) / (1.0 - deadzone), u)


def steady_state_speed(u: float) -> float:
    """Wheel speed at which net torque is zero (no external load).

    tau_stall*(u_dz - w/w0) = c + b*w  ->  w = (tau_stall*u_dz - c) / (tau_stall/w0 + b)
    """
    u_dz = apply_deadzone(u)
    if u_dz == 0.0:
        return 0.0
    s = sign(u_dz)
    w = (MOTOR_STALL_TORQUE_NM * abs(u_dz) - COULOMB_FRICTION_NM) / (
        MOTOR_STALL_TORQUE_NM / MOTOR_NO_LOAD_SPEED_RAD_S + VISCOUS_FRICTION_NM_S
    )
    return s * max(0.0, w)


@dataclass
class MotorModel:
    lagged_command: float = 0.0

    def reset(self) -> None:
        self.lagged_command = 0.0

    def update(self, command: float, wheel_omega_rad_s: float, dt_s: float) -> float:
        """Advance the motor by dt and return the net wheel torque [N m]."""
        if dt_s < 0.0:
            raise ValueError("dt_s must be >= 0")
        u_dz = apply_deadzone(command)
        if MOTOR_LAG_TAU_S <= 0.0:
            self.lagged_command = u_dz
        elif dt_s > 0.0:
            a = 1.0 - math.exp(-dt_s / MOTOR_LAG_TAU_S)  # exact discretisation
            self.lagged_command = clamp(
                self.lagged_command + a * (u_dz - self.lagged_command), -1.0, 1.0
            )
        u = self.lagged_command
        w = float(wheel_omega_rad_s)

        tau_m = clamp(
            MOTOR_STALL_TORQUE_NM * (u - w / MOTOR_NO_LOAD_SPEED_RAD_S),
            -MOTOR_STALL_TORQUE_NM,
            MOTOR_STALL_TORQUE_NM,
        )
        motion = sign(w)
        if motion == 0.0:
            motion = sign(u)  # at rest friction opposes the intended direction
        tau = tau_m - COULOMB_FRICTION_NM * motion - VISCOUS_FRICTION_NM_S * w
        return clamp(tau, -MOTOR_STALL_TORQUE_NM, MOTOR_STALL_TORQUE_NM)
