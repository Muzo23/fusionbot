"""
FusionBot wheel speed PI controller (step 3).

One controller per wheel, 100 Hz, synchronized with the encoder:

    omega_ref (from inverse kinematics)
    omega_enc (encoder, step 2) -> moving average over 5 samples -> omega_f
    e = omega_ref - omega_f
    u = Kp*e + Ki*integral(e)            clamped to [-1, 1]  -> motor command

Anti-windup (conditional integration): when the candidate output saturates
AND the error would push it further into saturation, the integral increment
of that tick is rejected. The integrator therefore never charges up while the
motor is already at its limit, and there is no overshoot caused by windup.

The 5-sample moving average smooths the +-1 count quantization of the encoder
speed (0.61 rad/s per sample) at the cost of ~20 ms of delay.

reset() clears the integrator and filter history. The navigation FSM requests
it whenever a motion primitive starts or stops (straight -> stop -> turn), so a
turn never inherits the integral built up during straight driving.
"""

from __future__ import annotations

from collections import deque
from dataclasses import dataclass

from core.robot_config import (
    ENCODER_RATE_HZ,
    WHEEL_PI_KI,
    WHEEL_PI_KP,
    WHEEL_SPEED_MOVING_AVERAGE_WINDOW,
)


@dataclass(frozen=True)
class PIOutput:
    command: float
    error_rad_s: float
    filtered_speed_rad_s: float
    integral: float
    saturated: bool


class WheelPI:
    def __init__(
        self,
        kp: float = WHEEL_PI_KP,
        ki: float = WHEEL_PI_KI,
        window: int = WHEEL_SPEED_MOVING_AVERAGE_WINDOW,
        period_s: float = 1.0 / ENCODER_RATE_HZ,
    ) -> None:
        if window <= 0 or period_s <= 0.0:
            raise ValueError("window and period_s must be > 0")
        self.kp, self.ki, self.period_s = float(kp), float(ki), float(period_s)
        self._hist: deque[float] = deque(maxlen=int(window))
        self.integral = 0.0

    def reset(self) -> None:
        self.integral = 0.0
        self._hist.clear()

    def update(self, target_rad_s: float, measured_rad_s: float, dt_s: float | None = None) -> PIOutput:
        dt = self.period_s if dt_s is None else float(dt_s)
        if dt <= 0.0:
            raise ValueError("dt must be > 0")
        self._hist.append(float(measured_rad_s))
        filtered = sum(self._hist) / len(self._hist)
        e = float(target_rad_s) - filtered

        candidate_i = self.integral + e * dt
        u_candidate = self.kp * e + self.ki * candidate_i
        winding_up = (u_candidate > 1.0 and e > 0.0) or (u_candidate < -1.0 and e < 0.0)
        if not winding_up:
            self.integral = candidate_i
        u = self.kp * e + self.ki * self.integral
        command = max(-1.0, min(1.0, u))
        return PIOutput(command, e, filtered, self.integral, command != u)
