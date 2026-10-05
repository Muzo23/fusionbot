"""
FusionBot motion controller (step 7).

Converts a motion primitive into a body-frame reference (v_ref, omega_ref).
It sits ABOVE the wheel PI (step 3) and BELOW the navigation FSM (step 9).

Primitives
----------
IDLE                      v = 0, omega = 0
STRAIGHT(v)               v = const, omega = 0 (the corridor guidance of step 8
                          later replaces omega while driving straight)
TURN(delta)               in-place relative turn, closed loop on ODOMETRY:

    accumulated += dtheta_odom              (every 100 Hz tick, never wrapped)
    e = delta - accumulated
    omega_ref = clamp(K * e, +-omega_max)
    done when |e| <= tolerance for N consecutive ticks

Validated parameters: K = 2.0 1/s, omega_max = 1.20 rad/s, tolerance 0.33 deg,
N = 5. Because the accumulation is not wrapped, +180 and -180 are different
manoeuvres (CCW vs CW), which the DFS needs to reproduce a path exactly.

No GT, no simulator, no absolute heading: only odometry increments.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

TURN_KP = 2.0
TURN_OMEGA_MAX_RAD_S = 1.20
TURN_TOLERANCE_RAD = math.radians(0.33)
TURN_REQUIRED_SAMPLES = 5
CONTROL_DT_S = 0.01
# Optional angular-acceleration limit for TURN (None = step to omega_max, the
# validated behaviour). Step-7b experiment: does a ramp reduce the per-turn slip?
TURN_ALPHA_MAX_RAD_S2: float | None = None


class MotionMode(str, Enum):
    IDLE = "IDLE"
    STRAIGHT = "STRAIGHT"
    TURN = "TURN"


@dataclass(frozen=True)
class MotionReference:
    linear_velocity_m_s: float
    angular_velocity_rad_s: float
    mode: MotionMode
    completed: bool = False                 # True only on the tick a TURN finishes
    turn_target_rad: float | None = None
    turn_accumulated_rad: float | None = None
    turn_error_rad: float | None = None


class MotionController:
    def __init__(
        self,
        *,
        kp: float = TURN_KP,
        omega_max_rad_s: float = TURN_OMEGA_MAX_RAD_S,
        tolerance_rad: float = TURN_TOLERANCE_RAD,
        required_samples: int = TURN_REQUIRED_SAMPLES,
        alpha_max_rad_s2: float | None = TURN_ALPHA_MAX_RAD_S2,
        dt_s: float = CONTROL_DT_S,
    ) -> None:
        if kp <= 0 or omega_max_rad_s <= 0 or tolerance_rad <= 0 or required_samples <= 0:
            raise ValueError("controller parameters must be > 0")
        self.kp, self.omega_max = float(kp), float(omega_max_rad_s)
        self.tolerance, self.required = float(tolerance_rad), int(required_samples)
        if alpha_max_rad_s2 is not None and alpha_max_rad_s2 <= 0:
            raise ValueError("alpha_max must be > 0 or None")
        self.dw_max = None if alpha_max_rad_s2 is None else float(alpha_max_rad_s2) * float(dt_s)
        self.stop()

    @property
    def mode(self) -> MotionMode:
        return self._mode

    def stop(self) -> None:
        self._mode = MotionMode.IDLE
        self._v = 0.0
        self._target = self._acc = 0.0
        self._in_tol = 0
        self._w = 0.0

    def start_straight(self, v_m_s: float) -> None:
        self.stop()
        self._mode = MotionMode.STRAIGHT
        self._v = float(v_m_s)

    def start_turn(self, delta_rad: float) -> None:
        self.stop()
        if abs(delta_rad) < 1e-12:
            return
        self._mode = MotionMode.TURN
        self._target = float(delta_rad)

    def start_turn_degrees(self, delta_deg: float) -> None:
        self.start_turn(math.radians(delta_deg))

    def update(self, odom_dtheta_rad: float = 0.0) -> MotionReference:
        if self._mode is MotionMode.IDLE:
            return MotionReference(0.0, 0.0, MotionMode.IDLE)
        if self._mode is MotionMode.STRAIGHT:
            return MotionReference(self._v, 0.0, MotionMode.STRAIGHT)

        self._acc += float(odom_dtheta_rad)
        e = self._target - self._acc
        self._in_tol = self._in_tol + 1 if abs(e) <= self.tolerance else 0
        if self._in_tol >= self.required:
            ref = MotionReference(0.0, 0.0, MotionMode.TURN, True, self._target, self._acc, e)
            self.stop()
            return ref
        w = max(-self.omega_max, min(self.omega_max, self.kp * e))
        if self.dw_max is not None:
            w = self._w + max(-self.dw_max, min(self.dw_max, w - self._w))
        self._w = w
        return MotionReference(0.0, w, MotionMode.TURN, False, self._target, self._acc, e)
