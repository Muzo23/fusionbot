"""
FusionBot PROX-NAV corridor guidance v2 (step 8).

Measurement model (step 8a, LIVE calibration in T1, 2168 samples,
|y| <= 40 mm, |alpha| <= 5.5 deg, GT used only as the calibration reference):

    s = (d_R - d_L)/2 = +1.0042*y + 0.1668*alpha + 0.00035   rms 0.021 mm
    e =  d_FL - d_FR  = -2.4648*y - 1.2633*alpha - 0.00453   rms 0.273 mm

Inverse (y left positive [m], alpha CCW positive [rad] w.r.t. the corridor axis):

    y_hat     = 1.4735*s + 0.1946*e + 0.00036    (rms 0.026 mm)
    alpha_hat = -2.8750*s - 1.1712*e - 0.00429   (rms 0.015 deg)

The offsets are a real sensor-mounting asymmetry (FR diagonal reads 4.5 mm more
than FL with the robot exactly centred and aligned); without them the robot
would be held permanently tilted by 0.25 deg.

Control law (FULL: all four lateral sensors see corridor walls)
    alpha_ref = clamp(-atan(y_hat / L), +-ALPHA_REF_MAX)      look-ahead L
    omega     = clamp(-K * (alpha_hat - alpha_ref), +-OMEGA_MAX)

Corridor-axis memory (HOLD: walls not all visible - openings, junctions)
    In FULL the corridor axis is stored in the ODOMETRY-relative heading frame,
        axis <- axis + G * wrap((theta_rel - alpha_hat) - axis)
    In HOLD the robot keeps that axis:
        omega = clamp(-K * wrap(theta_rel - axis), +-OMEGA_MAX)
    On a commanded turn the axis is advanced by the turn angle, so after the
    turn the robot already steers onto the new corridor axis.

theta_rel is the integral of odometry delta-theta (no absolute pose, no GT).
Same in every configuration A-E: it shapes the physical path only; it never
touches the pose estimate used for mapping and junction matching.
"""

from __future__ import annotations

import math
from dataclasses import dataclass

from perception.proximity_types import ProximitySnapshot

# ---- measurement model (step 8a live calibration)
Y_FROM_S, Y_FROM_E, Y_OFFSET = 1.4735, 0.1946, 0.00036
ALPHA_FROM_S, ALPHA_FROM_E, ALPHA_OFFSET = -2.8750, -1.1712, -0.00429

# ---- validity (centred readings: side 0.184 m, diagonal 0.340 m)
SIDE_MIN_M, SIDE_MAX_M = 0.05, 0.32
DIAG_MIN_M, DIAG_MAX_M = 0.10, 0.50
MIN_FRONT_CLEARANCE_M = 0.30
Y_HAT_MAX_M = 0.10                    # beyond this the reading is not a plain corridor
ALPHA_HAT_MAX_RAD = math.radians(12.0)

# ---- controller
LOOKAHEAD_M = 0.50
K_ALPHA_RAD_S = 3.0
ALPHA_REF_MAX_RAD = math.radians(6.0)
OMEGA_MAX_RAD_S = 0.60
AXIS_FILTER_GAIN = 0.2


def wrap(a: float) -> float:
    return (a + math.pi) % (2.0 * math.pi) - math.pi


@dataclass(frozen=True)
class CorridorMeasurement:
    y_m: float
    alpha_rad: float
    s_m: float
    e_m: float


@dataclass(frozen=True)
class GuidanceOutput:
    mode: str                         # FULL / HOLD / NONE
    omega_ref_rad_s: float
    y_hat_m: float | None
    alpha_hat_rad: float | None
    axis_error_rad: float | None      # FULL: alpha_hat, HOLD: theta_rel - axis


def _dist(r):
    return float(r.distance_m) if r.detected and r.distance_m is not None else None


def measure_corridor(snapshot: ProximitySnapshot) -> CorridorMeasurement | None:
    """(y, alpha) if all four lateral sensors see plain corridor walls, else None."""
    dl, dr = _dist(snapshot.left), _dist(snapshot.right)
    fl, fr = _dist(snapshot.front_left), _dist(snapshot.front_right)
    if None in (dl, dr, fl, fr):
        return None
    if not (SIDE_MIN_M <= dl <= SIDE_MAX_M and SIDE_MIN_M <= dr <= SIDE_MAX_M):
        return None
    if not (DIAG_MIN_M <= fl <= DIAG_MAX_M and DIAG_MIN_M <= fr <= DIAG_MAX_M):
        return None
    f = _dist(snapshot.front)
    if f is not None and f < MIN_FRONT_CLEARANCE_M:
        return None
    s = 0.5 * (dr - dl)
    e = fl - fr
    y = Y_FROM_S * s + Y_FROM_E * e + Y_OFFSET
    a = ALPHA_FROM_S * s + ALPHA_FROM_E * e + ALPHA_OFFSET
    if abs(y) > Y_HAT_MAX_M or abs(a) > ALPHA_HAT_MAX_RAD:
        return None
    return CorridorMeasurement(y, a, s, e)


class CorridorGuidance:
    def __init__(
        self,
        *,
        lookahead_m: float = LOOKAHEAD_M,
        k_alpha: float = K_ALPHA_RAD_S,
        alpha_ref_max_rad: float = ALPHA_REF_MAX_RAD,
        omega_max_rad_s: float = OMEGA_MAX_RAD_S,
        axis_gain: float = AXIS_FILTER_GAIN,
    ) -> None:
        if min(lookahead_m, k_alpha, alpha_ref_max_rad, omega_max_rad_s) <= 0 or not 0 < axis_gain <= 1:
            raise ValueError("invalid guidance parameters")
        self.L, self.k = float(lookahead_m), float(k_alpha)
        self.alpha_ref_max, self.omega_max = float(alpha_ref_max_rad), float(omega_max_rad_s)
        self.axis_gain = float(axis_gain)
        self.reset()

    def reset(self, axis_known: bool = True) -> None:
        """START is placed aligned with its corridor, so the axis is 0 in the
        odometry-relative frame; axis_known=False starts without an axis."""
        self.theta_rel = 0.0
        self.axis: float | None = 0.0 if axis_known else None

    def update_odometry(self, dtheta_rad: float) -> None:
        self.theta_rel += float(dtheta_rad)

    def on_turn_started(self, angle_rad: float) -> None:
        if self.axis is not None:
            self.axis += float(angle_rad)

    def _sat(self, w: float) -> float:
        return max(-self.omega_max, min(self.omega_max, w))

    def command(self, snapshot: ProximitySnapshot) -> GuidanceOutput:
        m = measure_corridor(snapshot)
        if m is not None:
            axis_meas = self.theta_rel - m.alpha_rad
            if self.axis is None:
                self.axis = axis_meas
            else:
                self.axis += self.axis_gain * wrap(axis_meas - self.axis)
            a_ref = max(-self.alpha_ref_max, min(self.alpha_ref_max, -math.atan(m.y_m / self.L)))
            w = self._sat(-self.k * (m.alpha_rad - a_ref))
            return GuidanceOutput("FULL", w, m.y_m, m.alpha_rad, m.alpha_rad)
        if self.axis is not None:
            err = wrap(self.theta_rel - self.axis)
            return GuidanceOutput("HOLD", self._sat(-self.k * err), None, None, err)
        return GuidanceOutput("NONE", 0.0, None, None, None)
