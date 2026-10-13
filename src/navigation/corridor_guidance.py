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

Single wall (LEFT / RIGHT, step 13b-6): where one side is open (junctions, the
open cells of micromouse mazes) but the other side and its diagonal see a plain
wall, (y, alpha) are solved from that side alone with the per-sensor model fitted
on the live FULL samples of the T2 run (8. 10. 2026, 12 769 ticks):

    d_R  = 0.1842 + 0.9942 y + 0.1711 alpha      d_FR = 0.3397 + 1.2310 y + 0.6311 alpha
    d_L  = 0.1835 - 1.0142 y - 0.1626 alpha      d_FL = 0.3351 - 1.2341 y - 0.6323 alpha

and used like FULL (control + axis update), but only if alpha agrees with the
held axis within SINGLE_GATE_RAD (a diagonal reading at a wall end is rejected).
Without it a long stretch with one open side (HOLD only) lets the turn losses
pile up in the held axis until the robot touches the wall (offline M3, step 13b-6).

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

# ---- single-wall model (live FULL samples, T2, 8. 10. 2026): d = c0 + cy*y + ca*alpha
SINGLE_MODEL = {
    "right": ((0.1842, 0.9942, 0.1711), (0.3397, 1.2310, 0.6311)),     # (side sensor, diagonal)
    "left": ((0.1835, -1.0142, -0.1626), (0.3351, -1.2341, -0.6323)),
}
SINGLE_GATE_RAD = math.radians(8.0)
SINGLE_MIN_TICKS = 5                  # the single wall must be valid this many ticks in a row (wall ends)

# ---- controller
LOOKAHEAD_M = 0.50
K_ALPHA_RAD_S = 3.0
ALPHA_REF_MAX_RAD = math.radians(6.0)
OMEGA_MAX_RAD_S = 0.60
AXIS_FILTER_GAIN = 0.2
# Step 13b-8: (y, alpha) measurement filter. Between two ticks the robot's own motion moves the
# estimate (alpha += dtheta, y += ds * sin(alpha), both from the encoders); the new measurement is
# blended in with gain dt / (MEAS_TAU_S + dt). Removes the steering chatter that proximity noise
# causes (live T1 realistic: 64 omega sign changes per metre) without the lag of a plain low-pass,
# because the prediction follows the robot's own turning. MEAS_TAU_S = 0 -> no filter.
MEAS_TAU_S = 0.05
DT_S = 0.01


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
    mode: str                         # FULL / LEFT / RIGHT (single wall) / HOLD / NONE
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


def measure_single_wall(snapshot: ProximitySnapshot, side: str) -> CorridorMeasurement | None:
    """(y, alpha) from ONE wall (side sensor + diagonal of that side), else None."""
    d_s = _dist(getattr(snapshot, side))
    d_d = _dist(getattr(snapshot, "front_" + side))
    if d_s is None or d_d is None:
        return None
    if not (SIDE_MIN_M <= d_s <= SIDE_MAX_M and DIAG_MIN_M <= d_d <= DIAG_MAX_M):
        return None
    f = _dist(snapshot.front)
    if f is not None and f < MIN_FRONT_CLEARANCE_M:
        return None
    (s0, sy, sa), (d0, dy, da) = SINGLE_MODEL[side]
    det = sy * da - sa * dy
    bs, bd = d_s - s0, d_d - d0
    y = (bs * da - sa * bd) / det
    a = (sy * bd - bs * dy) / det
    if abs(y) > Y_HAT_MAX_M or abs(a) > ALPHA_HAT_MAX_RAD:
        return None
    return CorridorMeasurement(y, a, float("nan"), float("nan"))


class CorridorGuidance:
    def __init__(
        self,
        *,
        lookahead_m: float = LOOKAHEAD_M,
        k_alpha: float = K_ALPHA_RAD_S,
        alpha_ref_max_rad: float = ALPHA_REF_MAX_RAD,
        omega_max_rad_s: float = OMEGA_MAX_RAD_S,
        axis_gain: float = AXIS_FILTER_GAIN,
        meas_tau_s: float = MEAS_TAU_S,
    ) -> None:
        if min(lookahead_m, k_alpha, alpha_ref_max_rad, omega_max_rad_s) <= 0 or not 0 < axis_gain <= 1:
            raise ValueError("invalid guidance parameters")
        self.L, self.k = float(lookahead_m), float(k_alpha)
        self.alpha_ref_max, self.omega_max = float(alpha_ref_max_rad), float(omega_max_rad_s)
        self.axis_gain = float(axis_gain)
        if meas_tau_s < 0:
            raise ValueError("meas_tau_s must be >= 0")
        self.meas_gain = DT_S / (meas_tau_s + DT_S)
        self.reset()

    def reset(self, axis_known: bool = True) -> None:
        """START is placed aligned with its corridor, so the axis is 0 in the
        odometry-relative frame; axis_known=False starts without an axis."""
        self.theta_rel = 0.0
        self.axis: float | None = 0.0 if axis_known else None
        self._single_ticks = {"left": 0, "right": 0}
        self._f = None                    # filtered (y, alpha) and the mode it belongs to
        self._f_mode = None

    def update_odometry(self, dtheta_rad: float, ds_m: float = 0.0) -> None:
        self.theta_rel += float(dtheta_rad)
        if self._f is not None:                                  # predict with the robot's own motion
            y, a = self._f
            a += float(dtheta_rad)
            y += float(ds_m) * math.sin(a)
            self._f = (y, a)

    def _filter(self, mode: str, y: float, a: float) -> tuple[float, float]:
        if self._f is None or self._f_mode != mode:
            self._f, self._f_mode = (y, a), mode                 # new measurement source: start from it
        else:
            fy, fa = self._f
            g = self.meas_gain
            self._f = (fy + g * (y - fy), fa + g * (a - fa))
        return self._f

    def on_turn_started(self, angle_rad: float) -> None:
        if self.axis is not None:
            self.axis += float(angle_rad)

    def _sat(self, w: float) -> float:
        return max(-self.omega_max, min(self.omega_max, w))

    def command(self, snapshot: ProximitySnapshot) -> GuidanceOutput:
        m = measure_corridor(snapshot)
        if m is not None:
            self._single_ticks = {"left": 0, "right": 0}
            fy, fa = self._filter("FULL", m.y_m, m.alpha_rad)
            m = CorridorMeasurement(fy, fa, m.s_m, m.e_m)
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
            single = {}
            for side in ("left", "right"):
                m1 = measure_single_wall(snapshot, side)
                ok = m1 is not None and abs(wrap(m1.alpha_rad - err)) <= SINGLE_GATE_RAD
                self._single_ticks[side] = self._single_ticks[side] + 1 if ok else 0
                single[side] = m1
            for side in ("left", "right"):
                m1 = single[side]
                if self._single_ticks[side] >= SINGLE_MIN_TICKS:
                    fy, fa = self._filter(side, m1.y_m, m1.alpha_rad)
                    m1 = CorridorMeasurement(fy, fa, m1.s_m, m1.e_m)
                    self.axis += self.axis_gain * wrap((self.theta_rel - m1.alpha_rad) - self.axis)
                    a_ref = max(-self.alpha_ref_max, min(self.alpha_ref_max, -math.atan(m1.y_m / self.L)))
                    w = self._sat(-self.k * (m1.alpha_rad - a_ref))
                    return GuidanceOutput(side.upper(), w, m1.y_m, m1.alpha_rad, m1.alpha_rad)
            self._f = self._f_mode = None
            return GuidanceOutput("HOLD", self._sat(-self.k * err), None, None, err)
        return GuidanceOutput("NONE", 0.0, None, None, None)
