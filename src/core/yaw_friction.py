"""
Yaw (rotation) friction compensation (step 8).

Finding (live step 8c, run 1): while driving straight at 0.25 m/s the robot
barely follows small angular-velocity references (|omega| < ~0.1 rad/s): the
castors and tyre scrub act as a Coulomb/stiction torque on the DIFFERENTIAL
mode, and the wheel PIs build the missing torque only through their integrator,
over 1-2 s. A rotation-mode model fitted to the live log (offline plant,
yaw_inertia 0.004 kg m^2, yaw_friction 0.0875 N m per wheel) reproduces the
heading trajectory with rms 0.29 deg.

Integral action on the heading error was rejected (limit cycles with stiction).
Used instead: friction feed-forward on the motor commands, smoothly saturated
around omega = 0 to avoid chattering:

    u_ff = U_F * sat(omega_ref / OMEGA_BAND)     right command += u_ff, left -= u_ff

U_F = 0.12 (~80 % of the fitted friction / stall torque, deliberately under-
compensated); OMEGA_BAND = 0.05 rad/s. Offline it was the only candidate with
no chattering for yaw friction 0.06 ... 0.115 N m (+-30 %).

Applied to STRAIGHT (corridor guidance) only; in-place turns use large omega
and are validated without it (step 7).
"""

from __future__ import annotations

YAW_FF_COMMAND = 0.12
YAW_FF_BAND_RAD_S = 0.05


def yaw_friction_feedforward(
    omega_ref_rad_s: float,
    u_f: float = YAW_FF_COMMAND,
    band_rad_s: float = YAW_FF_BAND_RAD_S,
) -> float:
    """Differential motor-command feed-forward (add to right, subtract from left)."""
    if u_f == 0.0:
        return 0.0
    if band_rad_s <= 0.0:
        raise ValueError("band must be > 0")
    return u_f * max(-1.0, min(1.0, omega_ref_rad_s / band_rad_s))
