"""
Navigation constants (step 9). Validated live in FusionBot2 and re-checked here;
shared by ALL configurations A-E and never retuned to hide localisation errors.
"""

# Single navigation speed. Every threshold below (FRONT_BLOCKED stopping
# distance, D_TURN_ENTRY, the corridor measurement model, the yaw-friction
# feed-forward) was validated at 0.25 m/s. A faster corridor cruise is future work.
NAV_SPEED_M_S = 0.25

# Positioning for a turn at a junction whose STRAIGHT is OPEN: drive this far
# (odometry) past the SIDE-CONFIRMED reference, then brake -> axle stops at the
# junction centre.
D_TURN_ENTRY_M = 0.529676

# Event -> junction-centre projection (used by MazeMemory, step 10).
D_SIDE_CANONICAL_M = 0.530606
D_DEAD_END_CANONICAL_M = 0.018455

# Junction whose STRAIGHT is CLOSED: braking is triggered by FRONT_BLOCKED.
# Review K2: FusionBot2 used a fixed odometry guard (0.529676 m after the
# reference, expected trigger 0.512151 m, margin 17.5 mm). Step-9 offline
# finding: the trigger itself is precise, but the SIDE-CONFIRMED reference moves
# by ~ (W/2)*tan(alpha) with the heading error at the reference (+-12 mm at
# 1.4 deg), eating the fixed margin. New guard, per event: at the reference the
# FRONT sensor already sees the junction's far wall, so
#     expected trigger = d_front(reference) - FRONT_BLOCKED_DISTANCE
#     guard            = expected trigger + CLOSED_GUARD_MARGIN_M
# (offline residual +1.7 ... +2.3 mm, independent of alpha). Physical last line
# remains FRONT_SAFETY.
CLOSED_GUARD_MARGIN_M = 0.015
NOMINAL_CLOSED_FRONT_TRIGGER_M = D_SIDE_CANONICAL_M - D_DEAD_END_CANONICAL_M   # 0.512151 (reference only)

# Stop / settle before and after a turn.
STOP_ZERO_COUNT_SAMPLES = 20          # 20 x 10 ms with no encoder count on both wheels
PRE_TURN_HOLD_S = 0.50

# Leaving a junction: both side sensors see walls for this many ticks.
REACQUIRE_SAMPLES = 5

# Physical last line (review K5): moving forward with the FRONT reading below
# this -> SAFETY_ABORT. Nominal FRONT_BLOCKED (0.0576 m) + braking (~18 mm)
# leaves ~0.039 m, so this never fires unless FRONT_BLOCKED was missed.
FRONT_SAFETY_MIN_M = 0.030

UTURN_DEG = +180.0
