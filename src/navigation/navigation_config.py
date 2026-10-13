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
# Step 13b-6: the FRONT distance at the reference is not one reading but the median of the
# last FRONT_REF_WINDOW straight-driving samples, each moved to the reference with the odometry
# (d_k - (s_ref - s_k)); a single noisy reading (sigma ~ 7 mm at 0.55 m for the assumed sensor)
# would otherwise eat the 15 mm margin. Only samples from the last FRONT_REF_MAX_TRAVEL_M.
FRONT_REF_WINDOW = 15
FRONT_REF_MAX_TRAVEL_M = 0.10
NOMINAL_CLOSED_FRONT_TRIGGER_M = D_SIDE_CANONICAL_M - D_DEAD_END_CANONICAL_M   # 0.512151 (reference only)

# Stop / settle before and after a turn.
STOP_ZERO_COUNT_SAMPLES = 20          # at-rest window: 20 x 10 ms
STOP_COUNT_TOL = 1                    # 13b-8: +-1 count chatter per tick and net per window allowed
STOP_TIMEOUT_S = 3.0                  # 13b-8: BRAKE / SETTLE never longer than this
PRE_TURN_HOLD_S = 0.50

# Leaving a junction: both side sensors see walls for this many ticks.
REACQUIRE_SAMPLES = 5

# Physical last line (review K5): moving forward with the FRONT reading below
# this -> SAFETY_ABORT. Nominal FRONT_BLOCKED (0.0576 m) + braking (~18 mm)
# leaves ~0.039 m, so this never fires unless FRONT_BLOCKED was missed.
FRONT_SAFETY_MIN_M = 0.030

UTURN_DEG = +180.0

# ---- step 14b: obstacles that are not in the map (goal run). The mission gives the lower bound of the
# FRONT reading on the edge being driven (its own odometric edge length); a FRONT reading clearly
# shorter than that is an obstacle: stop, wait for it to go, or after OBSTACLE_WAIT_MAX_S turn back
# and plan around the edge. Detection uses the FRONT proximity sensor in every configuration.
CORRIDOR_WIDTH_M = 0.728712
LATTICE_PITCH_M = 1.085268
FRONT_SENSOR_AHEAD_M = 0.325          # FRONT sensor ahead of the axle (step 8a mount fit)
OBSTACLE_STOP_M = 0.25                # FRONT reading at which the robot stops for an obstacle
OBSTACLE_MARGIN_M = 0.15              # reading must be this much shorter than the map allows
OBSTACLE_CONFIRM_SAMPLES = 3          # consecutive ticks (sensor noise)
OBSTACLE_CLEAR_S = 1.0                # FRONT free for this long -> continue
OBSTACLE_HYST_M = 0.05
OBSTACLE_WAIT_MAX_S = 15.0            # still blocked after this -> edge blocked, turn back, replan
