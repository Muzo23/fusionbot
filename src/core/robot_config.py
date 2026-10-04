"""
FusionBot - robot configuration (step 0).

The single source of truth for ROBOT-level constants. Every other module
imports from here; no physical constant is repeated anywhere else.

Scope rule: only properties of the robot belong here (geometry, drive,
sensors, scene paths). Maze geometry, navigation thresholds, estimator noise
and experiment settings live in their own modules.

Status of the values: all are carried over from the validated FusionBot
project (CAD model, MotorModel identification, odometry calibration). They are
not tuned in this repository.
"""

from __future__ import annotations

import math

# ---------------------------------------------------------------------------
# CoppeliaSim scene paths
# ---------------------------------------------------------------------------

ROBOT_PATH = "/FusionBot"
BASE_PATH = "/FusionBot/mainBase_phy"
LEFT_MOTOR_JOINT_PATH = "/FusionBot/left_motor_joint"
RIGHT_MOTOR_JOINT_PATH = "/FusionBot/right_motor_joint"

PROXIMITY_SENSOR_PATHS = {
    "left": "/FusionBot/mainBase_phy/prox_left",
    "front_left": "/FusionBot/mainBase_phy/prox_front_left",
    "front": "/FusionBot/mainBase_phy/prox_front",
    "front_right": "/FusionBot/mainBase_phy/prox_front_right",
    "right": "/FusionBot/mainBase_phy/prox_right",
}
PROXIMITY_SENSOR_ORDER = ("left", "front_left", "front", "front_right", "right")

# ---------------------------------------------------------------------------
# Frames and timing
# ---------------------------------------------------------------------------

# Physical forward of the robot is the base's local +Y axis, so
# physical heading = base yaw + pi/2. Heading 0 = world +X, CCW positive.
BASE_YAW_TO_PHYSICAL_HEADING_RAD = math.pi / 2.0

SIMULATION_TIMESTEP_S = 0.010   # CoppeliaSim pass = control period (100 Hz)
DYNAMICS_TIMESTEP_S = 0.002     # MuJoCo step, 5 substeps per pass (500 Hz)
CONTROL_RATE_HZ = 1.0 / SIMULATION_TIMESTEP_S

# ---------------------------------------------------------------------------
# Geometry
# ---------------------------------------------------------------------------

WHEEL_RADIUS_M = 0.052976

# Geometric axle separation (CAD).
WHEEL_SEPARATION_M = 0.292

# Effective rotational separation B_eff, identified from in-place rotation
# (0.340-0.342 m) and odometry calibration (0.3392 m). It absorbs contact,
# slip and compliance and is used BOTH by odometry and inverse kinematics.
# B and B_eff must never be mixed up.
EFFECTIVE_WHEEL_SEPARATION_M = 0.3392

# Farthest point of the visual (CAD) envelope from the axle midpoint; the
# in-place turning radius that must fit inside a corridor.
VISUAL_TURNING_RADIUS_M = 0.342291

# ---------------------------------------------------------------------------
# Drive: motor model (simulator side, 500 Hz)
#   tau = tau_stall * (u_eff - omega / omega_0) - friction
# ---------------------------------------------------------------------------

MOTOR_NO_LOAD_SPEED_RAD_S = 25.0
MOTOR_STALL_TORQUE_NM = 0.60
MOTOR_CONTINUOUS_TORQUE_NM = 0.25   # reference only, not a hard clamp
MOTOR_LAG_TAU_S = 0.05
COULOMB_FRICTION_NM = 0.01
VISCOUS_FRICTION_NM_S = 0.002
PWM_DEADZONE = 0.06
MAX_WHEEL_COMMAND_SPEED_RAD_S = 22.65

# ---------------------------------------------------------------------------
# Encoders (incremental, position-driven)
# ---------------------------------------------------------------------------

ENCODER_CPR = 1024
ENCODER_RATE_HZ = 100.0

# ---------------------------------------------------------------------------
# Wheel speed PI (100 Hz, encoder-derived speed feedback)
# ---------------------------------------------------------------------------

WHEEL_PI_KP = 0.08
WHEEL_PI_KI = 0.40
WHEEL_SPEED_MOVING_AVERAGE_WINDOW = 5

# ---------------------------------------------------------------------------
# Derived quantities (computed, never typed in by hand)
# ---------------------------------------------------------------------------

RAD_PER_COUNT = 2.0 * math.pi / ENCODER_CPR
METERS_PER_COUNT = WHEEL_RADIUS_M * RAD_PER_COUNT
MAX_WHEEL_LINEAR_SPEED_M_S = MAX_WHEEL_COMMAND_SPEED_RAD_S * WHEEL_RADIUS_M


def wheel_speeds_from_body(
    v_m_s: float,
    omega_rad_s: float,
    separation_m: float = EFFECTIVE_WHEEL_SEPARATION_M,
) -> tuple[float, float]:
    """Inverse kinematics: body (v, omega) -> (omega_left, omega_right) [rad/s]."""
    v_left = v_m_s - 0.5 * separation_m * omega_rad_s
    v_right = v_m_s + 0.5 * separation_m * omega_rad_s
    return v_left / WHEEL_RADIUS_M, v_right / WHEEL_RADIUS_M


def body_from_wheel_speeds(
    omega_left_rad_s: float,
    omega_right_rad_s: float,
    separation_m: float = EFFECTIVE_WHEEL_SEPARATION_M,
) -> tuple[float, float]:
    """Forward kinematics: (omega_left, omega_right) -> body (v, omega)."""
    v_left = omega_left_rad_s * WHEEL_RADIUS_M
    v_right = omega_right_rad_s * WHEEL_RADIUS_M
    return 0.5 * (v_left + v_right), (v_right - v_left) / separation_m


def clamp_wheel_command(omega_rad_s: float) -> float:
    return max(-MAX_WHEEL_COMMAND_SPEED_RAD_S, min(MAX_WHEEL_COMMAND_SPEED_RAD_S, omega_rad_s))
