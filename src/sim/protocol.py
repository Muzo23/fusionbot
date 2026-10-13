"""
FusionBot wire protocol 2.0 (step 5): state/command packet layout, no sockets.

State packet (bridge -> Python), 52 float64 values per 10 ms pass:
    [0] version  [1] state sequence  [2] sim time  [3,4] left/right joint position
    [5..19] proximity: (detected, distance, object handle) x 5, SENSOR_ORDER
    [20] eval seq  [21..23] GT axle x, y, heading     <- evaluation only
    [24..26] physical collision flag + handles   [27..30] motor callback counts, lag
    [31..34] applied command seq / source state seq / left / right
    [35..45] timing + contact diagnostics   [46..49] visual collision flag + handles
    [50] GT yaw rate (evaluation; the gyro itself comes from the IMU block below)
    [51] lockstep re-publish count
  Extension 2.1 (step 13b), appended; a 2.0 packet (52 values) is still accepted:
    [52] extension version (1)
    [53] IMU sample count (-1: no IMU in the scene)   [54..56] gyro x, y, z [rad/s]
    [57..59] accelerometer x, y, z (specific force) [m/s^2]   - IDEAL values, IMU frame
    [60] LiDAR scan count (-1: no LiDAR)   [61] n = rays attached to THIS packet
    [62 ..] n x (x, y, z, distance) in the Hokuyo frame - attached once per NEW scan; the LiDAR is
         rendered every 10th step (10 Hz, like the real URG-04LX), n = 0 otherwise
Command packet (Python -> bridge), 5 float64:
    version, command sequence (<0 = RELEASE), source state sequence, left, right
  Extension 2.2 (step 14b), optional, appended: n, then n x (index, x, y, z) - scene obstacle
    FB_MAZE_dyn_<index> is moved to (x, y, z) [world frame] before the step; an older bridge ignores it
"""

from __future__ import annotations

import struct
from dataclasses import dataclass

from perception.proximity_types import SENSOR_ORDER, ProximityReading, ProximitySnapshot


PROTOCOL_VERSION = 2.0
STATE_ENDPOINT = "tcp://127.0.0.1:23110"
COMMAND_ENDPOINT = "tcp://127.0.0.1:23111"

STATE_VALUE_COUNT = 52
COMMAND_VALUE_COUNT = 5

_STATE_STRUCT = struct.Struct("=" + "d" * STATE_VALUE_COUNT)
_COMMAND_STRUCT = struct.Struct("=" + "d" * COMMAND_VALUE_COUNT)


@dataclass(frozen=True)
class SimState:
    state_sequence: int
    simulation_time_s: float
    left_joint_position_rad: float
    right_joint_position_rad: float
    proximity: ProximitySnapshot

    # --- simulator truth: evaluation + virtual SENSOR MODELS only ---------
    gt_axle_x_m: float
    gt_axle_y_m: float
    gt_heading_rad: float
    gt_yaw_rate_rad_s: float

    collision: bool
    collision_robot_handle: int
    collision_wall_handle: int
    visual_collision: bool
    visual_collision_robot_handle: int
    visual_collision_wall_handle: int

    left_motor_callback_count: int
    right_motor_callback_count: int
    applied_source_state_sequence: int
    applied_left_command: float
    applied_right_command: float
    lockstep_republish_count: int

    # --- extension 2.1: IMU every step, LiDAR scan at 10 Hz (ideal simulator values) ---
    imu_count: int = -1
    imu_gyro_rad_s: tuple | None = None          # (x, y, z) IMU frame
    imu_accel_m_s2: tuple | None = None          # (x, y, z) specific force, IMU frame
    lidar_count: int = -1
    lidar_xyzd: tuple | None = None              # flat (x, y, z, d) * n in the Hokuyo frame, None = no scan


BASE_VALUE_COUNT = STATE_VALUE_COUNT
EXT_HEADER_COUNT = 10                            # [52..61]


def decode_state(data: bytes) -> SimState:
    if len(data) < _STATE_STRUCT.size or len(data) % 8:
        raise RuntimeError(
            f"State payload {len(data)} B, expected >= {_STATE_STRUCT.size} B "
            "(is the protocol-2.0/2.1 bridge installed? python -m sim.install_bridge)"
        )
    v = struct.unpack(f"={len(data) // 8}d", data)
    if abs(v[0] - PROTOCOL_VERSION) > 0.1:
        raise RuntimeError(f"Unsupported protocol version {v[0]!r}.")

    readings = {}
    for i, name in enumerate(SENSOR_ORDER):
        b = 5 + 3 * i
        detected = bool(int(round(v[b])))
        dist = float(v[b + 1])
        handle = int(round(v[b + 2]))
        readings[name] = ProximityReading(
            name=name,
            detected=detected,
            distance_m=dist if detected and dist >= 0.0 else None,
            detected_object_handle=handle if detected and handle >= 0 else None,
        )

    i = 20  # same layout as protocol 1.0, two values appended at the end
    return SimState(
        state_sequence=int(round(v[1])),
        simulation_time_s=float(v[2]),
        left_joint_position_rad=float(v[3]),
        right_joint_position_rad=float(v[4]),
        proximity=ProximitySnapshot(**readings),
        gt_axle_x_m=float(v[i + 1]),
        gt_axle_y_m=float(v[i + 2]),
        gt_heading_rad=float(v[i + 3]),
        collision=bool(int(round(v[i + 4]))),
        collision_robot_handle=int(round(v[i + 5])),
        collision_wall_handle=int(round(v[i + 6])),
        left_motor_callback_count=int(round(v[i + 7])),
        right_motor_callback_count=int(round(v[i + 8])),
        applied_source_state_sequence=int(round(v[i + 12])),
        applied_left_command=float(v[i + 13]),
        applied_right_command=float(v[i + 14]),
        visual_collision=bool(int(round(v[i + 26]))),
        visual_collision_robot_handle=int(round(v[i + 27])),
        visual_collision_wall_handle=int(round(v[i + 28])),
        gt_yaw_rate_rad_s=float(v[i + 30]),
        lockstep_republish_count=int(round(v[i + 31])),
        **_extension(v),
    )


def _extension(v) -> dict:
    if len(v) < BASE_VALUE_COUNT + EXT_HEADER_COUNT:
        return {}
    b = BASE_VALUE_COUNT
    imu_count = int(round(v[b + 1]))
    lidar_count = int(round(v[b + 8]))
    n = int(round(v[b + 9]))
    if len(v) < b + EXT_HEADER_COUNT + 4 * n:
        raise RuntimeError(f"State packet announces {n} LiDAR rays but carries {len(v) - b - EXT_HEADER_COUNT} values")
    return {
        "imu_count": imu_count,
        "imu_gyro_rad_s": tuple(v[b + 2:b + 5]) if imu_count >= 0 else None,
        "imu_accel_m_s2": tuple(v[b + 5:b + 8]) if imu_count >= 0 else None,
        "lidar_count": lidar_count,
        "lidar_xyzd": tuple(v[b + EXT_HEADER_COUNT:b + EXT_HEADER_COUNT + 4 * n]) if n > 0 else None,
    }


def encode_command(
    command_sequence: int, source_state_sequence: int, left: float, right: float,
    obstacles: list[tuple[int, float, float, float]] | None = None,
) -> bytes:
    base = _COMMAND_STRUCT.pack(
        PROTOCOL_VERSION,
        float(command_sequence),
        float(source_state_sequence),
        max(-1.0, min(1.0, float(left))),
        max(-1.0, min(1.0, float(right))),
    )
    if not obstacles:
        return base
    vals = [float(len(obstacles))]
    for i, x, y, z in obstacles:
        vals += [float(i), float(x), float(y), float(z)]
    return base + struct.pack(f"={len(vals)}d", *vals)


