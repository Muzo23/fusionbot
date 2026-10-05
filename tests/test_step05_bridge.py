"""Step 5 offline acceptance: wire protocol and generated bridge script.
(The live part is src/checks/step05_live_drive.py.)

    python -m unittest tests.test_step05_bridge -v
"""

import math
import struct
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from core import robot_config as rc  # noqa: E402
from sim.bridge_script import bridge_script_source  # noqa: E402
from sim.motor_script import COMMAND_PROPERTY  # noqa: E402
from sim.protocol import (  # noqa: E402
    _STATE_STRUCT, PROTOCOL_VERSION, STATE_VALUE_COUNT, decode_state, encode_command,
)


class TestProtocol(unittest.TestCase):
    def test_state_roundtrip(self):
        v = [0.0] * STATE_VALUE_COUNT
        v[0], v[1], v[2], v[3], v[4] = PROTOCOL_VERSION, 7, 0.07, 1.25, -2.5
        v[5], v[6], v[7] = 1, 0.301, 42          # left: detected
        v[8], v[9], v[10] = 0, -1, -1            # front_left: nothing
        v[21], v[22], v[23] = 0.1, -1.4, math.pi / 2
        v[24] = 1                                 # physical collision
        v[46] = 1                                 # visual collision
        v[50], v[51] = 0.123, 3
        s = decode_state(_STATE_STRUCT.pack(*v))
        self.assertEqual(s.state_sequence, 7)
        self.assertAlmostEqual(s.right_joint_position_rad, -2.5)
        self.assertTrue(s.proximity.left.detected)
        self.assertAlmostEqual(s.proximity.left.distance_m, 0.301)
        self.assertIsNone(s.proximity.front_left.distance_m)
        self.assertAlmostEqual(s.gt_heading_rad, math.pi / 2)
        self.assertTrue(s.collision and s.visual_collision)
        self.assertAlmostEqual(s.gt_yaw_rate_rad_s, 0.123)
        self.assertEqual(s.lockstep_republish_count, 3)

    def test_float64_precision(self):
        v = [0.0] * STATE_VALUE_COUNT
        v[0] = PROTOCOL_VERSION
        v[3] = 1264.123456789012   # large accumulated angle keeps full precision
        self.assertEqual(decode_state(_STATE_STRUCT.pack(*v)).left_joint_position_rad, v[3])

    def test_wrong_size_rejected(self):
        with self.assertRaises(RuntimeError):
            decode_state(b"\x00" * 16)

    def test_command_layout_and_clamp(self):
        vals = struct.unpack("=5d", encode_command(3, 7, 2.0, -0.4))
        self.assertEqual(vals, (PROTOCOL_VERSION, 3.0, 7.0, 1.0, -0.4))


class TestBridgeScript(unittest.TestCase):
    def setUp(self):
        self.lua = bridge_script_source(True)

    def test_all_placeholders_filled(self):
        self.assertNotIn("__", self.lua.replace("__index", ""))

    def test_lockstep_switch(self):
        self.assertIn("local LOCKSTEP = true", self.lua)
        self.assertIn("local LOCKSTEP = false", bridge_script_source(False))

    def test_paths_and_protocol(self):
        for p in (rc.ROBOT_PATH, rc.BASE_PATH, rc.LEFT_MOTOR_JOINT_PATH, rc.RIGHT_MOTOR_JOINT_PATH,
                  *rc.PROXIMITY_SENSOR_PATHS.values()):
            self.assertIn(p, self.lua)
        self.assertIn("local PROTOCOL_VERSION = 2.0", self.lua)
        self.assertIn("packDoubleTable", self.lua)

    def test_shares_command_property_with_motor_script(self):
        self.assertIn(f"'{COMMAND_PROPERTY}'", self.lua)


if __name__ == "__main__":
    unittest.main()
