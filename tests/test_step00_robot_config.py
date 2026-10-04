"""Step 0 acceptance: robot configuration is internally consistent.

    python -m unittest tests.test_step00_robot_config -v      (from the repo root)
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from core import robot_config as rc  # noqa: E402


class TestRobotConfig(unittest.TestCase):
    def test_timing(self):
        self.assertAlmostEqual(rc.SIMULATION_TIMESTEP_S / rc.DYNAMICS_TIMESTEP_S, 5.0)
        self.assertAlmostEqual(rc.CONTROL_RATE_HZ, rc.ENCODER_RATE_HZ)

    def test_encoder_resolution(self):
        # handoff: 1 count ~ 0.325057 mm of wheel travel
        self.assertAlmostEqual(rc.METERS_PER_COUNT * 1000.0, 0.325057, places=5)

    def test_speed_limits(self):
        # handoff: 1.00 m/s -> 18.876 rad/s (83.3 % of the command limit)
        wl, wr = rc.wheel_speeds_from_body(1.0, 0.0)
        self.assertAlmostEqual(wl, 18.876, places=3)
        self.assertAlmostEqual(wl / rc.MAX_WHEEL_COMMAND_SPEED_RAD_S, 0.833, places=3)
        self.assertLess(rc.MAX_WHEEL_COMMAND_SPEED_RAD_S, rc.MOTOR_NO_LOAD_SPEED_RAD_S)

    def test_kinematics_round_trip(self):
        for v, w in ((0.75, 0.0), (0.0, 1.2), (0.25, -0.6)):
            wl, wr = rc.wheel_speeds_from_body(v, w)
            v2, w2 = rc.body_from_wheel_speeds(wl, wr)
            self.assertAlmostEqual(v, v2, places=12)
            self.assertAlmostEqual(w, w2, places=12)

    def test_separation_convention(self):
        # in-place rotation: wheels opposite, |v_wheel| = omega * B_eff / 2
        wl, wr = rc.wheel_speeds_from_body(0.0, 1.0)
        self.assertAlmostEqual(wr * rc.WHEEL_RADIUS_M, 0.5 * rc.EFFECTIVE_WHEEL_SEPARATION_M)
        self.assertAlmostEqual(wl, -wr)
        self.assertGreater(rc.EFFECTIVE_WHEEL_SEPARATION_M, rc.WHEEL_SEPARATION_M)

    def test_sensor_paths(self):
        self.assertEqual(set(rc.PROXIMITY_SENSOR_ORDER), set(rc.PROXIMITY_SENSOR_PATHS))
        for p in rc.PROXIMITY_SENSOR_PATHS.values():
            self.assertTrue(p.startswith(rc.BASE_PATH + "/"))

    def test_heading_offset(self):
        self.assertAlmostEqual(rc.BASE_YAW_TO_PHYSICAL_HEADING_RAD, math.pi / 2)


if __name__ == "__main__":
    unittest.main()
