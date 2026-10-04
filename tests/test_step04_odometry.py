"""Step 4 acceptance: differential-drive odometry.

    python -m unittest tests.test_step04_odometry -v
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from core import robot_config as rc  # noqa: E402
from core.encoder import IncrementalEncoder, wrap_to_pi  # noqa: E402
from localization.odometry import DifferentialDriveOdometry  # noqa: E402

M = rc.METERS_PER_COUNT
B = rc.EFFECTIVE_WHEEL_SEPARATION_M


def drive_wheels(odo, v_left, v_right, seconds, dt=0.01):
    """Ideal wheels at constant linear speeds -> encoders -> odometry."""
    el, er = IncrementalEncoder(), IncrementalEncoder()
    el.reset(0.0)
    er.reset(0.0)
    ql = qr = 0.0
    for _ in range(int(round(seconds / dt))):
        ql += v_left / rc.WHEEL_RADIUS_M * dt
        qr += v_right / rc.WHEEL_RADIUS_M * dt
        sl = el.update(wrap_to_pi(ql), dt)
        sr = er.update(wrap_to_pi(qr), dt)
        odo.update(sl.delta_counts, sr.delta_counts)


class TestOdometry(unittest.TestCase):
    def test_straight_one_meter(self):
        o = DifferentialDriveOdometry(0.0, 0.0, math.pi / 2)
        drive_wheels(o, 0.5, 0.5, 2.0)
        p = o.pose
        self.assertAlmostEqual(p.x_m, 0.0, places=9)
        self.assertAlmostEqual(p.y_m, 1.0, delta=M)          # within one count
        self.assertAlmostEqual(p.theta_rad, math.pi / 2, places=12)
        self.assertAlmostEqual(o.signed_distance_m, o.path_length_m, places=12)

    def test_in_place_left_turn_90(self):
        o = DifferentialDriveOdometry()
        v = 0.5 * B * (math.pi / 2) / 1.5                     # 90 deg in 1.5 s
        drive_wheels(o, -v, v, 1.5)
        self.assertAlmostEqual(math.degrees(o.pose.theta_rad), 90.0, delta=0.15)
        # floor() of +/- angles can differ by one count -> half-count translation
        self.assertLess(math.hypot(o.pose.x_m, o.pose.y_m), M)
        self.assertAlmostEqual(o.signed_distance_m, 0.0, delta=2 * M)

    def test_unwrapped_heading_distinguishes_plus_minus_180(self):
        a, b = DifferentialDriveOdometry(), DifferentialDriveOdometry()
        v = 0.5 * B * math.pi / 2.0
        drive_wheels(a, -v, v, 2.0)   # +180
        drive_wheels(b, v, -v, 2.0)   # -180
        self.assertAlmostEqual(math.degrees(a.heading_unwrapped), 180.0, delta=0.2)
        self.assertAlmostEqual(math.degrees(b.heading_unwrapped), -180.0, delta=0.2)
        # wrapped poses are the same physical heading
        self.assertAlmostEqual(abs(wrap_to_pi(a.pose.theta_rad - b.pose.theta_rad)), 0.0, delta=0.01)

    def test_arc_midpoint_accuracy(self):
        """Quarter circle of radius 1 m; odometry vs exact geometry."""
        R, v = 1.0, 0.5
        w = v / R
        T = (math.pi / 2) / w
        o = DifferentialDriveOdometry(0.0, 0.0, 0.0)
        drive_wheels(o, v - w * B / 2, v + w * B / 2, T)
        p = o.pose
        self.assertAlmostEqual(p.x_m, 1.0, delta=0.002)       # quantization-level
        self.assertAlmostEqual(p.y_m, 1.0, delta=0.002)
        self.assertAlmostEqual(math.degrees(p.theta_rad), 90.0, delta=0.15)

    def test_reverse(self):
        o = DifferentialDriveOdometry()
        drive_wheels(o, -0.25, -0.25, 2.0)
        self.assertAlmostEqual(o.pose.x_m, -0.5, delta=M)
        self.assertAlmostEqual(o.signed_distance_m, -o.path_length_m, places=12)

    def test_single_increment_formula(self):
        o = DifferentialDriveOdometry()
        inc = o.update(100, 140)
        self.assertAlmostEqual(inc.ds_m, 120 * M)
        self.assertAlmostEqual(inc.dtheta_rad, 40 * M / B)

    def test_reset(self):
        o = DifferentialDriveOdometry()
        o.update(50, 80)
        o.reset(1.0, 2.0, 3.5)
        self.assertEqual((o.pose.x_m, o.pose.y_m), (1.0, 2.0))
        self.assertAlmostEqual(o.pose.theta_rad, wrap_to_pi(3.5))
        self.assertEqual(o.path_length_m, 0.0)


if __name__ == "__main__":
    unittest.main()
