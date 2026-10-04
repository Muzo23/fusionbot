"""Step 2 acceptance: incremental encoder.

    python -m unittest tests.test_step02_encoder -v
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from core import robot_config as rc  # noqa: E402
from core.encoder import IncrementalEncoder, wrap_to_pi  # noqa: E402

T = 1.0 / rc.ENCODER_RATE_HZ
Q = rc.RAD_PER_COUNT


def drive(enc, q0, omega, steps, dt=T):
    """Feed a wheel turning at constant omega; joint position wraps at +-pi."""
    out, q = [], q0
    for _ in range(steps):
        q += omega * dt
        s = enc.update(wrap_to_pi(q), dt)
        if s is not None:
            out.append(s)
    return out, q


class TestEncoder(unittest.TestCase):
    def test_first_update_only_initializes(self):
        e = IncrementalEncoder()
        self.assertIsNone(e.update(0.3, T))
        self.assertIsNotNone(e.update(0.3, T))

    def test_one_revolution_is_1024_counts(self):
        e = IncrementalEncoder()
        e.reset(0.0)
        # 1 rev at 10 rad/s, crossing the +-pi wrap point
        steps = int(round(2 * math.pi / 10.0 / T))
        samples, _ = drive(e, 0.0, 2 * math.pi / (steps * T), steps)
        self.assertEqual(e.total_counts, 1024)
        self.assertEqual(sum(s.delta_counts for s in samples), 1024)

    def test_no_subcount_loss(self):
        # 0.3 count per sample for 100 samples -> exactly 30 counts
        e = IncrementalEncoder()
        e.reset(0.0)
        drive(e, 0.0, 0.3 * Q / T, 100)
        self.assertEqual(e.total_counts, 30)

    def test_count_is_floor_of_absolute_angle_with_reversals(self):
        e = IncrementalEncoder()
        e.reset(0.0)
        theta = 0.0
        pattern = [0.35, -0.6, 0.2, 0.9, -1.4, 0.05, 2.3, -0.7]  # in counts
        for p in pattern * 20:
            theta += p * Q
            e.update(wrap_to_pi(theta), T)
            self.assertEqual(e.total_counts, math.floor(theta / Q + 1e-12))

    def test_speed_from_counts(self):
        e = IncrementalEncoder()
        e.reset(0.0)
        w = 14.16  # 0.75 m/s cruise
        samples, _ = drive(e, 0.0, w, 200)
        mean = sum(s.speed_rad_s for s in samples) / len(samples)
        self.assertAlmostEqual(mean, w, delta=Q / (len(samples) * T) + 1e-9)
        for s in samples:  # each sample within one count of the truth
            self.assertLessEqual(abs(s.speed_rad_s - w), Q / T + 1e-9)

    def test_reverse_is_symmetric(self):
        a, b = IncrementalEncoder(), IncrementalEncoder()
        a.reset(0.0)
        b.reset(0.0)
        drive(a, 0.0, +7.0, 150)
        drive(b, 0.0, -7.0, 150)
        self.assertLessEqual(abs(a.total_counts + b.total_counts), 1)  # floor asymmetry

    def test_max_speed_unwraps_correctly(self):
        e = IncrementalEncoder()
        e.reset(0.0)
        w = rc.MAX_WHEEL_COMMAND_SPEED_RAD_S
        _, q = drive(e, 0.0, w, 500)
        self.assertEqual(e.total_counts, math.floor(q / Q + 1e-12))

    def test_resolution(self):
        self.assertAlmostEqual(Q * rc.WHEEL_RADIUS_M * 1000, 0.325057, places=5)


if __name__ == "__main__":
    unittest.main()
