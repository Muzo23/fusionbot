"""Step 13a (offline): configuration matrix and sensor noise profiles.

    python -m unittest discover -s tests -p "test_step13a*" -v
"""

import math
import statistics as st
import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from core.configurations import CONFIGS, participation_table
from sensors.noise_models import GyroStartCalibration, LidarSpec, NoiseProfile


class TestConfigurations(unittest.TestCase):
    def test_matrix(self):
        self.assertEqual(CONFIGS["A"].sensors(), ("encoders",))
        self.assertEqual(CONFIGS["B"].sensors(), ("encoders", "proximity"))
        self.assertEqual(CONFIGS["C"].sensors(), ("encoders", "gyro"))
        self.assertEqual(CONFIGS["D"].sensors(), ("encoders", "lidar"))
        self.assertEqual(CONFIGS["E"].sensors(), ("encoders", "gyro", "proximity", "lidar"))
        self.assertEqual(CONFIGS["A"].estimator, "odometry")
        for n in "BCDE":
            self.assertEqual(CONFIGS[n].estimator, "ekf")
        for c in CONFIGS.values():                      # same navigation and map everywhere
            self.assertTrue(c.prox_guidance and c.lidar_map)
            self.assertEqual(c.distance_source, "encoders")
            self.assertEqual(c.gyro_zupt, c.uses_gyro)
        self.assertIn("| heading (prediction) | encoders | encoders | gyro | encoders | gyro |",
                      participation_table())


class TestNoise(unittest.TestCase):
    def test_ideal_is_pass_through(self):
        p = NoiseProfile("ideal", 7)
        self.assertEqual(p.gyro.measure(1.2345, 0.01), 1.2345)
        self.assertEqual(p.lidar.measure([0.5, None, 3.0]), [0.5, None, 3.0])
        self.assertEqual(p.proximity.measure(0.2), 0.2)

    def test_realistic_gyro_matches_datasheet(self):
        p = NoiseProfile("realistic", 1)
        g = p.gyro
        self.assertLessEqual(abs(g.bias), math.radians(20.0))
        m = [g.measure(0.0, 0.01) for _ in range(5000)]
        lsb = math.radians(1 / 131)
        for v in m[:50]:
            self.assertAlmostEqual(v / lsb, round(v / lsb), places=6)        # quantized
        sd = st.pstdev(m)
        self.assertAlmostEqual(sd, math.radians(0.05), delta=math.radians(0.01))
        self.assertAlmostEqual(st.mean(m), g.bias, delta=math.radians(0.02))

    def test_start_calibration_removes_the_initial_bias(self):
        p = NoiseProfile("realistic", 3)
        cal = GyroStartCalibration()
        for _ in range(50):                                 # 0.5 s standing
            cal.add(p.gyro.measure(0.0, 0.01))
        self.assertLess(abs(cal.bias - p.gyro.bias), math.radians(0.03))

    def test_realistic_lidar_sigma(self):
        p = NoiseProfile("realistic", 2)
        near = [r - 0.5 for r in p.lidar.measure([0.5] * 4000)]
        far = [r - 3.0 for r in p.lidar.measure([3.0] * 4000)]
        self.assertAlmostEqual(st.pstdev(near), 0.015, delta=0.002)
        self.assertAlmostEqual(st.pstdev(far), 0.045, delta=0.004)
        self.assertEqual(p.lidar.measure([None]), [None])
        self.assertAlmostEqual(LidarSpec().sigma(2.0), 0.03)

    def test_seed_reproducibility_and_independence(self):
        a, b, c = NoiseProfile("realistic", 5), NoiseProfile("realistic", 5), NoiseProfile("realistic", 6)
        self.assertEqual([a.gyro.measure(0.3, 0.01) for _ in range(10)], [b.gyro.measure(0.3, 0.01) for _ in range(10)])
        self.assertNotEqual(a.gyro.bias, c.gyro.bias)
        # using the LiDAR does not change the gyro stream (separate streams)
        d, e = NoiseProfile("realistic", 9), NoiseProfile("realistic", 9)
        d.lidar.measure([1.0] * 100)
        self.assertEqual(d.gyro.measure(0.1, 0.01), e.gyro.measure(0.1, 0.01))

    def test_unknown_profile(self):
        with self.assertRaises(ValueError):
            NoiseProfile("noisy")


if __name__ == "__main__":
    unittest.main()
