"""Step 15b (offline): estimator replay, noise tuning tools, covariance-intersection update.

    python -m unittest discover -s tests -p "test_step15b*" -v
"""

import math
import subprocess
import sys
import unittest
from pathlib import Path

import numpy as np

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from experiment import replay as RP
from experiment import tune as TU
from localization import ekf as E
from localization import scan_match as SM

ROOT = Path(__file__).resolve().parents[1]


class TestReplayExact(unittest.TestCase):
    """A replay with the run's own configuration and parameters reproduces the recorded estimate."""

    @classmethod
    def setUpClass(cls):
        cmd = [sys.executable, "-m", "run", "--offline", "--config", "E", "--noise", "realistic", "--maze", "T1",
               "--seed", "7", "--tag", "test15b", "--offline-lidar", "--max-time", "25", "--no-goal"]
        subprocess.run(cmd, cwd=ROOT / "src", capture_output=True, text=True, check=True)
        cls.run_dir = ROOT / "results" / "runs" / "T1_E_realistic_s7_offline_test15b"

    def test_exact(self):
        v = RP.verify(self.run_dir)
        self.assertGreater(v["ticks"], 1500)
        self.assertLess(v["max_pos_diff_mm"], 1e-6)
        self.assertLess(v["max_heading_diff_deg"], 1e-6)

    def test_other_configuration_and_restore(self):
        rec = RP.Recording(self.run_dir)
        before = SM.COV_INFLATE
        r = RP.replay(rec, "D", {"scan_match.COV_INFLATE": 7.0})
        self.assertEqual(SM.COV_INFLATE, before)                 # overrides are restored
        self.assertTrue(math.isfinite(r["L1_pos_rmse_m"]) and r["L5_nees_mean"] > 0)
        with self.assertRaises(KeyError):
            RP.replay(rec, "A", {"ekf.NO_SUCH": 1.0})


class TestCost(unittest.TestCase):
    def test_consistent_filter_costs_nothing(self):
        r = [{"run": "a", "L1_pos_rmse_m": 0.1, "L5_nees_xy_mean": 2.0, "L5_nees_th_mean": 1.0, "L5_nees_mean": 3.0}]
        self.assertAlmostEqual(TU.cost(r, "consistency", {"a": 0.1}), 0.0)
        self.assertAlmostEqual(TU.cost(r, "accuracy", {"a": 0.1}), 0.0)
        worse = [{**r[0], "L1_pos_rmse_m": 0.2}]
        self.assertAlmostEqual(TU.cost(worse, "accuracy", {"a": 0.1}), math.log(2.0))

    def test_overconfident_and_conservative_both_penalised(self):
        base = {"run": "a", "L1_pos_rmse_m": 0.1, "L5_nees_mean": 3.0}
        hi = TU.cost([{**base, "L5_nees_xy_mean": 8.0, "L5_nees_th_mean": 1.0}], "consistency", {"a": 0.1})
        lo = TU.cost([{**base, "L5_nees_xy_mean": 0.5, "L5_nees_th_mean": 1.0}], "consistency", {"a": 0.1})
        self.assertGreater(hi, 0.5)
        self.assertAlmostEqual(hi, lo)                            # symmetric in log space


class TestCovarianceIntersection(unittest.TestCase):
    def test_ci_never_overconfident_and_bias_untouched(self):
        f = E.PoseEKF(use_gyro=True, zupt=False)
        f.start(0.0, 0.0, 0.0, bias_sd=math.radians(0.01), pos_sd=0.05, th_sd=math.radians(2))
        H = np.hstack([np.eye(3), np.zeros((3, 1))])
        R = np.diag([0.05 ** 2, 0.05 ** 2, math.radians(2) ** 2])
        p_bias = f.P[3, 3]
        for _ in range(500):            # the SAME information fused again and again (fully correlated)
            f.update("lidar_ci", [0.0, 0.0, 0.0], list(f.x[:3]), H, R, angle_rows=(2,), ci=True)
        self.assertTrue(np.all(np.isfinite(f.P)))
        self.assertAlmostEqual(f.P[3, 3], p_bias, delta=1e-15)
        # CI: repeated fusion of correlated data cannot shrink the pose below the smaller of the two inputs
        self.assertGreaterEqual(f.P[0, 0], 0.5 * 0.05 ** 2)
        g = E.PoseEKF(use_gyro=True, zupt=False)
        g.start(0.0, 0.0, 0.0, pos_sd=0.05, th_sd=math.radians(2))
        for _ in range(500):            # an ordinary EKF update treats them as independent -> collapses
            g.update("lidar", [0.0, 0.0, 0.0], list(g.x[:3]), H, R, angle_rows=(2,))
        self.assertLess(g.P[0, 0], 0.01 * 0.05 ** 2)


class TestTunedValues(unittest.TestCase):
    def test_physically_plausible(self):
        q = math.pi / 2
        self.assertAlmostEqual(math.degrees(math.sqrt(E.Q_TH_ENC_PER_RAD * q)), 0.71, delta=0.01)
        self.assertAlmostEqual(math.degrees(math.sqrt(E.Q_TH_GYRO_PER_RAD * q)), 0.256, delta=0.005)
        self.assertAlmostEqual(1000 * math.sqrt(E.Q_DS_PER_M), 1.98, delta=0.01)
        self.assertEqual(E.GYRO_BIAS_RW, math.radians(0.002))   # datasheet value, not tuned
        self.assertEqual(SM.COV_INFLATE, 100.0)


if __name__ == "__main__":
    unittest.main()
