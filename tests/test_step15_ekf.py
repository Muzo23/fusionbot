"""Step 15 (offline): EKF core [x, y, theta, b_g], configuration C (gyro heading + ZUPT), NEES.

    python -m unittest discover -s tests -p "test_step15*" -v
"""

import math
import random
import unittest

import numpy as np

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from core.configurations import CONFIGS
from localization import ekf as E
from localization.estimators import EkfEstimator, TickInputs, make_estimator
from localization.odometry import DifferentialDriveOdometry


class TestPrediction(unittest.TestCase):
    def test_encoder_prediction_is_the_step4_odometry(self):
        """Configuration A: the EKF mean is exactly the odometry of step 4 (the filter only adds P)."""
        rng = random.Random(3)
        odo = DifferentialDriveOdometry(0.3, -1.2, 1.0)
        f = E.PoseEKF(use_gyro=False, zupt=False)
        f.start(0.3, -1.2, 1.0)
        for _ in range(3000):
            inc = odo.update(rng.randint(-3, 12), rng.randint(-3, 12))
            f.predict(inc.ds_m, inc.dtheta_rad, None, 0.01)
        p = odo.pose
        self.assertAlmostEqual(f.x[0], p.x_m, places=9)
        self.assertAlmostEqual(f.x[1], p.y_m, places=9)
        self.assertAlmostEqual(math.remainder(f.x[2] - p.theta_rad, 2 * math.pi), 0.0, places=9)

    def test_gyro_heading_removes_the_bias(self):
        f = E.PoseEKF(use_gyro=True, zupt=False)
        b = math.radians(-18.0)
        f.start(0.0, 0.0, 0.0, bias=b)
        for _ in range(100):                      # 1 s at +90 deg/s measured with the bias on top
            f.predict(0.0, 0.123, math.radians(90.0) + b, 0.01)   # encoder dtheta is ignored
        self.assertAlmostEqual(math.degrees(f.x[2]), 90.0, places=6)

    def test_covariance_grows_with_distance_and_angle_only(self):
        f = E.PoseEKF(use_gyro=False, zupt=False)
        f.start(0.0, 0.0, 0.0)
        P0 = f.P.copy()
        for _ in range(100):
            f.predict(0.0, 0.0, None, 0.01)       # standing still: the pose does not grow (only the bias walk)
        np.testing.assert_allclose(f.P[:3, :3], P0[:3, :3])
        for _ in range(400):                      # 1 m straight
            f.predict(0.0025, 0.0, None, 0.01)
        self.assertAlmostEqual(f.P[0, 0] - P0[0, 0], E.Q_DS_PER_M * 1.0, delta=1e-9)
        self.assertGreater(f.P[1, 1], P0[1, 1])   # heading uncertainty -> lateral uncertainty
        np.testing.assert_allclose(f.P, f.P.T, atol=1e-15)
        self.assertTrue(np.all(np.linalg.eigvalsh(f.P) > 0))


class TestUpdates(unittest.TestCase):
    def test_zupt_learns_the_bias(self):
        f = E.PoseEKF(use_gyro=True, zupt=True)
        f.start(0.0, 0.0, 0.0, bias=0.0, bias_sd=math.radians(0.5))
        rng = random.Random(1)
        b = math.radians(0.3)
        for _ in range(300):
            f.zupt_update(b + rng.gauss(0.0, E.GYRO_RATE_SD))
        self.assertAlmostEqual(math.degrees(f.x[3]), 0.3, delta=0.01)
        self.assertLess(math.sqrt(f.P[3, 3]), math.radians(0.01))

    def test_gate_rejects_an_outlier(self):
        f = E.PoseEKF(use_gyro=True, zupt=True)
        f.start(0.0, 0.0, 0.0, bias=0.0, bias_sd=math.radians(0.01))
        self.assertFalse(f.zupt_update(math.radians(5.0)))      # robot not really at rest
        self.assertEqual(f.x[3], 0.0)
        self.assertTrue(f.zupt_update(math.radians(0.01)))
        self.assertEqual(f.stats["zupt"], [1, 1])

    def test_rest_detection_tolerates_chatter_not_creep(self):
        f = E.PoseEKF(use_gyro=True, zupt=True)
        r = [f.at_rest(1 if i % 2 else -1, 0) for i in range(E.REST_WINDOW)]
        self.assertTrue(r[-1])
        self.assertFalse(any(r[:-1]))
        g = E.PoseEKF(use_gyro=True, zupt=True)
        self.assertFalse(any(g.at_rest(-1, 0) for _ in range(3 * E.REST_WINDOW)))


class TestEstimator(unittest.TestCase):
    def test_configurations(self):
        self.assertEqual(make_estimator(CONFIGS["A"], DifferentialDriveOdometry()).name, "odometry+cov")
        self.assertEqual(make_estimator(CONFIGS["C"], DifferentialDriveOdometry()).name, "ekf")
        for c in "":
            with self.assertRaises(NotImplementedError):
                make_estimator(CONFIGS[c], DifferentialDriveOdometry())

    def test_c_uses_gyro_and_zupt(self):
        odo = DifferentialDriveOdometry()
        est = EkfEstimator(CONFIGS["C"], odo)
        est.start(0.0, 0.0, 0.0, gyro_bias=0.01, gyro_samples=50)
        self.assertAlmostEqual(est.gyro_bias, 0.01)
        for _ in range(E.REST_WINDOW + 5):
            est.update(TickInputs(0.0, odo.update(0, 0), 0.0105, 0.01, None, None))
        self.assertGreater(est.zupt_ticks, 0)
        self.assertGreater(est.gyro_bias, 0.01)


class TestNees(unittest.TestCase):
    def test_consistent_samples_give_mean_three(self):
        from evaluation.metrics import _nees
        rng = np.random.default_rng(0)
        n = 20000
        P = np.diag([0.01, 0.02, 0.003])
        e = rng.multivariate_normal(np.zeros(3), P, n)
        T = {"P_xx": np.full(n, 0.01), "P_yy": np.full(n, 0.02), "P_tt": np.full(n, 0.003),
             "P_xy": np.zeros(n), "P_xt": np.zeros(n), "P_yt": np.zeros(n)}
        m = _nees(T, e[:, 0], e[:, 1], e[:, 2], step=1)
        self.assertAlmostEqual(m["L5_nees_mean"], 3.0, delta=0.1)
        self.assertAlmostEqual(m["L5_nees_in_95pct"], 95.0, delta=1.0)


class TestRun(unittest.TestCase):
    def test_c_beats_a_on_t1(self):
        import run as runner
        a = runner.main(["--offline", "--config", "A", "--noise", "realistic", "--maze", "T1", "--seed", "2",
                         "--tag", "test15"])
        c = runner.main(["--offline", "--config", "C", "--noise", "realistic", "--maze", "T1", "--seed", "2",
                         "--tag", "test15"])
        self.assertTrue(a["S4_mission_success"] and c["S4_mission_success"])
        self.assertLess(c["L1_pos_rmse_m"], a["L1_pos_rmse_m"])
        self.assertGreater(c["estimator_stats"]["updates"]["zupt"]["accepted"], 100)


if __name__ == "__main__":
    unittest.main()
