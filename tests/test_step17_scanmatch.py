"""Step 17 (offline): LiDAR scan-to-map matching against the robot's own occupancy grid (D, E).

    python -m unittest discover -s tests -p "test_step17*" -v
"""

import math
import unittest

import numpy as np

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from core.configurations import CONFIGS
from localization import scan_match as SM
from localization.estimators import make_estimator
from localization.odometry import DifferentialDriveOdometry
from mapping.occupancy_grid import OccupancyGrid
from mazes.maze_geometry import WallMap
from mazes.maze_spec import MAZES

MOUNT = (0.18, 0.0, 0.0)
ANG = np.linspace(-2.09, 2.09, 684)


def scan_at(wm, x, y, th):
    cx, cy = x + MOUNT[0] * math.cos(th), y + MOUNT[0] * math.sin(th)
    r = [wm.raycast(cx, cy, th + a, 5.0) for a in ANG]
    return [v if math.isfinite(v) else None for v in r]


def mapped(spec, poses):
    wm = WallMap(spec)
    g = OccupancyGrid(*spec.start_pose[:2])
    for x, y, th in poses:
        cx, cy = x + MOUNT[0] * math.cos(th), y + MOUNT[0] * math.sin(th)
        for _ in range(3):
            g.update(cx, cy, th, ANG, scan_at(wm, x, y, th))
    return wm, g


class TestField(unittest.TestCase):
    def test_distance_is_exact_for_an_axis_wall(self):
        occ = np.zeros((60, 60), bool)
        occ[30, :] = True
        d, _ = SM._distance(occ, 0.02)
        self.assertAlmostEqual(d[35, 10], 0.10, places=9)
        self.assertAlmostEqual(d[0, 0], SM.D_MAX_M)

    def test_attribute_carried_from_the_nearest_cell(self):
        occ = np.zeros((40, 40), bool)
        occ[10, 10] = occ[30, 30] = True
        t = np.zeros((40, 40))
        t[10, 10], t[30, 30] = 1.0, 2.0
        _, (a,) = SM._distance(occ, 0.02, (t,))
        self.assertEqual(a[12, 11], 1.0)
        self.assertEqual(a[28, 27], 2.0)


class TestMatch(unittest.TestCase):
    def setUp(self):
        self.spec = MAZES["T2"]
        x, y, th = self.spec.start_pose
        # map the START corridor and the first junction from three poses along it
        self.poses = [(x, y + d, th) for d in (0.0, 0.6, 1.2)]
        self.wm, self.g = mapped(self.spec, self.poses)

    def test_recovers_a_lateral_and_heading_error(self):
        x, y, th = self.poses[1]
        m = SM.ScanMatcher(self.g, MOUNT)
        sc = scan_at(self.wm, x, y, th)
        res = m.match((x + 0.05, y, th + math.radians(2.0)), ANG, sc)
        self.assertIsNotNone(res)
        (px, py, pth), cov = res
        self.assertAlmostEqual(px, x, delta=0.01)
        self.assertAlmostEqual(math.degrees(pth - th), 0.0, delta=0.3)
        self.assertTrue(np.all(np.linalg.eigvalsh(cov) > 0))

    def test_too_large_correction_is_not_a_match(self):
        x, y, th = self.poses[1]
        m = SM.ScanMatcher(self.g, MOUNT)
        res = m.match((x + 0.0, y, th + math.radians(25.0)), ANG, scan_at(self.wm, x, y, th))
        if res is not None:                       # either refused, or found the right answer
            self.assertAlmostEqual(res[0][2], th, delta=math.radians(1.0))

    def test_empty_map_gives_no_match(self):
        g = OccupancyGrid(0.0, 0.0)
        m = SM.ScanMatcher(g, MOUNT)
        self.assertIsNone(m.match((0.0, 0.0, 0.0), ANG, [1.0] * len(ANG)))
        self.assertEqual(m.last["reason"], "map too small")


class TestEstimator(unittest.TestCase):
    def test_all_configurations_available(self):
        for c in "ABCDE":
            make_estimator(CONFIGS[c], DifferentialDriveOdometry())

    def test_d_has_a_matcher_after_attach(self):
        est = make_estimator(CONFIGS["D"], DifferentialDriveOdometry())
        est.start(0.0, 0.0, 0.0)
        est.attach_lidar(OccupancyGrid(0.0, 0.0), MOUNT)
        self.assertIsNotNone(est.matcher)
        b = make_estimator(CONFIGS["B"], DifferentialDriveOdometry())
        b.attach_lidar(OccupancyGrid(0.0, 0.0), MOUNT)
        self.assertIsNone(b.matcher)


class TestRun(unittest.TestCase):
    def test_d_beats_a_on_t2(self):
        import run as runner
        a = runner.main(["--offline", "--offline-lidar", "--config", "A", "--noise", "realistic", "--maze", "T2",
                         "--seed", "1", "--tag", "test17"])
        d = runner.main(["--offline", "--offline-lidar", "--config", "D", "--noise", "realistic", "--maze", "T2",
                         "--seed", "1", "--tag", "test17"])
        self.assertTrue(d["S4_mission_success"])
        self.assertLess(d["L1_pos_rmse_m"], a["L1_pos_rmse_m"])
        self.assertGreater(d["estimator_stats"]["updates"]["lidar_plain"]["accepted"], 500)


if __name__ == "__main__":
    unittest.main()
