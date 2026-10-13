"""Step 14 (offline): occupancy grid of the LiDAR at the estimated pose and the K2 metric.

    python -m unittest discover -s tests -p "test_step14*" -v
"""

import math
import shutil
import tempfile
import unittest
from pathlib import Path

import numpy as np

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from evaluation.metrics import Geometry, _grid_metrics
from mapping import occupancy_grid as og
from mapping.occupancy_grid import OccupancyGrid
from mazes.maze_geometry import WallMap
from mazes.maze_spec import MAZES


def wall_scan(dist, n=181):
    """Scan from (0, 0) heading 0 of a straight wall x = dist (front half only)."""
    a = np.linspace(-1.2, 1.2, n)
    return a, [dist / math.cos(v) for v in a]


class TestGrid(unittest.TestCase):
    def test_one_beam(self):
        g = OccupancyGrid(0.0, 0.0)
        g.update(0.0, 0.0, 0.0, [0.0], [1.0])
        iy, ix = g.cell_of(np.array([0.5, 1.0 + 0.5 * g.res]), np.array([0.01, 0.01]))
        self.assertTrue(g.free()[iy[0], ix[0]])
        self.assertTrue(g.occupied()[iy[1], ix[1]])
        behind = g.cell_of(np.array([1.3]), np.array([0.01]))
        self.assertFalse(g.free()[behind[0][0], behind[1][0]] or g.occupied()[behind[0][0], behind[1][0]])

    def test_no_return_clears_up_to_free_max(self):
        g = OccupancyGrid(0.0, 0.0)
        g.update(0.0, 0.0, 0.0, [0.0], [None])
        iy, ix = g.cell_of(np.array([og.FREE_MAX_M - 0.05, og.FREE_MAX_M + 0.1]), np.array([0.01, 0.01]))
        self.assertTrue(g.free()[iy[0], ix[0]])
        self.assertFalse(g.free()[iy[1], ix[1]])

    def test_grows_and_keeps_cells(self):
        g = OccupancyGrid(0.0, 0.0, half_size_m=1.0)
        g.update(0.0, 0.0, 0.0, *wall_scan(0.5))
        before = int(g.occupied().sum())
        g.update(0.0, 0.0, math.pi, [0.0], [3.5])               # beam to x = -3.5 leaves the grid
        x0, x1, y0, y1 = g.extent()
        self.assertLess(x0, -3.5)
        self.assertGreaterEqual(int(g.occupied().sum()), before + 1)
        iy, ix = g.cell_of(np.array([0.5 + 0.5 * g.res]), np.array([0.0]))
        self.assertTrue(g.occupied()[iy[0], ix[0]])             # old wall still at the same world place

    def test_dynamic_obstacle_appears_and_clears(self):
        """Free corridor -> obstacle at 0.8 m -> obstacle gone: the clamp lets the map follow (step 14b)."""
        g = OccupancyGrid(0.0, 0.0)
        a, far = wall_scan(2.0)
        for _ in range(50):
            g.update(0.0, 0.0, 0.0, a, far)                     # cells at x = 0.8 saturate free
        cell = g.cell_of(np.array([0.8 + 0.5 * g.res]), np.array([0.0]))
        c = (cell[0][0], cell[1][0])
        self.assertTrue(g.free()[c])
        _, near = wall_scan(0.8)
        n_occ = next(i for i in range(1, 30) if (g.update(0.0, 0.0, 0.0, a, near) or g.occupied()[c]))
        self.assertLessEqual(n_occ, 6)
        for _ in range(50):
            g.update(0.0, 0.0, 0.0, a, near)                    # obstacle stays: saturates occupied
        n_free = next(i for i in range(1, 30) if (g.update(0.0, 0.0, 0.0, a, far) or not g.occupied()[c]))
        self.assertLessEqual(n_free, 9)

    def test_save_load(self):
        g = OccupancyGrid(1.0, 2.0)
        g.update(1.0, 2.0, 0.3, *wall_scan(0.7))
        d = Path(tempfile.mkdtemp())
        try:
            g.save(d / "g.npz")
            h = OccupancyGrid.load(d / "g.npz")
            self.assertEqual((h.ox, h.oy, h.res, h.scans), (g.ox, g.oy, g.res, g.scans))
            np.testing.assert_array_equal(h.L, g.L)
        finally:
            shutil.rmtree(d)


class TestK2(unittest.TestCase):
    def _grid(self, spec, pose_err=(0.0, 0.0, 0.0)):
        wm = WallMap(spec)
        g = OccupancyGrid(*spec.start_pose[:2])
        ang = np.linspace(-2.09, 2.09, 342)
        for (x, y) in spec.nodes_m.values():
            for th in (0.0, math.pi / 2, math.pi, -math.pi / 2):
                r = [wm.raycast(x, y, th + a, 5.0) for a in ang]
                r = [None if not math.isfinite(v) else v for v in r]
                g.update(x + pose_err[0], y + pose_err[1], th + pose_err[2], ang, r)
        return g

    def _k2(self, spec, g):
        d = Path(tempfile.mkdtemp())
        try:
            g.save(d / "map_grid.npz")
            return _grid_metrics({"dir": d, "scans": None}, Geometry(spec), spec)
        finally:
            shutil.rmtree(d)

    def test_true_pose_gives_a_correct_grid(self):
        spec = MAZES["T1"]
        m = self._k2(spec, self._grid(spec))
        self.assertGreater(m["K2_agreement_pct"], 99.5)
        self.assertGreater(m["K2_coverage_pct"], 95.0)

    def test_pose_error_lowers_k2(self):
        spec = MAZES["T1"]
        m = self._k2(spec, self._grid(spec, pose_err=(0.10, -0.05, math.radians(3))))
        self.assertLess(m["K2_agreement_pct"], 90.0)


if __name__ == "__main__":
    unittest.main()
