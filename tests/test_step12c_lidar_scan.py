"""Step 12c (offline): LiDAR scan geometry, validation against the maze, mount fit.

A synthetic fastHokuyo scan (2 perspective sensors x 256 rays, 120 deg each,
pixel rays uniform in tan) is built from the true walls, encoded exactly like the
FusionBotLidarScan script (x, y, z in the Hokuyo frame + distance) and decoded.

    python -m unittest discover -s tests -p "test_step12c*" -v
"""

import math
import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from mazes.maze_geometry import WallMap
from mazes.maze_spec import MAZES
from perception.lidar_scan import (RangeModel, apply_range_model, evaluate_scan, fit_range_model, fit_scan_pose,
                                   scan_from_xyzd, scan_points)

MAX = 5.0
MOUNT_FWD = 0.1786
HOK_YAW = math.radians(-90.0)     # Hokuyo object frame turned w.r.t. the scan forward (arbitrary)


def bearings():
    out = []
    for axis in (-math.radians(60), math.radians(60)):
        for k in range(256):
            u = (k + 0.5) / 256 * 2 - 1
            out.append(axis + math.atan(u * math.tan(math.radians(60))))
    return out


def synthetic(walls, cx, cy, heading, z=0.2128, quant=None, model=None):
    """Returns xyzd and the Hokuyo world matrix."""
    yaw_h = heading + HOK_YAW
    c, s = math.cos(yaw_h), math.sin(yaw_h)
    m = [c, -s, 0.0, cx, s, c, 0.0, cy, 0.0, 0.0, 1.0, z]
    xyzd = []
    for b in bearings():
        a = heading + b
        r = walls.raycast(cx, cy, a, MAX)
        d = r if math.isfinite(r) else MAX
        if model is not None and math.isfinite(r):
            d = d + model.error(d)
        if quant and math.isfinite(r):
            d = round(d / quant) * quant
        wx, wy = d * math.cos(a), d * math.sin(a)                 # relative to the centre, world axes
        hx, hy = c * wx + s * wy, -s * wx + c * wy                 # into the Hokuyo frame
        xyzd += [hx, hy, 0.0, d]
    return xyzd, m


class TestLidarScan(unittest.TestCase):
    def setUp(self):
        self.walls = WallMap(MAZES["T1"])
        x, y, h = MAZES["T1"].start_pose
        self.h = h
        self.c = (x + MOUNT_FWD * math.cos(h), y + MOUNT_FWD * math.sin(h), 0.2128)

    def test_perfect_scan_matches_the_walls(self):
        xyzd, m = synthetic(self.walls, self.c[0], self.c[1], self.h)
        rays = scan_from_xyzd(xyzd, m, self.c, self.h, MAX)
        ev = evaluate_scan(rays, self.walls, self.c, self.h, MAX, plane_z_m=self.c[2])
        self.assertEqual(ev["rays"], 512)
        self.assertAlmostEqual(ev["bearing_min_deg"], -119.7, delta=0.5)
        self.assertAlmostEqual(ev["bearing_max_deg"], 119.7, delta=0.5)
        self.assertEqual((ev["missed"], ev["phantom"]), (0, 0))
        self.assertLess(ev["max_abs_m"], 1e-9)
        self.assertLess(ev["max_point_to_wall_m"], 1e-9)
        self.assertGreater(ev["compared"], 400)
        self.assertGreater(ev["min_range_m"], 0.3)

    def test_quantised_scan_and_wrong_wall_detected(self):
        xyzd, m = synthetic(self.walls, self.c[0], self.c[1], self.h, quant=0.001)
        ev = evaluate_scan(scan_from_xyzd(xyzd, m, self.c, self.h, MAX), self.walls, self.c, self.h, MAX)
        self.assertLess(ev["median_abs_m"], 0.0006)
        # a robot standing 2 cm further north than believed: ranges ahead are 2 cm short
        ev = evaluate_scan(scan_from_xyzd(xyzd, m, self.c, self.h, MAX), self.walls,
                           (self.c[0], self.c[1] - 0.02), self.h, MAX)
        self.assertGreater(ev["p95_abs_m"], 0.015)

    def test_mount_fit_recovers_the_scan_pose(self):
        xyzd, m = synthetic(self.walls, self.c[0], self.c[1], self.h, quant=0.001)
        rays = scan_from_xyzd(xyzd, m, self.c, self.h, MAX)
        pts = scan_points(rays)
        guess = (self.c[0] + 0.005, self.c[1] - 0.003, self.h + math.radians(0.5))
        x, y, th, rms, n = fit_scan_pose(pts, self.walls, guess)
        self.assertGreater(n, 300)
        self.assertLess(math.hypot(x - self.c[0], y - self.c[1]), 0.0003)
        self.assertLess(abs(math.degrees(th - self.h)), 0.01)
        self.assertLess(rms, 0.001)

    def test_range_calibration_removes_the_systematic_error(self):
        """Live-like sensor (step 12c: +1.4 mm at 0.4 m, +10 mm at 1.7 m): the
        uncalibrated scan pulls the scan-to-wall fit ~9 mm backwards; the fitted
        model e = a r + b r^2 removes it."""
        from checks.step12d_lidar_calibration import choose_poses
        from checks.lidar_live import analyse
        true = RangeModel(0.0031, 0.0017)
        spec = MAZES["T3"]
        walls = WallMap(spec)
        mount = {"x_forward_m": MOUNT_FWD, "y_left_m": 0.0, "yaw_offset_rad": 0.0}
        poses = choose_poses(spec, walls, mount, 10)
        fronts = [p[0] for p in poses[:-1]]
        self.assertLess(min(fronts), 0.3)
        self.assertGreater(max(fronts), 4.5)                   # long ranges are calibrated too
        scans, samples = [], []
        for _, _, (x, y, h) in poses:
            cx, cy = x + MOUNT_FWD * math.cos(h), y + MOUNT_FWD * math.sin(h)
            xyzd, m = synthetic(walls, cx, cy, h, quant=0.001, model=true)
            sc = dict(xyzd=xyzd, m=m, centre=[cx, cy, 0.2128], fwd=h, axle=[x, y, 0.0], heading=h)
            scans.append(sc)
            samples += analyse(sc, walls, mount)["raw"]["samples"]
        model, n = fit_range_model(samples)
        self.assertAlmostEqual(model.a, true.a, delta=2e-4)
        self.assertAlmostEqual(model.b, true.b, delta=1e-4)
        start = analyse(scans[-1], walls, mount, model)
        self.assertLess(start["raw"]["mount_fwd_m"], -0.005)    # raw: biased like live (-9.4 mm)
        self.assertLess(abs(start["cor"]["mount_fwd_m"]), 0.0005)
        self.assertLess(start["cor"]["ng_median_abs_m"], 0.001)

    def test_robust_fit_ignores_mixed_pixel_outliers(self):
        """3 % of the rays 0.05-0.3 m too long (mixed pixels at wall corners): the
        robust range fit and the gated pose fit are not pulled by them."""
        import random
        rnd = random.Random(1)
        true = RangeModel(0.004, 0.0)
        xyzd, m = synthetic(self.walls, self.c[0], self.c[1], self.h, quant=0.001, model=true)
        for i in range(0, len(xyzd), 4):
            if xyzd[i + 3] < 4.9 and rnd.random() < 0.03:
                k = 1 + rnd.uniform(0.05, 0.3) / xyzd[i + 3]
                xyzd[i] *= k
                xyzd[i + 1] *= k
                xyzd[i + 3] *= k
        rays = scan_from_xyzd(xyzd, m, self.c, self.h, MAX)
        ev = evaluate_scan(rays, self.walls, self.c, self.h, MAX)
        model, n = fit_range_model(ev["samples"])
        self.assertAlmostEqual(model.error(1.0), true.error(1.0), delta=0.0005)
        naive, _ = fit_range_model(ev["samples"], robust=False)
        self.assertGreater(abs(naive.error(1.0) - true.error(1.0)), abs(model.error(1.0) - true.error(1.0)))
        x, y, th, rms, k = fit_scan_pose(scan_points(apply_range_model(rays, self.c, self.h, model)),
                                         self.walls, (self.c[0], self.c[1], self.h))
        self.assertLess(math.hypot(x - self.c[0], y - self.c[1]), 0.001)
        self.assertLess(abs(math.degrees(th - self.h)), 0.05)

    def test_correct_inverts_the_model(self):
        m = RangeModel(0.003, 0.0017)
        for r in (0.2, 1.0, 4.9):
            self.assertAlmostEqual(m.correct(r + m.error(r)), r, delta=2e-6)
        rays = apply_range_model(scan_from_xyzd(*synthetic(self.walls, self.c[0], self.c[1], self.h, model=m),
                                                self.c, self.h, MAX), self.c, self.h, m)
        ev = evaluate_scan(rays, self.walls, self.c, self.h, MAX)
        self.assertLess(ev["max_point_to_wall_m"], 1e-4)



class TestHousingDetection(unittest.TestCase):
    def test_upright_cylinder_on_top_is_found(self):
        from sim.lidar_setup import round_candidates
        axle, floor = (0.0, -1.4226, 0.05), 0.0
        boxes = [("chassis_visual", 1, (0.0, -1.30, 0.10), (0.40, 0.34, 0.20)),
                 ("wheel", 2, (0.17, -1.42, 0.05), (0.03, 0.10, 0.10)),
                 ("lidar_housing", 3, (0.0, -1.244, 0.19), (0.07, 0.07, 0.04)),
                 ("box_on_plate", 4, (0.05, -1.10, 0.20), (0.10, 0.05, 0.03))]
        got = round_candidates(boxes, floor, axle, math.pi / 2)
        self.assertEqual([g[0] for g in got], ["lidar_housing"])


if __name__ == "__main__":
    unittest.main()
