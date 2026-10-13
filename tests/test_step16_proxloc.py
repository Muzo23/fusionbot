"""Step 16 (offline): PROX-LOC - configuration B (corridor walls and junctions on the maze lattice).

    python -m unittest discover -s tests -p "test_step16*" -v
"""

import math
import unittest

import numpy as np

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from core.configurations import CONFIGS
from localization import ekf as E
from localization.estimators import make_estimator
from localization.odometry import DifferentialDriveOdometry
from localization.prox_loc import Lattice, ProxLoc, corridor_measurement
from navigation.junction_types import ReferenceKind
from perception.proximity_types import ProximityReading, ProximitySnapshot

# a robot exactly on the centre line, aligned: the step-8a corridor model readings
CENTRED = dict(left=0.1835, front_left=0.3351, front=None, front_right=0.3397, right=0.1842)


def snap(**kw):
    d = {**CENTRED, **kw}
    return ProximitySnapshot(**{n: ProximityReading(n, d[n] is not None, d[n]) for n in d})


def filt(x, y, th, sd_pos=0.05, sd_th=math.radians(3)):
    f = E.PoseEKF(use_gyro=False, zupt=False)
    f.start(x, y, th, pos_sd=sd_pos, th_sd=sd_th)
    return f


class TestCorridor(unittest.TestCase):
    def test_centred_reading(self):
        y, a, R, kind = corridor_measurement(snap())
        self.assertEqual(kind, "full")
        self.assertAlmostEqual(y, 0.0, delta=0.002)
        self.assertAlmostEqual(math.degrees(a), 0.0, delta=0.3)
        self.assertTrue(np.all(np.linalg.eigvalsh(R) > 0))

    def test_lateral_and_heading_pulled_to_the_corridor(self):
        """Robot believes it is 4 cm left of the centre line x = 0 heading north + 2 deg; the walls say
        centred and aligned -> the estimate moves to the line and the axis."""
        f = filt(-0.04, 0.5, math.pi / 2 + math.radians(2.0))
        pl = ProxLoc(f, Lattice(0.0, 0.0, 1.085268))
        for _ in range(20 * 10):
            pl.corridor(snap())
        self.assertAlmostEqual(f.x[0], 0.0, delta=0.005)
        self.assertAlmostEqual(math.degrees(f.x[2]), 90.0, delta=0.3)
        self.assertEqual(f.stats["prox_corridor"][1], 0)

    def test_side_obstacle_rejected(self):
        """DYN3: an object 10 cm in front of the left wall -> y jumps by ~5 cm -> gated out."""
        f = filt(0.0, 0.5, math.pi / 2, sd_pos=0.003, sd_th=math.radians(0.3))
        pl = ProxLoc(f, Lattice(0.0, 0.0, 1.085268))
        for _ in range(30):
            pl.corridor(snap(left=0.0835, front_left=0.2351))
        self.assertGreater(f.stats["prox_corridor"][1], 0)
        self.assertEqual(f.stats["prox_corridor"][0], 0)
        self.assertAlmostEqual(f.x[0], 0.0, delta=1e-9)

    def test_no_measurement_in_openings(self):
        f = filt(0.0, 0.5, math.pi / 2)
        pl = ProxLoc(f, Lattice(0.0, 0.0, 1.085268))
        for _ in range(30):
            pl.corridor(snap(left=None, front_left=None, right=None, front_right=None))
        self.assertEqual(f.stats, {})


class TestJunction(unittest.TestCase):
    def test_side_event_fixes_the_along_coordinate(self):
        lat = Lattice(0.0, 0.0, 1.085268)
        f = filt(0.0, 1.085268 - 0.530606 + 0.06, math.pi / 2)   # 6 cm too far north
        ProxLoc(f, lat).junction(ReferenceKind.SIDE_CONFIRMED)
        self.assertLess(abs(f.x[1] - (1.085268 - 0.530606)), 0.01)

    def test_start_dead_end_is_not_a_node(self):
        lat = Lattice(0.0, 0.0, 1.085268)
        f = filt(0.0, -1.422634 + 0.02, -math.pi / 2)
        self.assertIsNone(ProxLoc(f, lat).junction(ReferenceKind.DEAD_END))


class TestRun(unittest.TestCase):
    def test_b_on_m5(self):
        """M5 (14 loops): with PROX-LOC the map is correct and the pose error stays at millimetres."""
        import run as runner
        s = runner.main(["--offline", "--config", "B", "--noise", "realistic", "--maze", "M5", "--seed", "2",
                         "--tag", "test16"])
        self.assertTrue(s["S4_mission_success"])
        self.assertTrue(s["T_map_correct"])
        self.assertLess(s["L1_pos_rmse_m"], 0.02)

    def test_gating_switch(self):
        est = make_estimator(CONFIGS["B"], DifferentialDriveOdometry(), gating=False)
        self.assertTrue(math.isinf(est.ekf.gate[2]))


if __name__ == "__main__":
    unittest.main()
