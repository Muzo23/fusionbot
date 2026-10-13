"""Step 13b-2 (offline): experiment runner + recorder.

    python -m unittest discover -s tests -p "test_step13b_runner*" -v
"""

import contextlib
import gzip
import hashlib
import io
import json
import math
import shutil
import tempfile
import unittest
from pathlib import Path

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

import run as runner
from core.configurations import CONFIGS
from experiment.recorder import RUNS_DIR, RunRecorder, load_run, load_scans
from localization.estimators import make_estimator
from localization.odometry import DifferentialDriveOdometry


def offline(*args):
    with contextlib.redirect_stdout(io.StringIO()):
        return runner.main(["--offline", "--tag", "unittest", *args])


def folder(maze, cfg, noise, seed):
    return RUNS_DIR / f"{maze}_{cfg}_{noise}_s{seed}_offline_unittest"


class TestRunner(unittest.TestCase):
    @classmethod
    def tearDownClass(cls):
        for p in RUNS_DIR.glob("*_unittest"):
            shutil.rmtree(p, ignore_errors=True)

    def test_ideal_run_is_recorded(self):
        s = offline("--config", "A", "--noise", "ideal", "--maze", "T2")
        self.assertTrue(s["S4_mission_success"])
        self.assertTrue(s["T_map_correct"])
        d = folder("T2", "A", "ideal", 0)
        for f in ("meta.json", "ticks.csv.gz", "events.csv", "map_topo.json", "summary.json"):
            self.assertTrue((d / f).exists(), f)
        r = load_run(d)
        self.assertEqual(r["meta"]["configuration"]["name"], "A")
        self.assertEqual(r["meta"]["noise"]["profile"], "ideal")
        self.assertEqual(len(r["ticks"]), s["ticks"])
        self.assertEqual(len(r["events"]), s["events"])
        t0 = r["ticks"][0]
        for col in ("gt_x", "est_x", "gyro_ideal", "gyro_meas", "d_front", "dm_front", "collision"):
            self.assertIn(col, t0)
        # ideal profile: measured == ideal
        self.assertTrue(all(t["d_left"] == t["dm_left"] for t in r["ticks"][:500]))
        # step 13b-3: all GT metrics computed after the run from the record
        m = json.loads((d / "metrics.json").read_text())
        self.assertTrue(m["S4_mission_success"])
        self.assertGreater(m["P7_clearance_min_mm"], 5.0)
        self.assertLess(m["P1_centreline_rms_mm"], 15.0)            # corridor guidance keeps the robot centred
        self.assertLess(m["P2_axis_angle_rms_deg"], 1.5)
        self.assertLess(m["P4_turn_err_max_deg"], 3.0)
        self.assertLess(m["P5_turn_point_max_mm"], 40.0)
        self.assertLess(abs(m["P3_length_ratio"] - 1.0), 0.05)
        self.assertGreater(m["L1_pos_max_m"], m["L1_pos_rmse_m"])
        self.assertEqual(m["T3_loop_closures"], m["T3_expected_cycles"])
        self.assertIsNotNone(m["L5_nees_in_95pct"])                   # step 15: A is the EKF prediction (has P)

    def test_realistic_reproducible_and_seeded(self):
        a = offline("--config", "A", "--noise", "realistic", "--maze", "T2", "--seed", "5", "--tag", "x1")
        h1 = hashlib.md5(gzip.open(RUNS_DIR / "T2_A_realistic_s5_offline_x1" / "ticks.csv.gz").read()).hexdigest()
        b = offline("--config", "A", "--noise", "realistic", "--maze", "T2", "--seed", "5", "--tag", "x2")
        h2 = hashlib.md5(gzip.open(RUNS_DIR / "T2_A_realistic_s5_offline_x2" / "ticks.csv.gz").read()).hexdigest()
        for p in ("x1", "x2"):
            shutil.rmtree(RUNS_DIR / f"T2_A_realistic_s5_offline_{p}", ignore_errors=True)
        a.pop("wall_time_s")
        b.pop("wall_time_s")
        self.assertEqual(a, b)                       # same seed -> the same run, bit for bit
        self.assertEqual(h1, h2)
        self.assertGreater(abs(a["gyro_true_initial_bias_deg_s"]), 0.0)
        # start calibration recovers the drawn zero-rate offset (0.5 s, 0.05 deg/s noise)
        self.assertLess(abs(a["gyro_start_bias_estimate_deg_s"] - a["gyro_true_initial_bias_deg_s"]), 0.05)

    def test_other_configurations_wait_for_the_ekf(self):
        for name in "":                                               # every configuration runs since step 17
            with self.assertRaises(NotImplementedError):
                make_estimator(CONFIGS[name], DifferentialDriveOdometry())


class TestLidarFrame(unittest.TestCase):
    def test_scan_frame_of_a_rotated_hokuyo(self):
        """Points published in the Hokuyo BASE frame (any orientation) -> bearing / range in the scan
        frame, with the transform recorded by lidar_setup; offline scans through it match the walls."""
        import contextlib as cl
        from experiment.core import DEFAULT_MOUNT, RobotRun
        from experiment.drivers import offline_loop
        from mazes.maze_spec import MAZES
        from perception.lidar_scan import RangeModel
        from sensors.noise_models import NoiseProfile
        from sim.lidar_setup import scan_frame_in_base

        def rot(a, b):      # rotz(a) @ rotx(b)
            ca, sa, cb, sb = math.cos(a), math.sin(a), math.cos(b), math.sin(b)
            return [[ca, -sa * cb, sa * sb], [sa, ca * cb, -ca * sb], [0.0, sb, cb]]
        R, t = rot(0.7, math.pi / 2), (1.0, 2.0, 0.2)
        m = [R[0][0], R[0][1], R[0][2], t[0], R[1][0], R[1][1], R[1][2], t[1], R[2][0], R[2][1], R[2][2], t[2]]
        mount = {**DEFAULT_MOUNT, **scan_frame_in_base(m, (1.01, 2.02, 0.22), 0.3)}
        with tempfile.TemporaryDirectory() as tmp:
            rec = RunRecorder(Path(tmp))
            run = RobotRun(MAZES["T2"], CONFIGS["A"], NoiseProfile("ideal", 0), rec, max_time_s=6.0,
                           lidar_files=(mount, RangeModel(), None))
            with cl.redirect_stdout(io.StringIO()):
                offline_loop(run, MAZES["T2"], lidar=True)
            s = run.summary()
            rec.write_meta({"maze": {"id": "T2"}, "configuration": {"name": "A"}, "noise": {"profile": "ideal", "seed": 0},
                            "lidar": {"mount": mount, "extrinsic": [0.0, 0.0, 0.0]}})
            rec.close(s)
            from evaluation.metrics import compute
            m = compute(Path(tmp))
        self.assertGreater(s["lidar_scans"], 40)
        self.assertLess(m["lidar_gt_p90_mm"], 1.0)


class TestRecorder(unittest.TestCase):
    def test_scans_roundtrip(self):
        with tempfile.TemporaryDirectory() as tmp:
            rec = RunRecorder(Path(tmp))
            rec.write_meta({"x": 1})
            rec.tick({"k": 1, "v": None})
            rec.scan(0.1, 10, [-0.5, 0.0, 0.5], [1.23456, None, 4.9], [1.2, 0.5, None], (0, 0, 0), (0, 0, 0))
            rec.close({"ok": True, "nan": math.nan})
            z = load_scans(Path(tmp))
            self.assertAlmostEqual(z["ranges_ideal"][0, 0], 1.2346, places=4)   # 0.1 mm units
            self.assertTrue(math.isnan(z["ranges_ideal"][0, 1]))
            self.assertTrue(math.isnan(z["ranges_measured"][0, 2]))
            self.assertIsNone(json.loads((Path(tmp) / "summary.json").read_text())["nan"])


if __name__ == "__main__":
    unittest.main()
