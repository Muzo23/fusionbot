"""Step 19 (offline): disturbances D1-D7, noise scaling N1 / N2, ZUPT bias-fault recovery.

    python -m unittest discover -s tests -p "test_step19*" -v
"""

import math
import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from experiment import disturbances as DI
from experiment.step19 import job_args, jobs
from localization import ekf as E
from sensors.noise_models import NoiseProfile


def dist(text, seed=1, nodes=8):
    return DI.Disturbance(DI.parse(text), seed, nodes)


class TestWheelAngles(unittest.TestCase):
    def _drive(self, d, dl, dr, n=200):
        ql = qr = 3.0                                   # start near +pi: the angles wrap many times
        ol, or_ = d.joints(ql, qr)
        tl = tr = 0.0
        for _ in range(n):
            ql, qr = math.remainder(ql + dl, 2 * math.pi), math.remainder(qr + dr, 2 * math.pi)
            nl, nr = d.joints(ql, qr)
            tl += math.remainder(nl - ol, 2 * math.pi)
            tr += math.remainder(nr - or_, 2 * math.pi)
            ol, or_ = nl, nr
        return tl, tr

    def test_d2_scales_distance_only(self):
        tl, tr = self._drive(dist("D2=0.02"), 0.3, 0.3)
        self.assertAlmostEqual(tl, 1.02 * 60.0, places=6)
        self.assertAlmostEqual(tr, 1.02 * 60.0, places=6)
        tl, tr = self._drive(dist("D2=0.02"), -0.3, 0.3)          # pure rotation: unchanged
        self.assertAlmostEqual(tl, -60.0, places=6)
        self.assertAlmostEqual(tr, 60.0, places=6)

    def test_d3_scales_rotation_only(self):
        tl, tr = self._drive(dist("D3=-0.05"), -0.3, 0.3)
        self.assertAlmostEqual(tr, 0.95 * 60.0, places=6)
        tl, tr = self._drive(dist("D3=-0.05"), 0.3, 0.3)
        self.assertAlmostEqual(tl, 60.0, places=6)

    def test_no_disturbance_is_identity(self):
        d = DI.Disturbance(None, 1, 8)
        self.assertEqual(d.joints(1.0, -2.0), (1.0, -2.0))
        self.assertEqual(d.scan([0.5, None]), [0.5, None])
        self.assertEqual(d.proximity("left", 0.18), 0.18)
        self.assertIsNone(d.plant_slip())


class TestMidExplore(unittest.TestCase):
    def test_d7_starts_at_half_the_junctions_while_driving(self):
        d = dist("D7=L", nodes=8)
        noise = NoiseProfile("realistic", 1)
        self.assertEqual(d.proximity("left", 0.18), 0.18)
        self.assertIsNone(d.maybe_trigger(10.0, 3, True, noise))
        self.assertIsNone(d.maybe_trigger(11.0, 4, False, noise))        # standing / turning: not yet
        self.assertIsNotNone(d.maybe_trigger(12.0, 4, True, noise))
        self.assertAlmostEqual(d.proximity("left", 0.18), 0.18 + DI.D7_OFFSET_M)
        self.assertEqual(d.proximity("right", 0.18), 0.18)
        self.assertEqual(d.active_from_t, 12.0)

    def test_d5_adds_the_bias_step_once(self):
        d = dist("D5=0.5")
        noise = NoiseProfile("realistic", 1)
        b0 = noise.gyro.bias
        d.maybe_trigger(5.0, 9, True, noise)
        d.maybe_trigger(6.0, 9, True, noise)
        self.assertAlmostEqual(noise.gyro.bias - b0, math.radians(0.5))

    def test_d6_degrades_the_given_fraction(self):
        d = dist("D6=0.5")
        d.maybe_trigger(1.0, 9, True, NoiseProfile("realistic", 1))
        r = [2.0] * 20000
        out = d.scan(r)
        changed = sum(o != 2.0 for o in out) / len(r)
        none = sum(o is None for o in out) / len(r)
        self.assertAlmostEqual(changed, 0.5, delta=0.02)
        self.assertAlmostEqual(none, 0.25, delta=0.02)
        self.assertTrue(all(o is None or 0.05 <= o <= 2.0 for o in out))


class TestSlip(unittest.TestCase):
    def test_d4_loses_the_path_fraction(self):
        for level in (0.01, 0.03):
            slip = dist(f"D4={level}", seed=3).plant_slip()
            moved = wheel = 0.0
            h, v = 0.002, 0.25
            for _ in range(int(200 / v / h)):                 # 200 m straight
                fl, fr = slip(v, 0.0, h)
                moved += 0.5 * (fl + fr) * v * h
                wheel += v * h
            self.assertAlmostEqual(1.0 - moved / wheel, level, delta=0.35 * level)

    def test_d1_turn_loss(self):
        self.assertEqual(dist("D1=3").turn_loss(1.0), 3.0)
        self.assertEqual(dist("D2=0.01").turn_loss(1.0), 1.0)


class TestNoiseScale(unittest.TestCase):
    def test_same_draws_scaled_sigma(self):
        a = NoiseProfile("realistic", 4)
        b = NoiseProfile("realistic", 4, {"gyro": 2.0, "lidar": 2.0, "proximity": 2.0})
        self.assertAlmostEqual(b.gyro.scale_error, 2.0 * a.gyro.scale_error)
        self.assertEqual(a.gyro.bias, b.gyro.bias)                     # start offset: not scaled
        self.assertAlmostEqual(b.lidar.spec.sd_near_m, 2.0 * a.lidar.spec.sd_near_m)
        da, db = a.proximity.measure(0.3) - 0.3, b.proximity.measure(0.3) - 0.3
        self.assertAlmostEqual(db, 2.0 * da, delta=0.0011)              # 1 mm quantisation

    def test_labels(self):
        self.assertEqual(DI.noise_label({"gyro": 2.0, "lidar": 2.0, "proximity": 2.0}), "N1x2")
        self.assertEqual(DI.noise_label({"gyro": 4.0, "lidar": 1.0, "proximity": 1.0}), "N2gyrox4")
        self.assertIsNone(DI.noise_label({"gyro": 1.0, "lidar": 1.0, "proximity": 1.0}))
        self.assertEqual(DI.parse("D2=-0.02").label, "D2m0_02")
        self.assertEqual(job_args("N2prox", 8.0)[1], ["N2proxx8"])


class TestPlan(unittest.TestCase):
    def test_only_relevant_configurations(self):
        js = jobs(["D5", "D6"], ["T3"], [1], None)
        self.assertEqual({j["config"] for j in js if j["family"] == "D5"}, {"C", "E"})
        self.assertEqual({j["config"] for j in js if j["family"] == "D6"}, {"D", "E"})
        self.assertTrue(all("--offline-lidar" in j["extra"] for j in js if j["config"] in ("D", "E")))
        self.assertFalse(any("--offline-lidar" in j["extra"] for j in jobs(["D2"], ["T3"], [1], ["A"])))


class TestZuptRecovery(unittest.TestCase):
    def _filter(self, step_deg):
        f = E.PoseEKF(use_gyro=True, zupt=True)
        f.start(0.0, 0.0, 0.0, bias=0.01, bias_sd=math.radians(0.007))
        for _ in range(30):
            f.at_rest(0, 0)
        omega = 0.01 + math.radians(step_deg)
        for _ in range(200):                                    # 2 s at rest after the step
            f.predict(0.0, 0.0, omega, 0.01)
            if f.at_rest(0, 0):
                f.zupt_update(omega)
        return f, omega

    def test_bias_step_is_learned(self):
        f, omega = self._filter(0.5)
        self.assertEqual(len(f.bias_resets), 1)
        self.assertAlmostEqual(f.x[3], omega, delta=math.radians(0.02))

    def test_without_recovery_the_gate_blocks_it(self):
        old = E.ZUPT_RECOVERY
        E.ZUPT_RECOVERY = False
        try:
            f, omega = self._filter(0.5)
        finally:
            E.ZUPT_RECOVERY = old
        self.assertEqual(len(f.bias_resets), 0)
        self.assertGreater(abs(f.x[3] - omega), math.radians(0.4))

    def test_small_change_needs_no_reset(self):
        f, omega = self._filter(0.005)
        self.assertEqual(len(f.bias_resets), 0)


if __name__ == "__main__":
    unittest.main()
