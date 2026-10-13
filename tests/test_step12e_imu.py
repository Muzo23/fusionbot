"""Step 12e (offline): the IMU formulas of the FusionBotImuScript (Python mirror sim.imu_setup.imu_from_poses).

    python -m unittest discover -s tests -p "test_step12e*" -v
"""

import math
import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from sim.imu_setup import GRAVITY_M_S2, IMU_SCRIPT, imu_from_poses


def rz(a):
    c, s = math.cos(a), math.sin(a)
    return [[c, -s, 0.0], [s, c, 0.0], [0.0, 0.0, 1.0]]


class TestImu(unittest.TestCase):
    def test_at_rest(self):
        smp = [(0.01 * k, rz(0.7), [1.0, 2.0, 0.18]) for k in range(5)]
        out = imu_from_poses(smp)
        t, valid, g, a = out[-1]
        self.assertEqual(valid, 2)
        for v in g + a[:2]:
            self.assertAlmostEqual(v, 0.0, places=9)
        self.assertAlmostEqual(a[2], GRAVITY_M_S2, places=9)

    def test_turn_on_circle(self):
        """CCW circle, v = 0.25 m/s, omega = 1.2 rad/s: gyro z = omega,
        lateral specific force = v * omega (towards the centre), forward ~ 0."""
        v, w, dt = 0.25, 1.2, 0.01
        rho = v / w
        smp = []
        for k in range(10):
            th = w * dt * k
            smp.append((dt * k, rz(th), [rho * math.sin(th), rho * (1 - math.cos(th)), 0.18]))
        t, valid, g, a = imu_from_poses(smp)[-1]
        self.assertAlmostEqual(g[2], w, delta=1e-4)
        self.assertAlmostEqual(g[0], 0.0, places=9)
        self.assertAlmostEqual(a[1], v * w, delta=0.005)
        self.assertLess(abs(a[0]), 0.005)
        self.assertAlmostEqual(a[2], GRAVITY_M_S2, places=9)

    def test_forward_acceleration(self):
        acc, dt = 0.8, 0.01
        smp = [(dt * k, rz(math.pi / 2), [0.0, 0.5 * acc * (dt * k) ** 2, 0.18]) for k in range(6)]
        t, valid, g, a = imu_from_poses(smp)[-1]
        self.assertAlmostEqual(a[0], acc, delta=1e-6)       # body x = world y (heading 90 deg)
        self.assertAlmostEqual(a[1], 0.0, delta=1e-6)

    def test_expected_specific_force_from_gt(self):
        from checks.step12e_live_imu import expected_specific_force
        v, w, dt, r = 0.25, 0.6, 0.01, 0.0715
        rho = v / w
        rows = []
        for k in range(50):
            th = w * dt * k
            rows.append({"gt_x": rho * math.sin(th), "gt_y": rho * (1 - math.cos(th)),
                         "gt_th_unwrapped": th, "gt_w": w})
        ax, ay = expected_specific_force(rows, r)
        self.assertAlmostEqual(ay[25], v * w, delta=1e-3)
        self.assertAlmostEqual(ax[25], -w * w * r, delta=1e-3)

    def test_script_publishes_the_properties(self):
        for name in ("signal.fusionbotImuGyro", "signal.fusionbotImuAccel", "signal.fusionbotImuCount",
                     "sysCall_sensing", "sysCall_cleanup", "customData.fusionbotImuLogPath"):
            self.assertIn(name, IMU_SCRIPT)
        self.assertNotIn("{{", IMU_SCRIPT)


if __name__ == "__main__":
    unittest.main()
