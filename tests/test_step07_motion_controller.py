"""Step 7 acceptance: motion controller primitives in closed loop.

    python -m unittest tests.test_step07_motion_controller -v
"""

import math
import unittest

from plant import DrivePlant

from navigation.motion_controller import (
    TURN_OMEGA_MAX_RAD_S, TURN_TOLERANCE_RAD, MotionController, MotionMode,
)


def do_turn(deg, plant=None, settle=50, **mc_kw):
    p = plant or DrivePlant()
    mc = MotionController(**mc_kw)
    p.reset_pi()
    mc.start_turn_degrees(deg)
    th0_true, acc_odo = p.th, 0.0
    inc = p.step(0.0, 0.0)
    ticks, done, max_w = 0, None, 0.0
    while ticks < 2000:
        ref = mc.update(inc.dtheta_rad)
        max_w = max(max_w, abs(ref.angular_velocity_rad_s))
        if ref.completed:
            done = ref
            break
        inc = p.step(ref.linear_velocity_m_s, ref.angular_velocity_rad_s)
        acc_odo += inc.dtheta_rad
        ticks += 1
    p.reset_pi()
    for _ in range(settle):                   # coast/settle with zero reference
        acc_odo += p.step(0.0, 0.0).dtheta_rad
    return done, math.degrees(p.th - th0_true), math.degrees(acc_odo), ticks, max_w


class TestTurns(unittest.TestCase):
    def test_four_directed_turns(self):
        for deg in (90.0, -90.0, 180.0, -180.0):
            done, true_deg, odo_deg, ticks, max_w = do_turn(deg)
            self.assertIsNotNone(done, f"{deg}: turn did not complete")
            self.assertLessEqual(abs(done.turn_error_rad), TURN_TOLERANCE_RAD)
            # odometry-relative result incl. coasting after completion
            self.assertAlmostEqual(odo_deg, deg, delta=0.6, msg=f"{deg}")
            # sign preserved: +180 is CCW, -180 is CW
            self.assertEqual(math.copysign(1, true_deg), math.copysign(1, deg))
            self.assertLessEqual(max_w, TURN_OMEGA_MAX_RAD_S + 1e-12)
            # omega_max 1.2 rad/s -> 180 deg needs >= 2.6 s; P tail adds ~1.5 s
            self.assertLess(ticks * 0.01, 5.0, msg=f"{deg}")

    def test_alpha_limited_turns(self):
        for deg in (90.0, -180.0):
            done, true_deg, odo_deg, ticks, _ = do_turn(deg, alpha_max_rad_s2=2.4)
            self.assertIsNotNone(done)
            self.assertAlmostEqual(odo_deg, deg, delta=0.6)
            self.assertLess(ticks * 0.01, 5.0)

    def test_alpha_limit_bounds_slew(self):
        mc = MotionController(alpha_max_rad_s2=2.4)
        mc.start_turn_degrees(90.0)
        w = [mc.update(0.0).angular_velocity_rad_s for _ in range(5)]
        self.assertAlmostEqual(w[0], 0.024)
        self.assertTrue(all(abs(b - a) <= 0.024 + 1e-12 for a, b in zip([0.0] + w, w)))

    def test_completed_flag_once_then_idle(self):
        mc = MotionController()
        mc.start_turn_degrees(1.0)
        steps = [0.5, 0.5] + [0.0] * 8          # reach 1 deg, then stand still
        refs = [mc.update(math.radians(d)) for d in steps]
        self.assertEqual(sum(r.completed for r in refs), 1)
        self.assertIs(mc.mode, MotionMode.IDLE)
        self.assertEqual(refs[-1].angular_velocity_rad_s, 0.0)

    def test_zero_turn_is_idle(self):
        mc = MotionController()
        mc.start_turn(0.0)
        self.assertIs(mc.mode, MotionMode.IDLE)

    def test_straight_and_stop(self):
        mc = MotionController()
        mc.start_straight(0.25)
        r = mc.update(0.3)              # odometry dtheta ignored in STRAIGHT
        self.assertEqual((r.linear_velocity_m_s, r.angular_velocity_rad_s), (0.25, 0.0))
        mc.stop()
        self.assertEqual(mc.update().linear_velocity_m_s, 0.0)

    def test_turn_does_not_wrap(self):
        mc = MotionController()
        mc.start_turn_degrees(-180.0)
        r = mc.update(0.0)
        self.assertLess(r.angular_velocity_rad_s, 0.0)   # CW, not the short way


if __name__ == "__main__":
    unittest.main()
