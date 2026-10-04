"""Step 3 acceptance: wheel speed PI in closed loop with motor + encoder.

    python -m unittest tests.test_step03_wheel_pi -v

The loop below is the real chain at real rates: PI at 100 Hz on encoder speed,
motor model at 500 Hz on a wheel with lumped inertia J (offline stand-in for
MuJoCo; the live check follows in step 5).
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from core import robot_config as rc  # noqa: E402
from core.encoder import IncrementalEncoder, wrap_to_pi  # noqa: E402
from core.motor_model import MotorModel  # noqa: E402
from core.wheel_pi import WheelPI  # noqa: E402

J = 0.004  # lumped reflected inertia per wheel [kg m^2]
CRUISE = rc.wheel_speeds_from_body(0.75, 0.0)[0]   # 14.16 rad/s
SLOW = rc.wheel_speeds_from_body(0.25, 0.0)[0]     # 4.72 rad/s


def closed_loop(targets, pi=None):
    """targets: list of wheel speed references, one per 10 ms tick."""
    m, e, pi = MotorModel(), IncrementalEncoder(), pi or WheelPI()
    e.reset(0.0)
    w = q = u = 0.0
    out = []
    for ref in targets:
        for _ in range(5):
            w += (m.update(u, w, rc.DYNAMICS_TIMESTEP_S) / J) * rc.DYNAMICS_TIMESTEP_S
            q += w * rc.DYNAMICS_TIMESTEP_S
        s = e.update(wrap_to_pi(q), rc.SIMULATION_TIMESTEP_S)
        r = pi.update(ref, s.speed_rad_s)
        u = r.command
        out.append((w, r))
    return out


def settle_time(ws, target, band=0.02):
    for i in range(len(ws)):
        if all(abs(x - target) <= band * abs(target) for x in ws[i:]):
            return i * rc.SIMULATION_TIMESTEP_S
    return math.inf


class TestWheelPI(unittest.TestCase):
    def test_step_to_cruise(self):
        ws = [w for w, _ in closed_loop([CRUISE] * 300)]
        self.assertLess(max(ws) / CRUISE - 1.0, 0.10)              # overshoot < 10 %
        self.assertLess(settle_time(ws, CRUISE), 1.0)              # 2 % band within 1 s
        self.assertAlmostEqual(sum(ws[-50:]) / 50, CRUISE, delta=0.1)

    def test_step_to_junction_speed(self):
        ws = [w for w, _ in closed_loop([SLOW] * 300)]
        self.assertAlmostEqual(sum(ws[-50:]) / 50, SLOW, delta=0.1)

    def test_reverse_is_symmetric(self):
        a = [w for w, _ in closed_loop([CRUISE] * 200)]
        b = [w for w, _ in closed_loop([-CRUISE] * 200)]
        self.assertAlmostEqual(sum(a[-50:]) / 50, -sum(b[-50:]) / 50, delta=0.05)

    def test_anti_windup(self):
        """2 s of an unreachable target, then cruise: no windup overshoot."""
        log = closed_loop([30.0] * 200 + [CRUISE] * 300)
        ws = [w for w, _ in log]
        integral_at_limit = log[199][1].integral
        self.assertLess(integral_at_limit * rc.WHEEL_PI_KI, 1.0 + 1e-9)   # never charged past full command
        after = ws[200:]
        self.assertLess(settle_time(after, CRUISE), 1.0)

    def test_reset(self):
        pi = WheelPI()
        closed_loop([CRUISE] * 100, pi)
        self.assertGreater(pi.integral, 0.0)
        pi.reset()
        self.assertEqual(pi.integral, 0.0)
        r = pi.update(0.0, 0.0)
        self.assertEqual(r.command, 0.0)

    def test_moving_average(self):
        pi = WheelPI(kp=0.0, ki=0.0)
        vals = [1.0, 2.0, 3.0, 4.0, 5.0, 6.0]
        f = [pi.update(0.0, v).filtered_speed_rad_s for v in vals]
        self.assertAlmostEqual(f[0], 1.0)
        self.assertAlmostEqual(f[4], 3.0)
        self.assertAlmostEqual(f[5], 4.0)   # window of 5


if __name__ == "__main__":
    unittest.main()
