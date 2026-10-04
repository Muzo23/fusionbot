"""Step 1 acceptance: motor model behaves as the identified physical model.

    python -m unittest tests.test_step01_motor_model -v
"""

import math
import sys
import unittest
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from core import robot_config as rc  # noqa: E402
from core.motor_model import MotorModel, apply_deadzone, steady_state_speed  # noqa: E402
from sim.motor_script import motor_script_source  # noqa: E402

DT = rc.DYNAMICS_TIMESTEP_S


class TestDeadzone(unittest.TestCase):
    def test_deadzone(self):
        self.assertEqual(apply_deadzone(0.06), 0.0)
        self.assertEqual(apply_deadzone(-0.05), 0.0)
        self.assertAlmostEqual(apply_deadzone(1.0), 1.0)
        self.assertAlmostEqual(apply_deadzone(-1.0), -1.0)
        self.assertAlmostEqual(apply_deadzone(0.53), 0.5)       # (0.53-0.06)/0.94
        self.assertAlmostEqual(apply_deadzone(2.0), 1.0)        # saturates first


class TestLag(unittest.TestCase):
    def test_first_order_time_constant(self):
        m = MotorModel()
        t = 0.0
        while t < rc.MOTOR_LAG_TAU_S - 1e-12:
            m.update(1.0, 0.0, DT)
            t += DT
        # exact discretisation: 1 - e^-1 after one time constant
        self.assertAlmostEqual(m.lagged_command, 1.0 - math.exp(-1.0), places=9)


class TestTorqueSpeed(unittest.TestCase):
    def test_stall_and_no_load(self):
        m = MotorModel(lagged_command=1.0)
        # rest, full command: stall torque minus Coulomb friction
        self.assertAlmostEqual(m.update(1.0, 0.0, 0.0), rc.MOTOR_STALL_TORQUE_NM - rc.COULOMB_FRICTION_NM)
        # at no-load speed the motor gives nothing, friction brakes
        tau = m.update(1.0, rc.MOTOR_NO_LOAD_SPEED_RAD_S, 0.0)
        self.assertLess(tau, 0.0)

    def test_steady_state_speed_matches_command_limit(self):
        # full command, friction only: ~22.69 rad/s, i.e. the 22.65 rad/s
        # command limit is the speed the motor can actually hold
        w = steady_state_speed(1.0)
        self.assertAlmostEqual(w, 22.69, delta=0.01)
        self.assertLess(abs(w - rc.MAX_WHEEL_COMMAND_SPEED_RAD_S), 0.1)

    def test_wheel_converges_to_steady_state(self):
        """Wheel with lumped inertia J driven at u = 0.5 for 2 s."""
        J = 0.004
        m, w = MotorModel(), 0.0
        for _ in range(int(2.0 / DT)):
            w += m.update(0.5, w, DT) / J * DT
        self.assertAlmostEqual(w, steady_state_speed(0.5), delta=0.01)

    def test_symmetry(self):
        a, b = MotorModel(), MotorModel()
        for _ in range(100):
            ta = a.update(0.7, 5.0, DT)
            tb = b.update(-0.7, -5.0, DT)
            self.assertAlmostEqual(ta, -tb, places=12)

    def test_torque_bounded(self):
        m = MotorModel(lagged_command=-1.0)
        self.assertGreaterEqual(m.update(-1.0, 30.0, DT), -rc.MOTOR_STALL_TORQUE_NM)


class TestLuaScript(unittest.TestCase):
    def test_constants_injected(self):
        src = motor_script_source()
        self.assertNotIn("__", src.replace("__index", ""))
        self.assertIn("function sysCall_joint", src)
        self.assertIn(f"{rc.MOTOR_STALL_TORQUE_NM:.17g}", src)


if __name__ == "__main__":
    unittest.main()
