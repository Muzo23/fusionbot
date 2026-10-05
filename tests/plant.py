"""Offline drive plant for unit tests (not a validation of the real robot).

Two wheels with the step-1 motor model and lumped inertia, ideal unicycle
kinematics with B_eff, step-2 encoders, step-3 wheel PI and step-4 odometry,
at the real rates (motor 500 Hz, control 100 Hz).
"""

import math
import sys
from pathlib import Path

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))

from core import robot_config as rc  # noqa: E402
from core.encoder import IncrementalEncoder, wrap_to_pi  # noqa: E402
from core.motor_model import MotorModel  # noqa: E402
from core.wheel_pi import WheelPI  # noqa: E402
from localization.odometry import DifferentialDriveOdometry  # noqa: E402


class DrivePlant:
    J = 0.004

    def __init__(self, x=0.0, y=0.0, th=0.0, track=rc.EFFECTIVE_WHEEL_SEPARATION_M):
        self.x, self.y, self.th, self.track = x, y, th, track
        self.ml, self.mr = MotorModel(), MotorModel()
        self.wl = self.wr = self.ql = self.qr = 0.0
        self.el, self.er = IncrementalEncoder(), IncrementalEncoder()
        self.el.reset(0.0)
        self.er.reset(0.0)
        self.pl, self.pr = WheelPI(), WheelPI()
        self.odo = DifferentialDriveOdometry(x, y, th)
        self.ul = self.ur = 0.0

    def step(self, v_ref, w_ref):
        """One 10 ms control tick. Returns the odometry increment."""
        h = rc.DYNAMICS_TIMESTEP_S
        for _ in range(5):
            self.wl += self.ml.update(self.ul, self.wl, h) / self.J * h
            self.wr += self.mr.update(self.ur, self.wr, h) / self.J * h
            self.ql += self.wl * h
            self.qr += self.wr * h
            v = rc.WHEEL_RADIUS_M * (self.wl + self.wr) / 2
            w = rc.WHEEL_RADIUS_M * (self.wr - self.wl) / self.track
            self.x += v * math.cos(self.th + w * h / 2) * h
            self.y += v * math.sin(self.th + w * h / 2) * h
            self.th += w * h
        sl = self.el.update(wrap_to_pi(self.ql), rc.SIMULATION_TIMESTEP_S)
        sr = self.er.update(wrap_to_pi(self.qr), rc.SIMULATION_TIMESTEP_S)
        inc = self.odo.update(sl.delta_counts, sr.delta_counts)
        wl_ref, wr_ref = rc.wheel_speeds_from_body(v_ref, w_ref)
        self.ul = self.pl.update(wl_ref, sl.speed_rad_s).command
        self.ur = self.pr.update(wr_ref, sr.speed_rad_s).command
        self.last = (sl, sr)
        return inc

    def reset_pi(self):
        self.pl.reset()
        self.pr.reset()

    def wheels_still(self):
        sl, sr = self.last
        return sl.delta_counts == 0 and sr.delta_counts == 0
