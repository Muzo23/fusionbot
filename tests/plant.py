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
    """yaw_inertia / yaw_friction model the DIFFERENTIAL (rotation) mode:
    extra reflected inertia [kg m^2 per wheel] and a Coulomb friction torque
    with stiction [N m per wheel] (castors, tyre scrub). Default 0 = the
    original uncoupled plant."""
    J = 0.004

    def __init__(self, x=0.0, y=0.0, th=0.0, track=rc.EFFECTIVE_WHEEL_SEPARATION_M,
                 yaw_inertia=0.0, yaw_friction=0.0):
        self.x, self.y, self.th, self.track = x, y, th, track
        self.jy, self.fy = float(yaw_inertia), float(yaw_friction)
        self.ml, self.mr = MotorModel(), MotorModel()
        self.wl = self.wr = self.ql = self.qr = 0.0
        self.el, self.er = IncrementalEncoder(), IncrementalEncoder()
        self.el.reset(0.0)
        self.er.reset(0.0)
        self.pl, self.pr = WheelPI(), WheelPI()
        self.odo = DifferentialDriveOdometry(x, y, th)
        self.ul = self.ur = 0.0

    def step(self, v_ref, w_ref, u_diff_ff=0.0):
        """One 10 ms control tick. Returns the odometry increment.
        u_diff_ff: differential feed-forward added to the motor commands
        (right +, left -), applied from the next tick like the PI output."""
        h = rc.DYNAMICS_TIMESTEP_S
        for _ in range(5):
            tl = self.ml.update(self.ul, self.wl, h)
            tr = self.mr.update(self.ur, self.wr, h)
            if self.jy == 0.0 and self.fy == 0.0:
                self.wl += tl / self.J * h
                self.wr += tr / self.J * h
            else:
                wc, wd = 0.5 * (self.wl + self.wr), 0.5 * (self.wr - self.wl)
                tc, td = 0.5 * (tl + tr), 0.5 * (tr - tl)
                wc += tc / self.J * h
                if abs(wd) < 1e-9 and abs(td) <= self.fy:
                    wd = 0.0                                     # stiction
                else:
                    sgn = math.copysign(1.0, wd if abs(wd) >= 1e-9 else td)
                    wd_new = wd + (td - self.fy * sgn) / (self.J + self.jy) * h
                    wd = 0.0 if wd_new * wd < 0 else wd_new      # friction cannot reverse motion
                self.wl, self.wr = wc - wd, wc + wd
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
        self.ul = max(-1.0, min(1.0, self.pl.update(wl_ref, sl.speed_rad_s).command - u_diff_ff))
        self.ur = max(-1.0, min(1.0, self.pr.update(wr_ref, sr.speed_rad_s).command + u_diff_ff))
        self.last = (sl, sr)
        return inc

    def reset_pi(self):
        self.pl.reset()
        self.pr.reset()

    def wheels_still(self):
        sl, sr = self.last
        return sl.delta_counts == 0 and sr.delta_counts == 0
