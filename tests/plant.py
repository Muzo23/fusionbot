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
                 yaw_inertia=0.0, yaw_friction=0.0, turn_slip=0.0, turn_deficit=None):
        self.x, self.y, self.th, self.track = x, y, th, track
        self.jy, self.fy = float(yaw_inertia), float(yaw_friction)
        # turn_slip: in in-place rotation the BODY turns (1 - turn_slip) of what the
        # wheels (encoders) say; live step 7: ~0.6 deg per 90 deg -> 0.0067
        self.turn_slip = float(turn_slip)
        # turn_deficit=(ccw_rad, cw_rad): every in-place rotation the BODY loses a
        # FIXED angle w.r.t. the encoders, independent of the turn angle (live:
        # 68 turns in the step-11 T1/T2/T3 runs, CCW 0.47 deg, CW 0.66 deg)
        self.turn_deficit = turn_deficit
        self._deficit_left = 0.0
        self._rotating = False
        # step 19 D4: slip(v, w, h) -> (left, right) contact factors (1 = no slip); None = no slip
        self.slip = None
        self.ml, self.mr = MotorModel(), MotorModel()
        self.wl = self.wr = self.ql = self.qr = 0.0
        self.el, self.er = IncrementalEncoder(), IncrementalEncoder()
        self.el.reset(0.0)
        self.er.reset(0.0)
        self.pl, self.pr = WheelPI(), WheelPI()
        self.odo = DifferentialDriveOdometry(x, y, th)
        self.ul = self.ur = 0.0
        self.last = None

    def step(self, v_ref, w_ref, u_diff_ff=0.0):
        """One 10 ms control tick. Returns the odometry increment.
        u_diff_ff: differential feed-forward added to the motor commands
        (right +, left -), applied from the next tick like the PI output.
        NOTE: refs passed here are used by the PI AFTER this tick's measurement;
        for exact live ordering use advance() -> controller -> control()."""
        inc = self.advance()
        self.control(v_ref, w_ref, u_diff_ff)
        return inc

    def advance(self):
        """Simulate 10 ms with the held motor commands, read encoders, update
        odometry. Returns the odometry increment (encoder samples in self.last)."""
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
            if self.slip is not None:                    # wheels spin, the body moves less (step 19 D4)
                fl, fr = self.slip(v, w, h)
                v = rc.WHEEL_RADIUS_M * (fl * self.wl + fr * self.wr) / 2
                w = rc.WHEEL_RADIUS_M * (fr * self.wr - fl * self.wl) / self.track
            if self.turn_slip and abs(v) < 0.02:
                w *= 1.0 - self.turn_slip
            if self.turn_deficit is not None:
                in_place = abs(v) < 0.02 and abs(w) > 0.05
                if in_place and not self._rotating:
                    self._deficit_left = self.turn_deficit[0 if w > 0 else 1]
                self._rotating = in_place or (self._rotating and abs(w) > 0.005)
                if self._deficit_left > 0.0 and abs(w) > 0.0:
                    cut = min(self._deficit_left, 0.05 * abs(w) * h)   # spread over ~0.2 s
                    self._deficit_left -= cut
                    w -= math.copysign(cut / h, w)
            self.x += v * math.cos(self.th + w * h / 2) * h
            self.y += v * math.sin(self.th + w * h / 2) * h
            self.th += w * h
        sl = self.el.update(wrap_to_pi(self.ql), rc.SIMULATION_TIMESTEP_S)
        sr = self.er.update(wrap_to_pi(self.qr), rc.SIMULATION_TIMESTEP_S)
        self.last = (sl, sr)
        return self.odo.update(sl.delta_counts, sr.delta_counts)

    def control(self, v_ref, w_ref, u_diff_ff=0.0):
        """Wheel PIs on the latest encoder samples -> motor commands held for the next tick."""
        sl, sr = self.last
        wl_ref, wr_ref = rc.wheel_speeds_from_body(v_ref, w_ref)
        self.ul = max(-1.0, min(1.0, self.pl.update(wl_ref, sl.speed_rad_s).command - u_diff_ff))
        self.ur = max(-1.0, min(1.0, self.pr.update(wr_ref, sr.speed_rad_s).command + u_diff_ff))

    def reset_pi(self):
        self.pl.reset()
        self.pr.reset()

    def wheels_still(self):
        sl, sr = self.last
        return sl.delta_counts == 0 and sr.delta_counts == 0
