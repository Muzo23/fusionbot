"""
Step 5 LIVE acceptance test (CoppeliaSim, lockstep bridge).

Exercises steps 1-5 together on the real robot model:
    motor model (Lua, 500 Hz) -> MuJoCo -> encoders (step 2) -> wheel PI (step 3)
    -> odometry (step 4), over the lockstep bridge (step 5).

Run from src/ with the scene open, robot at START, simulation STOPPED:

    python -m sim.install_bridge          # once, then Ctrl+S in CoppeliaSim
    python -m checks.step05_live_drive

Three simulation runs (the scene resets to START between runs):
    run 1  wheel step: both wheels 0 -> 14.16 rad/s (0.75 m/s) for 1.0 s, then 0
    run 2  straight 1.000 m at 0.25 m/s (stop on ODOMETRY distance), settle,
           then in-place +90 deg at 0.6 rad/s (stop on ODOMETRY heading), settle
    run 3  identical to run 2 -> determinism check

GT is read ONLY for the report. Nothing in the control loop sees it.
CSV logs: results/step05/run{1,2,3}.csv
"""

from __future__ import annotations

import csv
import math
import time
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from core import robot_config as rc
from core.encoder import IncrementalEncoder
from core.wheel_pi import WheelPI
from localization.odometry import DifferentialDriveOdometry, wrap_to_pi
from sim.sim_utils import prepare_lockstep, wait_until_stopped
from sim.transport import LockstepSession

RESULTS = Path(__file__).resolve().parents[2] / "results" / "step05"
SETTLE_START_S = 0.5
CRUISE_W = rc.wheel_speeds_from_body(0.75, 0.0)[0]


class Stack:
    """Encoders + odometry + two wheel PIs: the drive part of the robot."""

    def __init__(self) -> None:
        self.el, self.er = IncrementalEncoder(), IncrementalEncoder()
        self.odo = DifferentialDriveOdometry()
        self.pl, self.pr = WheelPI(), WheelPI()
        self.started = False

    def start(self, ql: float, qr: float) -> None:
        self.el.reset(ql)
        self.er.reset(qr)
        self.odo.reset()
        self.pl.reset()
        self.pr.reset()
        self.started = True

    def tick(self, ql, qr, wl_ref, wr_ref, dt=rc.SIMULATION_TIMESTEP_S):
        sl, sr = self.el.update(ql, dt), self.er.update(qr, dt)
        inc = self.odo.update(sl.delta_counts, sr.delta_counts)
        cl = self.pl.update(wl_ref, sl.speed_rad_s).command
        cr = self.pr.update(wr_ref, sr.speed_rad_s).command
        return cl, cr, sl, sr, inc


def run(sim, plan, label: str):
    """plan(t_rel, stack) -> (wl_ref, wr_ref, phase, done)."""
    stack = Stack()
    rows = []
    link = LockstepSession()
    wall0 = time.perf_counter()
    try:
        time.sleep(0.2)
        sim.startSimulation()
        while True:
            s = link.next_state(timeout_s=10.0)
            t = s.simulation_time_s
            if not stack.started:
                if t >= SETTLE_START_S:
                    stack.start(s.left_joint_position_rad, s.right_joint_position_rad)
                    t0, gt0 = t, (s.gt_axle_x_m, s.gt_axle_y_m, s.gt_heading_rad)
                link.send(s.state_sequence, 0.0, 0.0)
                continue
            wl, wr, phase, done = plan(t - t0, stack)
            cl, cr, sl, sr, inc = stack.tick(
                s.left_joint_position_rad, s.right_joint_position_rad, wl, wr
            )
            link.send(s.state_sequence, cl, cr)
            p = stack.odo.pose
            rows.append({
                "t": t - t0, "phase": phase, "wl_ref": wl, "wr_ref": wr,
                "wl_enc": sl.speed_rad_s, "wr_enc": sr.speed_rad_s, "cmd_l": cl, "cmd_r": cr,
                "odo_x": p.x_m, "odo_y": p.y_m, "odo_th_unwrapped": stack.odo.heading_unwrapped,
                "odo_s": stack.odo.signed_distance_m,
                "gt_x": s.gt_axle_x_m - gt0[0], "gt_y": s.gt_axle_y_m - gt0[1],
                "gt_dth": wrap_to_pi(s.gt_heading_rad - gt0[2]), "gt_heading": s.gt_heading_rad,
                "collision": int(s.collision or s.visual_collision),
            })
            if done:
                break
        wall = time.perf_counter() - wall0
    finally:
        link.release()
        sim.stopSimulation()
        wait_until_stopped(sim)
        link.close()
    RESULTS.mkdir(parents=True, exist_ok=True)
    with (RESULTS / f"{label}.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    return rows, gt0, wall


# ---------------------------------------------------------------- plans
def plan_wheel_step(t, stack):
    if t < 1.0:
        return CRUISE_W, CRUISE_W, "step", False
    return 0.0, 0.0, "stop", t >= 2.0


class StraightThenTurn:
    V, W = 0.25, 0.6
    DIST, ANGLE = 1.000, math.radians(90.0)
    SETTLE = 1.0

    def __init__(self) -> None:
        self.phase, self.t_mark = "straight", None

    def __call__(self, t, stack):
        odo = stack.odo
        if self.phase == "straight":
            if odo.signed_distance_m >= self.DIST:
                self.phase, self.t_mark = "settle1", t
            else:
                wl, wr = rc.wheel_speeds_from_body(self.V, 0.0)
                return wl, wr, self.phase, False
        if self.phase == "settle1":
            if t - self.t_mark >= self.SETTLE:
                self.phase, self.th0 = "turn", odo.heading_unwrapped
            else:
                return 0.0, 0.0, self.phase, False
        if self.phase == "turn":
            if odo.heading_unwrapped - self.th0 >= self.ANGLE:
                self.phase, self.t_mark = "settle2", t
            else:
                wl, wr = rc.wheel_speeds_from_body(0.0, self.W)
                return wl, wr, self.phase, False
        return 0.0, 0.0, "settle2", t - self.t_mark >= self.SETTLE


# ---------------------------------------------------------------- report
def report_wheel_step(rows):
    step = [r for r in rows if r["phase"] == "step"]
    ws = [0.5 * (r["wl_enc"] + r["wr_enc"]) for r in step]
    peak = max(ws)
    tail = ws[-20:]
    ss = sum(tail) / len(tail)
    settle = next(
        (step[i]["t"] for i in range(len(ws)) if all(abs(x - CRUISE_W) <= 0.05 * CRUISE_W for x in ws[i:])),
        math.inf,
    )
    return {"peak": peak, "overshoot_pct": 100 * (peak / CRUISE_W - 1), "ss": ss, "settle5_s": settle}


def report_drive(rows):
    s1 = [r for r in rows if r["phase"] == "settle1"][-1]
    s2 = rows[-1]
    gt_dist = math.hypot(s1["gt_x"], s1["gt_y"])
    odo_dist = math.hypot(s1["odo_x"], s1["odo_y"])
    gt_turn = math.degrees(wrap_to_pi(s2["gt_dth"] - s1["gt_dth"]))
    odo_turn = math.degrees(s2["odo_th_unwrapped"] - s1["odo_th_unwrapped"])
    return {
        "gt_dist": gt_dist, "odo_dist": odo_dist, "dist_err_mm": 1000 * (odo_dist - gt_dist),
        "straight_gt_heading_change_deg": math.degrees(s1["gt_dth"]),
        "gt_turn_deg": gt_turn, "odo_turn_deg": odo_turn, "turn_err_deg": odo_turn - gt_turn,
        "collision": any(r["collision"] for r in rows),
    }


def main() -> None:
    sim = RemoteAPIClient().require("sim")
    original_rt = prepare_lockstep(sim)
    try:
        r1, _, w1 = run(sim, plan_wheel_step, "run1")
        r2, _, w2 = run(sim, StraightThenTurn(), "run2")
        r3, _, w3 = run(sim, StraightThenTurn(), "run3")
    finally:
        sim.setBoolProperty(sim.handle_scene, "realtimeSimulation", original_rt)

    ws = report_wheel_step(r1)
    d2, d3 = report_drive(r2), report_drive(r3)
    keys = ("odo_x", "odo_y", "gt_x", "gt_y", "gt_dth")
    det = max(abs(a[k] - b[k]) for a, b in zip(r2, r3) for k in keys) if len(r2) == len(r3) else math.inf

    print("\nStep 5 live check")
    print("-----------------")
    print(f"wheel step 0->{CRUISE_W:.2f} rad/s : peak {ws['peak']:.2f} (overshoot {ws['overshoot_pct']:+.1f} %), "
          f"5 % settle {ws['settle5_s']:.2f} s, steady {ws['ss']:.2f} rad/s")
    print(f"straight (stop @1 m): GT {d2['gt_dist']:.4f} m, ODOM {d2['odo_dist']:.4f} m, "
          f"error {d2['dist_err_mm']:+.2f} mm, GT heading change {d2['straight_gt_heading_change_deg']:+.3f} deg")
    print(f"turn +90 deg       : GT {d2['gt_turn_deg']:.3f} deg, ODOM {d2['odo_turn_deg']:.3f} deg, "
          f"error {d2['turn_err_deg']:+.3f} deg")
    print(f"collision          : {d2['collision']}")
    print(f"determinism run2/3 : max diff {det:.3e}, samples {len(r2)}/{len(r3)}")
    print(f"speed              : run2 {r2[-1]['t']:.1f} s sim in {w2:.1f} s wall")
    ok = (
        abs(ws["overshoot_pct"]) < 15 and ws["settle5_s"] < 1.0 and abs(ws["ss"] - CRUISE_W) < 0.3
        and abs(d2["dist_err_mm"]) < 5.0 and abs(d2["turn_err_deg"]) < 2.0
        and not d2["collision"] and det == 0.0
    )
    print(f"verdict            : {'PASS' if ok else 'CHECK (see values)'}")
    print(f"logs               : {RESULTS}")


if __name__ == "__main__":
    main()
