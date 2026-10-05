"""
Step 7 LIVE check: MotionController primitives on the real robot model.

Scenario (one simulation run, scene T1, robot at START, simulation STOPPED):
    0. START guard   left and right sensors must see walls and the front must be
                     free, otherwise the robot is not at START -> abort
    1. STRAIGHT      0.25 m/s north until ODOMETRY distance = 1.422634 m
                     (START -> centre of the ROOT junction), settle
    2. TURN +90      CCW, closed loop on odometry, settle
    3. TURN -90      CW  (back to north), settle
    4. TURN +180     CCW, settle
    5. TURN -180     CW  (must turn back the SAME way, not continue), settle

Settle = wheel PIs reset, zero reference until both encoders report 0 counts
for 20 consecutive ticks (min 0.3 s, max 2.0 s). The turn result is measured
AFTER the settle, so it includes the coasting after `completed`.

GT is read ONLY for the report; the GT heading is unwrapped tick by tick so a
180 deg turn is not ambiguous.

    python -m checks.step07_live_turns      (from src/)

CSV log: results/step07/turns.csv
"""

from __future__ import annotations

import csv
import math
import time
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from checks.step05_live_drive import Stack
from core import robot_config as rc
from core.encoder import wrap_to_pi
from navigation.motion_controller import MotionController, MotionMode
from sim.sim_utils import prepare_lockstep, wait_until_stopped
from sim.transport import LockstepSession

RESULTS = Path(__file__).resolve().parents[2] / "results" / "step07"
START_TO_ROOT_M = 1.422634
V = 0.25
TURNS_DEG = (90.0, -90.0, 180.0, -180.0)
SETTLE_STILL_TICKS, SETTLE_MIN_S, SETTLE_MAX_S = 30, 0.5, 3.0
START_SIDE_MAX_M, START_FRONT_MIN_M = 0.40, 0.50

# acceptance of the CONTROLLER (odometry-relative; the controller cannot do better
# than its sensor). Odometry-vs-GT is a characterisation of odometry, not of the
# controller: on the real model the wheels slip during in-place turns (live run 1:
# GT/odo = 0.989-0.996, CW worse than CCW, 8-14 mm axle drift per turn).
TOL_ODO_VS_TARGET_DEG = 0.6
TOL_TURN_TIME_S = 5.0


class StartGuardError(RuntimeError):
    pass


def start_guard(p) -> None:
    def side_ok(r):
        return r.detected and r.distance_m is not None and r.distance_m < START_SIDE_MAX_M
    front_free = (not p.front.detected) or (p.front.distance_m or 0.0) > START_FRONT_MIN_M
    if not (side_ok(p.left) and side_ok(p.right) and front_free):
        raise StartGuardError(
            "robot does not look like it is at START "
            f"(left={p.left.distance_m}, right={p.right.distance_m}, front={p.front.distance_m}). "
            "Reload the scene / put the robot back at START and rerun."
        )


class Mission:
    """Sequencer: straight -> settle -> (turn -> settle) x 4 -> done."""

    def __init__(self, turns=TURNS_DEG, alpha=None, primitives=None, speed=V) -> None:
        """primitives: list of ("straight", distance_m) / ("turn", deg); default is
        START->ROOT followed by `turns`."""
        self.mc = MotionController(alpha_max_rad_s2=alpha)
        self.speed = speed
        if primitives is None:
            primitives = [("straight", START_TO_ROOT_M)] + [("turn", d) for d in turns]
        self.queue = list(primitives)
        self.phase, self.label = "idle", ""
        self.t_mark = 0.0
        self.still = 0
        self.results = []         # one dict per primitive
        self.cur = None

    def next_primitive(self, t, stack, gt):
        if not self.queue:
            self.phase = "done"
            return
        kind, deg = self.queue.pop(0)
        self.cur = {"kind": kind, "target_deg": deg if kind == "turn" else None,
                    "distance_m": deg if kind == "straight" else None, "t0": t,
                    "odo_th0": stack.odo.heading_unwrapped, "odo_s0": stack.odo.signed_distance_m,
                    "gt_th0": gt["th_unwrapped"], "gt_x0": gt["x"], "gt_y0": gt["y"]}
        stack.pl.reset()          # every primitive starts from a clean PI state
        stack.pr.reset()
        if kind == "straight":
            self.mc.start_straight(self.speed)
            self.label = "straight"
        else:
            self.mc.start_turn_degrees(deg)
            self.label = f"turn{deg:+.0f}"
        self.phase = "move"

    def step(self, t, stack, inc, gt, sl, sr):
        """Returns (v_ref, w_ref)."""
        if self.phase == "idle":
            self.next_primitive(t, stack, gt)
        if self.phase == "move":
            if self.mc.mode is MotionMode.STRAIGHT and \
                    stack.odo.signed_distance_m - self.cur["odo_s0"] >= self.cur["distance_m"]:
                self.mc.stop()
                self._to_settle(t, stack)
            else:
                ref = self.mc.update(inc.dtheta_rad)
                if ref.completed:
                    self.cur["completed_t"] = t - self.cur["t0"]
                    self.cur["final_err_deg"] = math.degrees(ref.turn_error_rad)
                    self._to_settle(t, stack)
                else:
                    return ref.linear_velocity_m_s, ref.angular_velocity_rad_s
        if self.phase == "settle":
            self.still = self.still + 1 if (sl.delta_counts == 0 and sr.delta_counts == 0) else 0
            el = t - self.t_mark
            if (el >= SETTLE_MIN_S and self.still >= SETTLE_STILL_TICKS) or el >= SETTLE_MAX_S:
                c = self.cur
                c["settle_s"] = el
                c["odo_dth_deg"] = math.degrees(stack.odo.heading_unwrapped - c["odo_th0"])
                c["odo_ds_m"] = stack.odo.signed_distance_m - c["odo_s0"]
                c["gt_dth_deg"] = math.degrees(gt["th_unwrapped"] - c["gt_th0"])
                c["gt_dxy_m"] = math.hypot(gt["x"] - c["gt_x0"], gt["y"] - c["gt_y0"])
                self.results.append(c)
                self.next_primitive(t, stack, gt)    # first reference on the next tick
        return 0.0, 0.0

    def _to_settle(self, t, stack):
        stack.pl.reset()
        stack.pr.reset()
        self.phase, self.t_mark, self.still = "settle", t, 0


def run_mission(turns, csv_name, alpha=None, primitives=None, speed=V, hook=None):
    """One simulation run: START -> ROOT -> turns. Returns (results, rows).

    hook(mission, state, inc, v, w) -> (w_override | None, extra_columns: dict)
    is called every tick after the mission (used by step 8 for the guidance)."""
    sim = RemoteAPIClient().require("sim")
    original_rt = prepare_lockstep(sim)
    stack, mission = Stack(), Mission(turns, alpha, primitives, speed)
    rows, gt = [], None
    link = LockstepSession()
    try:
        time.sleep(0.2)
        sim.startSimulation()
        while mission.phase != "done":
            s = link.next_state(timeout_s=10.0)
            t = s.simulation_time_s
            if gt is None:
                gt = {"x": s.gt_axle_x_m, "y": s.gt_axle_y_m,
                      "th": s.gt_heading_rad, "th_unwrapped": s.gt_heading_rad}
            else:
                gt["th_unwrapped"] += wrap_to_pi(s.gt_heading_rad - gt["th"])
                gt["x"], gt["y"], gt["th"] = s.gt_axle_x_m, s.gt_axle_y_m, s.gt_heading_rad
            if not stack.started:
                if t >= 0.5:
                    start_guard(s.proximity)
                    stack.start(s.left_joint_position_rad, s.right_joint_position_rad)
                link.send(s.state_sequence, 0.0, 0.0)
                continue
            # encoders/odometry first, then the reference for the NEXT tick
            sl = stack.el.update(s.left_joint_position_rad, rc.SIMULATION_TIMESTEP_S)
            sr = stack.er.update(s.right_joint_position_rad, rc.SIMULATION_TIMESTEP_S)
            inc = stack.odo.update(sl.delta_counts, sr.delta_counts)
            v, w = mission.step(t, stack, inc, gt, sl, sr)
            extra = {}
            if hook is not None:
                w_over, extra = hook(mission, s, inc, v, w)
                if w_over is not None:
                    w = w_over
            wl, wr = rc.wheel_speeds_from_body(v, w)
            cl = stack.pl.update(wl, sl.speed_rad_s).command
            cr = stack.pr.update(wr, sr.speed_rad_s).command
            link.send(s.state_sequence, cl, cr)
            rows.append({
                "t": t, "phase": f"{mission.phase}:{mission.label}", "v_ref": v, "w_ref": w,
                "odo_th_unwrapped": stack.odo.heading_unwrapped, "odo_s": stack.odo.signed_distance_m,
                "gt_x": gt["x"], "gt_y": gt["y"], "gt_th_unwrapped": gt["th_unwrapped"],
                "collision": int(s.collision or s.visual_collision),
                **{f"d_{n}": (getattr(s.proximity, n).distance_m
                              if getattr(s.proximity, n).detected else None)
                   for n in ("left", "front_left", "front", "front_right", "right")},
                **extra,
            })
            if t > 180.0:
                raise RuntimeError("mission timeout (180 s simulated)")
    finally:
        link.release()
        sim.stopSimulation()
        wait_until_stopped(sim)
        link.close()
        sim.setBoolProperty(sim.handle_scene, "realtimeSimulation", original_rt)

    RESULTS.mkdir(parents=True, exist_ok=True)
    with (RESULTS / csv_name).open("w", newline="") as f:
        wcsv = csv.DictWriter(f, fieldnames=list(rows[0]))
        wcsv.writeheader()
        wcsv.writerows(rows)
    return mission.results, rows


def turn_table(results):
    """Prints one line per turn; returns list of (target, odo, gt, t_done, transl_mm)."""
    print(f"{'turn':>8} {'odo':>9} {'GT':>9} {'odo-tgt':>8} {'odo-GT':>8} {'GT/odo':>7} {'t_done':>7} {'transl':>7}")
    out = []
    for c in results:
        if c["kind"] != "turn":
            continue
        d_tgt = c["odo_dth_deg"] - c["target_deg"]
        d_gt = c["odo_dth_deg"] - c["gt_dth_deg"]
        done_t = c.get("completed_t", math.inf)
        transl = 1000 * c["gt_dxy_m"]
        print(f"{c['target_deg']:+8.0f} {c['odo_dth_deg']:+9.3f} {c['gt_dth_deg']:+9.3f} "
              f"{d_tgt:+8.3f} {d_gt:+8.3f} {c['gt_dth_deg'] / c['odo_dth_deg']:7.4f} "
              f"{done_t:6.2f}s {transl:5.1f}mm")
        out.append((c["target_deg"], c["odo_dth_deg"], c["gt_dth_deg"], done_t, transl))
    return out


def main() -> None:
    results, rows = run_mission(TURNS_DEG, "turns.csv")
    print("\nStep 7 live check (MotionController at the T1 ROOT junction)")
    print("-------------------------------------------------------------")
    st = results[0]
    print(f"straight : odo {st['odo_ds_m']:.4f} m, GT {st['gt_dxy_m']:.4f} m "
          f"(err {1000 * (st['odo_ds_m'] - st['gt_dxy_m']):+.1f} mm), "
          f"GT heading change {st['gt_dth_deg']:+.3f} deg")
    tab = turn_table(results)
    coll = any(r["collision"] for r in rows)
    ctrl_ok = all(
        abs(odo - tgt) <= TOL_ODO_VS_TARGET_DEG and t <= TOL_TURN_TIME_S
        and math.copysign(1, gt) == math.copysign(1, tgt)
        for tgt, odo, gt, t, _ in tab
    ) and len(tab) == len(TURNS_DEG)
    print(f"net GT heading after the turns: {sum(r[2] for r in tab):+.3f} deg (ideal 0)")
    print(f"collision: {coll}")
    print(f"controller verdict : {'PASS' if ctrl_ok and not coll else 'FAIL'}"
          f"  (|odo-target| <= {TOL_ODO_VS_TARGET_DEG} deg, t <= {TOL_TURN_TIME_S} s, sign, no collision)")
    print("odometry vs GT     : characterisation only -> run checks.step07b_turn_calibration")
    print(f"log      : {RESULTS / 'turns.csv'}")


if __name__ == "__main__":
    main()
