"""
Step 9 LIVE check: navigation FSM through T1 with the left-hand policy.

The full left-hand traversal of the T1 tree is 44 junction events: every
branch, back to ROOT, U-turn in the dead end behind START (about 8 min of
simulated time). Everything runs as in the final runtime: encoders -> odometry
-> FSM (perception, motion controller, corridor guidance) -> yaw-friction
feed-forward -> wheel PIs, wheel-PI reset on every primitive change, the
simulator contact flag as the external safety event.

GT only for the report:
  - every junction event is mapped to the nearest maze node (GT pose projected
    to the junction centre) and compared with the ground-truth traversal
    (node, left/straight/right availability, decision);
  - closed-junction FRONT_BLOCKED trigger vs the expectation from the FRONT
    reading at the reference (K2, margin 15 mm);
  - body-to-wall clearance from the GT pose and the maze geometry.

    python -m checks.step09_live_fsm             (from src/, scene T1, robot at START)
    python -m checks.step09_live_fsm --events 12 (shorter run)

Logs: results/step09/fsm_ticks.csv, results/step09/fsm_events.csv
"""

from __future__ import annotations

import argparse
import csv
import math
import time
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from checks.step05_live_drive import Stack
from checks.step07_live_turns import start_guard
from core import robot_config as rc
from core.yaw_friction import yaw_friction_feedforward
from mazes.maze_geometry import WallMap
from mazes.maze_spec import MAZES
from mazes.topology import expected_left_hand_events
from navigation import navigation_config as cfg
from navigation.junction_types import ReferenceKind
from navigation.motion_controller import MotionMode
from navigation.navigation_fsm import NavigationFSM, NavState
from navigation.simple_policies import LeftHandPolicy
from sim.sim_utils import prepare_lockstep, wait_until_stopped
from sim.transport import LockstepSession

RESULTS = Path(__file__).resolve().parents[2] / "results" / "step09"
BODY_POINTS = ((0.335, 0.0), (0.33, 0.09), (0.33, -0.09), (0.15, 0.17), (0.15, -0.17),
               (-0.10, 0.17), (-0.10, -0.17), (0.0, 0.17), (0.0, -0.17))


def nearest_node(spec, x, y, th, ahead):
    px, py = x + ahead * math.cos(th), y + ahead * math.sin(th)
    nodes = spec.nodes_m
    best = min(nodes, key=lambda n: math.hypot(nodes[n][0] - px, nodes[n][1] - py))
    if py < nodes[spec.root][1] - 0.6 and abs(px - nodes[spec.root][0]) < 0.5:
        return "__START__"
    return best


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--events", type=int, default=44)
    ap.add_argument("--maze", default="T1")
    args = ap.parse_args()
    spec = MAZES[args.maze]
    walls = WallMap(spec)
    expected = expected_left_hand_events(spec, args.events)

    sim = RemoteAPIClient().require("sim")
    original_rt = prepare_lockstep(sim)
    stack = Stack()
    policy = LeftHandPolicy(max_events=args.events)
    fsm = NavigationFSM(policy)
    ticks, events = [], []
    min_clear, wall0 = math.inf, time.perf_counter()
    link = LockstepSession()
    try:
        time.sleep(0.2)
        sim.startSimulation()
        while True:
            s = link.next_state(timeout_s=10.0)
            t = s.simulation_time_s
            if not stack.started:
                if t >= 0.5:
                    start_guard(s.proximity)
                    stack.start(s.left_joint_position_rad, s.right_joint_position_rad)
                link.send(s.state_sequence, 0.0, 0.0)
                continue
            sl = stack.el.update(s.left_joint_position_rad, rc.SIMULATION_TIMESTEP_S)
            sr = stack.er.update(s.right_joint_position_rad, rc.SIMULATION_TIMESTEP_S)
            inc = stack.odo.update(sl.delta_counts, sr.delta_counts)
            coll = bool(s.collision or s.visual_collision)
            st = fsm.update(s.proximity, odo_distance_m=stack.odo.signed_distance_m,
                            odo_dtheta_rad=inc.dtheta_rad, left_counts=sl.delta_counts,
                            right_counts=sr.delta_counts, safety_event=coll)
            if st.reset_wheel_pi:
                stack.pl.reset()
                stack.pr.reset()
            wl, wr = rc.wheel_speeds_from_body(st.v_ref_m_s, st.omega_ref_rad_s)
            ff = yaw_friction_feedforward(st.omega_ref_rad_s) if st.motion_mode is MotionMode.STRAIGHT else 0.0
            cl = max(-1.0, min(1.0, stack.pl.update(wl, sl.speed_rad_s).command - ff))
            cr = max(-1.0, min(1.0, stack.pr.update(wr, sr.speed_rad_s).command + ff))
            link.send(s.state_sequence, cl, cr)

            x, y, th = s.gt_axle_x_m, s.gt_axle_y_m, s.gt_heading_rad
            c, sn = math.cos(th), math.sin(th)
            for bx, by in BODY_POINTS:
                px, py = x + c * bx - sn * by, y + sn * bx + c * by
                if walls.is_free(px, py):
                    min_clear = min(min_clear, walls.distance_to_walls(px, py))
                else:
                    min_clear = min(min_clear, 0.0)
            g = st.guidance
            ticks.append({
                "t": t, "state": st.state.value, "phase": st.phase.value if st.phase else "",
                "v_ref": st.v_ref_m_s, "w_ref": st.omega_ref_rad_s, "u_ff": ff,
                "g_mode": g.mode if g else "", "g_y_hat": g.y_hat_m if g else None,
                "g_alpha_hat": g.alpha_hat_rad if g else None,
                "odo_s": stack.odo.signed_distance_m, "odo_th": stack.odo.heading_unwrapped,
                "gt_x": x, "gt_y": y, "gt_th": th, "collision": int(coll),
                **{f"d_{n}": (getattr(s.proximity, n).distance_m if getattr(s.proximity, n).detected else None)
                   for n in ("left", "front_left", "front", "front_right", "right")},
            })
            if st.observation is not None:
                o = st.observation
                ahead = cfg.D_SIDE_CANONICAL_M if o.kind is ReferenceKind.SIDE_CONFIRMED else cfg.D_DEAD_END_CANONICAL_M
                node = nearest_node(spec, x, y, th, ahead)
                k = len(events)
                got = (node, o.left_available, o.straight_available, o.right_available, st.decision.value)
                exp = tuple(expected[k]) if k < len(expected) else (node, o.left_available, o.straight_available,
                                                                     o.right_available, "FINISH")
                events.append({"k": k, "t": t, "node": node, "kind": o.kind.value,
                               "left": o.left_available, "straight": o.straight_available,
                               "right": o.right_available, "decision": st.decision.value,
                               "front_at_ref": o.front_distance_m, "match": got == exp,
                               "expected": exp, "closed_trigger": None, "closed_expected": None})
                print(f"  event {k:2d} t={t:6.1f}s {node:>9} L{int(o.left_available)} S{int(o.straight_available)} "
                      f"R{int(o.right_available)} -> {st.decision.value:8} {'ok' if got == exp else 'MISMATCH ' + str(exp)}")
            if st.closed_front_trigger_m is not None and events:
                events[-1]["closed_trigger"] = st.closed_front_trigger_m
                events[-1]["closed_expected"] = st.closed_front_expected_m
            if st.state in (NavState.FINISHED, NavState.SAFETY_ABORT) or t > 1500.0:
                break
    finally:
        link.release()
        sim.stopSimulation()
        wait_until_stopped(sim)
        link.close()
        sim.setBoolProperty(sim.handle_scene, "realtimeSimulation", original_rt)

    RESULTS.mkdir(parents=True, exist_ok=True)
    for name, rows in (("fsm_ticks.csv", ticks), ("fsm_events.csv", events)):
        if rows:
            with (RESULTS / name).open("w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)

    res = [e["closed_trigger"] - e["closed_expected"] for e in events if e["closed_trigger"] is not None]
    nav = [e for e in events if e["decision"] != "FINISH"]
    n_ok = sum(e["match"] for e in nav)
    coll = any(r["collision"] for r in ticks)
    print(f"\nStep 9 live check ({args.maze}, left-hand policy, {args.events} events)")
    print("------------------------------------------------------------")
    print(f"final state      : {fsm.state.value}  {fsm.fault or ''}")
    print(f"events matched   : {n_ok} / {len(nav)} (expected {args.events}; then FINISH)")
    if res:
        print(f"closed junctions : {len(res)}, trigger - expected: {1000 * min(res):+.1f} .. "
              f"{1000 * max(res):+.1f} mm (guard margin {1000 * cfg.CLOSED_GUARD_MARGIN_M:.0f} mm)")
    print(f"min body clearance (GT): {1000 * min_clear:.1f} mm")
    print(f"collision        : {coll}")
    print(f"simulated / wall time: {ticks[-1]['t']:.0f} s / {time.perf_counter() - wall0:.0f} s")
    ok = (fsm.state is NavState.FINISHED and n_ok == len(nav) == args.events and not coll
          and (not res or max(abs(r) for r in res) < 0.010) and min_clear > 0.005)
    print(f"verdict          : {'PASS' if ok else 'CHECK (see values)'}")
    print(f"logs             : {RESULTS}")


if __name__ == "__main__":
    main()
