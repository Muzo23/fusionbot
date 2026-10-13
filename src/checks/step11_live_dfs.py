"""
Step 11 LIVE check: autonomous DFS exploration (configuration A), then (step 13d) the goal run:
after the exploration the robot is back at START and drives its shortest known path to the GOAL.

The robot explores the maze on its own: FSM (step 9) + DFS (step 11) over
MazeMemory (step 10), every junction placed with the ODOMETRY pose estimate
(odometry initialised at the known START pose). When everything is explored it
returns to START and stops (FINISH). Expected for the T1 tree: 43 junction
events = the left-hand traversal, then FINISH in the dead end behind START.

GT only for the report: true node of every event, evaluation of the memory
against the maze graph (duplicates / false merges / edges), clearance.
At the end the topological map is saved (JSON) and drawn over the true walls.

    python -m checks.step11_live_dfs              (from src/, scene T1, robot at START)
    python -m sim.maze_scene T3  then  python -m checks.step11_live_dfs --maze T3

Logs: results/step11/dfs_ticks.csv, dfs_events.csv, graph_<maze>_A.json/.png
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
from mazes.topology import evaluate_memory, expected_left_hand_events
from mapping.graph_map import draw_graph, save_graph
from navigation.maze_decision import DfsExplorer
from navigation.mission import MissionProvider, optimal_goal_route
from navigation.maze_memory import MazeMemory
from navigation import navigation_config as cfg
from navigation.junction_types import ReferenceKind
from navigation.motion_controller import MotionMode
from navigation.navigation_fsm import NavigationFSM, NavState
from sim.maze_scene import scene_maze_id
from sim.sim_utils import prepare_lockstep, wait_until_stopped
from sim.transport import LockstepSession

RESULTS = Path(__file__).resolve().parents[2] / "results" / "step11"
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
    ap.add_argument("--budget", type=int, default=200)
    ap.add_argument("--maze", default="T1")
    ap.add_argument("--no-goal", action="store_true", help="explore only (no goal run after the exploration)")
    args = ap.parse_args()
    spec = MAZES[args.maze]
    walls = WallMap(spec)
    expected = expected_left_hand_events(spec, 43) if args.maze == "T1" else []

    sim = RemoteAPIClient().require("sim")
    scene_id = scene_maze_id(sim)
    if scene_id is not None and scene_id != args.maze:
        raise SystemExit(f"the scene contains maze {scene_id}, not {args.maze}: "
                         f"run  python -m sim.maze_scene {args.maze}  or pass --maze {scene_id}")
    if scene_id is None:
        print(f"(scene has no maze id - assuming it is {args.maze}, e.g. built by the old generator)")
    original_rt = prepare_lockstep(sim)
    stack = Stack()
    memory = MazeMemory()
    odo = stack.odo
    dfs = DfsExplorer(memory, lambda: (odo.pose.x_m, odo.pose.y_m, odo.pose.theta_rad),
                      start_pose=spec.start_pose, max_events=args.budget)
    mission = None
    if not args.no_goal:
        mission = MissionProvider(memory, lambda: (odo.pose.x_m, odo.pose.y_m, odo.pose.theta_rad),
                                  start_pose=spec.start_pose, goal_xy=spec.nodes_m[spec.goal], max_events=args.budget)
        dfs = mission.dfs
    provider = mission or dfs
    fsm = NavigationFSM(provider)
    goal_t0, goal_gt_len, prev_gt = None, 0.0, None
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
                    stack.odo.reset(*spec.start_pose)          # odometry starts at the known START pose
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
            if mission is not None and mission.phase == "to_goal":
                if goal_t0 is None:
                    goal_t0 = t
                if prev_gt is not None:
                    goal_gt_len += math.hypot(x - prev_gt[0], y - prev_gt[1])
                prev_gt = (x, y)
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
                "phase": mission.phase if mission else "explore",
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
                exp = tuple(expected[k]) if k < len(expected) else ("__START__", False, False, False, "FINISH")
                step_ = provider.steps[-1]
                arr = step_.arrival
                ph = getattr(step_, "phase", "explore")
                ok_ev = (got == exp) if ph == "explore" else (k >= len(expected) or got[:4] == exp[:4])
                events.append({"k": k, "t": t, "node": node, "kind": o.kind.value,
                               "left": o.left_available, "straight": o.straight_available,
                               "right": o.right_available, "decision": st.decision.value,
                               "front_at_ref": o.front_distance_m, "match": ok_ev,
                               "junction": arr.junction, "association": arr.association,
                               "match_distance": arr.match_distance_m, "inconsistent": arr.inconsistent,
                               "dfs_reason": step_.reason, "phase": getattr(step_, "phase", "explore"),
                               "odo_x": odo.pose.x_m, "odo_y": odo.pose.y_m, "odo_th": odo.pose.theta_rad,
                               "expected": exp, "closed_trigger": None, "closed_expected": None})
                md = "" if arr.match_distance_m is None else f" d={arr.match_distance_m:.3f}"
                print(f"  event {k:2d} t={t:6.1f}s {node:>9} -> J{arr.junction:<2} {arr.association:5}{md:8} "
                      f"{step_.reason:20} {st.decision.value:8} "
                      f"{'ok' if ok_ev else ('MISMATCH ' + str(exp) if expected else '')}")
            if st.closed_front_trigger_m is not None and events:
                events[-1]["closed_trigger"] = st.closed_front_trigger_m
                events[-1]["closed_expected"] = st.closed_front_expected_m
            if st.state in (NavState.FINISHED, NavState.SAFETY_ABORT) or t > 6000.0:
                break
    finally:
        link.release()
        sim.stopSimulation()
        wait_until_stopped(sim)
        link.close()
        sim.setBoolProperty(sim.handle_scene, "realtimeSimulation", original_rt)

    RESULTS.mkdir(parents=True, exist_ok=True)
    for name, rows in ((f"dfs_{args.maze}_ticks.csv", ticks), (f"dfs_{args.maze}_events.csv", events)):
        if rows:
            with (RESULTS / name).open("w", newline="") as f:
                w = csv.DictWriter(f, fieldnames=list(rows[0]))
                w.writeheader()
                w.writerows(rows)

    gt_nodes = [e["node"] for e in events]
    ev_report = evaluate_memory(memory, spec, gt_nodes)
    save_graph(memory, RESULTS / f"graph_{args.maze}_A.json")
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        labels = {0: "START"}
        for a, g in zip(memory.log, gt_nodes):
            labels.setdefault(a.junction, g.replace("__START__", "START"))
        dup_ids = {j for j, g in labels.items()
                   if g in ev_report["duplicates"] or (g == "START" and "__START__" in ev_report["duplicates"])}
        fig, ax = plt.subplots(figsize=(8, 7.5))
        draw_graph(memory, ax, walls=walls, gt_labels=labels, duplicates=dup_ids,
                   title=f"{args.maze} live DFS, configuration A (odometry): {ev_report['memory_junctions']} junctions, "
                         f"duplicates {ev_report['duplicates'] or 'none'}")
        fig.tight_layout()
        fig.savefig(RESULTS / f"graph_{args.maze}_A.png", dpi=150)
    except ImportError:
        print("(matplotlib not installed - figure skipped)")

    res = [e["closed_trigger"] - e["closed_expected"] for e in events if e["closed_trigger"] is not None]
    nav = [e for e in events if e["decision"] != "FINISH"]
    n_ok = sum(e["match"] for e in events)
    coll = any(r["collision"] for r in ticks)
    summ = memory.summary()
    print(f"\nStep 11 live check ({args.maze}, DFS exploration, configuration A)")
    print("--------------------------------------------------------------")
    print(f"final state      : {fsm.state.value}  {fsm.fault or ''}  reason={dfs.finished_reason}  "
          f"at junction J{dfs.steps[-1].arrival.junction if dfs.steps else '-'}")
    if expected:
        print(f"events vs T1 traversal: {n_ok} / {len(events)} ok (expected 43 + FINISH)")
    print(f"memory           : {ev_report['memory_junctions']} junctions, edges ok/wrong "
          f"{ev_report['edges_correct']}/{ev_report['edges_wrong']}, duplicates {ev_report['duplicates']}, "
          f"false merges {ev_report['false_merges']}, loop closures {summ['loop_closures']}")
    print(f"associations     : {summ['edge_associations']} by known edge ({summ['inconsistent_edge_associations']} "
          f"inconsistent: distance > radius), max distance {ev_report['max_match_distance_m']:.3f} m")
    if res:
        print(f"closed junctions : {len(res)}, trigger - expected: {1000 * min(res):+.1f} .. {1000 * max(res):+.1f} mm")
    print(f"min body clearance (GT): {1000 * min_clear:.1f} mm")
    print(f"collision        : {coll}")
    print(f"simulated / wall time: {ticks[-1]['t']:.0f} s / {time.perf_counter() - wall0:.0f} s")
    explored = dfs.finished_reason == "complete" and (not expected or n_ok == len(events))
    nav_ok = fsm.state is NavState.FINISHED and explored and not coll
    if mission is None:
        nav_ok = nav_ok and dfs.steps[-1].arrival.junction == 0
    else:
        last = ticks[-1]
        gnode = nearest_node(spec, last["gt_x"], last["gt_y"], last["gt_th"], cfg.D_SIDE_CANONICAL_M)
        gnode2 = nearest_node(spec, last["gt_x"], last["gt_y"], last["gt_th"], cfg.D_DEAD_END_CANONICAL_M)
        at_goal = spec.goal in (gnode, gnode2)
        opt_path, opt_len = optimal_goal_route(spec)
        print(f"goal run         : reason {mission.finished_reason}, robot at true node {gnode} "
              f"(goal {spec.goal}: {'REACHED' if at_goal else 'NOT reached'}), "
              f"{(ticks[-1]['t'] - goal_t0) if goal_t0 else 0:.0f} s, path {goal_gt_len:.2f} m "
              f"(optimal {opt_len:.2f} m, {len(opt_path)} nodes), goal junction J{mission.goal_junction} "
              f"{mission.goal_distance_m if mission.goal_distance_m is not None else float('nan'):.3f} m from the goal position")
        nav_ok = nav_ok and mission.finished_reason == "goal_reached" and at_goal
    print(f"navigation       : {'PASS' if nav_ok else 'CHECK'}  (explored, back at START, "
          f"{'goal reached, ' if mission else ''}no collision)")
    print(f"topological map  : {'CORRECT' if ev_report['correct'] else 'INCORRECT'}  "
          f"(a measured result of the configuration, not a pass criterion: in T3 configuration A is "
          f"expected to miss the ring closure)")
    print(f"logs + map       : {RESULTS}")


if __name__ == "__main__":
    main()
