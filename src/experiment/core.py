"""
One run of the robot (step 13b-2), independent of where the measurements come from.

RobotRun.step(state) is called once per 10 ms tick with a state object that has
the fields of sim.protocol.SimState (the live bridge packet, or the offline
plant in experiment.drivers). It applies the noise profile, runs the drive
stack, the estimator of the configuration, the mission (explore -> START ->
goal) and the FSM, returns the two motor commands and records everything.

Order inside one tick (the same as all live checks since step 5):
    sensors (+ noise) -> encoders / odometry -> estimator -> FSM -> wheel PIs -> commands
GT (pose, true node) is used only for the record and the summary; everything else that needs
GT (body clearance, LiDAR vs true walls, trajectory metrics) is computed AFTER the run from the
record by evaluation.metrics (step 13b-3), so the live loop only does what the robot does.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from core import robot_config as rc
from core.configurations import Configuration
from core.encoder import IncrementalEncoder
from core.wheel_pi import WheelPI
from core.yaw_friction import yaw_friction_feedforward
from localization.estimators import TickInputs, make_estimator
from localization.odometry import DifferentialDriveOdometry, wrap_to_pi
from mapping.occupancy_grid import OccupancyGrid
from mazes.maze_geometry import WallMap
from mazes.topology import evaluate_memory
from navigation import navigation_config as cfg
from navigation.junction_types import ReferenceKind
from navigation.maze_decision import DfsExplorer
from navigation.maze_memory import MazeMemory
from navigation.mission import MissionProvider, optimal_goal_route
from navigation.motion_controller import MotionMode
from navigation.navigation_fsm import NavigationFSM, NavState
from perception.lidar_scan import RangeModel
from perception.proximity_types import ProximityReading, ProximitySnapshot
from sensors.noise_models import GyroStartCalibration, NoiseProfile

ROOT = Path(__file__).resolve().parents[2]
SENSORS = rc.PROXIMITY_SENSOR_ORDER
HOLD_S = 0.5                       # robot stands still before the start (gyro start calibration)
BODY_POINTS = ((0.335, 0.0), (0.33, 0.09), (0.33, -0.09), (0.15, 0.17), (0.15, -0.17),
               (-0.10, 0.17), (-0.10, -0.17), (0.0, 0.17), (0.0, -0.17))
LIDAR_MAX_M = 5.0
DEFAULT_MOUNT = {"x_forward_m": 0.1815, "y_left_m": 0.0014, "z_above_floor_m": 0.22, "yaw_offset_rad": 0.0}


def load_lidar_files():
    """(mount dict, RangeModel, calibration dict | None) from results/ (defaults if missing)."""
    mount, cal = dict(DEFAULT_MOUNT), None
    p = ROOT / "results" / "lidar_mount.json"
    if p.exists():
        mount.update(json.loads(p.read_text()))
    p = ROOT / "results" / "lidar_calibration.json"
    model = RangeModel()
    if p.exists():
        cal = json.loads(p.read_text())
        model = RangeModel(float(cal.get("a", 0.0)), float(cal.get("b", 0.0)))
    return mount, model, cal


def nearest_node(spec, x, y, th, ahead):
    """True junction node in front of the robot (GT, evaluation only)."""
    px, py = x + ahead * math.cos(th), y + ahead * math.sin(th)
    nodes = spec.nodes_m
    best = min(nodes, key=lambda n: math.hypot(nodes[n][0] - px, nodes[n][1] - py))
    if py < nodes[spec.root][1] - 0.6 and abs(px - nodes[spec.root][0]) < 0.5:
        return "__START__"
    return best


class DriveStack:
    def __init__(self) -> None:
        self.el, self.er = IncrementalEncoder(), IncrementalEncoder()
        self.odo = DifferentialDriveOdometry()
        self.pl, self.pr = WheelPI(), WheelPI()

    def start(self, ql, qr, pose) -> None:
        self.el.reset(ql)
        self.er.reset(qr)
        self.odo.reset(*pose)
        self.pl.reset()
        self.pr.reset()


class RobotRun:
    def __init__(self, spec, config: Configuration, noise: NoiseProfile, recorder, *, budget: int = 3000,
                 goal: bool = True, max_time_s: float = 6000.0, lidar_files=None, gating: bool = True,
                 disturbance=None) -> None:
        self.spec, self.config, self.noise, self.rec = spec, config, noise, recorder
        from experiment.disturbances import Disturbance
        self.dist = disturbance or Disturbance(None, noise.seed, len(spec.nodes_grid))     # step 19
        self._junctions_seen: set = set()
        self.walls = WallMap(spec)
        self.max_time_s = max_time_s
        self.mount, self.range_model, self.lidar_cal = lidar_files or load_lidar_files()
        self.ext = (0.0, 0.0, 0.0)
        if self.lidar_cal:
            self.ext = (self.lidar_cal.get("extrinsic_forward_m", 0.0), self.lidar_cal.get("extrinsic_left_m", 0.0),
                        self.lidar_cal.get("extrinsic_yaw_rad", 0.0))
        self.drive = DriveStack()
        self.estimator = make_estimator(config, self.drive.odo, gating=gating)
        self.memory = MazeMemory()
        self.grid = OccupancyGrid(*spec.start_pose[:2])     # metric map (step 14), LiDAR at the ESTIMATED pose
        pose_fn = self.estimator.pose
        if goal:
            self.mission = MissionProvider(self.memory, pose_fn, start_pose=spec.start_pose,
                                           goal_xy=spec.nodes_m[spec.goal], max_events=budget)
            self.dfs = self.mission.dfs
        else:
            self.mission = None
            self.dfs = DfsExplorer(self.memory, pose_fn, start_pose=spec.start_pose, max_events=budget)
        self.fsm = NavigationFSM(self.mission or self.dfs)
        self.gyro_cal = GyroStartCalibration()
        self.started = False
        self.done = False
        self.k = 0
        self.t = 0.0
        self.t_start = None
        self.goal_t0 = None
        self.goal_len_gt = 0.0
        self.path_len_gt = 0.0
        self._prev_gt = None
        self.collision = False
        self.gt_nodes: list[str] = []
        self.ev_err = []                  # (position error, heading error) at every junction event
        self.n_scans = 0
        self.last_step = None
        self.dyn_scenario: str | None = None      # step 14b
        self.dyn_obstacles: list = []
        self.dyn_log: list[dict] = []
        self._prev_nav_state = None
        self._last_gt = tuple(spec.start_pose)

    # ------------------------------------------------------------------ step 14b: dynamic environment
    def attach_dynamic(self, scenario: str, obstacles: list) -> None:
        self.dyn_scenario, self.dyn_obstacles = scenario, obstacles

    def planned_gt_route(self) -> list[str] | None:
        """The goal route the robot planned, as true maze nodes (each junction of the robot's map ->
        the true node it was observed at most often). Experiment control only (where to put obstacles)."""
        if self.mission is None or not self.mission.route:
            return None
        from collections import Counter
        seen: dict[int, Counter] = {}
        for e in self.rec.events:
            seen.setdefault(e["junction"], Counter())[e["gt_node"]] += 1
        out = []
        for jid, _ in self.mission.route:
            c = seen.get(jid)
            out.append(c.most_common(1)[0][0] if c else "?")
        return out

    def goal_running(self) -> bool:
        return self.mission is not None and self.mission.phase == "to_goal"

    def note_dynamic(self, name: str, t: float) -> None:
        x, y, _ = self._last_gt
        self.dyn_log.append({"t": round(t, 3), "event": "obstacle_triggered", "obstacle": name,
                             "gt_x": x, "gt_y": y})
        print(f"  dynamic t={t:7.1f}s {name} appears")

    def _log_state(self, st, t, gt, est) -> None:
        prev, now = self._prev_nav_state, st.state
        self._prev_nav_state = now
        if prev is None or prev == now or NavState.BLOCKED not in (prev, now):
            return
        ev = "obstacle_stop" if now is NavState.BLOCKED else (
            "obstacle_turn_back" if now is NavState.POSITION else
            "goal_blocked" if now is NavState.FINISHED else "obstacle_resume")
        self.dyn_log.append({"t": round(t, 3), "event": ev, "odo_s": self.drive.odo.signed_distance_m,
                             "gt_x": gt[0], "gt_y": gt[1], "gt_th": gt[2], "est_x": est[0], "est_y": est[1],
                             "est_th": est[2], "front_m": self._front_now})
        print(f"  dynamic t={t:7.1f}s {ev}")

    # ------------------------------------------------------------------ helpers
    def _noisy_proximity(self, snap):
        out = {}
        for n in SENSORS:
            r = getattr(snap, n)
            d = self.dist.proximity(n, self.noise.proximity.measure(r.distance_m if r.detected else None))
            out[n] = ProximityReading(n, r.detected and d is not None, d, r.detected_object_handle)
        return ProximitySnapshot(**out)

    def _scan_centre(self, x, y, th):
        f = self.mount["x_forward_m"] + self.ext[0]
        l = self.mount["y_left_m"] + self.ext[1]
        return (x + f * math.cos(th) - l * math.sin(th), y + f * math.sin(th) + l * math.cos(th),
                th + self.mount.get("yaw_offset_rad", 0.0) + self.ext[2])

    def _scan(self, xyzd, gt, est):
        n = len(xyzd) // 4
        c = self.mount.get("scan_centre_in_base") or (0.0, 0.0, 0.0)
        R = self.mount.get("scan_rotation_from_base") or ((1.0, 0.0, 0.0), (0.0, 1.0, 0.0), (0.0, 0.0, 1.0))
        angles, ideal = [], []
        for i in range(n):
            px, py, pz, d = xyzd[4 * i:4 * i + 4]
            q = (px - c[0], py - c[1], pz - c[2])
            xs = R[0][0] * q[0] + R[0][1] * q[1] + R[0][2] * q[2]
            ys = R[1][0] * q[0] + R[1][1] * q[1] + R[1][2] * q[2]
            angles.append(math.atan2(ys, xs))
            # range = horizontal distance from the scan centre (as in the step-12d calibration)
            ideal.append(None if d >= 0.9999 * LIDAR_MAX_M or d <= 0.0 else self.range_model.correct(math.hypot(xs, ys)))
        measured = self.dist.scan(self.noise.lidar.measure(ideal))          # step 19 D6
        self.rec.scan(self.t, self.k, angles, ideal, measured, gt, est)
        self.n_scans += 1
        return {"angles": angles, "ranges": measured}

    # ------------------------------------------------------------------ one tick
    def step(self, s):
        """-> (left command, right command, done)."""
        self.t = t = s.simulation_time_s
        gt = (s.gt_axle_x_m, s.gt_axle_y_m, s.gt_heading_rad)
        self._last_gt = gt
        ev = self.dist.maybe_trigger(t, len(self._junctions_seen),               # step 19: D5-D7 mid-explore
                                     self.last_step is not None and self.last_step.state is NavState.FOLLOW, self.noise)
        if ev:
            self.dyn_log.append({"t": round(t, 3), "event": "disturbance_start", "label": self.dist.spec.label,
                                 "gt_x": gt[0], "gt_y": gt[1]})
            print(f"  disturbance t={t:7.1f}s {ev}")
        ql, qr = self.dist.joints(s.left_joint_position_rad, s.right_joint_position_rad)   # step 19 D2 / D3
        gyro_ideal = s.imu_gyro_rad_s[2] if s.imu_gyro_rad_s else None
        gyro_meas = self.noise.gyro.measure(gyro_ideal, rc.SIMULATION_TIMESTEP_S) if gyro_ideal is not None else None
        if not self.started:
            if gyro_meas is not None:
                self.gyro_cal.add(gyro_meas)
            if t >= HOLD_S:
                self.drive.start(ql, qr, self.spec.start_pose)
                if hasattr(self.estimator, "start"):
                    from mazes.maze_spec import PITCH_M, START_OFFSET_FROM_ROOT_M
                    sx, sy, _ = self.spec.start_pose            # known START cell + cell size (micromouse rules)
                    self.estimator.start(*self.spec.start_pose, gyro_bias=self.gyro_cal.bias if self.gyro_cal.n else None,
                                         gyro_samples=self.gyro_cal.n,
                                         lattice=(sx, sy + START_OFFSET_FROM_ROOT_M, PITCH_M))
                if hasattr(self.estimator, "attach_lidar"):
                    self.estimator.attach_lidar(self.grid, (self.mount["x_forward_m"] + self.ext[0],
                                                            self.mount["y_left_m"] + self.ext[1],
                                                            self.mount.get("yaw_offset_rad", 0.0) + self.ext[2]))
                self.started, self.t_start = True, t
            return 0.0, 0.0, False
        self.k += 1
        prox = self._noisy_proximity(s.proximity)
        self._front_now = prox.front.distance_m if prox.front.detected else None   # the perception is frozen in TURN/EXIT
        sl = self.drive.el.update(ql, rc.SIMULATION_TIMESTEP_S)
        sr = self.drive.er.update(qr, rc.SIMULATION_TIMESTEP_S)
        inc = self.drive.odo.update(sl.delta_counts, sr.delta_counts)
        est_before = self.estimator.pose()
        scan = self._scan(s.lidar_xyzd, gt, est_before) if s.lidar_xyzd else None
        self.estimator.update(TickInputs(t, inc, gyro_meas, self.gyro_cal.bias if self.gyro_cal.n else None,
                                         prox, scan))
        if scan is not None:                     # map update AFTER the estimator (D / E matched the scan first)
            cx, cy, ch = self._scan_centre(*self.estimator.pose())
            P = self.estimator.covariance()
            self.grid.update(cx, cy, ch, scan["angles"], scan["ranges"], t=t,
                             pose_var=(0.5 * float(P[0, 0] + P[1, 1]), float(P[2, 2])) if P is not None else (0.0, 0.0))
        coll = bool(s.collision or s.visual_collision)
        st = self.fsm.update(prox, odo_distance_m=self.drive.odo.signed_distance_m, odo_dtheta_rad=inc.dtheta_rad,
                             left_counts=sl.delta_counts, right_counts=sr.delta_counts, safety_event=coll)
        self.last_step = st
        if st.observation is not None and hasattr(self.estimator, "on_junction"):
            self.estimator.on_junction(st.observation.kind)
        if st.reset_wheel_pi:
            self.drive.pl.reset()
            self.drive.pr.reset()
        wl, wr = rc.wheel_speeds_from_body(st.v_ref_m_s, st.omega_ref_rad_s)
        ff = yaw_friction_feedforward(st.omega_ref_rad_s) if st.motion_mode is MotionMode.STRAIGHT else 0.0
        cl = max(-1.0, min(1.0, self.drive.pl.update(wl, sl.speed_rad_s).command - ff))
        cr = max(-1.0, min(1.0, self.drive.pr.update(wr, sr.speed_rad_s).command + ff))

        # ---- GT bookkeeping (evaluation only)
        x, y, th = gt
        if self._prev_gt is not None:
            dl = math.hypot(x - self._prev_gt[0], y - self._prev_gt[1])
            self.path_len_gt += dl
            if self.mission is not None and self.mission.phase in ("to_goal", "done") and self.goal_t0 is not None:
                self.goal_len_gt += dl
        self._prev_gt = (x, y)
        if self.mission is not None and self.mission.phase == "to_goal" and self.goal_t0 is None:
            self.goal_t0 = t
        self.collision |= coll
        ex, ey, eth = self.estimator.pose()
        if st.observation is not None:
            self._event(st, t, gt, (ex, ey, eth))
        self._log_state(st, t, gt, (ex, ey, eth))
        g = st.guidance
        P = self.estimator.covariance()
        self.rec.tick({
            "k": self.k, "t": round(t, 3), "state": st.state.value, "nav_phase": st.phase.value if st.phase else "",
            "mission_phase": self.mission.phase if self.mission else "explore",
            "v_ref": st.v_ref_m_s, "w_ref": st.omega_ref_rad_s, "cmd_l": cl, "cmd_r": cr, "u_ff": ff,
            "enc_l": sl.delta_counts, "enc_r": sr.delta_counts, "odo_s": self.drive.odo.signed_distance_m,
            "est_x": ex, "est_y": ey, "est_th": eth,
            "P_xx": P[0, 0] if P is not None else None, "P_xy": P[0, 1] if P is not None else None,
            "P_yy": P[1, 1] if P is not None else None, "P_xt": P[0, 2] if P is not None else None,
            "P_yt": P[1, 2] if P is not None else None, "P_tt": P[2, 2] if P is not None else None,
            "est_bg": getattr(self.estimator, "gyro_bias", None),
            "ekf_rej": getattr(self.estimator, "rejected_total", 0),
            "gt_x": x, "gt_y": y, "gt_th": th,
            "gt_w": s.gt_yaw_rate_rad_s, "gyro_ideal": gyro_ideal, "gyro_meas": gyro_meas,
            "acc_x": s.imu_accel_m_s2[0] if s.imu_accel_m_s2 else None,
            "acc_y": s.imu_accel_m_s2[1] if s.imu_accel_m_s2 else None,
            "acc_z": s.imu_accel_m_s2[2] if s.imu_accel_m_s2 else None,
            **{f"d_{n}": (getattr(s.proximity, n).distance_m if getattr(s.proximity, n).detected else None)
               for n in SENSORS},
            **{f"dm_{n}": (getattr(prox, n).distance_m if getattr(prox, n).detected else None) for n in SENSORS},
            "g_mode": g.mode if g else "", "g_y_hat": g.y_hat_m if g else None,
            "g_alpha_hat": g.alpha_hat_rad if g else None,
            "collision": int(coll), "scan": self.n_scans - 1 if scan else -1,
        })
        if st.state in (NavState.FINISHED, NavState.SAFETY_ABORT) or t > self.max_time_s:
            self.done = True
        return cl, cr, self.done

    def _event(self, st, t, gt, est):
        o = st.observation
        ahead = cfg.D_SIDE_CANONICAL_M if o.kind is ReferenceKind.SIDE_CONFIRMED else cfg.D_DEAD_END_CANONICAL_M
        node = nearest_node(self.spec, *gt, ahead)
        self.gt_nodes.append(node)
        provider = self.mission or self.dfs
        self._junctions_seen.add(provider.steps[-1].arrival.junction)
        step_ = provider.steps[-1]
        arr = step_.arrival
        e_pos = math.hypot(est[0] - gt[0], est[1] - gt[1])
        e_th = wrap_to_pi(est[2] - gt[2])
        self.ev_err.append((e_pos, e_th))
        self.rec.event({
            "k": len(self.gt_nodes) - 1, "tick": self.k, "t": round(t, 3), "phase": getattr(step_, "phase", "explore"),
            "gt_node": node, "kind": o.kind.value, "left": o.left_available, "straight": o.straight_available,
            "right": o.right_available, "decision": st.decision.value, "reason": step_.reason,
            "junction": arr.junction, "association": arr.association, "match_distance": arr.match_distance_m,
            "inconsistent": arr.inconsistent, "est_x": est[0], "est_y": est[1], "est_th": est[2],
            "gt_x": gt[0], "gt_y": gt[1], "gt_th": gt[2], "err_pos_m": e_pos, "err_th_rad": e_th,
        })
        md = "" if arr.match_distance_m is None else f" d={arr.match_distance_m:.3f}"
        print(f"  event {len(self.gt_nodes) - 1:3d} t={t:7.1f}s {node:>10} -> J{arr.junction:<3} {arr.association:5}"
              f"{md:9} {step_.reason:20} {st.decision.value:8} pos err {1000 * e_pos:5.0f} mm")

    # ------------------------------------------------------------------ end of the run
    def summary(self) -> dict:
        spec, fsm = self.spec, self.fsm
        ev = evaluate_memory(self.memory, spec, self.gt_nodes)
        summ = self.memory.summary()
        explored = self.dfs.finished_reason == "complete"
        at_goal, goal_info = None, {}
        if self.mission is not None:
            last = self._last_gt
            x, y = last[0], last[1]
            gx, gy = spec.nodes_m[spec.goal]
            nodes = {nearest_node(spec, *last, cfg.D_SIDE_CANONICAL_M), nearest_node(spec, *last, cfg.D_DEAD_END_CANONICAL_M)}
            at_goal = self.mission.finished_reason == "goal_reached" and spec.goal in nodes
            opt_path, opt_len = optimal_goal_route(spec)
            goal_info = {
                "mission_reason": self.mission.finished_reason, "goal_node": spec.goal,
                "robot_true_nodes_at_end": sorted(nodes), "dist_to_goal_m": math.hypot(x - gx, y - gy),
                "goal_junction": self.mission.goal_junction, "goal_junction_dist_m": self.mission.goal_distance_m,
                "E2_goal_run_s": (self.t - self.goal_t0) if self.goal_t0 else None,
                "goal_run_length_gt_m": self.goal_len_gt if self.goal_t0 else None,
                "optimal_length_m": opt_len, "optimal_nodes": len(opt_path),
                "E3_route_ratio": (self.goal_len_gt / opt_len) if self.goal_t0 else None,
            }
        e_pos = [e[0] for e in self.ev_err]
        e_th = [abs(e[1]) for e in self.ev_err]
        ex, ey, eth = self.estimator.pose()
        gx_, gy_, gth_ = self._last_gt
        s2 = not self.collision
        out = {
            "maze": spec.maze_id, "configuration": self.config.name, "noise": self.noise.name, "seed": self.noise.seed,
            "final_state": fsm.state.value, "fault": fsm.fault, "stop_timeouts": fsm.stop_timeouts, "explore_reason": self.dfs.finished_reason,
            "simulated_s": self.t, "ticks": self.k, "events": len(self.gt_nodes),
            "S1_explored": explored, "S2_no_collision": s2,
            "safety_abort": fsm.state is NavState.SAFETY_ABORT,
            "S3_goal_reached": at_goal, "S4_mission_success": bool(explored and s2 and (at_goal is not False)),
            "T_map_correct": ev["correct"], "T_memory_junctions": ev["memory_junctions"],
            "T_true_nodes_visited": ev["true_nodes_visited"], "T_duplicates": ev["duplicates"],
            "T_false_merges": ev["false_merges"], "T_edges_correct": ev["edges_correct"],
            "T_edges_wrong": ev["edges_wrong"], "T_loop_closures": summ["loop_closures"],
            "T_max_match_distance_m": ev["max_match_distance_m"],
            "E1_explore_s": ((self.goal_t0 or self.t) - (self.t_start or 0.0)),
            "path_length_gt_m": self.path_len_gt,
            "L_event_pos_err_mean_m": sum(e_pos) / len(e_pos) if e_pos else None,
            "L_event_pos_err_max_m": max(e_pos) if e_pos else None,
            "L_event_heading_err_max_deg": math.degrees(max(e_th)) if e_th else None,
            "L_final_pos_err_m": math.hypot(ex - gx_, ey - gy_),
            "L_final_heading_err_deg": math.degrees(wrap_to_pi(eth - gth_)),
            "lidar_scans": self.n_scans,
            "estimator": self.estimator.name,
            "estimator_stats": self.estimator.stats() if hasattr(self.estimator, "stats") else None,
            "gyro_start_bias_estimate_deg_s": math.degrees(self.gyro_cal.bias) if self.gyro_cal.n else None,
            "gyro_start_bias_rad_s": self.gyro_cal.bias if self.gyro_cal.n else None,     # step 15b replay
            "gyro_start_samples": self.gyro_cal.n,
            "gyro_true_initial_bias_deg_s": math.degrees(self.noise.gyro.initial_bias),
            "dyn_scenario": self.dyn_scenario,
            "dyn_obstacles": [o.name for o in self.dyn_obstacles],
            "dyn_triggered": [o.name for o in self.dyn_obstacles if o.triggered_t is not None],
            "dyn_stops": fsm.obstacle_stops, "dyn_resumes": fsm.obstacle_resumes,
            "dyn_turnbacks": fsm.obstacle_turnbacks,
            "dyn_blocked": self.mission.blocked_log if self.mission is not None else [],
            **self.dist.summary(),
            "noise_scale": self.noise.scale,
            **goal_info,
        }
        return out

    def save_map(self) -> None:
        from mapping.graph_map import draw_graph, save_graph
        save_graph(self.memory, self.rec.dir / "map_topo.json")
        if self.grid.scans:
            self.grid.save(self.rec.dir / "map_grid.npz")
        if self.dyn_scenario:
            (self.rec.dir / "dynamic.json").write_text(json.dumps({
                "scenario": self.dyn_scenario, "obstacles": [o.to_dict() for o in self.dyn_obstacles],
                "log": self.dyn_log, "blocked": self.mission.blocked_log if self.mission else []}, indent=1))
        try:
            import matplotlib
            matplotlib.use("Agg")
            import matplotlib.pyplot as plt
        except ImportError:
            return
        ev = evaluate_memory(self.memory, self.spec, self.gt_nodes)
        labels = {0: "START"}
        for a, g in zip(self.memory.log, self.gt_nodes):
            labels.setdefault(a.junction, g.replace("__START__", "START"))
        dup = {j for j, g in labels.items()
               if g in ev["duplicates"] or (g == "START" and "__START__" in ev["duplicates"])}
        fig, ax = plt.subplots(figsize=(9, 8))
        draw_graph(self.memory, ax, walls=self.walls, gt_labels=labels, duplicates=dup,
                   title=f"{self.spec.maze_id}  configuration {self.config.name}, noise {self.noise.name}, "
                         f"seed {self.noise.seed}: {ev['memory_junctions']} junctions, "
                         f"{'map correct' if ev['correct'] else 'map INCORRECT'}")
        fig.tight_layout()
        fig.savefig(self.rec.dir / "map_topo.png", dpi=140)
        plt.close(fig)
