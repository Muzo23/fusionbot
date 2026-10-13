"""
FusionBot experiment runner (step 13b-2): one run = configuration x noise profile x maze x seed.

    python -m run --config A --noise ideal --maze T1                 (live, CoppeliaSim, scene = maze)
    python -m run --config A --noise realistic --maze M2 --seed 3
    python -m run --config A --noise realistic --maze T3 --seed 1 --offline     (offline plant)

The run: hold 0.5 s (gyro start calibration) -> DFS exploration -> back at START ->
shortest route in the robot's own map to the goal -> FINISH. Everything is
recorded in results/runs/<maze>_<config>_<noise>_s<seed>/ (see experiment.recorder).

Before a live run: python -m sim.maze_scene <maze>, robot at START, simulation stopped.
"""

from __future__ import annotations

import argparse
import json
import math

from core import robot_config as rc
from core.configurations import CONFIGS
from experiment.core import RobotRun, load_lidar_files
from experiment.recorder import RUNS_DIR, RunRecorder, run_name
from mazes.maze_spec import MAZES
from sensors.noise_models import PROFILES, NoiseProfile
from experiment.disturbances import Disturbance, noise_label
from experiment.disturbances import parse as parse_dist


def build(args):
    spec = MAZES[args.maze]
    config = CONFIGS[args.config]
    scale = noise_scale(args)
    noise = NoiseProfile(args.noise, args.seed, scale)
    dspec = parse_dist(args.dist)
    dist = Disturbance(dspec, args.seed, len(spec.nodes_grid))
    tag = "_".join(x for x in (args.dyn, dspec.label if dspec else None, noise_label(scale),
                               "nogate" if args.no_gate else None, args.tag) if x) or None
    folder = RUNS_DIR / run_name(args.maze, args.config, args.noise, args.seed, tag, args.offline)
    rec = RunRecorder(folder)
    lidar_files = load_lidar_files()
    run = RobotRun(spec, config, noise, rec, budget=args.budget, goal=not args.no_goal,
                   max_time_s=args.max_time, lidar_files=lidar_files,
                   gating=not args.no_gate, disturbance=dist)
    mount, model, cal = lidar_files
    rec.write_meta({
        "run": folder.name, "mode": "offline" if args.offline else "live",
        "maze": {"id": spec.maze_id, "name": spec.name, "root": spec.root, "goal": spec.goal,
                 "nodes": len(spec.nodes_grid), "edges": len(spec.edges), "cycles": spec.expected_cycle_count,
                 "start_pose": spec.start_pose},
        "configuration": {"name": config.name, "description": config.description, "estimator": config.estimator,
                          "pose_sensors": config.sensors(), "heading_source": config.heading_source,
                          "gyro_zupt": config.gyro_zupt, "prox_loc_update": config.prox_loc_update,
                          "lidar_update": config.lidar_update},
        "noise": noise.describe(),
        "estimator": {"gating": not args.no_gate},
        "mission": {"goal_run": not args.no_goal, "event_budget": args.budget, "max_time_s": args.max_time},
        "robot": {"wheel_radius_m": rc.WHEEL_RADIUS_M, "effective_wheel_separation_m": rc.EFFECTIVE_WHEEL_SEPARATION_M,
                  "geometric_wheel_separation_m": rc.WHEEL_SEPARATION_M, "dt_s": rc.SIMULATION_TIMESTEP_S},
        "lidar": {"mount": mount, "range_model": {"a": model.a, "b": model.b}, "extrinsic": list(run.ext),
                  "calibration_file": cal is not None},
        "dynamic": args.dyn,
        "disturbance": dspec.describe() if dspec else None,
        "offline": {"lidar": args.offline_lidar, "turn_loss_scale": args.turn_loss} if args.offline else None,
    })
    return spec, run, rec


def noise_scale(args) -> dict | None:
    """N1: --noise-scale k (every sensor); N2: --noise-scale-gyro / -lidar / -prox k (one sensor)."""
    k = {"gyro": args.noise_scale, "lidar": args.noise_scale, "proximity": args.noise_scale}
    for name, v in (("gyro", args.noise_scale_gyro), ("lidar", args.noise_scale_lidar), ("proximity", args.noise_scale_prox)):
        if v is not None:
            k[name] = v
    return None if all(v == 1.0 for v in k.values()) else k


def report(s: dict) -> None:
    print(f"\nRun {s['maze']} / configuration {s['configuration']} / noise {s['noise']} / seed {s['seed']}: "
          f"{s['final_state']} {s['fault'] or ''}  explore={s['explore_reason']}  mission={s.get('mission_reason', '-')}  "
          f"({s['simulated_s']:.0f} s simulated, {s['events']} events, wall {s.get('wall_time_s', 0):.0f} s, "
          f"real-time factor {s['simulated_s'] / max(s.get('wall_time_s') or 1e-9, 1e-9):.2f})")
    if s.get("gyro_start_bias_estimate_deg_s") is not None:
        print(f"gyro start cal.  : estimate {s['gyro_start_bias_estimate_deg_s']:+.3f} deg/s "
              f"(drawn bias {s['gyro_true_initial_bias_deg_s']:+.3f} deg/s)")


def merge_metrics(summary: dict, m: dict) -> dict:
    """The run summary takes the post-run GT metrics that the live loop no longer computes."""
    summary.update({
        "S2_no_collision": m["S2_no_collision"], "S4_mission_success": m["S4_mission_success"],
        "P_min_clearance_m": m["P7_clearance_min_mm"] / 1000.0,
        "lidar_qa_median_abs_mm": m["lidar_gt_median_mm"], "lidar_qa_p90_abs_mm": m["lidar_gt_p90_mm"],
        "L1_pos_rmse_m": m["L1_pos_rmse_m"], "K1_wall_rmse_mm": m["K1_wall_rmse_mm"],
    })
    return summary


def _write(path, d) -> None:
    path.write_text(json.dumps({k: (None if isinstance(v, float) and not math.isfinite(v) else v)
                                for k, v in d.items()}, indent=1))


def main(argv=None) -> dict:
    ap = argparse.ArgumentParser(description="FusionBot experiment runner")
    ap.add_argument("--config", default="A", choices=sorted(CONFIGS))
    ap.add_argument("--noise", default="ideal", choices=PROFILES)
    ap.add_argument("--maze", default="T1", choices=sorted(MAZES))
    ap.add_argument("--seed", type=int, default=0)
    ap.add_argument("--tag", default=None, help="extra name for the result folder")
    ap.add_argument("--budget", type=int, default=3000, help="junction-event budget of the exploration")
    ap.add_argument("--max-time", type=float, default=6000.0, help="simulated time limit [s]")
    ap.add_argument("--no-goal", action="store_true", help="explore only")
    ap.add_argument("--offline", action="store_true", help="offline plant instead of CoppeliaSim")
    ap.add_argument("--offline-lidar", action="store_true", help="offline: ray-cast LiDAR scans (slow)")
    ap.add_argument("--dyn", default=None, choices=("DYN1", "DYN2", "DYN3", "DYN4", "ALL"),
                    help="step 14b: dynamic obstacles in the goal run")
    ap.add_argument("--no-gate", action="store_true", help="step 16 ablation: EKF accepts every measurement")
    ap.add_argument("--turn-loss", type=float, default=1.0, help="offline: scale of the live turn loss (0 = none)")
    ap.add_argument("--dist", default=None, help="step 19 disturbance, e.g. D2=0.02, D5=0.5, D7=L (see experiment.disturbances)")
    ap.add_argument("--noise-scale", type=float, default=1.0, help="step 19 N1: every sensor's noise sigma x k")
    ap.add_argument("--noise-scale-gyro", type=float, default=None, help="step 19 N2: gyro noise x k")
    ap.add_argument("--noise-scale-lidar", type=float, default=None, help="step 19 N2: LiDAR noise x k")
    ap.add_argument("--noise-scale-prox", type=float, default=None, help="step 19 N2: proximity noise x k")
    args = ap.parse_args(argv)

    spec, run, rec = build(args)
    print(f"run {rec.dir.name}: {run.config.description}, estimator {run.estimator.name}, "
          f"noise {args.noise} (seed {args.seed}), maze {spec.maze_id} goal {spec.goal}")
    from experiment import drivers
    try:
        if args.offline:
            wall = drivers.offline_loop(run, spec, lidar=args.offline_lidar, turn_loss=args.turn_loss, dyn=args.dyn)
        else:
            wall = drivers.live_loop(run, spec, dyn=args.dyn)
    finally:
        summary = run.summary() if run.started else None
        if summary is not None:
            run.save_map()
        rec.close(summary)
    summary["wall_time_s"] = wall
    rec.update_meta(finished=True, wall_time_s=wall)
    _write(rec.dir / "summary.json", summary)
    report(summary)
    from evaluation.metrics import evaluate                  # step 13b-3: all GT metrics after the run
    m = evaluate(rec.dir)
    _write(rec.dir / "summary.json", merge_metrics(summary, m))
    print(f"recorded in      : {rec.dir}")
    return summary


if __name__ == "__main__":
    main()
