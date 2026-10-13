"""
Batch of offline runs (step 13b-6; later the Monte Carlo of step 20).

    python -m experiment.batch --config A --noise realistic --mazes T1 T2 T3 M1 M2 --seeds 1-5
    python -m experiment.batch --noise ideal realistic --mazes T1 T2 --seeds 1-3 --tag fsm_v2

Every run is an ordinary  python -m run ... --offline  (own process, so the runs are
independent and use all CPU cores); the summaries are collected into
results/batches/<tag>.csv and a short table is printed (S4 rate, failure reasons).
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import subprocess
import sys
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
SRC = ROOT / "src"
BATCH_DIR = ROOT / "results" / "batches"
COLUMNS = ("maze", "configuration", "noise", "seed", "S4_mission_success", "S1_explored", "S2_no_collision",
           "S3_goal_reached", "safety_abort", "fault", "explore_reason", "mission_reason", "T_map_correct",
           "T_duplicates", "T_false_merges", "L_event_pos_err_max_m", "L_final_pos_err_m", "P_min_clearance_m",
           "E1_explore_s", "E2_goal_run_s", "E3_route_ratio", "L1_pos_rmse_m", "L2_heading_rmse_deg",
           "K1_wall_rmse_mm", "K2_agreement_pct", "K2_coverage_pct", "P1_centreline_rms_mm", "P2_axis_angle_rms_deg", "P3_length_ratio",
           "P4_turn_err_rms_deg", "P5_turn_point_rms_mm", "P6_domega_rms_rad_s2", "P6_oscillations_per_m",
           "P7_clearance_min_mm", "P8_effort_per_m", "DYN_stops", "DYN_resumes", "DYN_turnbacks", "DYN_false_stops", "DYN_min_clearance_mm",
           "simulated_s", "wall_time_s", "run_dir")


def seeds_arg(text: str) -> list[int]:
    out = []
    for part in text.split(","):
        a, _, b = part.partition("-")
        out += list(range(int(a), int(b) + 1)) if b else [int(a)]
    return out


def one(job, tag, extra, dyn=None):
    maze, cfg, noise, seed = job
    parts = [dyn, "nogate" if "--no-gate" in extra else None, tag]
    name = f"{maze}_{cfg}_{noise}_s{seed}_offline_" + "_".join(p for p in parts if p)
    cmd = [sys.executable, "-m", "run", "--offline", "--config", cfg, "--noise", noise, "--maze", maze,
           "--seed", str(seed), "--tag", tag, *extra]
    t0 = time.perf_counter()
    p = subprocess.run(cmd, cwd=SRC, capture_output=True, text=True)
    summ = ROOT / "results" / "runs" / name / "summary.json"
    if p.returncode != 0 or not summ.exists():
        return {"maze": maze, "configuration": cfg, "noise": noise, "seed": seed, "S4_mission_success": False,
                "fault": f"RUN ERROR: {(p.stderr or p.stdout).strip().splitlines()[-1:]}",
                "wall_time_s": time.perf_counter() - t0, "run_dir": name}
    s = json.loads(summ.read_text())
    mf = summ.with_name("metrics.json")
    if mf.exists():
        s.update({k: v for k, v in json.loads(mf.read_text()).items() if k not in s or k.startswith(("P", "L", "K"))})
    s["run_dir"] = name
    return s


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--config", nargs="+", default=["A"])
    ap.add_argument("--noise", nargs="+", default=["realistic"])
    ap.add_argument("--mazes", nargs="+", default=["T1", "T2", "T3", "M1", "M2", "M3", "M4", "M5"])
    ap.add_argument("--seeds", default="1-3")
    ap.add_argument("--tag", default="batch")
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 2)
    ap.add_argument("--dyn", default=None, help="step 14b: dynamic scenario (DYN1..DYN4, ALL)")
    ap.add_argument("--extra", nargs=argparse.REMAINDER, default=[], help="passed to run (e.g. --turn-loss 2)")
    args = ap.parse_args(argv)
    jobs = [(m, c, n, s) for c in args.config for n in args.noise for m in args.mazes for s in seeds_arg(args.seeds)]
    print(f"{len(jobs)} offline runs, {args.workers} in parallel")
    rows, t0 = [], time.perf_counter()
    with ThreadPoolExecutor(args.workers) as ex:
        extra = args.extra + (["--dyn", args.dyn] if args.dyn else [])
        for r in ex.map(lambda j: one(j, args.tag, extra, args.dyn), jobs):
            rows.append(r)
            print(f"  {r['maze']:3} {r['configuration']} {r['noise']:9} s{r['seed']:<3} "
                  f"{'SUCCESS' if r.get('S4_mission_success') else 'FAILED '}  "
                  f"{r.get('fault') or r.get('mission_reason') or r.get('explore_reason') or ''}", flush=True)
    BATCH_DIR.mkdir(parents=True, exist_ok=True)
    stem = f"{args.dyn}_{args.tag}" if args.dyn else args.tag
    out = BATCH_DIR / f"{stem}_{''.join(args.config)}.csv"        # one file per configuration set (no overwrite)
    with out.open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=COLUMNS, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    print(f"\nS4 success rate per maze / noise ({time.perf_counter() - t0:.0f} s):")
    for c in args.config:
        for n in args.noise:
            line = []
            for m in args.mazes:
                sel = [r for r in rows if r["maze"] == m and r["noise"] == n and r["configuration"] == c]
                line.append(f"{m} {sum(bool(r.get('S4_mission_success')) for r in sel)}/{len(sel)}")
            print(f"  {c} {n:9}: " + "  ".join(line))
    print(f"saved {out}")
    return rows


if __name__ == "__main__":
    main()
