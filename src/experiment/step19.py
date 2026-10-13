"""
Step 19: the disturbance and noise-level experiment (D1-D7, N1, N2) - plan, runner and report.

    python -m experiment.step19 --dry-run                       how many runs, which ones
    python -m experiment.step19 --families D2 D5 --seeds 1      a part only
    python -m experiment.step19                                 everything (resumable: finished runs are skipped)
    python -m experiment.step19 --report                        tables + R2 failure limits

Plan (all offline, profile realistic, mazes T3 and M4, seeds 1-3; N2 on T3 only). Only the configurations
a disturbance can affect are run (D5 changes the gyro: only C and E use it; D6 the LiDAR: only D and E);
for the others the run would be identical to the nominal one. The LiDAR is ray-cast only for D and E
(the configurations that use it), which keeps A-C runs fast (their K metrics are not part of step 19).

R2 (failure limit): per family and configuration, the smallest level (in order of severity) at which the
mission success rate drops below the nominal one.
"""

from __future__ import annotations

import argparse
import csv
import json
import os
import time
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path

from experiment.batch import BATCH_DIR, COLUMNS, ROOT, one
from experiment.disturbances import DisturbanceSpec, noise_label

ALL = ("A", "B", "C", "D", "E")
LIDAR_CONFIGS = ("D", "E")
# family -> (levels in order of severity, configurations, mazes or None = default)
PLAN = {
    "NOM": ([None], ALL, None),
    "D1": ([2.0, 3.0, 5.0], ALL, None),
    "D2": ([0.01, -0.01, 0.02, -0.02], ALL, None),
    "D3": ([0.02, -0.02, 0.05, -0.05], ALL, None),
    "D4": ([0.01, 0.03], ALL, None),
    "D5": ([0.2, 0.5], ("C", "E"), None),
    "D6": ([0.2, 0.5], ("D", "E"), None),
    "D7": (["L", "R"], ALL, None),
    "N1": ([0.5, 2.0, 4.0, 8.0], ALL, None),
    "N2gyro": ([2.0, 4.0, 8.0], ("C", "E"), ("T3",)),
    "N2lidar": ([2.0, 4.0, 8.0], ("D", "E"), ("T3",)),
    "N2prox": ([2.0, 4.0, 8.0], ALL, ("T3",)),
}
MAZES = ("T3", "M4")


def job_args(family: str, level) -> tuple[list[str], list[str]]:
    """-> (extra command line for run.py, folder-name labels in run.py order)."""
    if family == "NOM":
        return [], []
    if family.startswith("D"):
        spec = DisturbanceSpec(family, level)
        return ["--dist", f"{family}={level}"], [spec.label]
    if family == "N1":
        scale = {"gyro": level, "lidar": level, "proximity": level}
        return ["--noise-scale", str(level)], [noise_label(scale)]
    sensor = {"N2gyro": "gyro", "N2lidar": "lidar", "N2prox": "proximity"}[family]
    flag = {"gyro": "--noise-scale-gyro", "lidar": "--noise-scale-lidar", "proximity": "--noise-scale-prox"}[sensor]
    return [flag, str(level)], [noise_label({sensor: level})]


def jobs(families, mazes, seeds, configs):
    out = []
    for fam in families:
        levels, cfgs, fam_mazes = PLAN[fam]
        for level in levels:
            for m in (fam_mazes or mazes):
                for c in cfgs:
                    if configs and c not in configs:
                        continue
                    for s in seeds:
                        extra, labels = job_args(fam, level)
                        if c in LIDAR_CONFIGS:
                            extra = extra + ["--offline-lidar"]
                        out.append({"family": fam, "level": level, "maze": m, "config": c, "seed": s,
                                    "extra": extra, "labels": labels})
    return out


def _name(j, tag):
    parts = [*j["labels"], tag]
    return f"{j['maze']}_{j['config']}_realistic_s{j['seed']}_offline_" + "_".join(p for p in parts if p)


def _row(j, tag):
    folder = ROOT / "results" / "runs" / _name(j, tag)
    if (folder / "summary.json").exists() and (folder / "metrics.json").exists():
        s = json.loads((folder / "summary.json").read_text())
        s.update({k: v for k, v in json.loads((folder / "metrics.json").read_text()).items()
                  if k not in s or k.startswith(("P", "L", "K"))})
        s["run_dir"] = folder.name
        s["reused"] = True
    else:
        s = one((j["maze"], j["config"], "realistic", j["seed"]), tag, j["extra"], None, tuple(j["labels"]))
    s.update({"family": j["family"], "dist_level": j["level"]})
    d = s.get("disturbance")
    s["dist"] = d.get("label") if isinstance(d, dict) else (j["labels"][0] if j["labels"] else "NOM")
    s["dist_active_from_s"] = s.get("disturbance_active_from_s")
    return s


def run(a):
    js = jobs(a.families, a.mazes, a.seeds, a.configs)
    n_lidar = sum(j["config"] in LIDAR_CONFIGS for j in js)
    print(f"step 19: {len(js)} offline runs ({n_lidar} with ray-cast LiDAR), {a.workers} in parallel")
    if a.dry_run:
        from collections import Counter
        for fam, n in Counter(j["family"] for j in js).items():
            print(f"  {fam:8} {n:4} runs")
        print(f"rough time: {(n_lidar * 4 + (len(js) - n_lidar) * 0.7) / a.workers / 60:.1f} h "
              f"(~4 min per LiDAR run, ~0.7 min without; measure on your machine)")
        return
    rows, t0 = [], time.perf_counter()
    BATCH_DIR.mkdir(parents=True, exist_ok=True)
    out = BATCH_DIR / f"step19_{a.tag}.csv"
    cols = ("family", *COLUMNS)
    with ThreadPoolExecutor(a.workers) as ex:
        for r in ex.map(lambda j: _row(j, a.tag), js):
            rows.append(r)
            print(f"  {r['family']:8} {str(r['dist_level']):5} {r['maze']:3} {r['configuration']} s{r['seed']}  "
                  f"{'SUCCESS' if r.get('S4_mission_success') else 'FAILED '}  map {'ok ' if r.get('T_map_correct') else 'BAD'}  "
                  f"L1 {1000 * (r.get('L1_pos_rmse_m') or float('nan')):7.1f} mm  "
                  f"{r.get('fault') or r.get('mission_reason') or ''}{'  (reused)' if r.get('reused') else ''}", flush=True)
            with out.open("w", newline="") as f:              # rewritten after every run (long batches)
                w = csv.DictWriter(f, fieldnames=cols, extrasaction="ignore")
                w.writeheader()
                w.writerows(rows)
    print(f"{len(rows)} runs in {(time.perf_counter() - t0) / 60:.0f} min, saved {out}")


# ---------------------------------------------------------------------------------------------- report
def _b(v) -> bool:
    return str(v) == "True"


def _f(v):
    try:
        return float(v)
    except (TypeError, ValueError):
        return float("nan")


def report(a):
    import numpy as np
    f = BATCH_DIR / f"step19_{a.tag}.csv"
    rows = list(csv.DictReader(f.open()))
    key = lambda r: (r["family"], r["dist_level"], r["configuration"])
    groups: dict = {}
    for r in rows:
        groups.setdefault(key(r), []).append(r)
    stat = {}
    lines = ["| smetnja | razina | konf. | vožnje | S4 | karta točna | L1 RMSE [mm] | L1 max [mm] | L2 [°] | ANEES |",
             "|---|---|---|---:|---:|---:|---:|---:|---:|---:|"]
    for fam in PLAN:
        levels = [str(lv) for lv in PLAN[fam][0]]
        for lv in levels:
            for c in ALL:
                g = groups.get((fam, lv if lv != "None" else "", c)) or groups.get((fam, lv, c))
                if not g:
                    continue
                s4 = np.mean([_b(r["S4_mission_success"]) for r in g])
                mp = np.mean([_b(r["T_map_correct"]) for r in g])
                l1 = 1000 * np.nanmean([_f(r["L1_pos_rmse_m"]) for r in g])
                l1m = 1000 * np.nanmax([_f(r["L1_pos_max_m"]) for r in g])
                l2 = np.nanmean([_f(r["L2_heading_rmse_deg"]) for r in g])
                ne = np.nanmean([_f(r["L5_nees_mean"]) for r in g])
                stat[(fam, lv, c)] = s4
                lines.append(f"| {fam} | {lv if lv != 'None' else '–'} | {c} | {len(g)} | {100 * s4:.0f} % | {100 * mp:.0f} % | "
                             f"{l1:.0f} | {l1m:.0f} | {l2:.2f} | {ne:.2f} |")
    print("\n".join(lines))
    print("\nR2 - failure limit (smallest level, in order of severity, with S4 below nominal):")
    r2 = ["| smetnja | " + " | ".join(ALL) + " |", "|---|" + "---|" * len(ALL)]
    for fam in PLAN:
        if fam == "NOM":
            continue
        cells = []
        for c in ALL:
            nom = stat.get(("NOM", "None", c))
            lv_fail = "–"
            have = False
            for lv in [str(x) for x in PLAN[fam][0]]:
                s = stat.get((fam, lv, c))
                if s is None:
                    continue
                have = True
                if nom is not None and s < nom:
                    lv_fail = lv
                    break
            cells.append(lv_fail if have else "n/a")
        r2.append(f"| {fam} | " + " | ".join(cells) + " |")
    print("\n".join(r2))
    (BATCH_DIR / f"step19_{a.tag}_report.md").write_text("\n".join(lines) + "\n\nR2\n\n" + "\n".join(r2) + "\n")
    print(f"saved {BATCH_DIR / f'step19_{a.tag}_report.md'}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--families", nargs="+", default=list(PLAN), choices=list(PLAN))
    ap.add_argument("--mazes", nargs="+", default=list(MAZES))
    ap.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3])
    ap.add_argument("--configs", nargs="+", default=None, help="only these configurations")
    ap.add_argument("--tag", default="s19")
    ap.add_argument("--workers", type=int, default=os.cpu_count() or 2)
    ap.add_argument("--dry-run", action="store_true")
    ap.add_argument("--report", action="store_true")
    a = ap.parse_args(argv)
    report(a) if a.report else run(a)


if __name__ == "__main__":
    main()
