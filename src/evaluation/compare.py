"""
Configuration comparison table from the recorded runs (any batch tag).

    python -m evaluation.compare --tag ekf15                 (all runs *_offline_ekf15)
    python -m evaluation.compare --tag ALL_dyn16 --keys S4_mission_success DYN_ekf_rejected_near

Groups the metrics.json of the matching run folders by configuration x noise (mean over mazes and
seeds; S4 and T as counts) and prints a table; --csv writes it to results/batches/compare_<tag>.csv.
"""

from __future__ import annotations

import argparse
import csv
import json
import statistics as st
from pathlib import Path

RUNS = Path(__file__).resolve().parents[2] / "results" / "runs"
DEFAULT = ("S4_mission_success", "T1_map_correct", "L1_pos_rmse_m", "L1_pos_max_m", "L2_heading_rmse_deg",
           "L3_junction_err_mean_m", "L5_nees_mean", "K2_agreement_pct", "P1_centreline_rms_mm")
COUNT = ("S4_mission_success", "T1_map_correct")


def collect(tag: str, mazes=None):
    rows = []
    for d in sorted(RUNS.glob(f"*_offline_{tag}")):
        f = d / "metrics.json"
        if f.exists():
            m = json.loads(f.read_text())
            if mazes and m.get("maze") not in mazes:
                continue
            rows.append(m)
    return rows


def table(rows, keys):
    groups = {}
    for m in rows:
        groups.setdefault((m["configuration"], m["noise"]), []).append(m)
    out = []
    for (c, n), g in sorted(groups.items()):
        r = {"configuration": c, "noise": n, "runs": len(g)}
        for k in keys:
            v = [x.get(k) for x in g if x.get(k) is not None]
            if k in COUNT:
                r[k] = f"{sum(bool(x) for x in v)}/{len(g)}"
            else:
                v = [float(x) for x in v if not isinstance(x, (list, dict))]
                r[k] = round(st.mean(v), 4) if v else None
        out.append(r)
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("--tag", required=True)
    ap.add_argument("--keys", nargs="+", default=list(DEFAULT))
    ap.add_argument("--mazes", nargs="+", default=None)
    ap.add_argument("--csv", action="store_true")
    a = ap.parse_args(argv)
    t = table(collect(a.tag, a.mazes), a.keys)
    cols = ["configuration", "noise", "runs", *a.keys]
    print(" | ".join(cols))
    for r in t:
        print(" | ".join(str(r.get(c)) for c in cols))
    if a.csv:
        p = RUNS.parent / "batches" / f"compare_{a.tag}.csv"
        with p.open("w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=cols)
            w.writeheader()
            w.writerows(t)
        print(f"saved {p}")
    return t


if __name__ == "__main__":
    main()
