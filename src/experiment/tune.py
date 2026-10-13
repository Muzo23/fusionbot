"""
Step 15b: systematic tuning of the EKF noise parameters against consistency (NEES) and accuracy (L1),
on Monte Carlo recordings, with an evolutionary optimiser (differential evolution, Storn & Price 1997).

The recordings (configuration E, every sensor recorded, map always correct) are replayed through the
estimator of the stage (experiment.replay), so one evaluation costs seconds instead of a full run.
Recordings are split into a TUNING set and a VALIDATION set (other mazes and seeds), so the result is
checked on data the optimiser never saw.

Stages (run in this order, each one fixes the result of the previous ones):
    enc     configuration A (encoders only): process noise of the odometry model
            ekf.Q_DS_PER_M, ekf.Q_TH_ENC_PER_RAD, ekf.Q_TH_ENC_PER_M
            L1 does not depend on Q (no corrections) -> pure consistency:
            J = mean_runs[ ln(L1 / L1_ref) + ln(NEES_xy / 2)^2 + ln(NEES_th / 1)^2 ]   (L1 term = 0 for A)
    gyro    configuration C: gyro heading noise  ekf.Q_TH_GYRO_PER_RAD (bias random walk = datasheet), same J
    lidar   configuration D: scan-match covariance  scan_match.COV_INFLATE, scan_match.SIGMA_MIN_M
            J = mean_runs[ ln(L1 / L1_ref) ] + W_NEES * ln(ANEES / 3)^2   (accuracy AND consistency)
The parameters are searched in log10 space between bounds of a factor ~30 around the start values.
A consistent filter has NEES_xy = 2, NEES_th = 1 and NEES(x, y, theta) = 3 on average (chi-square
means; Bar-Shalom et al. 2001, ch. 5.4).

    python -m experiment.tune enc   --tag rec15b
    python -m experiment.tune gyro  --tag rec15b --fix results/tuning/enc.json
    python -m experiment.tune lidar --tag rec15b --fix results/tuning/enc.json results/tuning/gyro.json
    python -m experiment.tune scan  --tag rec15b --config D --param scan_match.COV_INFLATE --values 100 300 1000 3000
    python -m experiment.tune table --tag rec15b --fix results/tuning/enc.json results/tuning/gyro.json --set scan_match.COV_INFLATE=100
"""

from __future__ import annotations

import argparse
import json
import math
import time
from multiprocessing import Pool
from pathlib import Path

import numpy as np

from experiment.replay import MODULES, Recording, replay

ROOT = Path(__file__).resolve().parents[2]
RUNS = ROOT / "results" / "runs"
OUT = ROOT / "results" / "tuning"
TUNE_MAZES = ("T1", "T3", "M2", "M5")          # tuning set
VALID_MAZES = ("T2", "M1", "M3")               # validation set (never seen by the optimiser)
W_NEES = 0.25

STAGES = {
    "enc": {"config": "A", "params": ("ekf.Q_DS_PER_M", "ekf.Q_TH_ENC_PER_RAD", "ekf.Q_TH_ENC_PER_M"),
            "cost": "consistency"},
    # GYRO_BIAS_RW is NOT tuned: it is the datasheet value the noise profile uses (tuning it to 3.8x bought
    # NEES with +6 % L1 on the tuning set, +10 % on validation -> results/tuning/gyro_with_bias_rw_rejected.json)
    "gyro": {"config": "C", "params": ("ekf.Q_TH_GYRO_PER_RAD",), "cost": "consistency"},
    "lidar": {"config": "D", "params": ("scan_match.COV_INFLATE", "scan_match.SIGMA_MIN_M"), "cost": "accuracy"},
}
SPAN = 1.5          # log10 half-width of the search box around the start value (factor ~30)

_CACHE: dict[str, Recording] = {}


def recordings(tag: str, mazes, seeds) -> list[Path]:
    out = []
    for m in mazes:
        for s in seeds:
            p = RUNS / f"{m}_E_realistic_s{s}_offline_{tag}"
            if (p / "summary.json").exists():
                out.append(p)
    return out


def _rec(path) -> Recording:
    key = str(path)
    if key not in _CACHE:
        _CACHE[key] = Recording(path)
    return _CACHE[key]


def _one(job):
    path, config, params = job
    r = replay(_rec(path), config, params)
    r.pop("stats", None)
    return r


def default(name: str) -> float:
    mod, _, attr = name.partition(".")
    return float(getattr(MODULES[mod], attr))


def cost(results: list[dict], kind: str, ref_l1: dict | None = None) -> float:
    acc = np.mean([math.log(r["L1_pos_rmse_m"] / ref_l1[r["run"]]) for r in results])
    if kind == "consistency":            # + accuracy: a parameter that also moves the estimate (ZUPT gain) must not cost L1
        return float(acc + np.mean([math.log(r["L5_nees_xy_mean"] / 2.0) ** 2 + math.log(r["L5_nees_th_mean"]) ** 2
                                    for r in results]))
    anees = np.mean([r["L5_nees_mean"] for r in results])
    return float(acc + W_NEES * math.log(anees / 3.0) ** 2)


def summarise(results: list[dict]) -> dict:
    g = lambda k: float(np.mean([r[k] for r in results]))
    return {"runs": len(results), "L1_mm": 1000 * g("L1_pos_rmse_m"), "L1_max_mm": 1000 * max(r["L1_pos_max_m"] for r in results),
            "L2_deg": g("L2_heading_rmse_deg"), "ANEES": g("L5_nees_mean"), "NEES_xy": g("L5_nees_xy_mean"),
            "NEES_th": g("L5_nees_th_mean"), "in95_pct": g("L5_nees_in_95pct")}


class Evaluator:
    def __init__(self, paths, config, fixed, names, kind, pool, ref_l1=None):
        self.paths, self.config, self.fixed, self.names, self.kind = paths, config, fixed, names, kind
        self.pool, self.ref_l1 = pool, ref_l1
        self.history = []

    def params(self, z) -> dict:
        return {**self.fixed, **{n: 10.0 ** float(v) for n, v in zip(self.names, z)}}

    def run(self, params: dict) -> list[dict]:
        return self.pool.map(_one, [(p, self.config, params) for p in self.paths])

    def __call__(self, z) -> float:
        res = self.run(self.params(z))
        c = cost(res, self.kind, self.ref_l1)
        self.history.append({"z": [float(v) for v in z], "cost": c, **summarise(res)})
        h = self.history[-1]
        print(f"  #{len(self.history):3d}  " + "  ".join(f"{n.split('.')[1]}={10 ** v:.3g}" for n, v in zip(self.names, z))
              + f"  J={c:.4f}  L1 {h['L1_mm']:.1f} mm  ANEES {h['ANEES']:.2f} (xy {h['NEES_xy']:.2f}, th {h['NEES_th']:.2f})",
              flush=True)
        return c


def load_fixed(files) -> dict:
    out = {}
    for f in files or []:
        out.update(json.loads(Path(f).read_text())["best"])
    return out


def run_stage(a):
    st = STAGES[a.stage]
    fixed = load_fixed(a.fix)
    names = st["params"]
    tune_p = recordings(a.tag, a.tune_mazes, a.seeds)
    valid_p = recordings(a.tag, a.valid_mazes, a.valid_seeds)
    print(f"stage {a.stage}: configuration {st['config']}, {len(tune_p)} tuning / {len(valid_p)} validation recordings")
    x0 = np.array([math.log10(default(n)) for n in names])
    t0 = time.perf_counter()
    with Pool(a.workers) as pool:
        # reference L1 per run = the start values (accuracy term 0 = as good as before)
        ref = {r["run"]: r["L1_pos_rmse_m"] for r in pool.map(_one, [(p, st["config"], fixed) for p in tune_p])}
        ev = Evaluator(tune_p, st["config"], fixed, names, st["cost"], pool, ref)
        start_cost = ev(x0)
        from scipy.optimize import differential_evolution
        bounds = [(v - SPAN, v + SPAN) for v in x0]
        res = differential_evolution(ev, bounds, x0=x0, popsize=a.popsize, maxiter=a.generations, tol=1e-3,
                                     mutation=(0.5, 1.0), recombination=0.7, seed=a.rng, polish=False,
                                     init="latinhypercube", updating="immediate")
        best = ev.params(res.x)
        best_only = {n: best[n] for n in names}
        tune_before, tune_after = ev.history[0], min(ev.history, key=lambda h: h["cost"])
        ev_v = Evaluator(valid_p, st["config"], fixed, names, st["cost"], pool)
        val_before = summarise(ev_v.run(ev_v.params(x0))) if valid_p else None
        val_after = summarise(ev_v.run(best)) if valid_p else None
    out = {"stage": a.stage, "configuration": st["config"], "tag": a.tag, "fixed": fixed,
           "start": {n: default(n) for n in names}, "best": best_only, "start_cost": start_cost,
           "best_cost": float(res.fun), "evaluations": len(ev.history), "generations": int(res.nit),
           "tuning_runs": [p.name for p in tune_p], "validation_runs": [p.name for p in valid_p],
           "tuning": {"before": tune_before, "after": tune_after},
           "validation": {"before": val_before, "after": val_after},
           "wall_s": time.perf_counter() - t0, "history": ev.history}
    OUT.mkdir(parents=True, exist_ok=True)
    f = OUT / f"{a.stage}.json"
    f.write_text(json.dumps(out, indent=1))
    print("\nbest: " + ", ".join(f"{k} = {v:.4g} (start {out['start'][k]:.4g})" for k, v in best_only.items()))
    for part in ("tuning", "validation"):
        b, c = out[part]["before"], out[part]["after"]
        if b and c:
            print(f"{part:10}: L1 {b['L1_mm']:.1f} -> {c['L1_mm']:.1f} mm, ANEES {b['ANEES']:.2f} -> {c['ANEES']:.2f}, "
                  f"NEES_xy {b['NEES_xy']:.2f} -> {c['NEES_xy']:.2f}, NEES_th {b['NEES_th']:.2f} -> {c['NEES_th']:.2f}")
    print(f"saved {f}")


def run_scan(a):
    """1-D sweep of one parameter (for the figure in the thesis and as a check of the optimiser)."""
    from experiment.replay import _parse_set
    fixed = {**load_fixed(a.fix), **_parse_set(a.set)}
    paths = recordings(a.tag, a.mazes or (a.tune_mazes + a.valid_mazes), sorted(set(a.seeds + a.valid_seeds)))
    rows = []
    with Pool(a.workers) as pool:
        for v in a.values:
            res = pool.map(_one, [(p, a.config, {**fixed, a.param: v}) for p in paths])
            s = summarise(res)
            rows.append({"value": v, **s, "per_run": [{k: r[k] for k in ("run", "L1_pos_rmse_m", "L5_nees_mean")} for r in res]})
            print(f"  {a.param} = {v:<10g} L1 {s['L1_mm']:6.1f} mm  max {s['L1_max_mm']:6.1f}  ANEES {s['ANEES']:8.2f} "
                  f"(xy {s['NEES_xy']:.2f}, th {s['NEES_th']:.2f})  in95 {s['in95_pct']:.0f} %", flush=True)
            OUT.mkdir(parents=True, exist_ok=True)                 # saved after every value (long sweeps)
            f = OUT / (a.out or f"scan_{a.config}_{a.param.split('.')[1]}.json")
            f.write_text(json.dumps({"config": a.config, "param": a.param, "fixed": fixed,
                                     "runs": [p.name for p in paths], "rows": rows}, indent=1))
    print(f"saved {f}")


def run_table(a):
    """Every configuration over every recording with one parameter set -> per-run rows (before / after table)."""
    from experiment.replay import _parse_set
    fixed = {**load_fixed(a.fix), **_parse_set(a.set)}
    paths = recordings(a.tag, a.mazes or (a.tune_mazes + a.valid_mazes), sorted(set(a.seeds + a.valid_seeds)))
    rows = []
    OUT.mkdir(parents=True, exist_ok=True)
    f = OUT / (a.out or "table.json")
    with Pool(a.workers) as pool:
        for c in a.configs:
            res = pool.map(_one, [(p, c, fixed) for p in paths])
            for r in res:
                r.pop("trace", None)
                r["set"] = "tuning" if r["run"].split("_")[0] in a.tune_mazes and int(r["run"].split("_s")[1].split("_")[0]) in a.seeds else (
                    "validation" if r["run"].split("_")[0] in a.valid_mazes else "other")
            rows += res
            for part in ("tuning", "validation"):
                sel = [r for r in res if r["set"] == part]
                if sel:
                    s_ = summarise(sel)
                    print(f"  {c} {part:10} ({len(sel):2}) L1 {s_['L1_mm']:7.1f} mm  max {s_['L1_max_mm']:7.1f}  L2 {s_['L2_deg']:5.2f} deg  "
                          f"ANEES {s_['ANEES']:8.2f} (xy {s_['NEES_xy']:.2f}, th {s_['NEES_th']:.2f})  in95 {s_['in95_pct']:.0f} %", flush=True)
            f.write_text(json.dumps({"fixed": fixed, "rows": rows}, indent=1))
    print(f"saved {f}")


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("stage", choices=(*STAGES, "scan", "table"))
    ap.add_argument("--configs", nargs="+", default=["A", "B", "C", "D", "E"])
    ap.add_argument("--tag", default="rec15b")
    ap.add_argument("--tune-mazes", nargs="+", default=list(TUNE_MAZES))
    ap.add_argument("--valid-mazes", nargs="+", default=list(VALID_MAZES))
    ap.add_argument("--seeds", nargs="+", type=int, default=[1, 2])
    ap.add_argument("--valid-seeds", nargs="+", type=int, default=[1, 2, 3])
    ap.add_argument("--fix", nargs="*", help="result files of earlier stages (their best values are fixed)")
    ap.add_argument("--popsize", type=int, default=6, help="DE population = popsize x number of parameters")
    ap.add_argument("--generations", type=int, default=12)
    ap.add_argument("--rng", type=int, default=15)
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--config", default="D")
    ap.add_argument("--param", default="scan_match.COV_INFLATE")
    ap.add_argument("--values", nargs="+", type=float, default=[100, 300, 1000, 3000, 10000])
    ap.add_argument("--set", nargs="*", default=[], help="scan: extra fixed parameters, e.g. estimators.SCAN_MODE=ci")
    ap.add_argument("--mazes", nargs="+", default=None, help="scan: only these mazes (default tuning + validation)")
    ap.add_argument("--out", default=None, help="scan: result file name in results/tuning")
    a = ap.parse_args(argv)
    {"scan": run_scan, "table": run_table}.get(a.stage, run_stage)(a)


if __name__ == "__main__":
    main()
