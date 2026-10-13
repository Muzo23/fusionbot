"""
Estimator replay (step 15b): run ONLY the pose estimator (and the metric map it matches against) again
over a recorded run, with other filter parameters or another configuration.

Why this is valid: the navigation (FSM, motion control, wheel PIs) runs on the encoders and the
proximity sensors only; the pose estimate enters the run only through the topological map (junction
association) and the goal search. As long as the robot's map stays correct, the true trajectory and every
measurement (encoders, gyro, proximity, LiDAR, junction events) are the same whatever the estimator does,
so a recorded run can be replayed through any estimator. The recordings are made with configuration E
(map always correct); a final real batch with the tuned values checks the closed loop (step 15b, last part).

Everything the estimator needs is in the run folder (step 13b-2 recorder):
    ticks.csv.gz   enc_l, enc_r (encoder counts), gyro_meas, dm_* (proximity with noise), gt_*
    events.csv     tick + kind of every junction event (PROX-LOC junction update)
    scans.npz      measured LiDAR ranges (0.1 mm) at the ticks where a scan arrived
    summary.json   start-up gyro bias calibration (gyro_start_bias_rad_s, gyro_start_samples)
The only difference to the live estimator: LiDAR ranges are stored with 0.1 mm resolution
(the URG-04LX itself has 1 mm), so a replay of the SAME parameters reproduces the run to within
a fraction of a millimetre (checked by  python -m experiment.replay --verify <run>).

Parameters are overridden by name, e.g.  {"ekf.Q_DS_PER_M": 4e-4, "scan_match.COV_INFLATE": 300}.

    python -m experiment.replay --verify ../results/runs/T1_E_realistic_s1_offline_rec15b
    python -m experiment.replay ../results/runs/T1_E_realistic_s1_offline_rec15b --config D --set scan_match.COV_INFLATE=300
"""

from __future__ import annotations

import argparse
import csv
import gzip
import json
import math
from pathlib import Path

import numpy as np

from core.configurations import CONFIGS
from experiment.recorder import load_scans
from localization import ekf as _ekf
from localization import prox_loc as _prox_loc
from localization import scan_match as _scan_match
from localization import estimators as _estimators
from localization.estimators import TickInputs, make_estimator
from localization.odometry import DifferentialDriveOdometry
from mapping.occupancy_grid import OccupancyGrid
from navigation.junction_types import ReferenceKind
from perception.proximity_types import ProximityReading, ProximitySnapshot

MODULES = {"ekf": _ekf, "prox_loc": _prox_loc, "scan_match": _scan_match, "estimators": _estimators}
SENSORS = ("left", "front_left", "front", "front_right", "right")
_TICK_COLS = ("k", "t", "enc_l", "enc_r", "gyro_meas", "gt_x", "gt_y", "gt_th", "est_x", "est_y", "est_th", "scan",
              *(f"dm_{n}" for n in SENSORS))


def _num(v: str) -> float:
    return float(v) if v != "" else math.nan


class Recording:
    """A recorded run, parsed once into numpy arrays (cached in <run>/replay_cache.npz)."""

    def __init__(self, folder) -> None:
        self.dir = Path(folder)
        self.meta = json.loads((self.dir / "meta.json").read_text())
        self.summary = json.loads((self.dir / "summary.json").read_text())
        cache = self.dir / "replay_cache.npz"
        if cache.exists() and cache.stat().st_mtime >= (self.dir / "ticks.csv.gz").stat().st_mtime:
            z = np.load(cache)
            self.T = {k: z[k] for k in z.files}
        else:
            cols: dict[str, list] = {c: [] for c in _TICK_COLS}
            with gzip.open(self.dir / "ticks.csv.gz", "rt") as f:
                for row in csv.DictReader(f):
                    for c in _TICK_COLS:
                        cols[c].append(_num(row[c]))
            self.T = {c: np.array(v) for c, v in cols.items()}
            np.savez(cache, **self.T)
        self.events: dict[int, str] = {}
        if (self.dir / "events.csv").exists():
            with (self.dir / "events.csv").open() as f:
                for e in csv.DictReader(f):
                    self.events[int(e["tick"])] = e["kind"]
        self.scans = load_scans(self.dir) if (self.dir / "scans.npz").exists() else None
        if self.scans is not None:
            self.scan_angles = [float(a) for a in self.scans["angles"]]
            # the noise model quantises the ranges to 1 mm (k * 0.001); rebuild exactly that float
            raw = np.load(self.dir / "scans.npz")["ranges_measured"]
            if raw.size and not np.any(raw[raw > 0] % 10):
                r = (raw // 10).astype(np.float64) * 0.001
                r[raw == 0] = np.nan
                self.scans["ranges_measured"] = r
        lid = self.meta.get("lidar", {})
        m, ext = lid.get("mount", {}), lid.get("extrinsic") or (0.0, 0.0, 0.0)
        self.mount = (m.get("x_forward_m", 0.1815) + ext[0], m.get("y_left_m", 0.0014) + ext[1],
                      m.get("yaw_offset_rad", 0.0) + ext[2])
        self.start_pose = tuple(self.meta["maze"]["start_pose"])
        self.name = self.dir.name

    @property
    def n(self) -> int:
        return len(self.T["k"])


def _apply(params: dict | None) -> dict:
    """Set module constants, return the old values (for restore)."""
    old = {}
    for key, val in (params or {}).items():
        mod, _, name = key.partition(".")
        m = MODULES[mod]
        if not hasattr(m, name):
            raise KeyError(f"unknown parameter {key}")
        old[key] = getattr(m, name)
        setattr(m, name, val)
    return old


def replay(rec: Recording, config: str, params: dict | None = None, *, gating: bool = True,
           keep_trace: bool = False) -> dict:
    """Run the estimator of `config` over the recording. -> metrics (L1, L2, L5 NEES, update counts)."""
    old = _apply(params)
    try:
        return _replay(rec, CONFIGS[config], gating, keep_trace)
    finally:
        _apply(old)


def _replay(rec, cfg, gating, keep_trace):
    from mazes.maze_spec import PITCH_M, START_OFFSET_FROM_ROOT_M
    T = rec.T
    odo = DifferentialDriveOdometry()
    odo.reset(*rec.start_pose)
    est = make_estimator(cfg, odo, gating=gating)
    grid = OccupancyGrid(*rec.start_pose[:2])
    s = rec.summary
    bias = s.get("gyro_start_bias_rad_s")
    if bias is None and s.get("gyro_start_bias_estimate_deg_s") is not None:
        bias = math.radians(s["gyro_start_bias_estimate_deg_s"])
    n_cal = s.get("gyro_start_samples") or 51
    sx, sy, _ = rec.start_pose
    est.start(*rec.start_pose, gyro_bias=bias, gyro_samples=n_cal if bias is not None else 0,
              lattice=(sx, sy + START_OFFSET_FROM_ROOT_M, PITCH_M))
    est.attach_lidar(grid, rec.mount)
    f, l, yaw = rec.mount
    n = rec.n
    E = np.zeros((n, 3))
    P = np.zeros((n, 3, 3))
    dm = [T[f"dm_{s_}"] for s_ in SENSORS]
    for i in range(n):
        inc = odo.update(int(T["enc_l"][i]), int(T["enc_r"][i]))
        readings = {}
        for name, col in zip(SENSORS, dm):
            v = col[i]
            ok = not math.isnan(v)
            readings[name] = ProximityReading(name, ok, float(v) if ok else None)
        prox = ProximitySnapshot(**readings)
        g = T["gyro_meas"][i]
        scan = None
        si = int(T["scan"][i])
        if si >= 0 and rec.scans is not None:
            r = rec.scans["ranges_measured"][si]
            scan = {"angles": rec.scan_angles, "ranges": [None if math.isnan(v) else float(v) for v in r]}
        est.update(TickInputs(T["t"][i], inc, None if math.isnan(g) else float(g), bias, prox, scan))
        Pi = est.covariance()
        if scan is not None and (est.matcher is not None or keep_trace):     # map only where it is used
            x, y, th = est.pose()
            cx, cy = x + f * math.cos(th) - l * math.sin(th), y + f * math.sin(th) + l * math.cos(th)
            grid.update(cx, cy, th + yaw, scan["angles"], scan["ranges"], t=T["t"][i],
                        pose_var=(0.5 * float(Pi[0, 0] + Pi[1, 1]), float(Pi[2, 2])) if Pi is not None else (0.0, 0.0))
        kind = rec.events.get(int(T["k"][i]))
        if kind is not None:
            est.on_junction(ReferenceKind(kind))
        E[i] = est.pose()
        P[i] = est.covariance()[:3, :3]
    out = metrics(T, E, P)
    out.update({"run": rec.name, "configuration": cfg.name, "stats": est.stats()})
    if keep_trace:
        out["trace"] = (E, P)
        out["grid"] = grid
    return out


def metrics(T, E, P, step: int = 10) -> dict:
    """The same L1 / L2 / L5 definitions as evaluation.metrics."""
    ex, ey = E[:, 0] - T["gt_x"], E[:, 1] - T["gt_y"]
    eth = (E[:, 2] - T["gt_th"] + math.pi) % (2 * math.pi) - math.pi
    pe = np.hypot(ex, ey)
    i = np.arange(0, len(ex), step)
    e = np.stack([ex[i], ey[i], eth[i]], axis=1)
    nees = np.einsum("ni,nij,nj->n", e, np.linalg.inv(P[i]), e)
    # position-only NEES (2 dof) and heading-only (1 dof): which part is inconsistent
    nees_xy = np.einsum("ni,nij,nj->n", e[:, :2], np.linalg.inv(P[i][:, :2, :2]), e[:, :2])
    nees_th = e[:, 2] ** 2 / P[i][:, 2, 2]
    return {"L1_pos_rmse_m": float(np.sqrt(np.mean(pe ** 2))), "L1_pos_max_m": float(pe.max()),
            "L2_heading_rmse_deg": float(np.degrees(np.sqrt(np.mean(eth ** 2)))),
            "L5_nees_mean": float(nees.mean()), "L5_nees_median": float(np.median(nees)),
            "L5_nees_in_95pct": 100.0 * float(np.mean(nees <= 7.815)),
            "L5_nees_xy_mean": float(nees_xy.mean()), "L5_nees_th_mean": float(nees_th.mean())}


def verify(folder) -> dict:
    """Replay a run with its own configuration and default parameters; compare with the recorded estimate."""
    rec = Recording(folder)
    cfg = rec.meta["configuration"]["name"]
    r = replay(rec, cfg, keep_trace=True)
    E = r["trace"][0]
    d = np.hypot(E[:, 0] - rec.T["est_x"], E[:, 1] - rec.T["est_y"])
    dth = np.degrees(np.abs((E[:, 2] - rec.T["est_th"] + math.pi) % (2 * math.pi) - math.pi))
    return {"run": rec.name, "configuration": cfg, "ticks": rec.n, "max_pos_diff_mm": 1000 * float(d.max()),
            "max_heading_diff_deg": float(dth.max()), "L1_replay_mm": 1000 * r["L1_pos_rmse_m"],
            "NEES_replay": r["L5_nees_mean"]}


def _parse_set(items):
    out = {}
    for it in items or []:
        k, _, v = it.partition("=")
        try:
            out[k] = float(v)
        except ValueError:
            out[k] = v                   # e.g. estimators.SCAN_MODE=ci
    return out


def main(argv=None):
    ap = argparse.ArgumentParser()
    ap.add_argument("run", nargs="+", help="run folder(s)")
    ap.add_argument("--verify", action="store_true", help="replay with the run's own configuration and compare")
    ap.add_argument("--config", nargs="+", default=["A", "B", "C", "D", "E"])
    ap.add_argument("--set", nargs="*", help="parameter overrides, e.g. scan_match.COV_INFLATE=300")
    ap.add_argument("--no-gate", action="store_true")
    a = ap.parse_args(argv)
    for folder in a.run:
        if a.verify:
            print(json.dumps(verify(folder)))
            continue
        rec = Recording(folder)
        for c in a.config:
            r = replay(rec, c, _parse_set(a.set), gating=not a.no_gate)
            print(f"{rec.name:45} {c}  L1 {1000 * r['L1_pos_rmse_m']:7.1f} mm  max {1000 * r['L1_pos_max_m']:7.1f} mm  "
                  f"L2 {r['L2_heading_rmse_deg']:5.2f} deg  NEES {r['L5_nees_mean']:9.2f} "
                  f"(xy {r['L5_nees_xy_mean']:.2f}, th {r['L5_nees_th_mean']:.2f}, {r['L5_nees_in_95pct']:.0f} % < 95 %)")


if __name__ == "__main__":
    main()
