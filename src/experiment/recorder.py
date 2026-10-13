"""
Run recorder (step 13b-2): everything a run produces, in one folder per run.

    results/runs/<maze>_<config>_<noise>_s<seed>[_<tag>]/
        meta.json         what was run: configuration, noise profile (+ drawn values), maze,
                          seed, robot constants, LiDAR mount / calibration, start / end, versions
        ticks.csv.gz      one row per 10 ms control tick (written while driving)
        events.csv        one row per junction event (FSM observation + decision)
        scans.npz         LiDAR at 10 Hz: ray angles (once), ideal and measured ranges (uint16, 0.1 mm),
                          GT and estimated pose at the scan (needs numpy)
        map_topo.json     the robot's topological map (MazeMemory)
        map_topo.png      the map over the true walls (evaluation picture)
        summary.json      basic results (step 13b-3 extends this to all metrics S/L/T/K/E/P/R)

The ideal AND the measured value of every noisy sensor are recorded, so the
effect of the noise can be analysed afterwards without driving again.
"""

from __future__ import annotations

import csv
import datetime as _dt
import gzip
import json
import math
import platform
from pathlib import Path

RUNS_DIR = Path(__file__).resolve().parents[2] / "results" / "runs"


def run_name(maze: str, config: str, noise: str, seed: int, tag: str | None = None, offline: bool = False) -> str:
    return f"{maze}_{config}_{noise}_s{seed}" + ("_offline" if offline else "") + (f"_{tag}" if tag else "")


def _clean(v):
    if isinstance(v, float) and not math.isfinite(v):
        return None
    if isinstance(v, dict):
        return {k: _clean(x) for k, x in v.items()}
    if isinstance(v, (list, tuple)):
        return [_clean(x) for x in v]
    return v


class RunRecorder:
    def __init__(self, folder: Path) -> None:
        self.dir = Path(folder)
        self.dir.mkdir(parents=True, exist_ok=True)
        for old in ("ticks.csv.gz", "events.csv", "scans.npz", "map_topo.json", "map_topo.png", "summary.json"):
            p = self.dir / old
            if p.exists():
                p.unlink()
        self._ticks_f = None
        self._ticks_w = None
        self.n_ticks = 0
        self.events: list[dict] = []
        self.scan_t, self.scan_tick = [], []
        self.scan_ideal, self.scan_meas = [], []
        self.scan_gt, self.scan_est = [], []
        self.scan_angles = None
        self.meta: dict = {}

    # ------------------------------------------------------------------ writing
    def write_meta(self, meta: dict) -> None:
        self.meta = {**meta, "created": _dt.datetime.now().isoformat(timespec="seconds"),
                     "python": platform.python_version(), "platform": platform.platform()}
        (self.dir / "meta.json").write_text(json.dumps(_clean(self.meta), indent=1))

    def update_meta(self, **kw) -> None:
        self.meta.update(kw)
        (self.dir / "meta.json").write_text(json.dumps(_clean(self.meta), indent=1))

    def tick(self, row: dict) -> None:
        if self._ticks_w is None:
            self._ticks_f = gzip.open(self.dir / "ticks.csv.gz", "wt", newline="")
            self._ticks_w = csv.DictWriter(self._ticks_f, fieldnames=list(row))
            self._ticks_w.writeheader()
        self._ticks_w.writerow({k: ("" if v is None else v) for k, v in row.items()})
        self.n_ticks += 1

    def event(self, row: dict) -> None:
        self.events.append(row)

    def scan(self, t, tick, angles, ideal, measured, gt_pose, est_pose) -> None:
        if self.scan_angles is None:
            self.scan_angles = list(angles)
        self.scan_t.append(t)
        self.scan_tick.append(tick)
        self.scan_ideal.append([math.nan if r is None else r for r in ideal])
        self.scan_meas.append([math.nan if r is None else r for r in measured])
        self.scan_gt.append(gt_pose)
        self.scan_est.append(est_pose)

    def close(self, summary: dict | None = None) -> None:
        if self._ticks_f is not None:
            self._ticks_f.close()
            self._ticks_f = None
        if self.events:
            with (self.dir / "events.csv").open("w", newline="") as f:
                keys = list(self.events[0])
                w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
                w.writeheader()
                w.writerows(self.events)
        if self.scan_t:
            try:
                import numpy as np
                np.savez_compressed(
                    self.dir / "scans.npz", t=np.array(self.scan_t), tick=np.array(self.scan_tick),
                    angles=np.array(self.scan_angles, dtype=np.float64),   # float64: exact replay (step 15b)
                    ranges_ideal=_to_u16(np, self.scan_ideal), ranges_measured=_to_u16(np, self.scan_meas),
                    range_unit_m=RANGE_UNIT_M,
                    gt_pose=np.array(self.scan_gt), est_pose=np.array(self.scan_est))
            except ImportError:                                   # pragma: no cover
                print("(numpy not installed - scans not saved)")
        if summary is not None:
            (self.dir / "summary.json").write_text(json.dumps(_clean(summary), indent=1))


RANGE_UNIT_M = 1e-4          # ranges stored as uint16 in 0.1 mm (0 = no return), max 6.55 m


def _to_u16(np, rows):
    a = np.array(rows, dtype=np.float64) / RANGE_UNIT_M
    a = np.where(np.isfinite(a), np.clip(np.round(a), 1, 65535), 0)
    return a.astype(np.uint16)


def load_scans(folder: Path):
    """-> dict of numpy arrays; ranges in metres with NaN for no return."""
    import numpy as np
    z = np.load(Path(folder) / "scans.npz")
    out = {k: z[k] for k in z.files}
    for k in ("ranges_ideal", "ranges_measured"):
        r = out[k].astype(np.float64) * float(out["range_unit_m"])
        r[out[k] == 0] = np.nan
        out[k] = r
    return out


def load_run(folder: Path) -> dict:
    """Read a run back: meta, summary, events (list of dict, strings), ticks (list of dict, strings)."""
    folder = Path(folder)
    out = {"meta": json.loads((folder / "meta.json").read_text())}
    if (folder / "summary.json").exists():
        out["summary"] = json.loads((folder / "summary.json").read_text())
    if (folder / "events.csv").exists():
        with (folder / "events.csv").open() as f:
            out["events"] = list(csv.DictReader(f))
    if (folder / "ticks.csv.gz").exists():
        with gzip.open(folder / "ticks.csv.gz", "rt") as f:
            out["ticks"] = list(csv.DictReader(f))
    return out
