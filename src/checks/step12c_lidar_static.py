"""
Step 12c LIVE check: the mounted LiDAR, robot standing at START.

Proves that what the LiDAR returns is geometrically right before it is used for
the map (step 14) and pose correction (step 18):

  1. data flow: signal.fusionbotLidarXYZD on the Hokuyo object, rays over
     240 deg (the model gives 684 rays, 0.351 deg, like the real URG-04LX),
     one new scan per simulation step;
  2. ranges vs the TRUE walls: every ray is compared with the ray cast through
     the known maze (mazes.maze_geometry.WallMap) from the true scan pose;
     rays that hit a wall end are excluded ("edge rays"), along-ray statistics use
     only non-grazing rays (incidence >= 30 deg), the error normal to the wall
     is reported for all rays;
  3. no self-hits (the robot is excluded from rendering), no missing returns,
     no phantom returns, points lie in the horizontal scan plane;
  4. mount: the scan pose predicted from the true axle pose + results/lidar_mount.json
     is compared with the pose obtained by aligning the scan with the walls;
  5. repeatability: scans of the standing robot are identical.

Range errors and the mount are judged AFTER the range calibration of step 12d
(results/lidar_calibration.json, e(r) = a r + b r^2); the raw values are printed
as the characterisation of the uncalibrated sensor.

    python -m checks.step12c_lidar_static          (from src/, robot at START, simulation stopped)

Logs: results/step12c/lidar_scan.csv, lidar_static.png
"""

from __future__ import annotations

import argparse
import csv
import json
import math
from pathlib import Path

from checks.lidar_live import analyse, grab_scans
from mazes.maze_geometry import WallMap
from mazes.maze_spec import MAZES
from perception.lidar_scan import RangeModel, expected_range
from sim.lidar_setup import MAX_RANGE_M, find_hokuyo
from sim.maze_scene import scene_maze_id

ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "results" / "step12c"
CALIBRATION = ROOT / "results" / "lidar_calibration.json"
N_SCANS = 20
SELF_HIT_M = 0.15
# acceptance after calibration (well inside the real URG-04LX spec of +-30 mm / 3 %)
MAX_MEDIAN_M, MAX_P95_M, MAX_NORMAL_P95_M = 0.002, 0.008, 0.006
MAX_MOUNT_M, MAX_MOUNT_DEG = 0.002, 0.2


def load_model():
    """(RangeModel, extrinsic (forward m, left m, yaw rad)) from step 12d, or (None, zeros)."""
    if not CALIBRATION.exists():
        return None, (0.0, 0.0, 0.0)
    d = json.loads(CALIBRATION.read_text())
    ext = (d.get("extrinsic_forward_m", 0.0), d.get("extrinsic_left_m", 0.0), d.get("extrinsic_yaw_rad", 0.0))
    return RangeModel(d["a"], d["b"]), ext


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--maze", default=None, help="default: the maze id stored in the scene")
    args = ap.parse_args()
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    sim = RemoteAPIClient().require("sim")
    maze = args.maze or scene_maze_id(sim) or "T1"
    walls = WallMap(MAZES[maze])
    mount = json.loads((ROOT / "results" / "lidar_mount.json").read_text())
    model, ext = load_model()
    hok = find_hokuyo(sim)
    scans = grab_scans(sim, hok, N_SCANS)

    first, last = scans[0], scans[-1]
    res = analyse(last, walls, mount, model, MAX_RANGE_M, extrinsic=ext)
    rays, raw = res["rays"], res["raw"]
    cor = res.get("cor")
    self_hits = sum(1 for r in rays if r.range_m is not None and r.range_m < SELF_HIT_M)
    steps = (last["t"] - first["t"]) / 0.01
    rate = (last["count"] - first["count"]) / steps if steps > 0 else math.nan
    r0 = [r.range_m for r in analyse(first, walls, mount, None, MAX_RANGE_M)["rays"]]
    rep = max((abs(a - b) for a, b in zip(r0, [r.range_m for r in rays]) if a is not None and b is not None),
              default=math.nan)
    moved = math.hypot(last["axle"][0] - first["axle"][0], last["axle"][1] - first["axle"][1])

    RESULTS.mkdir(parents=True, exist_ok=True)
    cx, cy = last["centre"][0], last["centre"][1]
    with (RESULTS / "lidar_scan.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["bearing_deg", "range_m", "expected_m", "residual_m", "x_m", "y_m", "z_m", "range_corrected_m"])
        crays = res.get("cor_rays", rays)
        for r, rc_ in zip(rays, crays):
            e = expected_range(walls, cx, cy, last["fwd"] + r.bearing_rad, MAX_RANGE_M)
            w.writerow([f"{math.degrees(r.bearing_rad):.4f}", r.range_m, e,
                        (r.range_m - e) if (r.range_m is not None and e is not None) else None,
                        r.x_m, r.y_m, r.z_m, rc_.range_m])
    try:
        _figure(walls, last, rays, RESULTS / "lidar_static.png", maze)
    except Exception as exc:                              # noqa: BLE001
        print(f"(figure not drawn: {exc})")

    ok = True

    def chk(cond, text):
        nonlocal ok
        if cond is None:
            print(f"  [-- ] {text}")
            return
        ok &= bool(cond)
        print(f"  [{'OK ' if cond else 'BAD'}] {text}")

    def rng(ev):
        return (f"non-grazing bias {1000 * ev['ng_bias_m']:+.2f} mm, median |e| {1000 * ev['ng_median_abs_m']:.2f} mm, "
                f"p95 {1000 * ev['ng_p95_abs_m']:.2f} mm ({ev['non_grazing']} rays); "
                f"normal to wall p95 {1000 * ev['normal_p95_m']:.2f} mm, max {1000 * ev['normal_max_m']:.2f} mm")

    def mnt(ev):
        return (f"forward {1000 * ev['mount_fwd_m']:+.2f} mm, left {1000 * ev['mount_left_m']:+.2f} mm, "
                f"yaw {ev['mount_yaw_deg']:+.3f} deg (fit rms {1000 * ev['mount_rms_m']:.2f} mm)")

    ax, h = last["axle"], last["heading"]
    print(f"\nStep 12c LiDAR static check (maze {maze}, {len(scans)} scans)")
    print("------------------------------------------------------")
    print(f"robot true pose        : axle ({ax[0]:+.4f}, {ax[1]:+.4f}) m, heading {math.degrees(h):+.3f} deg, "
          f"moved {1000 * moved:.2f} mm during the check")
    print(f"scan centre            : ({cx:+.4f}, {cy:+.4f}), plane {last['centre'][2]:.4f} m")
    chk(raw["rays"] >= 500, f"rays per scan          : {raw['rays']} (returns {raw['returns']})")
    chk(abs(raw["bearing_min_deg"] + 120) < 1.0 and abs(raw["bearing_max_deg"] - 120) < 1.0,
        f"field of view          : {raw['bearing_min_deg']:+.2f} .. {raw['bearing_max_deg']:+.2f} deg, "
        f"step {raw['bearing_step_deg']:.3f} deg")
    chk(abs(rate - 1.0) < 0.05, f"new scans per sim step : {rate:.3f}")
    chk(self_hits == 0, f"self hits (< {SELF_HIT_M} m)   : {self_hits}   (min range {raw['min_range_m']:.4f} m)")
    chk(raw["missed"] == 0 and raw["phantom"] == 0,
        f"missing / phantom      : {raw['missed']} / {raw['phantom']}   (edge rays excluded: {raw['edge_rays']})")
    chk(raw["max_plane_dev_m"] < 0.002, f"scan plane flatness    : max {1000 * raw['max_plane_dev_m']:.3f} mm")
    chk(rep < 0.001, f"repeatability          : max range change {1000 * rep:.3f} mm over {len(scans)} scans")
    chk(None, f"RAW range              : {rng(raw)}")
    chk(None, f"extrinsic (step 12d)   : forward {1000 * ext[0]:+.2f} mm, left {1000 * ext[1]:+.2f} mm, "
              f"yaw {math.degrees(ext[2]):+.3f} deg added to lidar_mount.json")
    chk(None, f"RAW mount (fit - mount): {mnt(raw)}")
    if cor is None:
        chk(False, "range calibration      : results/lidar_calibration.json missing -> run  python -m checks.step12d_lidar_calibration")
    else:
        chk(cor["ng_median_abs_m"] < MAX_MEDIAN_M and cor["ng_p95_abs_m"] < MAX_P95_M
            and cor["normal_p95_m"] < MAX_NORMAL_P95_M and abs(cor["ng_bias_m"]) < 0.001,
            f"CALIBRATED range       : {rng(cor)}")
        chk(abs(cor["mount_fwd_m"]) < MAX_MOUNT_M and abs(cor["mount_left_m"]) < MAX_MOUNT_M
            and abs(cor["mount_yaw_deg"]) < MAX_MOUNT_DEG, f"CALIBRATED mount       : {mnt(cor)}")
    print(f"verdict                : {'PASS' if ok else 'CHECK (see BAD lines)'}")
    print(f"logs                   : {RESULTS / 'lidar_scan.csv'}, {RESULTS / 'lidar_static.png'}")


def _figure(walls, sc, rays, path, maze):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(7, 7))
    for X, y0, y1 in walls.v:
        ax.plot([X, X], [y0, y1], color="0.2", lw=1.5)
    for Y, x0, x1 in walls.h:
        ax.plot([x0, x1], [Y, Y], color="0.2", lw=1.5)
    cx, cy = sc["centre"][0], sc["centre"][1]
    for r in rays[::8]:
        a = sc["fwd"] + r.bearing_rad
        L = r.range_m if r.range_m is not None else MAX_RANGE_M
        ax.plot([cx, cx + L * math.cos(a)], [cy, cy + L * math.sin(a)], color="tab:blue", lw=0.3, alpha=0.4)
    hx = [r.x_m for r in rays if r.range_m is not None]
    hy = [r.y_m for r in rays if r.range_m is not None]
    ax.scatter(hx, hy, s=3, color="tab:red", label="LiDAR returns", zorder=3)
    ax.plot(cx, cy, "k^", ms=8, label="scan centre")
    ax.set_aspect("equal")
    ax.set_xlim(cx - 3, cx + 3)
    ax.set_ylim(cy - 1, cy + 5)
    ax.legend(loc="upper right")
    ax.set_title(f"LiDAR scan at START, maze {maze} (walls = maze model)")
    ax.set_xlabel("x [m]")
    ax.set_ylabel("y [m]")
    fig.tight_layout()
    try:
        fig.savefig(path, dpi=130)
    except OSError:                       # the old picture is open in a viewer (Windows locks it)
        path = path.with_name(path.stem + "_new" + path.suffix)
        fig.savefig(path, dpi=130)
        print(f"(old figure is open in a viewer - saved as {path.name})")
    plt.close(fig)


if __name__ == "__main__":
    main()
