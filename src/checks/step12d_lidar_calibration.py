"""
Step 12d LIVE: range calibration of the simulated LiDAR.

Step 12c showed that the simulated Hokuyo returns ranges that are slightly too
long, by an amount that grows with range (about +1.4 mm at 0.4 m, +10 mm at
1.7 m), deterministic and identical in every scan. Such a systematic error is
calibrated, not filtered:

  1. the robot is placed (simulation stopped) at junction centres of the maze,
     in the four directions, chosen so that the wall straight ahead lies at
     ranges spread over 0.2 .. 4.9 m;
  2. at every pose a scan is taken and every non-grazing ray (incidence >= 30
     deg, not at a wall end) is compared with the ray cast through the true
     walls (GT used for calibration only);
  3. the error model  e(r) = a r + b r^2  is fitted by least squares over all
     poses and saved to results/lidar_calibration.json;
  4. verification: residuals before / after the correction per range bin and
     per bearing sector, and the mount fit (scan-to-wall alignment vs
     lidar_mount.json) at every pose, raw and corrected.

The robot is put back to START at the end.

    python -m checks.step12d_lidar_calibration     (from src/, simulation stopped; T3 has the longest corridors)

Logs: results/step12d/calibration_poses.csv, results/lidar_calibration.json, lidar_range_error.png
"""

from __future__ import annotations

import argparse
import csv
import json
import math
import statistics as st
from pathlib import Path

from checks.lidar_live import analyse, grab_scans
from mazes.maze_geometry import WallMap
from mazes.maze_spec import MAZES
from perception.lidar_scan import GRAZING_SIN, fit_range_model, robust_inliers
from sim.lidar_setup import MAX_RANGE_M, find_hokuyo
from sim.maze_scene import place_robot, scene_maze_id

ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "results" / "step12d"
CALIBRATION = ROOT / "results" / "lidar_calibration.json"
N_POSES = 10
POSE_DEV_M, POSE_DEV_DEG = 0.003, 0.2
BINS = ((0.0, 0.5), (0.5, 1.0), (1.0, 2.0), (2.0, 3.0), (3.0, 5.0))


def candidate_poses(spec):
    """Junction centres in 4 directions + corridor centrelines every 0.25 m, facing along the corridor."""
    out = []
    for name, (x, y) in spec.nodes_m.items():
        out += [(name, (x, y, k * math.pi / 2)) for k in range(4)]
    for a, b in spec.edges:
        (xa, ya), (xb, yb) = spec.nodes_m[a], spec.nodes_m[b]
        L = math.hypot(xb - xa, yb - ya)
        h = math.atan2(yb - ya, xb - xa)
        for i in range(1, int(L / 0.25)):
            t = i * 0.25 / L
            x, y = xa + t * (xb - xa), ya + t * (yb - ya)
            out += [(f"{a}{b}{i}", (x, y, h)), (f"{a}{b}{i}", (x, y, h + math.pi))]
    return out


def choose_poses(spec, walls, mount, n):
    """Poses whose wall straight ahead is spread evenly over the available ranges; START always included."""
    cands = []
    for name, (x, y, h) in candidate_poses(spec):
        h = math.remainder(h, 2 * math.pi)
        cx, cy = x + mount["x_forward_m"] * math.cos(h), y + mount["x_forward_m"] * math.sin(h)
        r = walls.raycast(cx, cy, h, MAX_RANGE_M * 0.98)
        if math.isfinite(r):
            cands.append((r, name, (x, y, h)))
    cands.sort()
    chosen = []
    if cands:
        lo, hi = cands[0][0], cands[-1][0]
        for i in range(n - 1):
            target = lo + (hi - lo) * i / max(1, n - 2)
            best = min((c for c in cands if c not in chosen), key=lambda c: abs(c[0] - target))
            chosen.append(best)
    sx, sy, sh = spec.start_pose
    chosen.append((math.nan, "START", (sx, sy, sh)))
    return chosen


def main() -> None:
    ap = argparse.ArgumentParser()
    ap.add_argument("--maze", default=None, help="default: the maze id stored in the scene")
    ap.add_argument("--poses", type=int, default=N_POSES)
    args = ap.parse_args()
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    sim = RemoteAPIClient().require("sim")
    maze = args.maze or scene_maze_id(sim) or "T1"
    spec = MAZES[maze]
    walls = WallMap(spec)
    mount = json.loads((ROOT / "results" / "lidar_mount.json").read_text())
    hok = find_hokuyo(sim)
    if sim.getSimulationState() != sim.simulation_stopped:
        raise SystemExit("stop the simulation first")

    poses = choose_poses(spec, walls, mount, args.poses)
    print(f"\nStep 12d LiDAR range calibration (maze {maze}, {len(poses)} poses)")
    data = []
    try:
        for i, (r_front, name, (x, y, h)) in enumerate(poses):
            place_robot(sim, x, y, h)
            scan = grab_scans(sim, hok, 3)[-1]
            data.append((name, h, r_front, scan))
            print(f"  pose {i + 1:2d}/{len(poses)}: {name:>7} heading {math.degrees(h):6.1f} deg, "
                  f"wall ahead {r_front:5.2f} m")
    finally:
        place_robot(sim, *spec.start_pose)

    # ---- fit over all poses
    samples = []
    RESULTS.mkdir(parents=True, exist_ok=True)
    with (RESULTS / "calibration_rays.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pose", "heading_deg", "bearing_deg", "true_range_m", "sin_incidence", "residual_m"])
        for name, h, r_front, scan in data:
            smp = analyse(scan, walls, mount, None, MAX_RANGE_M)["raw"]["samples"]
            samples += smp
            for e, si, res, b in smp:
                w.writerow([name, f"{math.degrees(h):.1f}", f"{math.degrees(b):.3f}", f"{e:.5f}", f"{si:.4f}", f"{res:.6f}"])
    model, n = fit_range_model(samples)
    # statistics over the inliers the model was fitted on (mixed-pixel outliers reported separately)
    ng_all = [s for s in samples if s[1] >= GRAZING_SIN]
    mask, thr = robust_inliers([s[2] - model.error(s[0]) for s in ng_all])
    ng = [s for s, keep in zip(ng_all, mask) if keep]
    n_out = len(ng_all) - len(ng)
    r_max = max(s[0] for s in ng)

    def bin_stats(res_of):
        rows = []
        for lo, hi in BINS:
            v = [res_of(s) for s in ng if lo <= s[0] < hi]
            rows.append((lo, hi, len(v), st.median(v) if v else math.nan,
                         sorted(abs(x) for x in v)[int(0.95 * len(v))] if v else math.nan))
        return rows

    before = bin_stats(lambda s: s[2])
    after = bin_stats(lambda s: model.correct(s[0] + s[2]) - s[0])

    # ---- per pose verification
    RESULTS.mkdir(parents=True, exist_ok=True)
    per_pose = []
    with (RESULTS / "calibration_poses.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["pose", "heading_deg", "wall_ahead_m", "raw_ng_median_mm", "cor_ng_median_mm", "cor_ng_p95_mm",
                    "raw_fwd_mm", "raw_left_mm", "raw_yaw_deg", "cor_fwd_mm", "cor_left_mm", "cor_yaw_deg"])
        for name, h, r_front, scan in data:
            a = analyse(scan, walls, mount, model, MAX_RANGE_M)
            raw, cor = a["raw"], a["cor"]
            per_pose.append((name, h, r_front, raw, cor))
            w.writerow([name, f"{math.degrees(h):.1f}", f"{r_front:.3f}",
                        f"{1000 * raw['ng_median_abs_m']:.2f}", f"{1000 * cor['ng_median_abs_m']:.2f}",
                        f"{1000 * cor['ng_p95_abs_m']:.2f}",
                        f"{1000 * raw['mount_fwd_m']:+.2f}", f"{1000 * raw['mount_left_m']:+.2f}",
                        f"{raw['mount_yaw_deg']:+.3f}", f"{1000 * cor['mount_fwd_m']:+.2f}",
                        f"{1000 * cor['mount_left_m']:+.2f}", f"{cor['mount_yaw_deg']:+.3f}"])

    # bearing sectors after correction
    sectors = []
    for lo in range(-120, 120, 30):
        v = [model.correct(s[0] + s[2]) - s[0] for s in ng if lo <= math.degrees(s[3]) < lo + 30]
        sectors.append((lo, len(v), st.median(v) if v else math.nan))

    # extrinsic correction: mean scan-to-wall offset of the calibrated scans over all poses
    # (effective scan origin / heading of the sensor w.r.t. lidar_mount.json)
    ext = tuple(st.mean(p_[4][k] for p_ in per_pose) for k in ("mount_fwd_m", "mount_left_m", "mount_yaw_deg"))
    ext_sd = tuple(st.pstdev(p_[4][k] for p_ in per_pose) for k in ("mount_fwd_m", "mount_left_m", "mount_yaw_deg"))

    rms_b = math.sqrt(sum(s[2] ** 2 for s in ng) / len(ng))
    rms_a = math.sqrt(sum((model.correct(s[0] + s[2]) - s[0]) ** 2 for s in ng) / len(ng))
    CALIBRATION.write_text(json.dumps({
        "model": "measured = true + a*r + b*r^2 (r in m), correction r_true = RangeModel.correct(r_measured)",
        "a": model.a, "b": model.b, "maze": maze, "poses": len(data), "non_grazing_rays": n,
        "calibrated_range_max_m": r_max, "rms_before_m": rms_b, "rms_after_m": rms_a,
        "extrinsic": "effective scan pose = lidar_mount.json + (forward, left, yaw) below",
        "extrinsic_forward_m": ext[0], "extrinsic_left_m": ext[1], "extrinsic_yaw_rad": math.radians(ext[2]),
        "extrinsic_sd": {"forward_m": ext_sd[0], "left_m": ext_sd[1], "yaw_deg": ext_sd[2]},
    }, indent=1))
    try:
        _figure(ng, model, RESULTS / "lidar_range_error.png")
    except Exception as exc:                                  # noqa: BLE001
        print(f"(figure not drawn: {exc})")

    # ---- report
    ok = True
    print("--------------------------------------------------------------")
    print(f"model e(r) = a r + b r^2 : a = {1000 * model.a:+.3f} mm/m, b = {1000 * model.b:+.3f} mm/m^2 "
          f"({n} non-grazing rays, ranges up to {r_max:.2f} m; {n_out} outliers > {1000 * thr:.1f} mm rejected)")
    print(f"  e(0.5 m) = {1000 * model.error(0.5):+.1f} mm, e(1 m) = {1000 * model.error(1):+.1f} mm, "
          f"e(2 m) = {1000 * model.error(2):+.1f} mm, e(4 m) = {1000 * model.error(4):+.1f} mm")
    print(f"rms over all non-grazing rays: before {1000 * rms_b:.2f} mm, after {1000 * rms_a:.2f} mm")
    print("range bin      n   median before   median after   p95 |e| after")
    for (lo, hi, k, mb, _), (_, _, _, ma, pa) in zip(before, after):
        print(f"  {lo:.1f}-{hi:.1f} m {k:5d}   {1000 * mb:+9.2f} mm   {1000 * ma:+9.2f} mm   {1000 * pa:9.2f} mm")
    print("bearing sector (after)   " + "  ".join(f"{lo:+4d}:{1000 * m:+.1f}" for lo, k, m in sectors if k))
    print("pose        wall ahead   raw mount fwd/left/yaw          calibrated mount fwd/left/yaw   cal. median/p95")
    worst = [0.0, 0.0, 0.0]          # deviation of each pose from the mean (extrinsic) correction
    for name, h, r_front, raw, cor in per_pose:
        print(f"  {name:>7} {math.degrees(h):5.0f}  {r_front:6.2f} m   "
              f"{1000 * raw['mount_fwd_m']:+6.2f} {1000 * raw['mount_left_m']:+6.2f} mm {raw['mount_yaw_deg']:+.3f} deg   "
              f"{1000 * cor['mount_fwd_m']:+6.2f} {1000 * cor['mount_left_m']:+6.2f} mm {cor['mount_yaw_deg']:+.3f} deg   "
              f"{1000 * cor['ng_median_abs_m']:.2f}/{1000 * cor['ng_p95_abs_m']:.2f} mm")
        worst = [max(worst[0], abs(cor["mount_fwd_m"] - ext[0])), max(worst[1], abs(cor["mount_left_m"] - ext[1])),
                 max(worst[2], abs(cor["mount_yaw_deg"] - ext[2]))]
    all_med = max(abs(m) for _, _, k, m, _ in after if k)
    c1 = all_med < 0.002
    c2 = abs(ext[0]) < 0.005 and abs(ext[1]) < 0.005 and abs(ext[2]) < 0.5
    # per-pose scatter = fit noise: the forward offset of one scan rests on the few rays that hit
    # walls ahead (often 2-5 m away, 1.4 mm range noise) -> 3 mm; still 10x below the real sensor spec
    c3 = worst[0] < POSE_DEV_M and worst[1] < POSE_DEV_M and worst[2] < POSE_DEV_DEG
    print(f"extrinsic correction (mean over poses): forward {1000 * ext[0]:+.2f} +- {1000 * ext_sd[0]:.2f} mm, "
          f"left {1000 * ext[1]:+.2f} +- {1000 * ext_sd[1]:.2f} mm, yaw {ext[2]:+.3f} +- {ext_sd[2]:.3f} deg")
    print(f"  [{'OK ' if c1 else 'BAD'}] calibrated median per range bin within 2 mm (worst {1000 * all_med:.2f} mm)")
    print(f"  [{'OK ' if c2 else 'BAD'}] extrinsic correction small (< 5 mm / 0.5 deg): the mount json is right")
    print(f"  [{'OK ' if c3 else 'BAD'}] every pose within {1000 * POSE_DEV_M:.0f} mm / {POSE_DEV_DEG} deg of it "
          f"(worst {1000 * worst[0]:.2f} / {1000 * worst[1]:.2f} mm, {worst[2]:.3f} deg)")
    ok = c1 and c2 and c3
    print(f"verdict: {'PASS' if ok else 'CHECK'}")
    print(f"saved  : {CALIBRATION}")
    print(f"logs   : {RESULTS / 'calibration_poses.csv'}, {RESULTS / 'lidar_range_error.png'}")
    print("robot is back at START (Ctrl+S not needed)")


def _figure(ng, model, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(1, 2, figsize=(11, 4.2))
    r = [s[0] for s in ng]
    e = [1000 * s[2] for s in ng]
    ec = [1000 * (model.correct(s[0] + s[2]) - s[0]) for s in ng]
    ax[0].scatter(r, e, s=2, alpha=0.4, label="measured - true")
    rr = [i * 0.05 for i in range(0, 101)]
    ax[0].plot(rr, [1000 * model.error(x) for x in rr], "r", label="model a r + b r$^2$")
    ax[0].set_xlabel("true range [m]")
    ax[0].set_ylabel("range error [mm]")
    ax[0].set_title("raw (non-grazing rays)")
    ax[0].legend()
    ax[1].scatter(r, ec, s=2, alpha=0.4, color="tab:green")
    ax[1].axhline(0, color="k", lw=0.8)
    ax[1].set_xlabel("true range [m]")
    ax[1].set_ylabel("range error [mm]")
    ax[1].set_title("after calibration")
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
