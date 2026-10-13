"""
Step 13b LIVE check: protocol 2.1 - the IMU and the LiDAR arrive in the bridge packet.

The robot drives the step-7 mission (START -> ROOT, turns +90, -90, +180, -180).
Every state packet is inspected:

  1. IMU block in EVERY packet, the IMU count grows by 1 per packet;
  2. gyro z from the packet == GT yaw rate of the same packet (< 1e-3 rad/s);
  3. accelerometer in the packet, standstill value printed (full check: step 12e);
  4. LiDAR scan attached to every 10th packet (10 Hz) with 684 rays, no packet
     in between carries a scan;
  5. the LiDAR scan is FRESH: (LiDAR count - IMU count) is the same in every
     packet with a scan (both scripts ran in the same step, before the bridge);
  6. the scan is plausible: >= 90 % of the rays hit a wall inside 5 m, the
     shortest return >= 0.05 m;
  7. the step-7 turn table again (13b-4: did grouping into mainBase_phy change
     the turn behaviour?).

    python -m checks.step13b_live_sensors      (from src/, robot at START, simulation stopped)

Log: results/step13b/packets.csv (one line per packet, scan size, counts, gyro, GT)
"""

from __future__ import annotations

import csv
import math
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "results" / "step13b"
LIDAR_DIVIDER = 10
LIDAR_RAYS = 684
LIDAR_MAX_M = 5.0
G = 9.81


def main() -> None:
    from checks.step07_live_turns import TURNS_DEG, run_mission, turn_table

    packets = []

    def hook(mission, s, inc, v, w):
        n = len(s.lidar_xyzd) // 4 if s.lidar_xyzd else 0
        d = [s.lidar_xyzd[4 * k + 3] for k in range(n)] if n else []
        packets.append({
            "t": s.simulation_time_s, "seq": s.state_sequence,
            "imu_count": s.imu_count, "lidar_count": s.lidar_count, "rays": n,
            "gz": s.imu_gyro_rad_s[2] if s.imu_gyro_rad_s else math.nan,
            "gt_w": s.gt_yaw_rate_rad_s,
            "ax": s.imu_accel_m_s2[0] if s.imu_accel_m_s2 else math.nan,
            "ay": s.imu_accel_m_s2[1] if s.imu_accel_m_s2 else math.nan,
            "az": s.imu_accel_m_s2[2] if s.imu_accel_m_s2 else math.nan,
            "hits": sum(1 for x in d if 0 < x < 0.9999 * LIDAR_MAX_M),
            "d_min": min((x for x in d if x > 0), default=math.nan),
            "phase": f"{mission.phase}:{mission.label}",
        })
        return None, {}

    results, rows = run_mission(TURNS_DEG, "step13b_mission.csv", hook=hook)
    RESULTS.mkdir(parents=True, exist_ok=True)
    with (RESULTS / "packets.csv").open("w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(packets[0]))
        w.writeheader()
        w.writerows(packets)

    ok = True

    def chk(cond, text):
        nonlocal ok
        ok &= bool(cond)
        print(f"  [{'OK ' if cond else 'BAD'}] {text}")

    P = packets
    print(f"\nStep 13b live check, protocol 2.1 ({len(P)} control packets, {P[-1]['t']:.2f} s simulated)")
    print("------------------------------------------------------------------------")
    no_imu = sum(1 for p in P if p["imu_count"] < 0)
    if no_imu == len(P) and all(p["lidar_count"] < 0 for p in P):
        print("  no extension block in the packets - the old (2.0) bridge is still in the scene:")
        print("  python -m sim.install_bridge, then Ctrl+S, and run again")
        raise SystemExit(1)
    jumps = [b["imu_count"] - a["imu_count"] for a, b in zip(P, P[1:])]
    chk(no_imu == 0 and all(j == 1 for j in jumps),
        f"IMU in every packet    : {len(P) - no_imu}/{len(P)} packets, count step "
        f"min {min(jumps)} max {max(jumps)} (expected 1)")
    dz = [p["gz"] - p["gt_w"] for p in P]
    chk(max(abs(x) for x in dz) < 1e-3,
        f"gyro z vs GT yaw rate  : max |diff| {max(abs(x) for x in dz):.2e} rad/s "
        f"(peak rate {max(abs(p['gt_w']) for p in P):.3f} rad/s)")
    # rest: settle phases with the body not turning any more (|GT yaw rate| < 1e-4 rad/s)
    rest = [p for p in P if p["phase"].startswith(("idle", "settle")) and abs(p["gt_w"]) < 1e-4] or P[:1]
    rz = sum(p["az"] for p in rest) / len(rest)
    rxy = max(math.hypot(p["ax"], p["ay"]) for p in rest)
    print(f"  [inf] accel at standstill: z {rz:.4f} m/s^2 (g = {G}), max |xy| {rxy:.4f} m/s^2 "
          f"({len(rest)} packets; full accelerometer check = step 12e)")

    scans = [p for p in P if p["rays"] > 0]
    gaps = [b["seq"] - a["seq"] for a, b in zip(scans, scans[1:])]
    sizes = sorted({p["rays"] for p in scans})
    expected = len(P) / LIDAR_DIVIDER
    chk(scans and sizes == [LIDAR_RAYS] and all(g == LIDAR_DIVIDER for g in gaps)
        and abs(len(scans) - expected) <= 1,
        f"LiDAR at 10 Hz         : {len(scans)} scans in {len(P)} packets (expected ~{expected:.0f}), "
        f"gap {min(gaps, default=0)}..{max(gaps, default=0)} packets, rays per scan {sizes}")
    # scan k is rendered in step 10 (k - 1) + 1 (10 Hz, step 13b-7): IMU count - 10 * (scan count - 1) is the same
    # for every packet that carries a scan if the scan comes from the same step as the IMU sample
    offs = sorted({p["imu_count"] - LIDAR_DIVIDER * (p["lidar_count"] - 1) for p in scans})
    chk(len(offs) == 1,
        f"LiDAR fresh (same step): IMU count - 10 x (scan count - 1) = {offs} (one value = in sync)")
    hit = min(p["hits"] / p["rays"] for p in scans) if scans else 0
    dmin = min(p["d_min"] for p in scans) if scans else math.nan
    chk(hit >= 0.9 and dmin >= 0.05,
        f"scan plausible         : worst scan {100 * hit:.1f} % hits, shortest return {dmin:.3f} m")
    print(f"verdict                : {'PASS' if ok else 'CHECK (see BAD lines)'}")

    print("\n13b-4  turns after grouping into mainBase_phy (compare with step 7 / 7b):")
    turn_table(results)
    coll = any(r["collision"] for r in rows)
    print(f"collision: {coll}")
    print(f"log      : {RESULTS / 'packets.csv'}")


if __name__ == "__main__":
    main()
