"""
Step 8c LIVE check: corridor guidance v2 in the T1 START corridor.

Four simulation runs, each from START (scene restarts):
    G0    no disturbance, guidance ON
    G+5   in-place +5 deg disturbance at START, guidance ON
    G-5   in-place -5 deg disturbance at START, guidance ON
    O+5   +5 deg disturbance, guidance OFF (open-loop reference)
After the disturbance the robot drives straight at 0.25 m/s until the ODOMETRY
distance reaches the ROOT centre (1.422634 m) and stops. The disturbance turn is
NOT reported to the guidance (no on_turn_started), so it must correct it.

All four lateral walls are visible only for the first ~0.55 m (then the
diagonals look into the ROOT opening -> HOLD on the learnt axis).

GT only for the report: y (lateral, left +) and alpha w.r.t. the START corridor
axis, at the FULL->HOLD switch and at the stop.

Offline expectation (step-8a mounts): G+-5 -> |y| ~ 1.5 mm, |alpha| ~ 0 deg at
ROOT; O+5 -> y ~ 117 mm, alpha ~ 4.7 deg.

    python -m checks.step08c_live_guidance     (from src/, scene T1, robot at START)
"""

from __future__ import annotations

import math

from checks.step07_live_turns import RESULTS as _R7, START_TO_ROOT_M, run_mission
from navigation.corridor_guidance import CorridorGuidance

RESULTS = _R7.parent / "step08"
RUNS = (("G0", 0.0, True), ("G+5", 5.0, True), ("G-5", -5.0, True), ("O+5", 5.0, False))

# acceptance at the ROOT stop (GT)
TOL_Y0_MM, TOL_A0_DEG = 3.0, 0.3          # undisturbed
TOL_Y5_MM, TOL_A5_DEG = 10.0, 0.5         # after a 5 deg disturbance


def make_hook(guided: bool):
    g = CorridorGuidance()

    def hook(mission, state, inc, v, w):
        g.update_odometry(inc.dtheta_rad)
        out = g.command(state.proximity)
        straight = mission.phase == "move" and mission.label == "straight"
        extra = {
            "g_mode": out.mode,
            "g_y_hat": out.y_hat_m, "g_alpha_hat": out.alpha_hat_rad,
            "g_omega": out.omega_ref_rad_s,
            "g_axis": g.axis, "g_theta_rel": g.theta_rel,
        }
        return (out.omega_ref_rad_s if (guided and straight) else None), extra
    return hook


def gt_pose(r, r0):
    dx, dy = r["gt_x"] - r0["gt_x"], r["gt_y"] - r0["gt_y"]
    th0 = r0["gt_th_unwrapped"]
    along = dx * math.cos(th0) + dy * math.sin(th0)
    lat = -dx * math.sin(th0) + dy * math.cos(th0)
    return along, lat, r["gt_th_unwrapped"] - th0


def main() -> None:
    RESULTS.mkdir(parents=True, exist_ok=True)
    report, ok_all = [], True
    for name, delta, guided in RUNS:
        prims = ([("turn", delta)] if delta else []) + [("straight", START_TO_ROOT_M)]
        csv_name = f"guidance_{name}.csv"
        _, rows = run_mission((), csv_name, primitives=prims, hook=make_hook(guided))
        (_R7 / csv_name).replace(RESULTS / csv_name)
        r0 = rows[0]
        drive = [r for r in rows if r["phase"] == "move:straight"]
        sw = next((r for r in drive if r["g_mode"] != "FULL"), None)
        end = rows[-1]
        _, y_sw, a_sw = gt_pose(sw, r0) if sw else (0, math.nan, math.nan)
        along_end, y_end, a_end = gt_pose(end, r0)
        coll = any(r["collision"] for r in rows)
        full_m = sum(1 for r in drive if r["g_mode"] == "FULL") * 0.01 * 0.25
        report.append((name, 1000 * y_sw, math.degrees(a_sw), 1000 * y_end,
                       math.degrees(a_end), along_end, full_m, coll))
        if guided:
            ty, ta = (TOL_Y0_MM, TOL_A0_DEG) if delta == 0 else (TOL_Y5_MM, TOL_A5_DEG)
            ok_all &= abs(1000 * y_end) <= ty and abs(math.degrees(a_end)) <= ta and not coll

    print("\nStep 8c live check (corridor guidance v2, T1 START corridor)")
    print("-------------------------------------------------------------")
    print(f"{'run':>4} | {'at FULL->HOLD':^20} | {'at ROOT stop':^20} | {'along':>6} {'FULL':>6} coll")
    print(f"{'':>4} | {'y [mm]':>9} {'a [deg]':>9}  | {'y [mm]':>9} {'a [deg]':>9}  | {'[m]':>6} {'[m]':>6}")
    for name, ys, as_, ye, ae, al, fm, c in report:
        print(f"{name:>4} | {ys:+9.2f} {as_:+9.3f}  | {ye:+9.2f} {ae:+9.3f}  | {al:6.3f} {fm:6.2f} {c}")
    o = report[-1]
    g5 = max(abs(report[1][3]), abs(report[2][3]))
    print(f"lateral error after 5 deg: guided {g5:.1f} mm vs open loop {abs(o[3]):.1f} mm "
          f"(x{abs(o[3]) / max(g5, 1e-3):.0f} smaller)")
    print(f"verdict: {'PASS' if ok_all else 'CHECK (see values)'}  "
          f"(G0: |y|<={TOL_Y0_MM} mm, |a|<={TOL_A0_DEG} deg; G+-5: |y|<={TOL_Y5_MM} mm, "
          f"|a|<={TOL_A5_DEG} deg; no collision)")
    print(f"logs: {RESULTS}")


if __name__ == "__main__":
    main()
