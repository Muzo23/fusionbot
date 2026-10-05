"""
Step 8c LIVE check: corridor guidance v2 in the T1 START corridor.

Six simulation runs, each from START (scene restarts):
    G0    no disturbance, guidance ON
    G+5   in-place +5 deg disturbance at START, guidance ON
    G-5   in-place -5 deg disturbance at START, guidance ON
    G+1   +1 deg (realistic: one turn residual is ~0.6 deg), guidance ON
    G-1   -1 deg, guidance ON
    O+5   +5 deg disturbance, guidance OFF (open-loop reference)
Guidance runs use the yaw-friction feed-forward (core/yaw_friction.py).

Live run 1 (without feed-forward) failed: G+-5 ended at |y| 12 mm, |alpha| 0.6 deg
because the robot barely follows small omega (castor/scrub stiction).
After the disturbance the robot drives straight at 0.25 m/s until the ODOMETRY
distance reaches the ROOT centre (1.422634 m) and stops. The disturbance turn is
NOT reported to the guidance (no on_turn_started), so it must correct it.

All four lateral walls are visible only for the first ~0.55 m (then the
diagonals look into the ROOT opening -> HOLD on the learnt axis).

GT only for the report: y (lateral, left +) and alpha w.r.t. the START corridor
axis, at the FULL->HOLD switch and at the stop.

Offline expectation (plant with the fitted yaw friction, feed-forward ON):
G+-5 -> |y| ~ 3 mm, |alpha| ~ 0.4 deg; G+-1 -> |y| ~ 4 mm, |alpha| < 0.1 deg;
O+5 -> y ~ 117 mm, alpha ~ 4.7 deg.

    python -m checks.step08c_live_guidance     (from src/, scene T1, robot at START)
"""

from __future__ import annotations

import math

from checks.step07_live_turns import RESULTS as _R7, START_TO_ROOT_M, run_mission
from core.yaw_friction import yaw_friction_feedforward
from navigation.corridor_guidance import CorridorGuidance

RESULTS = _R7.parent / "step08"
RUNS = (("G0", 0.0, True), ("G+5", 5.0, True), ("G-5", -5.0, True),
        ("G+1", 1.0, True), ("G-1", -1.0, True), ("O+5", 5.0, False))

# acceptance at the ROOT stop (GT)
TOL_Y0_MM, TOL_A0_DEG = 3.0, 0.3          # undisturbed
TOL_Y5_MM, TOL_A5_DEG = 10.0, 0.5         # after a 5 deg disturbance
TOL_Y1_MM, TOL_A1_DEG = 5.0, 0.3          # after a 1 deg disturbance


def make_hook(guided: bool):
    g = CorridorGuidance()

    def hook(mission, state, inc, v, w):
        g.update_odometry(inc.dtheta_rad)
        out = g.command(state.proximity)
        straight = mission.phase == "move" and mission.label == "straight"
        active = guided and straight
        extra = {
            "u_ff": yaw_friction_feedforward(out.omega_ref_rad_s) if active else 0.0,
            "g_mode": out.mode,
            "g_y_hat": out.y_hat_m, "g_alpha_hat": out.alpha_hat_rad,
            "g_omega": out.omega_ref_rad_s,
            "g_axis": g.axis, "g_theta_rel": g.theta_rel,
        }
        return (out.omega_ref_rad_s if active else None), extra
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
            ty, ta = {0.0: (TOL_Y0_MM, TOL_A0_DEG), 1.0: (TOL_Y1_MM, TOL_A1_DEG),
                      5.0: (TOL_Y5_MM, TOL_A5_DEG)}[abs(delta)]
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
          f"(G0: {TOL_Y0_MM} mm/{TOL_A0_DEG} deg; G+-1: {TOL_Y1_MM} mm/{TOL_A1_DEG} deg; "
          f"G+-5: {TOL_Y5_MM} mm/{TOL_A5_DEG} deg; no collision)")
    print(f"logs: {RESULTS}")


if __name__ == "__main__":
    main()
