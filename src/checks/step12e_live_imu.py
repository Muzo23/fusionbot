"""
Step 12e LIVE check: the IMU in the IMU_phy housing.

The robot drives START -> ROOT (0.25 m/s) and turns +90, -90, +180, -180 deg in
place (the step-7 mission). The IMU script logs every sample (customData
.fusionbotImuLogPath); the bridge gives the true body yaw rate (GT). Compared:

  1. data flow: one sample per simulation step, valid from the 3rd step;
  2. gyro z  vs GT yaw rate (same step, no lag): must agree to < 1e-3 rad/s;
     gyro x, y (roll / pitch rate of a flat robot): small;
  3. accelerometer at rest: (0, 0, +9.81) m/s^2;
  4. accelerometer while driving vs the specific force expected at the IMU point
     from the GT axle trajectory:  a_x = du/dt - w^2 r,  a_y = u w + dw/dt r
     (r = IMU ahead of the axle), both smoothed over 50 ms;
  5. lag between gyro z and GT yaw rate (cross-correlation) = 0 steps.

    python -m checks.step12e_live_imu        (from src/, robot at START, simulation stopped)

Logs: results/step12e/imu_log.csv, imu_vs_gt.csv, imu_live.png
"""

from __future__ import annotations

import csv
import json
import math
import statistics as st
from pathlib import Path

ROOT = Path(__file__).resolve().parents[2]
RESULTS = ROOT / "results" / "step12e"
G = 9.81
SMOOTH = 5                       # samples (50 ms)


def _smooth(v, n=SMOOTH):
    out, h = [], n // 2
    for i in range(len(v)):
        w = v[max(0, i - h):i + h + 1]
        out.append(sum(w) / len(w))
    return out


def _rms(v):
    return math.sqrt(sum(x * x for x in v) / len(v)) if v else math.nan


def expected_specific_force(rows, r_fwd, dt=0.01):
    """GT axle trajectory -> expected (a_x, a_y) at a point r_fwd ahead of the axle, body frame."""
    n = len(rows)
    th = [r["gt_th_unwrapped"] for r in rows]
    u = [0.0] * n
    for i in range(1, n - 1):
        vx = (rows[i + 1]["gt_x"] - rows[i - 1]["gt_x"]) / (2 * dt)
        vy = (rows[i + 1]["gt_y"] - rows[i - 1]["gt_y"]) / (2 * dt)
        u[i] = vx * math.cos(th[i]) + vy * math.sin(th[i])
    w = [r["gt_w"] for r in rows]
    ax, ay = [0.0] * n, [0.0] * n
    for i in range(1, n - 1):
        du = (u[i + 1] - u[i - 1]) / (2 * dt)
        dw = (w[i + 1] - w[i - 1]) / (2 * dt)
        ax[i] = du - w[i] ** 2 * r_fwd
        ay[i] = u[i] * w[i] + dw * r_fwd
    return ax, ay


def main() -> None:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    from checks.step07_live_turns import run_mission
    from sim.imu_setup import LOG_PATH_PROPERTY

    mount = json.loads((ROOT / "results" / "imu_mount.json").read_text())
    RESULTS.mkdir(parents=True, exist_ok=True)
    log = RESULTS / "imu_log.csv"
    if log.exists():
        log.unlink()
    sim = RemoteAPIClient().require("sim")
    sim.setStringProperty(sim.handle_scene, LOG_PATH_PROPERTY, str(log).replace("\\", "/"))
    try:
        _, rows = run_mission(
            (90.0, -90.0, 180.0, -180.0), "step12e_imu_mission.csv",
            hook=lambda mission, s, inc, v, w: (None, {"gt_w": s.gt_yaw_rate_rad_s}))
    finally:
        sim.setStringProperty(sim.handle_scene, LOG_PATH_PROPERTY, "")
    if not log.exists():
        raise SystemExit(f"no IMU log written ({log}) - is FusionBotImuScript under /FusionBot/mainBase_phy/FusionBotIMU? "
                         "(python -m sim.imu_setup --apply, then Ctrl+S)")
    imu = {}
    with log.open() as f:
        for r in csv.DictReader(f):
            imu[round(float(r["t"]), 4)] = {k: float(v) for k, v in r.items()}

    # ---- join on simulation time
    joined, missing = [], 0
    for r in rows:
        m = imu.get(round(r["t"], 4))
        if m is None:
            missing += 1
            continue
        joined.append((r, m))
    rr = [j[0] for j in joined]
    ii = [j[1] for j in joined]
    gz = [m["gz"] for m in ii]
    gw = [r["gt_w"] for r in rr]
    dz = [a - b for a, b in zip(gz, gw)]
    gxy = [math.hypot(m["gx"], m["gy"]) for m in ii]

    # lag (steps) maximising the correlation gyro z <-> GT
    def corr_at(k):
        p = [(gz[i + k], gw[i]) for i in range(max(0, -k), min(len(gz), len(gz) - k))]
        return sum(a * b for a, b in p)
    lag = max(range(-3, 4), key=corr_at)

    all_imu = sorted(imu.values(), key=lambda m: m["t"])

    # moving: expected specific force from GT
    ex, ey = expected_specific_force(rr, mount["x_forward_m"])
    mx, my = _smooth([m["ax"] for m in ii]), _smooth([m["ay"] for m in ii])
    ex, ey = _smooth(ex), _smooth(ey)
    sel = range(3, len(rr) - 3)
    # rest = robot really standing: before the start and in the settle phases with no expected
    # acceleration (a zero command alone is not rest - the robot may still be braking)
    rest = [m for m in all_imu if 0.3 <= m["t"] <= 0.5 and m["valid"] >= 2]
    rest += [m for i, (r, m) in enumerate(joined) if r["phase"].startswith("settle")
             and abs(ex[i]) < 0.02 and abs(ey[i]) < 0.02]
    rest_z = st.mean(m["az"] for m in rest) if rest else math.nan
    rest_xy = [math.hypot(m["ax"], m["ay"]) for m in rest]
    gxy_rest = [math.hypot(m["gx"], m["gy"]) for r, m in joined if r["phase"].startswith("settle")]
    err_x = [mx[i] - ex[i] for i in sel]
    err_y = [my[i] - ey[i] for i in sel]
    peak_y = max(abs(ey[i]) for i in sel)

    with (RESULTS / "imu_vs_gt.csv").open("w", newline="") as f:
        w = csv.writer(f)
        w.writerow(["t", "phase", "gyro_z", "gt_yaw_rate", "gyro_x", "gyro_y", "acc_x", "acc_y", "acc_z",
                    "acc_x_s", "acc_y_s", "exp_x_s", "exp_y_s"])
        for i, (r, m) in enumerate(joined):
            w.writerow([f"{r['t']:.3f}", r["phase"], m["gz"], r["gt_w"], m["gx"], m["gy"], m["ax"], m["ay"],
                        m["az"], mx[i], my[i], ex[i], ey[i]])
    try:
        _figure(rr, gz, gw, mx, my, ex, ey, RESULTS / "imu_live.png")
    except Exception as exc:                                  # noqa: BLE001
        print(f"(figure not drawn: {exc})")

    ok = True

    def chk(cond, text):
        nonlocal ok
        ok &= bool(cond)
        print(f"  [{'OK ' if cond else 'BAD'}] {text}")

    print(f"\nStep 12e IMU live check ({len(rows)} control steps, {len(imu)} IMU samples)")
    print("------------------------------------------------------------")
    print(f"IMU mount              : {mount['x_forward_m']:+.4f} m ahead, {mount['y_left_m']:+.4f} m left, "
          f"{mount['z_above_floor_m']:.4f} m high; runs first in the step: {mount.get('runs_first')}")
    chk(missing == 0, f"one sample per step    : {len(joined)} joined, {missing} control steps without an IMU sample")
    chk(max(abs(x) for x in dz) < 1e-3,
        f"gyro z vs GT yaw rate  : max |diff| {max(abs(x) for x in dz):.2e} rad/s, rms {_rms(dz):.2e} "
        f"(peak rate {max(abs(x) for x in gw):.3f} rad/s)")
    chk(lag == 0, f"lag gyro vs GT         : {lag} steps")
    # roll/pitch rate: ~0 at rest; while moving the body rocks on its contacts (real motion, not
    # a sensor error - the planar EKF uses only gyro z)
    chk(max(gxy_rest) < 0.002 and max(gxy) < 0.1,
        f"gyro x/y (roll/pitch)  : at rest max {max(gxy_rest):.5f} rad/s; moving rms {_rms(gxy):.4f}, "
        f"max {max(gxy):.4f} rad/s (body rocking)")
    chk(abs(rest_z - G) < 0.05 and st.mean(rest_xy) < 0.02 and max(rest_xy) < 0.1,
        f"accel at rest          : z {rest_z:.4f} m/s^2 (g = {G}), |xy| mean {st.mean(rest_xy):.4f}, "
        f"max {max(rest_xy):.4f} m/s^2 ({len(rest)} samples)")
    chk(_rms(err_x) < 0.05 and _rms(err_y) < 0.05,
        f"accel vs GT (50 ms)    : rms error x {_rms(err_x):.4f}, y {_rms(err_y):.4f} m/s^2 "
        f"(peak expected lateral {peak_y:.3f} m/s^2)")
    print(f"verdict                : {'PASS' if ok else 'CHECK (see BAD lines)'}")
    print(f"logs                   : {log}, {RESULTS / 'imu_vs_gt.csv'}, {RESULTS / 'imu_live.png'}")


def _figure(rr, gz, gw, mx, my, ex, ey, path):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    t = [r["t"] for r in rr]
    fig, ax = plt.subplots(3, 1, figsize=(10, 8), sharex=True)
    ax[0].plot(t, gw, "k", lw=2.5, alpha=0.35, label="GT yaw rate")
    ax[0].plot(t, gz, "tab:blue", lw=1, label="IMU gyro z")
    ax[0].set_ylabel("rad/s")
    ax[0].legend(loc="upper right")
    ax[1].plot(t, ex, "k", lw=2.5, alpha=0.35, label="expected from GT")
    ax[1].plot(t, mx, "tab:orange", lw=1, label="IMU accel x (forward)")
    ax[1].set_ylabel("m/s$^2$")
    ax[1].legend(loc="upper right")
    ax[2].plot(t, ey, "k", lw=2.5, alpha=0.35, label="expected from GT")
    ax[2].plot(t, my, "tab:green", lw=1, label="IMU accel y (left)")
    ax[2].set_ylabel("m/s$^2$")
    ax[2].set_xlabel("simulation time [s]")
    ax[2].legend(loc="upper right")
    fig.suptitle("IMU (FusionBotIMU in IMU_phy) vs ground truth, 50 ms smoothing for acceleration")
    fig.tight_layout()
    try:
        fig.savefig(path, dpi=130)
    except OSError:
        path = path.with_name(path.stem + "_new" + path.suffix)
        fig.savefig(path, dpi=130)
    plt.close(fig)


if __name__ == "__main__":
    main()
