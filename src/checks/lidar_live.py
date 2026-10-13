"""Shared live helper for the LiDAR checks (steps 12c / 12d).

grab_scans(sim, hok, n): starts the simulation, holds the robot 0.5 s in
lockstep (zero commands), releases the bridge, reads n consecutive scans in
client-stepping mode (every read between two sim.step() calls belongs to the
same simulation step) and stops the simulation. The robot does not move.
"""

from __future__ import annotations

import time

from sim.lidar_setup import COUNT_PROPERTY, XYZD_PROPERTY, _axle, _heading, scan_geometry

HOLD_S = 0.5


def _collect(sim, hok, n):
    scans, diag = [], []
    sim.setStepping(True)
    try:
        for _ in range(10 * n + 20):
            c = sim.getIntProperty(hok, COUNT_PROPERTY, {"noError": True})
            xyzd = sim.getFloatArrayProperty(hok, XYZD_PROPERTY, {"noError": True})
            t = sim.getSimulationTime()
            diag.append((t, c, None if xyzd is None else len(xyzd)))
            if c is not None and xyzd and (not scans or c != scans[-1]["count"]):   # new scans only (10 Hz)
                centre, fwd, tilt, _ = scan_geometry(sim, hok)
                scans.append(dict(count=c, t=t, xyzd=list(xyzd), m=list(sim.getObjectMatrix(hok, sim.handle_world)),
                                  centre=centre, fwd=fwd, tilt=tilt, axle=_axle(sim), heading=_heading(sim)))
                if len(scans) >= n:
                    break
            sim.step()
    finally:
        sim.setStepping(False)
    return scans, diag


def grab_scans(sim, hok, n):
    from sim.sim_utils import prepare_lockstep, wait_until_stopped
    from sim.transport import LockstepSession

    original_rt = prepare_lockstep(sim)
    link = LockstepSession()
    try:
        time.sleep(0.2)
        sim.startSimulation()
        while True:
            s = link.next_state(timeout_s=10.0)
            link.send(s.state_sequence, 0.0, 0.0)
            if s.simulation_time_s >= HOLD_S:
                break
        link.release()
        time.sleep(0.1)
        scans, diag = _collect(sim, hok, n)
    finally:
        link.release()
        sim.stopSimulation()
        wait_until_stopped(sim)
        link.close()
        sim.setBoolProperty(sim.handle_scene, "realtimeSimulation", original_rt)
    if not scans:
        print("\nNO LiDAR DATA. Reads (sim time, scan count, values in XYZD):")
        for t, c, k in diag[:5]:
            print(f"  t={t:.3f}  count={c}  xyzd={k}")
        names = [str(sim.getObjectAlias(h)) for h in sim.getObjectsInTree(hok, sim.handle_all, 0)]
        print(f"  objects under the Hokuyo: {names}")
        print("  -> look at the CoppeliaSim status bar (bottom) for a red Lua error of FusionBotLidarScan")
        raise SystemExit(f"NO LiDAR DATA on property {XYZD_PROPERTY}")
    return scans


def predicted_scan_pose(scan, mount, extrinsic=(0.0, 0.0, 0.0)):
    """Scan pose from the TRUE axle pose + the mount in results/lidar_mount.json
    (+ the extrinsic correction (forward m, left m, yaw rad) found by the step 12d calibration)."""
    import math
    ax, h = scan["axle"], scan["heading"]
    c, s = math.cos(h), math.sin(h)
    f, l_ = mount["x_forward_m"] + extrinsic[0], mount["y_left_m"] + extrinsic[1]
    return (ax[0] + c * f - s * l_, ax[1] + s * f + c * l_, h + mount["yaw_offset_rad"] + extrinsic[2])


def analyse(scan, walls, mount, model=None, max_range_m=5.0, extrinsic=(0.0, 0.0, 0.0)):
    """Raw (and corrected, if a RangeModel is given) evaluation + mount fit of one scan."""
    import math
    from perception.lidar_scan import apply_range_model, evaluate_scan, fit_scan_pose, scan_from_xyzd, scan_points
    rays = scan_from_xyzd(scan["xyzd"], scan["m"], scan["centre"], scan["fwd"], max_range_m)
    out = {"rays": rays}
    variants = [("raw", rays)]
    if model is not None:
        variants.append(("cor", apply_range_model(rays, scan["centre"], scan["fwd"], model)))
    pred = predicted_scan_pose(scan, mount, extrinsic)
    h = scan["heading"]
    c, s = math.cos(h), math.sin(h)
    for name, rr in variants:
        ev = evaluate_scan(rr, walls, scan["centre"], scan["fwd"], max_range_m, plane_z_m=scan["centre"][2])
        fx, fy, fth, rms, n = fit_scan_pose(scan_points(rr), walls, pred)
        ev["mount_fwd_m"] = c * (fx - pred[0]) + s * (fy - pred[1])
        ev["mount_left_m"] = -s * (fx - pred[0]) + c * (fy - pred[1])
        ev["mount_yaw_deg"] = math.degrees(math.remainder(fth - pred[2], 2 * math.pi))
        ev["mount_rms_m"], ev["mount_n"] = rms, n
        out[name] = ev
        out[name + "_rays"] = rr
    return out
