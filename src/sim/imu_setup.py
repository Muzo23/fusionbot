"""
IMU setup (step 12e): puts a 6-axis IMU (gyro + accelerometer) into the robot's
IMU housing (shape IMU_phy) and installs its script. Simulation must be STOPPED.

    python -m sim.imu_setup              dry run: prints the plan
    python -m sim.imu_setup --apply      performs it (then Ctrl+S in CoppeliaSim)

What --apply does
-----------------
1. creates the dummy FusionBotIMU (child of the moving body mainBase_phy) at the centre of the
   IMU_phy housing, axes = robot axes (x forward, y left, z up);
2. installs the script FusionBotImuScript under it. Every simulation step
   (sysCall_sensing, i.e. after the physics step) it measures the motion of its
   own frame from the poses of the last three steps:
     gyro  [rad/s]  : body-frame angular velocity from R(k-1)^T R(k)
     accel [m/s^2]  : specific force f = R(k)^T (a_world - g), a_world from the
                      second difference of the position (at rest f = (0, 0, +9.81))
   and publishes on the dummy
     signal.fusionbotImuGyro   (3 floats)    signal.fusionbotImuAccel (3 floats)
     signal.fusionbotImuCount  (int, increments every step)
   These are the IDEAL (noise-free) values of the simulated sensor; the noise
   model of a real MEMS IMU is added in Python (step 13), like for the LiDAR.
   If the scene string customData.fusionbotImuLogPath is set, the script also
   writes every sample to that CSV when the simulation stops (live check 12e).
3. writes the mount (relative to the axle frame) to results/imu_mount.json.

The script asks to run FIRST in each step, so that the bridge (step 13) reads
the sample of the same step.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from core import robot_config as rc
from sim.lidar_setup import _alias, _axle, _children, _heading, shape_boxes

IMU_ALIAS = "FusionBotIMU"
SCRIPT_ALIAS = "FusionBotImuScript"
HOUSING_ALIAS = "IMU_phy"
GYRO_PROPERTY = "signal.fusionbotImuGyro"
ACCEL_PROPERTY = "signal.fusionbotImuAccel"
COUNT_PROPERTY = "signal.fusionbotImuCount"
LOG_PATH_PROPERTY = "customData.fusionbotImuLogPath"
GRAVITY_M_S2 = 9.81
RESULTS = Path(__file__).resolve().parents[2] / "results"

IMU_SCRIPT = f"""sim=require'sim'
-- FusionBot IMU script (installed by sim/imu_setup.py, step 12e)
local G={GRAVITY_M_S2}

local function mat(h) return sim.getObjectMatrix(h,sim.handle_world) end

function sysCall_init()
    self=sim.getObject('..')
    hist={{}}            -- last three {{t, matrix}}
    count=0
    logPath=sim.getStringProperty(sim.handle_scene,'{LOG_PATH_PROPERTY}',{{noError=true}})
    logRows={{}}
end

function sysCall_sensing()
    local t=sim.getSimulationTime()
    local m=mat(self)
    table.insert(hist,1,{{t,m}})
    if #hist>3 then table.remove(hist) end
    local gyro={{0,0,0}}
    local acc={{0,0,G}}
    local valid=0
    if #hist>=2 then
        local dt=hist[1][1]-hist[2][1]
        if dt>0 then
            local a,b=hist[2][2],hist[1][2]          -- R(k-1), R(k): columns = axes
            -- Rrel = R(k-1)^T R(k); omega_body = vee(Rrel - Rrel^T)/2/dt
            local function R(M,i,j) return M[4*(i-1)+j] end
            local Rr={{}}
            for i=1,3 do Rr[i]={{}} for j=1,3 do
                local s=0 for k=1,3 do s=s+R(a,k,i)*R(b,k,j) end Rr[i][j]=s end end
            gyro={{(Rr[3][2]-Rr[2][3])/(2*dt),(Rr[1][3]-Rr[3][1])/(2*dt),(Rr[2][1]-Rr[1][2])/(2*dt)}}
            valid=1
            if #hist>=3 then
                local c=hist[3][2]
                local dt2=hist[2][1]-hist[3][1]
                if dt2>0 then
                    local aw={{}}
                    for i=1,3 do
                        local p0,p1,p2=c[4*i],a[4*i],b[4*i]
                        aw[i]=((p2-p1)/dt-(p1-p0)/dt2)/(0.5*(dt+dt2))
                    end
                    aw[3]=aw[3]+G                          -- specific force: a - g, g=(0,0,-G)
                    acc={{0,0,0}}
                    for j=1,3 do for i=1,3 do acc[j]=acc[j]+R(b,i,j)*aw[i] end end
                    valid=2
                end
            end
        end
    end
    count=count+1
    sim.setFloatArrayProperty(self,'{GYRO_PROPERTY}',gyro)
    sim.setFloatArrayProperty(self,'{ACCEL_PROPERTY}',acc)
    sim.setIntProperty(self,'{COUNT_PROPERTY}',count)
    if logPath and logPath~='' then
        logRows[#logRows+1]=string.format('%.6f,%d,%.9g,%.9g,%.9g,%.9g,%.9g,%.9g',
            t,valid,gyro[1],gyro[2],gyro[3],acc[1],acc[2],acc[3])
    end
end

function sysCall_cleanup()
    if logPath and logPath~='' and #logRows>0 then
        local f=io.open(logPath,'w')
        if f then
            f:write('t,valid,gx,gy,gz,ax,ay,az\\n')
            f:write(table.concat(logRows,'\\n'))
            f:write('\\n')
            f:close()
        end
    end
end
"""


def _set_first(sim, script) -> bool:
    """Ask CoppeliaSim to run this script first in every step (API names differ between versions)."""
    for setter in (
        lambda: sim.setObjectInt32Param(script, sim.scriptintparam_execpriority, sim.scriptexecorder_first),
        lambda: sim.setIntProperty(script, "executionPriority", sim.scriptexecorder_first),
        lambda: sim.setIntProperty(script, "execPriority", sim.scriptexecorder_first),
    ):
        try:
            setter()
            return True
        except Exception:                              # noqa: BLE001
            continue
    return False


def main() -> None:
    ap = argparse.ArgumentParser(description="Mount the FusionBot IMU in the IMU_phy housing.")
    ap.add_argument("--apply", action="store_true", help="perform the changes (default: dry run)")
    ap.add_argument("--housing", default=HOUSING_ALIAS, help="robot shape that holds the IMU")
    args = ap.parse_args()

    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    sim = RemoteAPIClient().require("sim")
    if sim.getSimulationState() != sim.simulation_stopped:
        raise SystemExit("stop the simulation first")

    robot = int(sim.getObject(rc.ROBOT_PATH))
    axle = _axle(sim)
    floor_z = axle[2] - rc.WHEEL_RADIUS_M
    heading = _heading(sim)
    existing = [int(h) for h in sim.getObjectsInTree(robot, sim.handle_all, 0) if _alias(sim, h) == IMU_ALIAS]
    from sim.robot_parts import PARTS, find_part
    mount_file = RESULTS / "imu_mount.json"
    c0, s0 = math.cos(heading), math.sin(heading)
    if args.housing != HOUSING_ALIAS:               # explicitly named shape
        hits = [b for b in shape_boxes(sim, robot, set()) if b[0] == args.housing]
        if len(hits) != 1:
            raise SystemExit(f"robot shape '{args.housing}' not found (python -m sim.lidar_setup --list-shapes)")
        label, centre, size = hits[0][0], hits[0][2], hits[0][3]
    else:
        try:
            part = find_part(sim, robot, "imu", floor_z)
            label, centre, size = f"{part.label} ({part.method})", part.centre, part.size
        except RuntimeError as exc:
            print(f"({exc})")
            if existing:                            # keep the IMU where it is
                label, size = f"existing {IMU_ALIAS} position kept", (0.0, 0.0, 0.0)
                centre = list(sim.getObjectPosition(existing[0], sim.handle_world))
            elif mount_file.exists():               # rebuild from the saved mount
                mm = json.loads(mount_file.read_text())
                label, size = "position from results/imu_mount.json", (0.0, 0.0, 0.0)
                centre = [axle[0] + c0 * mm["x_forward_m"] - s0 * mm["y_left_m"],
                          axle[1] + s0 * mm["x_forward_m"] + c0 * mm["y_left_m"], floor_z + mm["z_above_floor_m"]]
            else:
                raise SystemExit(f"IMU housing {PARTS['imu']['alias']} not found; pass --housing <alias>")
    fwd = (centre[0] - axle[0]) * c0 + (centre[1] - axle[1]) * s0
    left = -(centre[0] - axle[0]) * s0 + (centre[1] - axle[1]) * c0

    print("\nFusionBot IMU setup" + ("" if args.apply else "  (DRY RUN - nothing changed)"))
    print("-----------------------------------------------")
    print(f"housing                : {label}  size {size[0]:.3f} x {size[1]:.3f} x {size[2]:.3f} m")
    print(f"robot heading          : {math.degrees(heading):+.3f} deg")
    print(f"PLAN: IMU frame at the housing centre ({fwd:+.4f} m ahead, {left:+.4f} m left of the axle, "
          f"{centre[2] - floor_z:.4f} m above floor), axes = robot axes")
    print(f"existing {IMU_ALIAS}  : {'yes (will be replaced)' if existing else 'no'}")
    if not args.apply:
        print("\nrun again with --apply to perform it")
        return

    # remove the old IMU WITH its children (sim.removeObject alone re-parents the children), and any
    # stray IMU script left elsewhere by an earlier version of this tool
    stale = set()
    for h in existing:
        stale |= {int(x) for x in sim.getObjectsInTree(h, sim.handle_all, 0)}
    stale |= {int(h) for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)
              if _alias(sim, h) == SCRIPT_ALIAS}
    for h in sorted(stale, reverse=True):
        try:
            sim.removeObject(h)
        except Exception:                              # noqa: BLE001 - already removed
            pass
    if stale:
        print(f"removed {len(stale)} old IMU object(s)/script(s)")
    d = int(sim.createDummy(0.02))
    sim.setObjectAlias(d, IMU_ALIAS)
    sim.setObjectPosition(d, list(centre), sim.handle_world)
    sim.setObjectOrientation(d, [0.0, 0.0, heading], sim.handle_world)
    sim.setObjectParent(d, int(sim.getObject(rc.BASE_PATH)), True)   # moving body, not the static /FusionBot
    sh = int(sim.createScript(sim.scripttype_simulation, IMU_SCRIPT, 0, "lua"))
    sim.setObjectAlias(sh, SCRIPT_ALIAS)
    sim.setObjectParent(sh, d, False)               # script icon sits on the IMU (local pose 0)
    first = _set_first(sim, sh)

    m = list(sim.getObjectMatrix(d, sim.handle_world))
    yaw = math.atan2(m[4], m[0])
    mount = {
        "imu_path": f"{rc.BASE_PATH}/{IMU_ALIAS}",
        "x_forward_m": fwd, "y_left_m": left, "z_above_floor_m": centre[2] - floor_z,
        "yaw_offset_rad": math.remainder(yaw - heading, 2 * math.pi),
        "gyro_property": GYRO_PROPERTY, "accel_property": ACCEL_PROPERTY, "count_property": COUNT_PROPERTY,
        "gravity_m_s2": GRAVITY_M_S2, "runs_first": first,
    }
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "imu_mount.json").write_text(json.dumps(mount, indent=1))
    print(f"\nDONE: {IMU_ALIAS} + {SCRIPT_ALIAS} created, script runs first in each step: {first}")
    print(f"mount: forward {fwd:+.4f} m, left {left:+.4f} m, {centre[2] - floor_z:.4f} m above floor, "
          f"yaw offset {math.degrees(mount['yaw_offset_rad']):+.4f} deg")
    print(f"saved {RESULTS / 'imu_mount.json'}")
    print("now save the scene in CoppeliaSim (Ctrl+S)")


def imu_from_poses(samples, g: float = GRAVITY_M_S2):
    """Python reference of the Lua IMU (used by the offline test).
    samples: [(t, R 3x3 world-from-body as rows, p world)] oldest first.
    Returns [(t, valid, gyro_body, accel_body)] with the same formulas as the script."""
    out = []
    for k in range(len(samples)):
        t, Rb, pb = samples[k]
        gyro, acc, valid = [0.0, 0.0, 0.0], [0.0, 0.0, g], 0
        if k >= 1:
            ta, Ra, pa = samples[k - 1]
            dt = t - ta
            Rr = [[sum(Ra[kk][i] * Rb[kk][j] for kk in range(3)) for j in range(3)] for i in range(3)]
            gyro = [(Rr[2][1] - Rr[1][2]) / (2 * dt), (Rr[0][2] - Rr[2][0]) / (2 * dt), (Rr[1][0] - Rr[0][1]) / (2 * dt)]
            valid = 1
            if k >= 2:
                tc, _, pc = samples[k - 2]
                dt2 = ta - tc
                aw = [((pb[i] - pa[i]) / dt - (pa[i] - pc[i]) / dt2) / (0.5 * (dt + dt2)) for i in range(3)]
                aw[2] += g
                acc = [sum(Rb[i][j] * aw[i] for i in range(3)) for j in range(3)]
                valid = 2
        out.append((t, valid, gyro, acc))
    return out


if __name__ == "__main__":
    main()
