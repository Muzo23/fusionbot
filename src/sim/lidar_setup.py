"""
LiDAR setup (step 12b): mounts the 'Hokuyo URG 04LX UG01_Fast' model on the
robot and installs the FusionBot scan script. Simulation must be STOPPED.

    python -m sim.lidar_setup            dry run: measures and prints the plan
    python -m sim.lidar_setup --apply    performs it (then Ctrl+S in CoppeliaSim)
    python -m sim.lidar_setup --onto auto [--apply]
                                         scan centre on the axis of the LiDAR housing cylinder
                                         on top of the robot, plane at its mid-height
    python -m sim.lidar_setup --list-shapes   robot shapes with their positions / sizes

What --apply does
-----------------
1. finds the Hokuyo model (the object whose children are sensor1 / sensor2) and
   makes it a child of /FusionBot (like the proximity sensors);
2. turns it so that the centre of its 240 deg field of view points along the
   robot heading, and moves it so that the scan centre lies above the wheel-axle
   midpoint and the SCAN PLANE is at --height (default: 0.03 m above the top of
   the robot body, never above the wall top 0.30 m minus 0.04 m);
3. makes every shape of the LiDAR static and NOT respondable, so the robot's
   mass and dynamics stay exactly as calibrated in steps 1-11;
   With --keep-position the LiDAR stays exactly where you put it (only its yaw
   is aligned with the robot heading; the scan plane must stay below the wall top).
4. replaces the model's MAIN script (the script directly under the model; the
   filter scripts under sensor1 / sensor2 are KEPT - they convert the depth
   image into ranges) by the FusionBot scan script:
     - the WHOLE robot is excluded from rendering (the original excluded only
       the LiDAR, so the robot body could appear in the scan),
     - no drawing of rays (speed), far clipping = 5 m,
     - every simulation step publishes on the Hokuyo object
         signal.fusionbotLidarXYZD : float array, 4 values per ray
                                     (x, y, z in the Hokuyo frame, distance;
                                      distance >= 0.9999*5 m = no hit)
         signal.fusionbotLidarScanCount : int, increments every scan
     (the lockstep bridge will read these in step 18);
5. writes the mount (relative to the axle frame) to results/lidar_mount.json.
"""

from __future__ import annotations

import argparse
import json
import math
from pathlib import Path

from core import robot_config as rc

MAX_RANGE_M = 5.0
WALL_TOP_ABOVE_FLOOR_M = 0.30
SCRIPT_ALIAS = "FusionBotLidarScan"
XYZD_PROPERTY = "signal.fusionbotLidarXYZD"
COUNT_PROPERTY = "signal.fusionbotLidarScanCount"
RESULTS = Path(__file__).resolve().parents[2] / "results"

LIDAR_SCRIPT = f"""sim=require'sim'
-- FusionBot LiDAR scan script (installed by sim/lidar_setup.py, step 12b)
local MAX_RANGE={MAX_RANGE_M}
local XYZD_PROPERTY='{XYZD_PROPERTY}'
local COUNT_PROPERTY='{COUNT_PROPERTY}'

function sysCall_init()
    self=sim.getObject('..')
    visionSensors={{sim.getObject('../sensor1'),sim.getObject('../sensor2')}}
    local robot=sim.getObject('{rc.ROBOT_PATH}')
    local collection=sim.createCollection(0)
    sim.addItemToCollection(collection,sim.handle_all,-1,0)
    sim.addItemToCollection(collection,sim.handle_tree,robot,1)   -- never see the robot itself
    for i=1,2 do
        sim.setObjectInt32Param(visionSensors[i],sim.visionintparam_entity_to_render,collection)
        sim.setObjectFloatParam(visionSensors[i],sim.visionfloatparam_far_clipping,MAX_RANGE)
    end
    scanCount=0
end

function sysCall_sensing()
    local data={{}}
    local invBase=sim.getMatrixInverse(sim.getObjectMatrix(self))
    for i=1,2 do
        local r,t,u=sim.readVisionSensor(visionSensors[i])
        if u then
            local m=sim.multiplyMatrices(invBase,sim.getObjectMatrix(visionSensors[i]))
            for j=0,u[2]-1 do
                for k=0,u[1]-1 do
                    local w=2+4*(j*u[1]+k)
                    local p=sim.multiplyVector(m,{{u[w+1],u[w+2],u[w+3]}})
                    data[#data+1]=p[1]
                    data[#data+1]=p[2]
                    data[#data+1]=p[3]
                    data[#data+1]=u[w+4]
                end
            end
        end
    end
    scanCount=scanCount+1
    sim.setFloatArrayProperty(self,XYZD_PROPERTY,data)
    sim.setIntProperty(self,COUNT_PROPERTY,scanCount)
end
"""


def _type(sim, name):
    return getattr(sim, name, None)


def _children(sim, h):
    out, i = [], 0
    while True:
        c = sim.getObjectChild(h, i)
        if c == -1:
            return out
        out.append(int(c))
        i += 1


def _alias(sim, h) -> str:
    return str(sim.getObjectAlias(h))


def find_hokuyo(sim) -> int:
    hits = []
    for h in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0):
        names = {_alias(sim, c) for c in _children(sim, h)}
        if {"sensor1", "sensor2"} <= names:
            hits.append(int(h))
    if len(hits) != 1:
        raise RuntimeError(f"expected exactly one Hokuyo model (object with children sensor1/sensor2), found {len(hits)}")
    return hits[0]


def _matrix(sim, h):
    return list(sim.getObjectMatrix(h, sim.handle_world))


def _pos(sim, h):
    return list(sim.getObjectPosition(h, sim.handle_world))


def _axle(sim):
    lj, rj = sim.getObject(rc.LEFT_MOTOR_JOINT_PATH), sim.getObject(rc.RIGHT_MOTOR_JOINT_PATH)
    a, b = _pos(sim, lj), _pos(sim, rj)
    return [0.5 * (a[i] + b[i]) for i in range(3)]


def _heading(sim):
    base = sim.getObject(rc.BASE_PATH)
    return math.remainder(sim.getObjectOrientation(base, sim.handle_world)[2] + rc.BASE_YAW_TO_PHYSICAL_HEADING_RAD,
                          2 * math.pi)


def scan_geometry(sim, hok):
    """World: scan centre (mean of the two sensor origins), forward direction (bisector
    of the two sensor view axes = their z axes), and how horizontal the scan is."""
    kids = {_alias(sim, c): c for c in _children(sim, hok)}
    s1, s2 = kids["sensor1"], kids["sensor2"]
    m1, m2 = _matrix(sim, s1), _matrix(sim, s2)
    centre = [0.5 * (m1[3] + m2[3]), 0.5 * (m1[7] + m2[7]), 0.5 * (m1[11] + m2[11])]
    z1, z2 = (m1[2], m1[6], m1[10]), (m2[2], m2[6], m2[10])
    fwd = [z1[i] + z2[i] for i in range(3)]
    tilt = max(abs(z1[2]), abs(z2[2]))
    return centre, math.atan2(fwd[1], fwd[0]), tilt, (s1, s2)


def robot_top_z(sim, robot, exclude: set[int]) -> float:
    shape_t = _type(sim, "sceneobject_shape")
    if shape_t is None:
        shape_t = _type(sim, "object_shape_type")
    top = -math.inf
    for h in sim.getObjectsInTree(robot, sim.handle_all, 0):
        h = int(h)
        if h in exclude or sim.getObjectType(h) != shape_t:
            continue
        try:
            zmax = sim.getObjectFloatParam(h, sim.objfloatparam_objbbox_max_z)
        except Exception:                              # noqa: BLE001
            continue
        top = max(top, _pos(sim, h)[2] + float(zmax))
    return top


def shape_boxes(sim, robot, exclude: set[int]):
    """[(alias, handle, world bbox centre, (dx, dy, dz))] of the robot shapes (object-frame bbox)."""
    shape_t = _type(sim, "sceneobject_shape")
    if shape_t is None:
        shape_t = _type(sim, "object_shape_type")
    out = []
    for h in sim.getObjectsInTree(robot, sim.handle_all, 0):
        h = int(h)
        if h in exclude or sim.getObjectType(h) != shape_t:
            continue
        try:
            lo = [float(sim.getObjectFloatParam(h, getattr(sim, f"objfloatparam_objbbox_min_{a}"))) for a in "xyz"]
            hi = [float(sim.getObjectFloatParam(h, getattr(sim, f"objfloatparam_objbbox_max_{a}"))) for a in "xyz"]
        except Exception:                              # noqa: BLE001
            continue
        c = [0.5 * (lo[i] + hi[i]) for i in range(3)]
        m = _matrix(sim, h)
        w = [m[4 * r] * c[0] + m[4 * r + 1] * c[1] + m[4 * r + 2] * c[2] + m[4 * r + 3] for r in range(3)]
        out.append((_alias(sim, h), h, w, tuple(hi[i] - lo[i] for i in range(3))))
    return out


def round_candidates(boxes, floor_z, axle, heading):
    """Upright cylinders 3-15 cm across on the upper part of the robot (LiDAR housing)."""
    out = []
    c0, s0 = math.cos(heading), math.sin(heading)
    for alias, h, w, (dx, dy, dz) in boxes:
        fwd = (w[0] - axle[0]) * c0 + (w[1] - axle[1]) * s0
        if 0.03 <= dx <= 0.15 and abs(dx - dy) <= 0.15 * max(dx, dy) and dz >= 0.015 \
                and w[2] - floor_z > 0.12 and abs(fwd) < 0.40:
            out.append((alias, h, w, (dx, dy, dz)))
    return out


def main() -> None:
    ap = argparse.ArgumentParser(description="Mount the Hokuyo LiDAR on FusionBot and install the scan script.")
    ap.add_argument("--apply", action="store_true", help="perform the changes (default: dry run)")
    ap.add_argument("--height", type=float, default=None, help="scan plane height above the floor [m]")
    ap.add_argument("--forward", type=float, default=0.0, help="scan centre ahead of the axle midpoint [m]")
    ap.add_argument("--onto", default=None, metavar="ALIAS|auto",
                    help="put the scan centre on the axis of this robot shape (e.g. the LiDAR housing "
                         "cylinder); 'auto' finds the upright cylinder on top of the robot")
    ap.add_argument("--list-shapes", action="store_true", help="list the robot shapes with their boxes")
    ap.add_argument("--keep-position", action="store_true",
                    help="keep the LiDAR where you placed it; only align its yaw with the robot heading")
    args = ap.parse_args()

    from coppeliasim_zmqremoteapi_client import RemoteAPIClient
    sim = RemoteAPIClient().require("sim")
    if sim.getSimulationState() != sim.simulation_stopped:
        raise SystemExit("stop the simulation first")

    robot = int(sim.getObject(rc.ROBOT_PATH))
    hok = find_hokuyo(sim)
    hok_tree = {int(h) for h in sim.getObjectsInTree(hok, sim.handle_all, 0)}
    axle = _axle(sim)
    floor_z = axle[2] - rc.WHEEL_RADIUS_M
    heading = _heading(sim)
    top = robot_top_z(sim, robot, hok_tree)
    centre, fwd_yaw, tilt, sensors = scan_geometry(sim, hok)
    max_plane = WALL_TOP_ABOVE_FLOOR_M - 0.04
    onto_xy = None
    if args.list_shapes or args.onto:
        boxes = shape_boxes(sim, robot, hok_tree)
        c0, s0 = math.cos(heading), math.sin(heading)
        if args.list_shapes:
            print("\nrobot shapes (centre relative to the axle: forward / left / height above floor; size)")
            for alias, h, w, (dx, dy, dz) in boxes:
                f_ = (w[0] - axle[0]) * c0 + (w[1] - axle[1]) * s0
                l_ = -(w[0] - axle[0]) * s0 + (w[1] - axle[1]) * c0
                print(f"  {alias:28} {f_:+.4f} {l_:+.4f} {w[2] - floor_z:.4f} m   size {dx:.3f} x {dy:.3f} x {dz:.3f} m")
            if not args.onto:
                return
        if args.onto == "auto":
            cands = round_candidates(boxes, floor_z, axle, heading)
            if len(cands) != 1:
                names = ", ".join(c[0] for c in cands) or "none"
                raise SystemExit(f"--onto auto found {len(cands)} cylinders ({names}); run with --list-shapes "
                                 f"and pass --onto <alias> (click the cylinder in CoppeliaSim to see its name)")
            target = cands[0]
        else:
            hits = [b for b in boxes if b[0] == args.onto]
            if len(hits) != 1:
                raise SystemExit(f"robot shape '{args.onto}' not found ({len(hits)} matches); see --list-shapes")
            target = hits[0]
        onto_xy = (target[2][0], target[2][1])
        print(f"housing                : {target[0]}  size {target[3][0]:.3f} x {target[3][1]:.3f} x {target[3][2]:.3f} m, "
              f"centre {target[2][2] - floor_z:.4f} m above floor")
    if args.keep_position:
        plane = centre[2] - floor_z
    elif onto_xy is not None:
        plane = args.height if args.height is not None else min(target[2][2] - floor_z, max_plane)
    else:
        plane = args.height if args.height is not None else min(max(top - floor_z + 0.03, 0.10), max_plane)

    print("\nFusionBot LiDAR setup" + ("" if args.apply else "  (DRY RUN - nothing changed)"))
    print("-----------------------------------------------")
    print(f"Hokuyo model           : {_alias(sim, hok)}  parent: {_alias(sim, sim.getObjectParent(hok))}")
    print(f"axle midpoint (world)  : ({axle[0]:+.4f}, {axle[1]:+.4f}), floor z {floor_z:+.4f} m")
    print(f"robot heading          : {math.degrees(heading):+.3f} deg")
    print(f"robot body top         : {top - floor_z:.4f} m above floor")
    print(f"scan centre now        : ({centre[0]:+.4f}, {centre[1]:+.4f}), {centre[2] - floor_z:.4f} m above floor")
    print(f"scan forward now       : {math.degrees(fwd_yaw):+.3f} deg, plane tilt {math.degrees(math.asin(min(1.0, tilt))):.3f} deg")
    for s in sensors:
        try:
            res = sim.getVisionSensorRes(s)
            ang = sim.getObjectFloatParam(s, sim.visionfloatparam_perspective_angle)
            print(f"  {_alias(sim, s)}: resolution {res}, view angle {math.degrees(ang):.1f} deg")
        except Exception:                              # noqa: BLE001
            pass
    if onto_xy is not None and not args.keep_position:
        c0, s0 = math.cos(heading), math.sin(heading)
        dx, dy = onto_xy[0] - axle[0], onto_xy[1] - axle[1]
        print(f"PLAN: scan centre on the housing axis ({dx * c0 + dy * s0:+.4f} m ahead, {-dx * s0 + dy * c0:+.4f} m "
              f"left of the axle), plane {plane:.4f} m above floor (mid-height of the housing), forward = robot heading")
    elif args.keep_position:
        dx, dy = centre[0] - axle[0], centre[1] - axle[1]
        c0, s0 = math.cos(heading), math.sin(heading)
        print(f"PLAN: keep the LiDAR where it is ({dx * c0 + dy * s0:+.3f} m ahead, {-dx * s0 + dy * c0:+.3f} m left "
              f"of the axle, plane {plane:.3f} m above floor); only turn it to the robot heading")
        if math.hypot(dx, dy) > 0.40:
            raise SystemExit("the LiDAR is more than 0.4 m from the robot - put it on the robot first")
    else:
        print(f"PLAN: scan centre {args.forward:+.3f} m ahead of the axle, plane {plane:.3f} m above floor "
              f"(wall top {WALL_TOP_ABOVE_FLOOR_M:.2f} m), forward = robot heading")
    script_t0 = _type(sim, "sceneobject_script") or _type(sim, "object_script_type")
    for h in sorted(hok_tree):
        if script_t0 is not None and sim.getObjectType(h) == script_t0:
            where = "main (will be replaced)" if sim.getObjectParent(h) == hok else "sensor filter (kept)"
            print(f"  script {_alias(sim, h):24} under {_alias(sim, sim.getObjectParent(h)):12} -> {where}")
    if plane > max_plane:
        raise SystemExit(f"scan plane {plane:.3f} m too high: the LiDAR would look over the walls")
    if tilt > 0.05:
        raise SystemExit("the scan plane of the model is not horizontal - check the model orientation")
    if not args.apply:
        print("\nrun again with --apply to perform it")
        return

    # 1. parent
    if sim.getObjectParent(hok) != robot:
        sim.setObjectParent(hok, robot, True)
    # 2. rotate about the world z axis through the scan centre, then translate
    dyaw = math.remainder(heading - fwd_yaw, 2 * math.pi)
    m = _matrix(sim, hok)
    m = sim.rotateAroundAxis(m, [0, 0, 1], centre, dyaw)
    sim.setObjectMatrix(hok, m, sim.handle_world)
    centre, fwd_yaw, tilt, _ = scan_geometry(sim, hok)
    if not args.keep_position:
        if onto_xy is not None:
            target = [onto_xy[0], onto_xy[1], floor_z + plane]
        else:
            target = [axle[0] + args.forward * math.cos(heading), axle[1] + args.forward * math.sin(heading),
                      floor_z + plane]
        p = _pos(sim, hok)
        sim.setObjectPosition(hok, [p[i] + target[i] - centre[i] for i in range(3)], sim.handle_world)
    # 3. static, not respondable
    shape_t = _type(sim, "sceneobject_shape") or _type(sim, "object_shape_type")
    n_shapes = 0
    for h in hok_tree:
        if sim.getObjectType(h) == shape_t:
            sim.setObjectInt32Param(h, sim.shapeintparam_static, 1)
            sim.setObjectInt32Param(h, sim.shapeintparam_respondable, 0)
            n_shapes += 1
    # 4. script
    script_t = _type(sim, "sceneobject_script") or _type(sim, "object_script_type")
    direct = {int(h) for h in _children(sim, hok)}
    kept = sum(1 for h in hok_tree if h != hok and h not in direct
               and script_t is not None and sim.getObjectType(h) == script_t)   # count BEFORE removing
    removed = 0
    for h in direct:                                   # ONLY scripts directly under the model
        try:
            if script_t is not None and sim.getObjectType(h) == script_t:
                sim.removeObject(h)
                removed += 1
        except Exception:                              # noqa: BLE001
            pass
    sh = int(sim.createScript(sim.scripttype_simulation, LIDAR_SCRIPT, 0, "lua"))
    sim.setObjectAlias(sh, SCRIPT_ALIAS)
    sim.setObjectParent(sh, hok, True)
    # 5. verify and record
    centre, fwd_yaw, tilt, _ = scan_geometry(sim, hok)
    dx, dy = centre[0] - axle[0], centre[1] - axle[1]
    c, s = math.cos(heading), math.sin(heading)
    mount = {
        "hokuyo_path": f"{rc.ROBOT_PATH}/{_alias(sim, hok)}",
        "x_forward_m": dx * c + dy * s, "y_left_m": -dx * s + dy * c,
        "z_above_floor_m": centre[2] - floor_z,
        "yaw_offset_rad": math.remainder(fwd_yaw - heading, 2 * math.pi),
        "max_range_m": MAX_RANGE_M, "xyzd_property": XYZD_PROPERTY, "count_property": COUNT_PROPERTY,
    }
    RESULTS.mkdir(parents=True, exist_ok=True)
    (RESULTS / "lidar_mount.json").write_text(json.dumps(mount, indent=1))
    print(f"\nDONE: {n_shapes} LiDAR shapes static / not respondable, {removed} main script(s) replaced by "
          f"{SCRIPT_ALIAS}, {kept} sensor filter script(s) kept")
    print(f"mount: forward {mount['x_forward_m']:+.4f} m, left {mount['y_left_m']:+.4f} m, "
          f"plane {mount['z_above_floor_m']:.4f} m above floor, yaw offset {math.degrees(mount['yaw_offset_rad']):+.4f} deg")
    print(f"saved {RESULTS / 'lidar_mount.json'}")
    print("now save the scene in CoppeliaSim (Ctrl+S)")


if __name__ == "__main__":
    main()
