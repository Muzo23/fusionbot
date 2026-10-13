"""
Robot geometry and mass properties as they are IN THE SCENE (for the thesis, section 3.3, and to
check the constants in core/robot_config.py).

    python -m checks.robot_geometry          (from src/, simulation stopped, robot at START)

Printed and saved to results/robot_geometry.json:
  * wheel joints: position in the robot frame (forward / left of the axle midpoint, height),
    the geometric wheel separation = distance between the two joints;
  * every shape of the robot: dynamic / respondable, mass, bounding-box size, position in the
    robot frame -> wheel diameter and width, caster, base parts;
  * total mass and the centre of mass of the dynamic shapes (robot frame);
  * comparison with WHEEL_RADIUS_M, WHEEL_SEPARATION_M and EFFECTIVE_WHEEL_SEPARATION_M.
Nothing in the scene is changed.
"""

from __future__ import annotations

import json
import math
from pathlib import Path

from core import robot_config as rc
from sim.lidar_setup import _alias, _first_type, _heading, _matrix, _pos

ROOT = Path(__file__).resolve().parents[2]


def _try(*fns, default=None):
    for f in fns:
        try:
            v = f()
            if v is not None:
                return v
        except Exception:                                        # noqa: BLE001 - API differs between versions
            continue
    return default


def _param(sim, h, *names):
    for n in names:
        p = getattr(sim, n, None)
        if p is None:
            continue
        v = _try(lambda: sim.getObjectInt32Param(h, p))
        if v is not None:
            return int(v)
    return None


def main() -> None:
    from coppeliasim_zmqremoteapi_client import RemoteAPIClient

    sim = RemoteAPIClient().require("sim")
    robot = sim.getObject(rc.ROBOT_PATH)
    lj, rj = sim.getObject(rc.LEFT_MOTOR_JOINT_PATH), sim.getObject(rc.RIGHT_MOTOR_JOINT_PATH)
    pl, pr = _pos(sim, lj), _pos(sim, rj)
    axle = [0.5 * (pl[i] + pr[i]) for i in range(3)]
    th = _heading(sim)
    c, s = math.cos(th), math.sin(th)

    def robot_frame(p):
        dx, dy = p[0] - axle[0], p[1] - axle[1]
        return dx * c + dy * s, -dx * s + dy * c, p[2]

    sep = math.dist(pl[:2], pr[:2])
    shape_t = _first_type(sim, "sceneobject_shape", "object_shape_type")
    shapes = []
    for h in sim.getObjectsInTree(robot, sim.handle_all, 0):
        h = int(h)
        if sim.getObjectType(h) != shape_t:
            continue
        lo = [_try(lambda a=a: sim.getObjectFloatParam(h, getattr(sim, f"objfloatparam_objbbox_min_{a}"))) for a in "xyz"]
        hi = [_try(lambda a=a: sim.getObjectFloatParam(h, getattr(sim, f"objfloatparam_objbbox_max_{a}"))) for a in "xyz"]
        size = [hi[i] - lo[i] for i in range(3)] if None not in lo + hi else None
        m = _matrix(sim, h)
        cl = [0.5 * (lo[i] + hi[i]) for i in range(3)] if size else [0, 0, 0]
        cw = [m[4 * r] * cl[0] + m[4 * r + 1] * cl[1] + m[4 * r + 2] * cl[2] + m[4 * r + 3] for r in range(3)]
        static = _param(sim, h, "shapeintparam_static")
        respondable = _param(sim, h, "shapeintparam_respondable")
        mass = _try(lambda: sim.getShapeMass(h), lambda: sim.getFloatProperty(h, "mass"))
        # centre of mass and principal inertia from the physics properties (not the bounding box)
        com_w, inertia = None, None
        ine = _try(lambda: sim.getShapeInertia(h))
        if ine and len(ine) == 2:
            I, T = list(ine[0]), list(ine[1])
            inertia = [round(I[0], 7), round(I[4], 7), round(I[8], 7)]
            com_w = [m[4 * r] * T[3] + m[4 * r + 1] * T[7] + m[4 * r + 2] * T[11] + m[4 * r + 3] for r in range(3)]
        f, l, z = robot_frame(cw)
        cf = robot_frame(com_w) if com_w else None
        shapes.append({"name": _alias(sim, h), "parent": _alias(sim, sim.getObjectParent(h)),
                       "dynamic": static == 0 if static is not None else None,
                       "respondable": bool(respondable) if respondable is not None else None,
                       "mass_kg": mass, "size_m": [round(v, 5) for v in size] if size else None,
                       "centre_forward_m": round(f, 5), "centre_left_m": round(l, 5), "centre_z_m": round(z, 5),
                       "com_robot_frame_m": [round(v, 5) for v in cf] if cf else None,
                       "inertia_diag_kg_m2": inertia})

    # wheels: respondable shapes in the joints' subtrees
    def subtree(j):
        return {int(x) for x in sim.getObjectsInTree(j, sim.handle_all, 0)}
    wheels = {}
    for side, j in (("left", lj), ("right", rj)):
        names = {_alias(sim, x) for x in subtree(j) if sim.getObjectType(x) == shape_t}
        cand = [sh for sh in shapes if sh["name"] in names and sh["respondable"] and sh["size_m"]]
        if cand:
            sz = sorted(cand[0]["size_m"])
            wheels[side] = {"shape": cand[0]["name"], "diameter_m": sz[2], "width_m": sz[0],
                            "radius_m": sz[2] / 2}
    floor_z = axle[2] - (wheels["left"]["radius_m"] if "left" in wheels else rc.WHEEL_RADIUS_M)
    dyn = [sh for sh in shapes if sh["dynamic"] and sh["mass_kg"]]
    total = sum(sh["mass_kg"] for sh in dyn)
    def c_of(sh):     # true centre of mass when the engine gives it, else the bounding-box centre
        return sh["com_robot_frame_m"] or [sh["centre_forward_m"], sh["centre_left_m"], sh["centre_z_m"]]
    com = [sum(sh["mass_kg"] * c_of(sh)[i] for sh in dyn) / total for i in range(3)] if total else None

    out = {
        "joints": {"left_robot_frame_m": [round(v, 5) for v in robot_frame(pl)],
                   "right_robot_frame_m": [round(v, 5) for v in robot_frame(pr)],
                   "axle_height_above_floor_m": round(axle[2] - floor_z, 5),
                   "separation_m": round(sep, 5)},
        "wheels": wheels, "total_dynamic_mass_kg": total,
        "com_robot_frame_m": [round(v, 5) for v in com] if com else None,
        "code_constants": {"WHEEL_RADIUS_M": rc.WHEEL_RADIUS_M, "WHEEL_SEPARATION_M": rc.WHEEL_SEPARATION_M,
                           "EFFECTIVE_WHEEL_SEPARATION_M": rc.EFFECTIVE_WHEEL_SEPARATION_M},
        "shapes": shapes,
    }
    (ROOT / "results").mkdir(exist_ok=True)
    (ROOT / "results" / "robot_geometry.json").write_text(json.dumps(out, indent=1))

    print("\nFusionBot geometry in the scene")
    print("-------------------------------")
    print(f"wheel joints (robot frame, forward / left / z): left {out['joints']['left_robot_frame_m']}, "
          f"right {out['joints']['right_robot_frame_m']}")
    print(f"joint separation      : {sep:.5f} m   (code WHEEL_SEPARATION_M {rc.WHEEL_SEPARATION_M}, "
          f"B_eff {rc.EFFECTIVE_WHEEL_SEPARATION_M})")
    for side, w in wheels.items():
        print(f"{side:5} wheel           : {w['shape']}, diameter {w['diameter_m']:.5f} m (r {w['radius_m']:.5f}), "
              f"width {w['width_m']:.5f} m   (code WHEEL_RADIUS_M {rc.WHEEL_RADIUS_M})")
    print(f"total dynamic mass    : {total:.3f} kg, centre of mass (forward, left, z) {out['com_robot_frame_m']}")
    for sh in dyn:
        print(f"  {sh['name']:20} mass {sh['mass_kg']:.3f} kg, centre of mass {sh['com_robot_frame_m']}, "
              f"principal inertia {sh['inertia_diag_kg_m2']} kg m^2")
    print("\nshapes (dyn = dynamic, resp = respondable; centre = bounding-box centre in the robot frame)")
    print(f"  {'name':28} {'parent':22} {'dyn':>4} {'resp':>4} {'mass kg':>8}  {'size x y z [m]':26} fwd / left / z")
    for sh in shapes:
        sz = " ".join(f"{v:.3f}" for v in sh["size_m"]) if sh["size_m"] else "-"
        m = f"{sh['mass_kg']:.3f}" if sh["mass_kg"] is not None else "-"
        print(f"  {sh['name'][:28]:28} {sh['parent'][:22]:22} {str(sh['dynamic'])[:4]:>4} {str(sh['respondable'])[:4]:>4} "
              f"{m:>8}  {sz:26} {sh['centre_forward_m']:+.3f} / {sh['centre_left_m']:+.3f} / {sh['centre_z_m']:.3f}")
    print(f"\nsaved {ROOT / 'results' / 'robot_geometry.json'}")


if __name__ == "__main__":
    main()
