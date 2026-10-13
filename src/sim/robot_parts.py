"""
Finding the sensor housings on the robot, also when the robot's parts are GROUPED
into one compound shape (e.g. everything merged into mainBase_phy).

    find_part(sim, robot, "imu")    -> Part(label, centre_world, size, method)
    find_part(sim, robot, "lidar")

1. a separate shape with the housing's alias (IMU_phy / lidar_phy), any case;
2. otherwise every robot shape is COPIED, the copy is ungrouped
   (sim.ungroupShape) and its components are compared with the housing's known
   size (sorted dimensions, tolerance 6 mm) and height above the floor; the
   copies are deleted again - the robot itself is never changed.

The housing sizes were measured on the ungrouped robot (step 12b/12e list of
robot shapes): IMU_phy 0.062 x 0.064 x 0.019 m, lidar_phy 0.075 x 0.075 x 0.045 m.
"""

from __future__ import annotations

from dataclasses import dataclass

from sim.lidar_setup import _alias, _first_type, _matrix, shape_boxes

PARTS = {
    "imu": {"alias": "IMU_phy", "size": (0.062, 0.064, 0.019), "min_height_m": 0.12},
    "lidar": {"alias": "lidar_phy", "size": (0.075, 0.075, 0.045), "min_height_m": 0.12},
}
SIZE_TOL_M = 0.006


@dataclass(frozen=True)
class Part:
    label: str
    centre: tuple            # world (x, y, z) of the bbox centre
    size: tuple              # (dx, dy, dz) in the part frame
    method: str              # "shape" | "component of <compound>"


def size_error(size, expected) -> float:
    """Largest difference of the sorted dimensions (orientation independent)."""
    return max(abs(a - b) for a, b in zip(sorted(size), sorted(expected)))


def pick_component(components, expected, floor_z, min_height, tol=SIZE_TOL_M):
    """components: [(label, centre, size)] -> best match or None; raises if two different places match."""
    hits = sorted(((size_error(s, expected), lab, c, s) for lab, c, s in components
                   if c[2] - floor_z >= min_height and size_error(s, expected) <= tol), key=lambda x: x[0])
    if not hits:
        return None
    best = hits[0]
    for other in hits[1:]:
        if max(abs(other[2][i] - best[2][i]) for i in range(2)) > 0.01:
            raise RuntimeError(f"two housings of this size at different places: {best[1]} and {other[1]}")
    return best


def _component_boxes(sim, handles):
    shape_t = _first_type(sim, "sceneobject_shape", "object_shape_type")
    out = []
    for h in handles:
        h = int(h)
        if sim.getObjectType(h) != shape_t:
            continue
        lo = [float(sim.getObjectFloatParam(h, getattr(sim, f"objfloatparam_objbbox_min_{a}"))) for a in "xyz"]
        hi = [float(sim.getObjectFloatParam(h, getattr(sim, f"objfloatparam_objbbox_max_{a}"))) for a in "xyz"]
        c = [0.5 * (lo[i] + hi[i]) for i in range(3)]
        m = _matrix(sim, h)
        w = tuple(m[4 * r] * c[0] + m[4 * r + 1] * c[1] + m[4 * r + 2] * c[2] + m[4 * r + 3] for r in range(3))
        out.append((_alias(sim, h), w, tuple(hi[i] - lo[i] for i in range(3))))
    return out


def find_part(sim, robot, part: str, floor_z: float, exclude: set[int] | None = None) -> Part:
    spec = PARTS[part]
    exclude = exclude or set()
    shapes = shape_boxes(sim, robot, exclude)
    for alias, h, w, s in shapes:                                   # 1. separate shape
        if alias.lower() == spec["alias"].lower():
            return Part(alias, tuple(w), tuple(s), "shape")
    components = []
    scene_before = {int(x) for x in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)}
    for alias, h, w, s in shapes:                                   # 2. inside a compound
        copies = [int(x) for x in sim.copyPasteObjects([h], 0)]
        parts = []
        try:
            parts = [int(x) for x in sim.ungroupShape(copies[0])]
            if len(parts) > 1:
                components += [(f"{alias}#{i}", c, sz) for i, (_, c, sz) in
                               enumerate(_component_boxes(sim, parts))]
        finally:
            for x in set(parts) | set(copies):
                try:
                    sim.removeObject(x)
                except Exception:                                   # noqa: BLE001 - already gone
                    pass
    # anything the copies brought along (e.g. children of a copied model base) is removed too
    for x in sorted({int(x) for x in sim.getObjectsInTree(sim.handle_scene, sim.handle_all, 0)} - scene_before,
                    reverse=True):
        try:
            sim.removeObject(x)
        except Exception:                                           # noqa: BLE001
            pass
    best = pick_component(components, spec["size"], floor_z, spec["min_height_m"])
    if best is None:
        raise RuntimeError(f"no {part} housing ({spec['alias']}, {spec['size']}) found as a shape or inside "
                           f"a grouped shape ({len(components)} components checked)")
    compound = best[1].split("#")[0]
    return Part(best[1], best[2], best[3], f"component of {compound}")
