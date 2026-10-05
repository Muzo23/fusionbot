"""
FusionBot topological maze memory (step 10).

Stores the explored maze as a graph: junction nodes (estimated centre, exits
per global direction) and edges (traversal counts). It does not choose
manoeuvres (DFS: step 11), read sensors, command motors or use GT.

Every junction event from the FSM (step 9) is placed with the POSE ESTIMATE of
the current configuration (A: odometry; B-E: fused). That is deliberate
(decision 5. 10. 2026, option 1): how reliably the robot recognises a junction
it has seen before depends on the quality of its pose estimate, and this
becomes a measured result of the thesis (topological map correctness per
configuration).

Geometry
--------
Frame: the estimate frame (odometry is initialised at the known START pose,
so it coincides with the maze frame up to the estimation error).
Global directions E=+x, N=+y, W=-x, S=-y; the robot heading estimate is
snapped to the nearest cardinal (decision boundary 45 deg; the live step-9
odometry heading drift was 11 deg after 526 s).
Junction centre estimate = reference pose + D * (cos th, sin th) with
D = D_SIDE_CANONICAL (side opening) or D_DEAD_END_CANONICAL (dead end).

Association (which junction is this?)
--------------------------------------
1. Along a KNOWN edge (both ends already stored): the arrival junction is the
   other end of that edge - topological, independent of the pose estimate
   (step 11 refinement of the 5. 10. decision: "when backtracking DFS relies on
   the graph, recognition is only for loop closure"). The metric distance is
   still logged as a consistency measure (association "EDGE").
2. Along a NEW edge: metric recognition (association "MATCH" or "NEW").
   With edge_association=False every event uses metric recognition (the
   step-10 variant, kept for comparison: with live odometry it duplicated ROOT
   and START after ~500 s in T1).

Metric recognition: an event is matched to a stored junction if
    1. the estimated centres are within JUNCTION_MATCH_RADIUS_M (0.373 m,
       half-way between the nominal scatter 0.204 m and half the pitch), and
    2. the exit sets agree (topological signature: the same global exits).
The nearest candidate wins (ties: lower id). Otherwise a new junction is
created. Condition 2 is checked separately so the log can tell a mismatch of
position from a mismatch of structure.

Edges
-----
depart(direction) is called when the robot leaves a junction through an exit.
The next event closes that edge at the arrival junction (arrival exit =
opposite of the arrival heading) and increments its traversal count. A loop
closure is an arrival at an already-known junction through an exit that was
not yet connected.
"""

from __future__ import annotations

import math
from dataclasses import dataclass, field
from enum import Enum

from navigation.junction_types import JunctionObservation, ReferenceKind
from navigation.navigation_config import D_DEAD_END_CANONICAL_M, D_SIDE_CANONICAL_M

JUNCTION_MATCH_RADIUS_M = 0.373424613


class Dir(str, Enum):
    E = "E"
    N = "N"
    W = "W"
    S = "S"


_ORDER = (Dir.E, Dir.N, Dir.W, Dir.S)          # CCW from +x
_ANGLE = {d: i * math.pi / 2 for i, d in enumerate(_ORDER)}


def snap_heading(theta_rad: float) -> Dir:
    k = round(theta_rad / (math.pi / 2)) % 4
    return _ORDER[int(k)]


def rotate(d: Dir, quarter_turns_ccw: int) -> Dir:
    return _ORDER[(_ORDER.index(d) + quarter_turns_ccw) % 4]


def opposite(d: Dir) -> Dir:
    return rotate(d, 2)


def relative_exits(obs: JunctionObservation, heading: Dir) -> dict[str, Dir]:
    """Robot-relative available exits (incl. BACK) -> global directions."""
    out = {"BACK": opposite(heading)}
    if obs.left_available:
        out["LEFT"] = rotate(heading, 1)
    if obs.straight_available:
        out["STRAIGHT"] = heading
    if obs.right_available:
        out["RIGHT"] = rotate(heading, -1)
    return out


@dataclass
class Junction:
    id: int
    x: float
    y: float
    kind: ReferenceKind | str
    exits: frozenset[Dir]
    edge_at: dict[Dir, int] = field(default_factory=dict)   # direction -> edge id
    visits: int = 1


@dataclass
class Edge:
    id: int
    a: int
    dir_a: Dir
    b: int | None = None
    dir_b: Dir | None = None
    traversals: int = 0

    def other(self, node: int) -> tuple[int | None, Dir | None]:
        return (self.b, self.dir_b) if node == self.a else (self.a, self.dir_a)


@dataclass(frozen=True)
class Arrival:
    junction: int
    is_new: bool
    heading: Dir                       # global heading on arrival
    arrival_exit: Dir                  # exit through which the robot entered (= opposite heading)
    exits: frozenset[Dir]
    centre_xy: tuple[float, float]
    match_distance_m: float | None     # distance to the matched / edge-associated junction (None if new)
    rejected_structure: tuple[int, ...]  # junctions within radius but with different exits
    edge: int | None                   # edge closed by this arrival
    loop_closure: bool                 # arrived at a known junction through a new edge
    association: str = "NEW"           # NEW / MATCH (metric) / EDGE (known edge)
    inconsistent: bool = False         # EDGE association with different exits or distance > radius


class MazeMemory:
    def __init__(self, match_radius_m: float = JUNCTION_MATCH_RADIUS_M, edge_association: bool = True) -> None:
        if match_radius_m <= 0:
            raise ValueError("match radius must be > 0")
        self.radius = float(match_radius_m)
        self.edge_association = bool(edge_association)
        self.junctions: dict[int, Junction] = {}
        self.edges: dict[int, Edge] = {}
        self.current: int | None = None
        self._pending: int | None = None            # edge being traversed
        self.log: list[Arrival] = []

    # ------------------------------------------------------------ start
    def start(self, x: float, y: float, theta: float) -> int:
        """The START pose is a node with one exit (forward); the robot departs it."""
        h = snap_heading(theta)
        j = Junction(0, x, y, "START", frozenset({h}))
        self.junctions = {0: j}
        self.edges = {}
        self.current = 0
        self._pending = None
        self.log = []
        self.depart(h)
        return 0

    # ------------------------------------------------------------ events
    @staticmethod
    def centre_estimate(obs: JunctionObservation, x: float, y: float, theta: float) -> tuple[float, float]:
        d = D_SIDE_CANONICAL_M if obs.kind is ReferenceKind.SIDE_CONFIRMED else D_DEAD_END_CANONICAL_M
        return x + d * math.cos(theta), y + d * math.sin(theta)

    def observe(self, obs: JunctionObservation, x: float, y: float, theta: float) -> Arrival:
        heading = snap_heading(theta)
        exits = frozenset(relative_exits(obs, heading).values())
        cx, cy = self.centre_estimate(obs, x, y, theta)
        arrival_exit = opposite(heading)

        if self.edge_association and self._pending is not None:
            e = self.edges[self._pending]
            if e.b is not None:
                src = self.current
                tgt, tgt_dir = e.other(src) if not (e.a == e.b) else (e.b, e.dir_b)
                j = self.junctions[tgt]
                d = math.hypot(j.x - cx, j.y - cy)
                inconsistent = (j.exits != exits) or tgt_dir != arrival_exit or d > self.radius
                if j.exits != exits:
                    j.exits = j.exits | exits          # keep every exit ever seen (robust depart)
                j.visits += 1
                e.traversals += 1
                self._pending = None
                self.current = j.id
                arr = Arrival(j.id, False, heading, arrival_exit, exits, (cx, cy), d, (), e.id, False,
                              "EDGE", inconsistent)
                self.log.append(arr)
                return arr

        best, best_d, rejected = None, None, []
        for j in sorted(self.junctions.values(), key=lambda j: j.id):
            d = math.hypot(j.x - cx, j.y - cy)
            if d > self.radius:
                continue
            if j.exits != exits:
                rejected.append(j.id)
                continue
            if best is None or d < best_d - 1e-12:
                best, best_d = j, d

        if best is None:
            jid = max(self.junctions) + 1
            best = Junction(jid, cx, cy, obs.kind, exits)
            self.junctions[jid] = best
            is_new = True
        else:
            best.visits += 1
            is_new = False

        edge_id, loop = None, False
        if self._pending is not None:
            e = self.edges[self._pending]
            edge_id = e.id
            known = best.edge_at.get(arrival_exit)
            if known is not None and known != e.id:
                # the arrival exit is already connected by another edge: merge into it
                k = self.edges[known]
                k.traversals += 1
                del self.edges[e.id]
                src = self.junctions[e.a]
                src.edge_at[e.dir_a] = known
                if k.b is None:
                    k.b, k.dir_b = e.a, e.dir_a
                edge_id = known
            else:
                if e.b is None:
                    loop = (not is_new) and (e.a != best.id or e.dir_a != arrival_exit)
                    e.b, e.dir_b = best.id, arrival_exit
                e.traversals += 1
                best.edge_at[arrival_exit] = e.id
            self._pending = None

        self.current = best.id
        arr = Arrival(best.id, is_new, heading, arrival_exit, exits, (cx, cy),
                      None if is_new else best_d, tuple(rejected), edge_id, loop,
                      "NEW" if is_new else "MATCH")
        self.log.append(arr)
        return arr

    def depart(self, direction: Dir) -> int:
        """Robot leaves the current junction through `direction`. Returns the edge id."""
        j = self.junctions[self.current]
        if direction not in j.exits:
            raise ValueError(f"junction {j.id} has no exit {direction.value}")
        eid = j.edge_at.get(direction)
        if eid is None:
            eid = max(self.edges, default=-1) + 1
            self.edges[eid] = Edge(eid, j.id, direction)
            j.edge_at[direction] = eid
        self._pending = eid
        return eid

    # ------------------------------------------------------------ queries
    def unexplored_exits(self, jid: int) -> list[Dir]:
        j = self.junctions[jid]
        return [d for d in _ORDER if d in j.exits and d not in j.edge_at]

    def neighbour(self, jid: int, direction: Dir) -> int | None:
        eid = self.junctions[jid].edge_at.get(direction)
        if eid is None:
            return None
        return self.edges[eid].other(jid)[0]

    def summary(self) -> dict:
        closed = [e for e in self.edges.values() if e.b is not None]
        return {
            "junctions": len(self.junctions),
            "edges": len(closed),
            "open_edges": len(self.edges) - len(closed),
            "loop_closures": sum(a.loop_closure for a in self.log),
            "structure_rejections": sum(bool(a.rejected_structure) for a in self.log),
            "edge_associations": sum(a.association == "EDGE" for a in self.log),
            "inconsistent_edge_associations": sum(a.inconsistent for a in self.log),
        }


DECISION_TURNS = {"LEFT": 1, "STRAIGHT": 0, "RIGHT": -1, "UTURN": 2}


class MemoryRecorder:
    """Decision-provider wrapper: records every junction event in MazeMemory
    with the current pose estimate, asks the inner policy, records the departure.
    (Step 11's DFS uses the memory directly; this wrapper serves reactive policies.)"""

    def __init__(self, inner, memory: MazeMemory, pose_fn) -> None:
        self.inner, self.memory, self.pose_fn = inner, memory, pose_fn

    def decide(self, obs: JunctionObservation):
        x, y, th = self.pose_fn()
        arr = self.memory.observe(obs, x, y, th)
        d = self.inner.decide(obs)
        if d.value in DECISION_TURNS:
            self.memory.depart(rotate(arr.heading, DECISION_TURNS[d.value]))
        return d
