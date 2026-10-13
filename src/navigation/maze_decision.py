"""
FusionBot maze exploration: depth-first search over MazeMemory (step 11).

Trémaux / DFS rules, evaluated at every junction event from the FSM:

  1. LOOP CLOSURE: arrived through a NEW corridor at a junction that is already
     known (metric recognition, MazeMemory association "MATCH")  ->  turn back
     through the same corridor (it is now traversed twice; the junction keeps
     its original DFS parent).
  2. EXPLORE: the junction has an exit that is not yet connected to any edge
     -> take it (preference LEFT, STRAIGHT, RIGHT relative to the robot,
     deterministic).
  3. BACKTRACK: everything explored -> leave through the exit by which the
     junction was FIRST entered (its DFS parent edge).
  4. FINISH: back at START (memory junction 0, reached as the dead end behind
     the start pose) with nothing left to explore. The robot stops there.

Consistency guard: if the chosen manoeuvre is not physically available in
the current observation (the memory no longer matches the world, e.g. the
heading estimate drifted past 45 deg so global directions are snapped wrongly),
the explorer stops with FINISH, reason "inconsistent" - a graceful, measurable
failure of the pose estimate instead of a safety abort.

A budget (max_events) stops the exploration with FINISH if it does not
terminate (e.g. a missed loop closure in configuration A makes the robot
re-explore a loop as if it were new); the reason is recorded.

Every junction event is placed with the pose estimate of the configuration
(pose_fn); returns along known corridors are associated topologically by
MazeMemory, so only loop closures depend on the pose estimate.
"""

from __future__ import annotations

from dataclasses import dataclass

from navigation.junction_types import JunctionObservation, ManeuverDecision
from navigation.maze_memory import Arrival, Dir, MazeMemory, rotate

PREFERENCE = (("LEFT", 1), ("STRAIGHT", 0), ("RIGHT", -1))
_REL = {0: ManeuverDecision.STRAIGHT, 1: ManeuverDecision.LEFT, 2: ManeuverDecision.UTURN, 3: ManeuverDecision.RIGHT}
_ORDER = (Dir.E, Dir.N, Dir.W, Dir.S)


def relative_decision(heading: Dir, target: Dir) -> ManeuverDecision:
    return _REL[(_ORDER.index(target) - _ORDER.index(heading)) % 4]


@dataclass(frozen=True)
class DfsStep:
    event: int
    arrival: Arrival
    reason: str                 # explore / backtrack / loop_closure_return / complete / budget
    direction: Dir | None
    decision: ManeuverDecision


class DfsExplorer:
    def __init__(self, memory: MazeMemory, pose_fn, *, start_pose=None, max_events: int | None = None) -> None:
        self.memory = memory
        self.pose_fn = pose_fn
        self.max_events = max_events
        if start_pose is not None:
            memory.start(*start_pose)
        if not memory.junctions:
            raise ValueError("memory must be started (start_pose) before exploring")
        self.parent_exit: dict[int, Dir] = {}
        self.steps: list[DfsStep] = []
        self.finished_reason: str | None = None

    def decide(self, obs: JunctionObservation) -> ManeuverDecision:
        x, y, th = self.pose_fn()
        arr = self.memory.observe(obs, x, y, th)
        jid = arr.junction
        if arr.is_new:
            self.parent_exit[jid] = arr.arrival_exit
        j = self.memory.junctions[jid]

        if self.max_events is not None and len(self.steps) >= self.max_events:
            return self._finish(arr, "budget")
        if arr.association == "MATCH" and arr.loop_closure:
            return self.decide_checked(obs, arr, arr.arrival_exit, "loop_closure_return")
        for _, q in PREFERENCE:
            d = rotate(arr.heading, q)
            if d in arr.exits and d not in j.edge_at:
                return self._go(arr, d, "explore")
        parent = self.parent_exit.get(jid)
        if jid == 0 or parent is None:
            return self._finish(arr, "complete")
        return self.decide_checked(obs, arr, parent, "backtrack")

    def decide_checked(self, obs, arr, d, reason):
        dec = relative_decision(arr.heading, d)
        if not obs.available(dec):
            return self._finish(arr, "inconsistent")
        return self._go(arr, d, reason)

    def _go(self, arr: Arrival, d: Dir, reason: str) -> ManeuverDecision:
        dec = relative_decision(arr.heading, d)
        self.memory.depart(d)
        self.steps.append(DfsStep(len(self.steps), arr, reason, d, dec))
        return dec

    def _finish(self, arr: Arrival, reason: str) -> ManeuverDecision:
        self.finished_reason = reason
        self.steps.append(DfsStep(len(self.steps), arr, reason, None, ManeuverDecision.FINISH))
        return ManeuverDecision.FINISH
