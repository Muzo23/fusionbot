"""
Mission (step 13d): explore the whole maze, come back to START, then drive to the GOAL.

    EXPLORE   DfsExplorer (step 11) until it finishes "complete" at START
              (any other end - budget / inconsistent - ends the mission there)
    TO_GOAL   shortest path in the robot's OWN topological map (MazeMemory,
              Dijkstra, edge length = distance between the estimated junction
              centres) from START to the junction nearest to the goal position;
              decisions are issued junction by junction, every arrival is
              associated in the memory as during exploration
    done      FINISH at the goal junction ("goal_reached"), or a graceful FINISH
              with the reason when the map does not match the world any more

The goal is given like in a micromouse contest: as a POSITION in the start frame
(here the world coordinates of the maze's goal node, since odometry starts at the
known START pose). Which junction of the robot's map is "the goal" therefore
depends on the configuration's pose estimate - a drifted map can put the goal
at the wrong junction, which the evaluation detects with GT.
"""

from __future__ import annotations

import heapq
import math
from dataclasses import dataclass

from navigation import navigation_config as cfg
from navigation.junction_types import ManeuverDecision, ReferenceKind
from navigation.maze_decision import DfsExplorer, relative_decision
from navigation.maze_memory import Arrival, Dir, MazeMemory, opposite

GOAL_SEARCH_RADIUS_M = 0.55          # half a lattice pitch: the goal junction must be this close


@dataclass(frozen=True)
class MissionStep:
    phase: str                       # explore / to_goal
    arrival: Arrival
    reason: str
    decision: ManeuverDecision


def shortest_path(memory: MazeMemory, src: int, dst: int):
    """Dijkstra over the closed edges of the memory. Returns [(junction, exit dir to take)], ending
    with (dst, None), or None if unreachable."""
    J = memory.junctions
    dist, prev = {src: 0.0}, {}
    pq = [(0.0, src)]
    while pq:
        d, u = heapq.heappop(pq)
        if u == dst:
            break
        if d > dist.get(u, math.inf):
            continue
        for direction, eid in J[u].edge_at.items():
            e = memory.edges[eid]
            if e.b is None or e.blocked:
                continue
            v = e.other(u)[0]
            w = math.hypot(J[u].x - J[v].x, J[u].y - J[v].y)
            if d + w < dist.get(v, math.inf) - 1e-12:
                dist[v], prev[v] = d + w, (u, direction)
                heapq.heappush(pq, (d + w, v))
    if dst not in dist:
        return None
    path, v = [(dst, None)], dst
    while v != src:
        u, direction = prev[v]
        path.append((u, direction))
        v = u
    return list(reversed(path)), dist[dst]


class MissionProvider:
    def __init__(self, memory: MazeMemory, pose_fn, *, start_pose, goal_xy, max_events: int | None = None,
                 goal_radius_m: float = GOAL_SEARCH_RADIUS_M) -> None:
        self.memory, self.pose_fn = memory, pose_fn
        self.dfs = DfsExplorer(memory, pose_fn, start_pose=start_pose, max_events=max_events)
        self.goal_xy = tuple(goal_xy)
        self.goal_radius = goal_radius_m
        self.phase = "explore"
        self.steps: list[MissionStep] = []
        self.finished_reason: str | None = None
        self.goal_junction: int | None = None
        self.goal_distance_m: float | None = None
        self.route: list | None = None
        self.route_length_m: float | None = None
        self.explore_events: int | None = None
        # step 14b: odometric edge lengths (junction centre -> junction centre, measured while driving)
        # and the expected FRONT reading on the edge being driven in the goal run
        self.edge_length_m: dict[int, float] = {}
        self._s_centre_last = 0.0                    # odometry at the centre of the junction left last (START = 0)
        self.expect: tuple[float, float, bool] | None = None    # (odometry at departure, edge length, open ahead)
        self.blocked_log: list[dict] = []

    # ------------------------------------------------------------------ helpers
    def _finish(self, arr: Arrival, reason: str) -> ManeuverDecision:
        self.finished_reason, self.phase = reason, "done"
        self.steps.append(MissionStep("to_goal", arr, reason, ManeuverDecision.FINISH))
        return ManeuverDecision.FINISH

    def _leave(self, obs, arr: Arrival, direction: Dir, reason: str) -> ManeuverDecision:
        dec = relative_decision(arr.heading, direction)
        if not obs.available(dec):
            return self._finish(arr, "inconsistent")
        eid = self.memory.depart(direction)
        self._set_expect(eid, direction)
        self.steps.append(MissionStep("to_goal", arr, reason, dec))
        return dec

    # ------------------------------------------------------------------ step 14b: obstacles
    @staticmethod
    def _centre_odometry(obs) -> float:
        d = cfg.D_SIDE_CANONICAL_M if obs.kind is ReferenceKind.SIDE_CONFIRMED else cfg.D_DEAD_END_CANONICAL_M
        return obs.reference_odom_distance_m + d

    def _measure_edge(self, obs, arr: Arrival) -> None:
        s_c = self._centre_odometry(obs)
        if arr.edge is not None and arr.association != "RETURN" and arr.edge not in self.edge_length_m:
            self.edge_length_m[arr.edge] = s_c - self._s_centre_last
        self._s_centre_last = s_c

    def _set_expect(self, eid: int, direction: Dir) -> None:
        e = self.memory.edges.get(eid)
        L = self.edge_length_m.get(eid)
        if e is None or L is None or e.b is None:
            self.expect = None
            return
        tgt = e.other(self.memory.current)[0]
        self.expect = (self._s_centre_last, L, direction in self.memory.junctions[tgt].exits)

    def front_expected_min(self, odo_s: float) -> float | None:
        """Lower bound of the FRONT reading on the edge being driven (goal run only), from the edge
        length the robot measured itself: the next junction's far wall (or farther if its straight is
        open). A FRONT reading clearly shorter than this is an obstacle that is not in the map."""
        if self.phase != "to_goal" or self.expect is None:
            return None
        s0, L, open_ahead = self.expect
        rem = L - (odo_s - s0)
        wall = rem + 0.5 * cfg.CORRIDOR_WIDTH_M - cfg.FRONT_SENSOR_AHEAD_M
        return wall + (cfg.LATTICE_PITCH_M - cfg.CORRIDOR_WIDTH_M if open_ahead else 0.0)

    def on_blocked(self, odo_s: float) -> ManeuverDecision:
        """The FSM waited OBSTACLE_WAIT_MAX_S and the obstacle is still there: treat the edge as blocked,
        plan around it from the junction behind and turn back (FINISH if there is no other way)."""
        s0, L, _ = self.expect if self.expect else (odo_s, 0.0, False)
        jid = self.memory.current
        eid = self.memory.turn_back()
        rec = {"junction": jid, "edge": eid, "travelled_m": odo_s - s0}
        self.blocked_log.append(rec)
        res = shortest_path(self.memory, jid, self.goal_junction) if self.goal_junction is not None else None
        if res is None:
            rec["result"] = "goal_blocked"
            self.finished_reason, self.phase = "goal_blocked", "done"
            self.expect = None
            return ManeuverDecision.FINISH
        self.route, _ = res
        rec["result"] = "replanned"
        self.expect = (odo_s, odo_s - s0, False)      # back to the junction behind
        return ManeuverDecision.UTURN

    def _plan(self, src: int) -> bool:
        J = self.memory.junctions
        cands = [(math.hypot(j.x - self.goal_xy[0], j.y - self.goal_xy[1]), j.id) for j in J.values() if j.id != 0]
        if not cands:
            return False
        d, gid = min(cands)
        self.goal_distance_m = d
        if d > self.goal_radius:
            return False
        self.goal_junction = gid
        res = shortest_path(self.memory, src, gid)
        if res is None:
            return False
        self.route, self.route_length_m = res
        return True

    # ------------------------------------------------------------------ provider
    def decide(self, obs) -> ManeuverDecision:
        if self.phase == "explore":
            dec = self.dfs.decide(obs)
            st = self.dfs.steps[-1]
            self._measure_edge(obs, st.arrival)
            self.steps.append(MissionStep("explore", st.arrival, st.reason, dec))
            if dec is not ManeuverDecision.FINISH:
                return dec
            self.explore_events = len(self.dfs.steps)
            if self.dfs.finished_reason != "complete":
                self.finished_reason, self.phase = f"explore_{self.dfs.finished_reason}", "done"
                return dec
            # explored, back at START: plan the goal run and leave START again
            self.steps.pop()
            self.phase = "to_goal"
            if not self._plan(st.arrival.junction):
                self.finished_reason, self.phase = "goal_not_in_map", "done"
                self.steps.append(MissionStep("to_goal", st.arrival, "goal_not_in_map", ManeuverDecision.FINISH))
                return ManeuverDecision.FINISH
            return self._leave(obs, st.arrival, self.route[0][1], "goal_run")

        # ---- to_goal: every arrival is associated in the memory like during exploration
        x, y, th = self.pose_fn()
        arr = self.memory.observe(obs, x, y, th)
        self._measure_edge(obs, arr)
        self.expect = None
        if arr.junction == self.goal_junction:
            return self._finish(arr, "goal_reached")
        nxt = {j: d for j, d in self.route[:-1]}
        if arr.junction not in nxt:                     # off the planned route: replan from here
            res = shortest_path(self.memory, arr.junction, self.goal_junction)
            if res is None:
                return self._finish(arr, "goal_lost")
            self.route, _ = res
            nxt = {j: d for j, d in self.route[:-1]}
        return self._leave(obs, arr, nxt[arr.junction], "goal_run")


def optimal_goal_route(spec) -> tuple[list[str], float]:
    """Ground-truth shortest path ROOT -> goal in the maze graph (evaluation only), length in metres
    including the START approach (start pose -> ROOT)."""
    from mazes.maze_spec import START_OFFSET_FROM_ROOT_M
    nodes = spec.nodes_m
    adj = {n: [] for n in nodes}
    for a, b in spec.edges:
        w = math.hypot(nodes[a][0] - nodes[b][0], nodes[a][1] - nodes[b][1])
        adj[a].append((b, w))
        adj[b].append((a, w))
    dist, prev, pq = {spec.root: 0.0}, {}, [(0.0, spec.root)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist[u]:
            continue
        for v, w in adj[u]:
            if d + w < dist.get(v, math.inf):
                dist[v], prev[v] = d + w, u
                heapq.heappush(pq, (d + w, v))
    path, v = [spec.goal], spec.goal
    while v != spec.root:
        v = prev[v]
        path.append(v)
    return list(reversed(path)), dist[spec.goal] + START_OFFSET_FROM_ROOT_M


__all__ = ["MissionProvider", "MissionStep", "shortest_path", "optimal_goal_route", "opposite"]
