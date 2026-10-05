"""Offline maze simulator for FSM tests (not a validation substitute).

DrivePlant with the rotation-mode friction fitted live in step 8, the maze
walls from mazes.maze_geometry, single-ray proximity sensors with the mounts
derived from the step-8a live fit, and a rough footprint collision check.
The loop reproduces the live runtime order: measure -> FSM -> PI -> apply.
"""

import math

from plant import DrivePlant

from corridor_world import MOUNTS
from core.yaw_friction import yaw_friction_feedforward
from mazes.maze_geometry import WallMap
from navigation.motion_controller import MotionMode
from perception.proximity_types import ProximityReading, ProximitySnapshot

LIVE_YAW = dict(yaw_inertia=0.004, yaw_friction=0.0875)
# rough robot outline (robot frame, m): front bumper, front corners, sides, rear corners
BODY_POINTS = ((0.335, 0.0), (0.33, 0.09), (0.33, -0.09), (0.15, 0.17), (0.15, -0.17),
               (-0.10, 0.17), (-0.10, -0.17), (0.0, 0.17), (0.0, -0.17))


class MazeSim:
    def __init__(self, spec, yaw=LIVE_YAW):
        self.walls = WallMap(spec)
        x, y, th = spec.start_pose
        self.plant = DrivePlant(x, y, th, **yaw)
        self.t = 0.0
        self.collision = False
        self.min_clearance = math.inf

    def snapshot(self):
        p = self.plant
        c, s = math.cos(p.th), math.sin(p.th)
        out = {}
        for n, (mx, my, ma, rng) in MOUNTS.items():
            d = self.walls.raycast(p.x + c * mx - s * my, p.y + s * mx + c * my, p.th + ma, rng)
            det = math.isfinite(d)
            out[n] = ProximityReading(n, det, d if det else None)
        return ProximitySnapshot(**out)

    def _check_body(self):
        p = self.plant
        c, s = math.cos(p.th), math.sin(p.th)
        for bx, by in BODY_POINTS:
            x, y = p.x + c * bx - s * by, p.y + s * bx + c * by
            if not self.walls.is_free(x, y):
                self.collision = True
            else:
                self.min_clearance = min(self.min_clearance, self.walls.distance_to_walls(x, y))

    def run(self, fsm, max_s=600.0, on_tick=None):
        """Run until the FSM terminates or max_s. Returns the list of NavSteps with an event."""
        events = []
        while self.t < max_s:
            inc = self.plant.advance()
            sl, sr = self.plant.last
            self.t += 0.01
            self._check_body()
            step = fsm.update(self.snapshot(), odo_distance_m=self.plant.odo.signed_distance_m,
                              odo_dtheta_rad=inc.dtheta_rad, left_counts=sl.delta_counts,
                              right_counts=sr.delta_counts, safety_event=self.collision)
            if step.reset_wheel_pi:
                self.plant.reset_pi()
            ff = yaw_friction_feedforward(step.omega_ref_rad_s) if step.motion_mode is MotionMode.STRAIGHT else 0.0
            self.plant.control(step.v_ref_m_s, step.omega_ref_rad_s, ff)
            if step.observation is not None:
                events.append((self.t, (self.plant.x, self.plant.y, self.plant.th), step))
            if on_tick:
                on_tick(self, step)
            if step.state.value in ("FINISHED", "SAFETY_ABORT"):
                break
        return events

    def nearest_node(self, x, y, th, ahead):
        """Junction node closest to the point `ahead` m in front of the robot."""
        px, py = x + ahead * math.cos(th), y + ahead * math.sin(th)
        nodes = self.walls.spec.nodes_m
        best = min(nodes, key=lambda n: math.hypot(nodes[n][0] - px, nodes[n][1] - py))
        sx, sy, _ = self.walls.spec.start_pose
        if math.hypot(sx - px, sy - py - 0.0) < math.hypot(nodes[best][0] - px, nodes[best][1] - py) \
                and py < nodes[self.walls.spec.root][1] - 0.6:
            return "__START__"
        return best
