"""Step 13d (offline): explore -> back at START -> shortest known path to the GOAL.

    python -m unittest discover -s tests -p "test_step13d*" -v
"""

import math
import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from maze_sim import LIVE_TURN_DEFICIT, MazeSim
from mazes.maze_spec import MAZES
from navigation.maze_memory import MazeMemory
from navigation.mission import MissionProvider, optimal_goal_route
from navigation.navigation_fsm import NavigationFSM, NavState


def run(maze_id, deficit=True):
    spec = MAZES[maze_id]
    sim = MazeSim(spec, turn_deficit=LIVE_TURN_DEFICIT if deficit else None)
    mem, odo = MazeMemory(), sim.plant.odo
    mis = MissionProvider(mem, lambda: (odo.pose.x_m, odo.pose.y_m, odo.pose.theta_rad),
                          start_pose=spec.start_pose, goal_xy=spec.nodes_m[spec.goal], max_events=500)
    fsm = NavigationFSM(mis)
    sim.run(fsm, max_s=5000)
    return spec, sim, fsm, mis


class TestMission(unittest.TestCase):
    def test_t2_explore_then_goal(self):
        spec, sim, fsm, mis = run("T2")
        self.assertIs(fsm.state, NavState.FINISHED)
        self.assertEqual(mis.finished_reason, "goal_reached")
        self.assertEqual(mis.dfs.finished_reason, "complete")
        self.assertFalse(sim.collision)
        gx, gy = spec.nodes_m[spec.goal]
        self.assertLess(math.hypot(sim.plant.x - gx, sim.plant.y - gy), 0.6)    # stops at the goal junction event
        phases = [s.phase for s in mis.steps]
        self.assertEqual(phases[0], "explore")
        self.assertEqual(phases[-1], "to_goal")
        # the goal run follows a shortest path: number of goal-run junctions = optimal route nodes
        n_goal = sum(p == "to_goal" for p in phases)
        self.assertLessEqual(n_goal, len(optimal_goal_route(spec)[0]) + 1)

    def test_m1_real_maze_goal(self):
        spec, sim, fsm, mis = run("M1")
        self.assertEqual(mis.finished_reason, "goal_reached")
        self.assertFalse(sim.collision)

    def test_optimal_route(self):
        path, length = optimal_goal_route(MAZES["T3"])
        self.assertEqual(path[0], "R")
        self.assertEqual(path[-1], "F")
        self.assertGreater(length, 10.0)


if __name__ == "__main__":
    unittest.main()
