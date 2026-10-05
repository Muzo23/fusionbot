"""Step 9 acceptance (offline): navigation FSM.

Unit tests drive the FSM with synthetic proximity snapshots; integration tests
run it through T1/T2/T3 in the offline maze simulator (plant with the live-fitted
rotation friction) with the left-hand policy and compare every junction event
with the ground-truth traversal of the maze graph.

    python -m unittest discover -s tests -p "test_step09*" -v
"""

import math
import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from maze_sim import MazeSim
from mazes.maze_spec import MAZES
from mazes.topology import expected_left_hand_events
from navigation import navigation_config as cfg
from navigation.junction_types import ManeuverDecision, ReferenceKind
from navigation.motion_controller import MotionMode
from navigation.navigation_fsm import NavigationFSM, NavState, Phase
from navigation.simple_policies import LeftHandPolicy
from perception.proximity_types import ProximityReading, ProximitySnapshot

WALL_SIDE, WALL_DIAG = 0.184, 0.340


def snap(left=WALL_SIDE, fl=WALL_DIAG, front=None, fr=WALL_DIAG, right=WALL_SIDE):
    def r(n, d):
        return ProximityReading(n, d is not None, d)
    return ProximitySnapshot(r("left", left), r("front_left", fl), r("front", front),
                             r("front_right", fr), r("right", right))


class Scripted:
    def __init__(self, *decisions):
        self.d = list(decisions)
        self.seen = []

    def decide(self, obs):
        self.seen.append(obs)
        return self.d.pop(0)


def tick(fsm, s, ds=0.0025, dth=0.0, counts=(8, 8), odo=[0.0]):
    odo[0] += ds
    return fsm.update(s, odo_distance_m=odo[0], odo_dtheta_rad=dth,
                      left_counts=counts[0], right_counts=counts[1])


class TestTransitions(unittest.TestCase):
    def setUp(self):
        tick.__defaults__[-1][0] = 0.0

    def test_init_to_follow(self):
        fsm = NavigationFSM(Scripted())
        st = tick(fsm, snap())
        self.assertIs(st.state, NavState.FOLLOW)
        self.assertTrue(st.reset_wheel_pi and st.reset_openings)
        self.assertEqual(st.v_ref_m_s, cfg.NAV_SPEED_M_S)
        self.assertIs(st.motion_mode, MotionMode.STRAIGHT)
        self.assertFalse(tick(fsm, snap()).reset_wheel_pi)       # one-shot

    def test_candidate_and_false_candidate(self):
        fsm = NavigationFSM(Scripted())
        tick(fsm, snap())
        self.assertIs(tick(fsm, snap(fl=None)).state, NavState.APPROACH)
        self.assertIs(tick(fsm, snap()).state, NavState.FOLLOW)  # diagonal back on the wall

    def test_closed_junction_left_turn_sequence(self):
        prov = Scripted(ManeuverDecision.LEFT)
        fsm = NavigationFSM(prov)
        tick(fsm, snap())
        tick(fsm, snap(fl=None, front=0.57))                     # CANDIDATE left
        st = tick(fsm, snap(left=None, fl=None, front=0.57))     # CONFIRMED -> event
        self.assertIsNotNone(st.observation)
        o = st.observation
        self.assertEqual((o.kind, o.left_available, o.straight_available, o.right_available),
                         (ReferenceKind.SIDE_CONFIRMED, True, False, False))
        self.assertIs(st.state, NavState.POSITION)
        self.assertIs(st.phase, Phase.ADVANCE)
        st = tick(fsm, snap(left=None, fl=None, front=0.05))      # FRONT_BLOCKED -> brake
        self.assertIs(st.phase, Phase.BRAKE)
        self.assertEqual(st.stop_guard, "FRONT_BLOCKED")
        self.assertTrue(st.reset_wheel_pi)
        self.assertEqual(st.v_ref_m_s, 0.0)
        for _ in range(cfg.STOP_ZERO_COUNT_SAMPLES):
            st = tick(fsm, snap(left=None, fl=None, front=0.04), ds=0, counts=(0, 0))
        self.assertIs(st.phase, Phase.HOLD)
        axis0 = fsm.guidance.axis
        for _ in range(int(cfg.PRE_TURN_HOLD_S / 0.01)):
            st = tick(fsm, snap(left=None, fl=None, front=0.04), ds=0, counts=(0, 0))
        self.assertIs(st.state, NavState.TURN)
        self.assertGreater(st.omega_ref_rad_s, 0.0)               # LEFT = CCW
        self.assertAlmostEqual(fsm.guidance.axis - axis0, math.pi / 2)

    def test_dead_end_stops_immediately(self):
        fsm = NavigationFSM(Scripted(ManeuverDecision.UTURN))
        tick(fsm, snap())
        st = tick(fsm, snap(front=0.05))
        self.assertEqual(st.observation.kind, ReferenceKind.DEAD_END)
        self.assertEqual(st.v_ref_m_s, 0.0)
        self.assertEqual((st.state, st.phase), (NavState.POSITION, Phase.BRAKE))

    def test_invalid_decision_aborts(self):
        fsm = NavigationFSM(Scripted(ManeuverDecision.RIGHT))
        tick(fsm, snap())
        tick(fsm, snap(fl=None, front=0.57))
        st = tick(fsm, snap(left=None, fl=None, front=0.57))
        self.assertIs(st.state, NavState.SAFETY_ABORT)
        self.assertIn("decision_not_available", st.fault)

    def test_straight_into_dead_end_aborts(self):
        fsm = NavigationFSM(Scripted(ManeuverDecision.STRAIGHT))
        tick(fsm, snap())
        self.assertIs(tick(fsm, snap(front=0.05)).state, NavState.SAFETY_ABORT)

    def test_finish(self):
        fsm = NavigationFSM(Scripted(ManeuverDecision.FINISH))
        tick(fsm, snap())
        st = tick(fsm, snap(front=0.05))
        self.assertIs(st.state, NavState.FINISHED)
        self.assertEqual(tick(fsm, snap()).v_ref_m_s, 0.0)

    def test_front_safety(self):
        fsm = NavigationFSM(Scripted())
        tick(fsm, snap())
        st = tick(fsm, snap(front=0.02))
        self.assertIs(st.state, NavState.SAFETY_ABORT)
        self.assertIn("front_safety", st.fault)

    def test_external_safety_event(self):
        fsm = NavigationFSM(Scripted())
        tick(fsm, snap())
        st = fsm.update(snap(), odo_distance_m=0.1, odo_dtheta_rad=0.0, left_counts=1, right_counts=1,
                        safety_event=True)
        self.assertIs(st.state, NavState.SAFETY_ABORT)

    def test_closed_guard_uses_front_at_reference(self):
        fsm = NavigationFSM(Scripted(ManeuverDecision.LEFT))
        tick(fsm, snap())
        tick(fsm, snap(fl=None, front=0.57))
        tick(fsm, snap(left=None, fl=None, front=0.57))          # reference, front 0.57
        expected = 0.57 - 0.057581
        st = None
        for _ in range(400):                                      # FRONT never blocks
            st = tick(fsm, snap(left=None, fl=None, front=0.40))
            if st.state is NavState.SAFETY_ABORT:
                break
        self.assertIn("closed_straight_guard", st.fault)
        travelled = tick.__defaults__[-1][0] - fsm.observation.reference_odom_distance_m
        self.assertAlmostEqual(travelled, expected + cfg.CLOSED_GUARD_MARGIN_M, delta=0.003)

    def test_exit_reacquire_needs_five_ticks_and_watches_front(self):
        fsm = NavigationFSM(Scripted(ManeuverDecision.STRAIGHT, ManeuverDecision.STRAIGHT))
        tick(fsm, snap())
        tick(fsm, snap(fl=None))
        st = tick(fsm, snap(left=None, fl=None))                  # straight open -> EXIT
        self.assertIs(st.state, NavState.EXIT)
        for _ in range(cfg.REACQUIRE_SAMPLES - 1):
            self.assertIs(tick(fsm, snap()).state, NavState.EXIT)
        st = tick(fsm, snap())
        self.assertIs(st.state, NavState.FOLLOW)
        self.assertTrue(st.reset_openings)
        # second junction: STRAIGHT, then a wall appears in front during EXIT
        tick(fsm, snap(fl=None))
        tick(fsm, snap(left=None, fl=None))
        st = tick(fsm, snap(left=None, fl=None, front=0.05))
        self.assertIn("front_blocked_during_exit", st.fault)


class TestMazes(unittest.TestCase):
    """Full runs in the offline maze simulator with the left-hand policy."""

    def _run(self, maze_id, n):
        spec = MAZES[maze_id]
        pol = LeftHandPolicy(max_events=n)
        fsm = NavigationFSM(pol)
        sim = MazeSim(spec)
        residual = []

        def on_tick(_, st):
            if st.closed_front_trigger_m is not None:
                residual.append(st.closed_front_trigger_m - st.closed_front_expected_m)
        events = sim.run(fsm, max_s=1500, on_tick=on_tick)
        self.assertIs(fsm.state, NavState.FINISHED, fsm.fault)
        self.assertFalse(sim.collision)
        self.assertGreater(sim.min_clearance, 0.010)
        expected = expected_left_hand_events(spec, n)
        got = []
        for _, (x, y, th), st in events[:n]:
            o = st.observation
            ahead = cfg.D_SIDE_CANONICAL_M if o.kind is ReferenceKind.SIDE_CONFIRMED else cfg.D_DEAD_END_CANONICAL_M
            got.append((sim.nearest_node(x, y, th, ahead), o.left_available, o.straight_available,
                        o.right_available, st.decision.value))
        self.assertEqual(got, [tuple(e) for e in expected])
        self.assertTrue(residual)
        self.assertLess(max(abs(r) for r in residual), 0.005)    # margin is 15 mm
        return sim

    def test_t1_full_left_hand_traversal(self):
        self._run("T1", 44)            # whole tree, back to ROOT, U-turn behind START

    def test_t2_loops(self):
        self._run("T2", 20)

    def test_t3_long_corridors(self):
        self._run("T3", 16)


if __name__ == "__main__":
    unittest.main()
