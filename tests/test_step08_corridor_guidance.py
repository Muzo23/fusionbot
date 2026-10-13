"""Step 8 acceptance (offline): corridor measurement model and guidance law.

    python -m unittest tests.test_step08_corridor_guidance -v
"""

import math
import unittest

from plant import DrivePlant  # noqa: F401  (adds src/ to sys.path)

from corridor_world import HALF_WIDTH_M, CorridorWorld
from navigation.corridor_guidance import (
    OMEGA_MAX_RAD_S, CorridorGuidance, measure_corridor,
)
from core.yaw_friction import YAW_FF_COMMAND, yaw_friction_feedforward
from navigation.motion_controller import MotionController
from perception.proximity_types import ProximityReading

V = 0.25
ROBOT_HALF_WIDTH_M = 0.17


# rotation-mode model fitted to live step-8c run 1 (rms 0.29 deg)
LIVE_YAW = dict(yaw_inertia=0.004, yaw_friction=0.0875)


def drive(world, x0=0.0, y0=0.0, th0=0.0, dist=2.0, guided=True, guidance=None,
          plant_kw=None, ff=False):
    """Straight at V with (or without) guidance. Returns list of (x, y, th, mode, w)."""
    p = DrivePlant(x0, y0, th0, **(plant_kw or {}))
    g = guidance or CorridorGuidance()
    trace, inc = [], p.step(0.0, 0.0)
    while p.x - x0 < dist:
        g.update_odometry(inc.dtheta_rad)
        out = g.command(world.snapshot(p.x, p.y, p.th))
        w = out.omega_ref_rad_s if guided else 0.0
        inc = p.step(V, w, yaw_friction_feedforward(w) if ff else 0.0)
        trace.append((p.x, p.y, p.th, out.mode, w))
    return trace, p, g


class TestMeasurementModel(unittest.TestCase):
    def test_inverse_reproduces_pose(self):
        """World mounts come from the live per-sensor fit; the inverse model
        (from the live s/e fit) must recover (y, alpha) over the calibrated range."""
        w = CorridorWorld()
        worst_y = worst_a = 0.0
        for y in (-0.04, -0.02, 0.0, 0.02, 0.04):
            for a_deg in (-5, -2.5, 0, 2.5, 5):
                m = measure_corridor(w.snapshot(1.0, y, math.radians(a_deg)))
                self.assertIsNotNone(m)
                worst_y = max(worst_y, abs(m.y_m - y))
                worst_a = max(worst_a, abs(math.degrees(m.alpha_rad) - a_deg))
        self.assertLess(worst_y, 0.001)       # < 1 mm   (live rms 0.026 mm)
        # corners (|y| 40 mm AND |alpha| 5 deg together) lie at the edge of the
        # live sample cloud, where the linearisation error is largest
        self.assertLess(worst_a, 0.15)        # live rms 0.015 deg

    def test_centred_reads_zero(self):
        m = measure_corridor(CorridorWorld().snapshot(1.0, 0.0, 0.0))
        self.assertAlmostEqual(m.y_m, 0.0, delta=2e-4)
        self.assertAlmostEqual(math.degrees(m.alpha_rad), 0.0, delta=0.02)

    def test_invalid_inputs(self):
        snap = CorridorWorld().snapshot(1.0, 0.0, 0.0)
        lost = ProximityReading("front_left", False, None)
        self.assertIsNone(measure_corridor(snap.__class__(**{**snap.as_dict(), "front_left": lost})))
        far = ProximityReading("right", True, 0.40)
        self.assertIsNone(measure_corridor(snap.__class__(**{**snap.as_dict(), "right": far})))
        near_front = ProximityReading("front", True, 0.20)
        self.assertIsNone(measure_corridor(snap.__class__(**{**snap.as_dict(), "front": near_front})))


class TestSingleWall(unittest.TestCase):
    """Step 13b-6: one side open, the other a plain wall."""

    def test_single_wall_model_recovers_pose(self):
        from navigation.corridor_guidance import measure_single_wall
        w = CorridorWorld()
        for side in ("left", "right"):
            for y in (-0.03, 0.0, 0.03):
                for a_deg in (-4, 0, 4):
                    m = measure_single_wall(w.snapshot(1.0, y, math.radians(a_deg)), side)
                    self.assertIsNotNone(m)
                    self.assertLess(abs(m.y_m - y), 0.004, (side, y, a_deg, m))
                                # linear per-sensor model: ~0.8 deg at the corner of the range (enough to stop a drift)
                    self.assertLess(abs(math.degrees(m.alpha_rad) - a_deg), 1.0, (side, y, a_deg, m))

    def test_wrong_held_axis_corrected_by_the_remaining_wall(self):
        """4 m with the left side open and a held axis 4 deg wrong (turn losses piled up):
        HOLD alone would steer into the right wall; the single wall keeps the robot centred."""
        world = CorridorWorld(gaps=(("left", -1.0, 5.0),))
        g = CorridorGuidance()
        g.axis = math.radians(4.0)
        tr, p, g = drive(world, dist=4.0, guidance=g)
        self.assertIn("RIGHT", {m for *_, m, _ in tr})
        self.assertLess(max(abs(y) for _, y, *_ in tr), 0.03)
        self.assertLess(abs(math.degrees(tr[-1][2])), 0.5)
        tr2, *_ = drive(world, dist=4.0, guidance=_HoldOnly())     # reference: HOLD only drifts
        self.assertGreater(max(abs(y) for _, y, *_ in tr2), 0.10)


class _HoldOnly(CorridorGuidance):
    def __init__(self):
        super().__init__()
        self.axis = math.radians(4.0)

    def command(self, snapshot):
        from navigation import corridor_guidance as cg
        m = cg.measure_corridor(snapshot)
        if m is not None:
            return super().command(snapshot)
        err = cg.wrap(self.theta_rel - self.axis)
        return cg.GuidanceOutput("HOLD", self._sat(-self.k * err), None, None, err)


class TestGuidance(unittest.TestCase):
    def test_converges_from_offset_and_tilt(self):
        tr, p, _ = drive(CorridorWorld(), y0=0.05, th0=math.radians(4.0), dist=2.0)
        self.assertLess(abs(p.y), 0.003)
        self.assertLess(abs(math.degrees(p.th)), 0.3)
        self.assertTrue(all(m == "FULL" for *_, m, _ in tr))
        self.assertLess(max(abs(y) for _, y, *_ in tr), HALF_WIDTH_M - ROBOT_HALF_WIDTH_M)
        self.assertLessEqual(max(abs(w) for *_, w in tr), OMEGA_MAX_RAD_S + 1e-12)

    def test_without_guidance_it_drifts(self):
        _, p, _ = drive(CorridorWorld(), th0=math.radians(2.0), dist=2.0, guided=False)
        self.assertGreater(abs(p.y), 0.05)                 # reference: open loop

    def test_short_corridor_tilt_removed(self):
        """START corridor in T1: only ~0.45 m with all four walls visible.
        Heading error must be mostly removed within that distance."""
        tr, *_ = drive(CorridorWorld(), th0=math.radians(5.0), dist=0.45)
        self.assertLess(abs(math.degrees(tr[-1][2])), 1.5)

    def test_hold_keeps_axis_through_gap(self):
        """1.2 m gap on both sides: no FULL measurement, HOLD keeps the heading
        that was learnt in the corridor before the gap."""
        world = CorridorWorld(gaps=(("left", 1.5, 2.7), ("right", 1.5, 2.7)))
        tr, p, g = drive(world, y0=0.03, th0=math.radians(3.0), dist=3.5)
        modes = {m for *_, m, _ in tr}
        # (step 13b-6: at the gap edges one side may briefly be a plain single wall -> LEFT / RIGHT)
        self.assertTrue({"FULL", "HOLD"} <= modes <= {"FULL", "HOLD", "LEFT", "RIGHT"}, modes)
        hold = [t for t in tr if t[3] == "HOLD"]
        # HOLD starts while the look-ahead approach is still tilting the robot;
        # after 0.2 m it must be aligned with the learnt axis and keep its offset
        settled = [t for t in hold if t[0] - hold[0][0] > 0.2]
        self.assertLess(max(abs(math.degrees(t[2])) for t in settled), 0.05)
        self.assertLess(abs(settled[-1][1] - settled[0][1]), 0.001)
        self.assertLess(abs(p.y), 0.005)

    def test_axis_advances_with_turn(self):
        """Learn the axis, turn +90 deg in place with on_turn_started(+90):
        after the turn HOLD error equals only the turn's residual (< 0.6 deg)."""
        world = CorridorWorld()
        _, p, g = drive(world, th0=math.radians(2.0), dist=1.0)
        mc = MotionController()
        p.reset_pi()
        mc.start_turn_degrees(90.0)
        g.on_turn_started(math.radians(90.0))
        inc = p.step(0.0, 0.0)
        while True:
            g.update_odometry(inc.dtheta_rad)
            ref = mc.update(inc.dtheta_rad)
            if ref.completed:
                break
            inc = p.step(0.0, ref.angular_velocity_rad_s)
        self.assertLess(abs(math.degrees(g.theta_rel - g.axis)), 0.6)

    def test_root_corridor_with_live_yaw_friction(self):
        """T1 START corridor (openings from 1.058 m), plant with the fitted yaw
        friction, robot 4 deg off: without feed-forward the residual tilt at the
        ROOT centre exceeds 0.5 deg (as live run 1); with it, it is within."""
        world = CorridorWorld(gaps=(("left", 1.058, 1.787), ("right", 1.058, 1.787)))
        res = {}
        for ff in (False, True):
            tr, p, _ = drive(world, th0=math.radians(4.0), dist=1.4226, plant_kw=LIVE_YAW, ff=ff)
            res[ff] = (abs(p.y), abs(math.degrees(p.th)), tr)
        self.assertGreater(res[False][1], 0.5)
        self.assertLess(res[True][0], 0.010)
        self.assertLess(res[True][1], 0.5)
        ws = [w for *_, w in res[True][2]]
        self.assertLessEqual(sum(1 for a, b in zip(ws, ws[1:]) if a * b < 0), 3)   # no chattering

    def test_none_without_axis(self):
        g = CorridorGuidance()
        g.reset(axis_known=False)
        snap = CorridorWorld(gaps=(("left", -1, 5), ("right", -1, 5))).snapshot(1.0, 0.0, 0.0)
        self.assertEqual(g.command(snap).mode, "NONE")
        self.assertEqual(g.command(snap).omega_ref_rad_s, 0.0)


class TestYawFeedForward(unittest.TestCase):
    def test_shape(self):
        self.assertEqual(yaw_friction_feedforward(0.0), 0.0)
        self.assertAlmostEqual(yaw_friction_feedforward(0.025), YAW_FF_COMMAND / 2)
        self.assertEqual(yaw_friction_feedforward(1.0), YAW_FF_COMMAND)
        self.assertEqual(yaw_friction_feedforward(-1.0), -YAW_FF_COMMAND)
        self.assertEqual(yaw_friction_feedforward(0.3, u_f=0.0), 0.0)


if __name__ == "__main__":
    unittest.main()
