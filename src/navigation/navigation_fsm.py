"""
FusionBot navigation state machine (step 9).

Sequences one junction after another, 100 Hz, from proximity perception
(step 6), the motion primitives (step 7) and the corridor guidance (step 8).
It does not estimate pose, read GT, command motors or keep a map; junction
decisions come from an external DecisionProvider (DFS in step 11).

States
------
    INIT ─► FOLLOW ◄──────────────────────────────── EXIT ◄── TURN
              │  ▲                                    ▲         ▲
     candidate│  │candidate gone                      │STRAIGHT │
              ▼  │                                    │         │
            APPROACH ── observation event ─► decide ──┴► POSITION
    (FOLLOW/APPROACH)                       │
                                            ├ FINISH ─► FINISHED
                                            └ invalid ─► SAFETY_ABORT

Observation events (instantaneous; the decision is taken on the same tick):
    SIDE_CONFIRMED   a side opening is CONFIRMED and no side is still CANDIDATE.
                     straight_available = FRONT not detected at that moment.
    DEAD_END         FRONT_BLOCKED with no confirmed side opening -> stop now.

POSITION (for LEFT / RIGHT / UTURN):
    ADVANCE  straight OPEN  : drive D_TURN_ENTRY past the reference (FRONT_BLOCKED
                              first -> abort "unexpected_front_blocked")
             straight CLOSED: drive until FRONT_BLOCKED; guard = (FRONT distance at
                              the reference - FRONT_BLOCKED) + 15 mm, reached first ->
                              abort "closed_straight_guard"; trigger distance logged
    BRAKE    20 ticks with zero encoder counts on both wheels
    HOLD     0.5 s
TURN: MotionController TURN (odometry), then 20 zero-count ticks; the corridor
axis of the guidance is advanced by the turn angle.
EXIT: straight with guidance (HOLD on the axis) and FROZEN opening semantics
until LEFT && RIGHT detected for 5 ticks; then reset_openings() and re-process
the SAME snapshot (it may already show the next junction's CANDIDATE).

Safety (priority over everything): external safety_event; FRONT_SAFETY (moving
forward with FRONT < 0.030 m); FRONT_BLOCKED during EXIT (unexpected).

Changes vs FusionBot2 (review): single speed 0.25 m/s (no cruise / pre-brake,
so the APPROACH->FOLLOW pre-brake issue disappears), FRONT_SAFETY added (K5),
EXIT watches FRONT, corridor guidance in every forward state, closed-straight
trigger distance logged (K2), observe+decide in one tick, explicit FINISH.

Every primitive change requests a wheel-PI reset (step-7 finding); the runtime
must honour `reset_wheel_pi` on the same tick.
"""

from __future__ import annotations

import math
from dataclasses import dataclass
from enum import Enum

from navigation import navigation_config as cfg
from navigation.corridor_guidance import CorridorGuidance, GuidanceOutput
from navigation.junction_types import (
    TURN_ANGLE_DEG, DecisionProvider, JunctionObservation, ManeuverDecision, ReferenceKind,
)
from navigation.motion_controller import MotionController, MotionMode
from perception.proximity_perception import PerceptionSnapshot, ProximityPerception
from perception.proximity_types import ProximitySnapshot


class NavState(str, Enum):
    INIT = "INIT"
    FOLLOW = "FOLLOW"
    APPROACH = "APPROACH"
    POSITION = "POSITION"
    TURN = "TURN"
    EXIT = "EXIT"
    FINISHED = "FINISHED"
    SAFETY_ABORT = "SAFETY_ABORT"


class Phase(str, Enum):
    ADVANCE = "ADVANCE"
    BRAKE = "BRAKE"
    HOLD = "HOLD"
    TURNING = "TURNING"
    SETTLE = "SETTLE"


@dataclass(frozen=True)
class NavStep:
    state: NavState
    phase: Phase | None
    v_ref_m_s: float
    omega_ref_rad_s: float
    motion_mode: MotionMode
    observation: JunctionObservation | None      # only on the event tick
    decision: ManeuverDecision | None            # only on the event tick
    reset_wheel_pi: bool
    reset_openings: bool
    guidance: GuidanceOutput | None
    perception: PerceptionSnapshot | None
    stop_guard: str | None                       # why the robot braked (POSITION)
    closed_front_trigger_m: float | None         # K2: FRONT_BLOCKED distance after the reference
    closed_front_expected_m: float | None        # K2: expected from the FRONT reading at the reference
    fault: str | None


class NavigationFSM:
    def __init__(
        self,
        decision_provider: DecisionProvider,
        *,
        perception: ProximityPerception | None = None,
        motion: MotionController | None = None,
        guidance: CorridorGuidance | None = None,
        speed_m_s: float = cfg.NAV_SPEED_M_S,
        uturn_deg: float = cfg.UTURN_DEG,
    ) -> None:
        if abs(abs(uturn_deg) - 180.0) > 1e-9:
            raise ValueError("uturn_deg must be +-180")
        self.provider = decision_provider
        self.perception = perception or ProximityPerception()
        self.motion = motion or MotionController()
        self.guidance = guidance or CorridorGuidance()
        self.speed = float(speed_m_s)
        self.uturn_deg = float(uturn_deg)
        self.reset()

    # ------------------------------------------------------------ lifecycle
    def reset(self) -> None:
        self.state, self.phase = NavState.INIT, None
        self.motion.stop()
        self.perception.reset_openings()
        self.guidance.reset()
        self.observation: JunctionObservation | None = None
        self.decision: ManeuverDecision | None = None
        self.fault: str | None = None
        self.stop_guard: str | None = None
        self._sem: PerceptionSnapshot | None = None
        self._zero = self._reacq = 0
        self._hold_s = 0.0
        self._pi_reset = True
        self._closed_trigger: float | None = None

    def _abort(self, reason: str) -> None:
        self.motion.stop()
        self._pi_reset = True
        self.state, self.phase, self.fault = NavState.SAFETY_ABORT, None, reason

    def _straight(self) -> None:
        self.motion.start_straight(self.speed)
        self._pi_reset = True

    def _stop(self, guard: str) -> None:
        self.motion.stop()
        self._pi_reset = True
        self.phase, self.stop_guard, self._zero = Phase.BRAKE, guard, 0

    def _turn_angle_deg(self) -> float:
        if self.decision is ManeuverDecision.UTURN:
            return self.uturn_deg
        return TURN_ANGLE_DEG[self.decision]

    # ------------------------------------------------------------ events
    def _observe(self, sem: PerceptionSnapshot, kind: ReferenceKind, odo_s: float):
        side = kind is ReferenceKind.SIDE_CONFIRMED
        obs = JunctionObservation(
            kind=kind,
            left_available=side and sem.left_open,
            straight_available=side and not sem.front_detected,
            right_available=side and sem.right_open,
            front_distance_m=sem.front_distance_m,
            reference_odom_distance_m=odo_s,
        )
        self.observation = obs
        if kind is ReferenceKind.DEAD_END:
            self.motion.stop()
            self._pi_reset = True
        try:
            dec = ManeuverDecision(self.provider.decide(obs))
        except Exception as exc:                       # noqa: BLE001 - any provider failure is a fault
            self._abort(f"decision_error: {exc}")
            return obs, None
        self.decision = dec
        if not obs.available(dec) or (dec is ManeuverDecision.STRAIGHT and not side):
            self._abort(f"decision_not_available: {dec.value}")
        elif dec is ManeuverDecision.FINISH:
            self.motion.stop()
            self._pi_reset = True
            self.state, self.phase = NavState.FINISHED, None
        elif dec is ManeuverDecision.STRAIGHT:
            self.state, self.phase, self._reacq = NavState.EXIT, None, 0
        else:
            self.state, self.stop_guard, self._closed_trigger = NavState.POSITION, None, None
            if kind is ReferenceKind.DEAD_END:
                self.phase, self.stop_guard, self._zero = Phase.BRAKE, "DEAD_END", 0
            else:
                self.phase = Phase.ADVANCE
        return obs, dec

    # ------------------------------------------------------------ main tick
    def update(
        self,
        raw: ProximitySnapshot,
        *,
        odo_distance_m: float,
        odo_dtheta_rad: float,
        left_counts: int,
        right_counts: int,
        dt_s: float = 0.01,
        safety_event: bool = False,
    ) -> NavStep:
        if not (math.isfinite(odo_distance_m) and math.isfinite(odo_dtheta_rad)) or dt_s <= 0:
            raise ValueError("invalid odometry input")
        self.guidance.update_odometry(odo_dtheta_rad)
        obs = dec = None
        reset_openings = False
        trigger = None
        still = left_counts == 0 and right_counts == 0

        # ---- safety first
        if self.state not in (NavState.FINISHED, NavState.SAFETY_ABORT):
            f = raw.front
            front_d = float(f.distance_m) if f.detected and f.distance_m is not None else None
            moving_fwd = self.motion.mode is MotionMode.STRAIGHT
            if safety_event:
                self._abort("external_safety_event")
            elif moving_fwd and front_d is not None and front_d < cfg.FRONT_SAFETY_MIN_M:
                self._abort(f"front_safety ({front_d:.4f} m)")

        st = self.state
        if st in (NavState.FINISHED, NavState.SAFETY_ABORT):
            ref = self.motion.update(odo_dtheta_rad)
            return self._out(ref, None, obs, dec, False, None, trigger)

        # ---- perception (opening semantics frozen while turning / leaving a junction)
        if st not in (NavState.TURN, NavState.EXIT):
            self._sem = self.perception.update(raw)
        sem = self._sem

        if st is NavState.INIT:
            self.perception.reset_openings()
            sem = self._sem = self.perception.update(raw)
            reset_openings = True
            self._straight()
            self.state = NavState.FOLLOW

        elif st in (NavState.FOLLOW, NavState.APPROACH):
            if (sem.left_open or sem.right_open) and not sem.any_candidate:
                obs, dec = self._observe(sem, ReferenceKind.SIDE_CONFIRMED, odo_distance_m)
            elif sem.front_blocked:
                obs, dec = self._observe(sem, ReferenceKind.DEAD_END, odo_distance_m)
            else:
                self.state = NavState.APPROACH if sem.any_candidate else NavState.FOLLOW

        elif st is NavState.POSITION:
            o = self.observation
            if self.phase is Phase.ADVANCE:
                ds = odo_distance_m - o.reference_odom_distance_m
                if not o.straight_available:
                    expected = o.front_distance_m - self.perception.front_blocked_distance_m
                    if sem.front_blocked:
                        self._closed_trigger = trigger = ds
                        self._stop("FRONT_BLOCKED")
                    elif ds >= expected + cfg.CLOSED_GUARD_MARGIN_M:
                        self._abort(f"closed_straight_guard (odometry {ds:.4f} m, expected FRONT_BLOCKED "
                                    f"at {expected:.4f} m)")
                else:
                    if sem.front_blocked:
                        self._abort(f"unexpected_front_blocked (odometry {ds:.4f} m)")
                    elif ds >= cfg.D_TURN_ENTRY_M:
                        self._stop("ODOM_TARGET")
            elif self.phase is Phase.BRAKE:
                self._zero = self._zero + 1 if still else 0
                if self._zero >= cfg.STOP_ZERO_COUNT_SAMPLES:
                    self.phase, self._hold_s = Phase.HOLD, 0.0
            elif self.phase is Phase.HOLD:
                self._hold_s += dt_s
                if self._hold_s >= cfg.PRE_TURN_HOLD_S - 1e-9:
                    ang = self._turn_angle_deg()
                    self.motion.start_turn_degrees(ang)
                    self.guidance.on_turn_started(math.radians(ang))
                    self._pi_reset = True
                    self.state, self.phase = NavState.TURN, Phase.TURNING

        elif st is NavState.EXIT:
            f = raw.front
            if f.detected and f.distance_m is not None and f.distance_m <= self.perception.front_blocked_distance_m:
                self._abort(f"front_blocked_during_exit ({f.distance_m:.4f} m)")
            else:
                both = raw.left.detected and raw.right.detected
                self._reacq = self._reacq + 1 if both else 0
                if self._reacq >= cfg.REACQUIRE_SAMPLES:
                    self.perception.reset_openings()
                    sem = self._sem = self.perception.update(raw)
                    reset_openings = True
                    self.observation = self.decision = None
                    self.stop_guard = None
                    self.state, self._reacq = NavState.FOLLOW, 0

        # ---- primitive: start straight on entering EXIT / FOLLOW from a stop
        if self.state is NavState.EXIT and self.motion.mode is not MotionMode.STRAIGHT and self.phase is None:
            self._straight()

        ref = self.motion.update(odo_dtheta_rad)

        if self.state is NavState.TURN:
            if self.phase is Phase.TURNING and ref.completed:
                self.phase, self._zero = Phase.SETTLE, 0
            elif self.phase is Phase.SETTLE:
                self._zero = self._zero + 1 if still else 0
                if self._zero >= cfg.STOP_ZERO_COUNT_SAMPLES:
                    self.state, self.phase, self._reacq = NavState.EXIT, None, 0
                    self._straight()
                    ref = self.motion.update(0.0)

        g = None
        if ref.mode is MotionMode.STRAIGHT and ref.linear_velocity_m_s > 0.0:
            g = self.guidance.command(raw)
        return self._out(ref, g, obs, dec, reset_openings, sem, trigger)

    def _out(self, ref, g, obs, dec, reset_openings, sem, trigger) -> NavStep:
        expected = None
        if trigger is not None and self.observation is not None:
            expected = self.observation.front_distance_m - self.perception.front_blocked_distance_m
        pi, self._pi_reset = self._pi_reset, False
        return NavStep(
            state=self.state, phase=self.phase,
            v_ref_m_s=ref.linear_velocity_m_s,
            omega_ref_rad_s=g.omega_ref_rad_s if g is not None else ref.angular_velocity_rad_s,
            motion_mode=ref.mode, observation=obs, decision=dec,
            reset_wheel_pi=pi, reset_openings=reset_openings, guidance=g, perception=sem,
            stop_guard=self.stop_guard, closed_front_trigger_m=trigger,
            closed_front_expected_m=expected, fault=self.fault,
        )
