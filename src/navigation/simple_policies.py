"""
Simple reactive decision providers (step 9 test/baseline; the DFS explorer with
memory is step 11).

LeftHandPolicy: LEFT if available, else STRAIGHT, else RIGHT, else UTURN.
In a tree maze it traverses every edge exactly twice and returns to the root
(wall following). max_events stops it with FINISH.
"""

from __future__ import annotations

from navigation.junction_types import JunctionObservation, ManeuverDecision


class LeftHandPolicy:
    def __init__(self, max_events: int | None = None) -> None:
        self.max_events = max_events
        self.events = 0
        self.log: list[tuple[JunctionObservation, ManeuverDecision]] = []

    def decide(self, obs: JunctionObservation) -> ManeuverDecision:
        if self.max_events is not None and self.events >= self.max_events:
            d = ManeuverDecision.FINISH
        elif obs.left_available:
            d = ManeuverDecision.LEFT
        elif obs.straight_available:
            d = ManeuverDecision.STRAIGHT
        elif obs.right_available:
            d = ManeuverDecision.RIGHT
        else:
            d = ManeuverDecision.UTURN
        self.events += 1
        self.log.append((obs, d))
        return d
