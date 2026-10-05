"""
Proximity data containers (step 5).

Raw readings as delivered by the bridge for the five robot sensors. The
semantic layer (openings, front blocked) is step 6.
"""

from __future__ import annotations

from dataclasses import dataclass

from core.robot_config import PROXIMITY_SENSOR_ORDER as SENSOR_ORDER  # noqa: F401


@dataclass(frozen=True)
class ProximityReading:
    name: str
    detected: bool
    distance_m: float | None          # None whenever detected is False
    detected_object_handle: int | None = None


@dataclass(frozen=True)
class ProximitySnapshot:
    left: ProximityReading
    front_left: ProximityReading
    front: ProximityReading
    front_right: ProximityReading
    right: ProximityReading

    def as_dict(self) -> dict[str, ProximityReading]:
        return {n: getattr(self, n) for n in SENSOR_ORDER}
