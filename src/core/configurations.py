"""
FusionBot sensor configurations A-E (step 13a): which sensor takes part how.

Every configuration drives the SAME robot with the SAME navigation (FSM, corridor
guidance on the proximity sensors, DFS exploration) and draws the SAME metric
map with the LiDAR; configurations differ only in what the POSE ESTIMATE uses:

  role                      A      B      C      D      E
  ----------------------------------------------------------
  distance (prediction)     enc    enc    enc    enc    enc
  heading  (prediction)     enc    enc    gyro   enc    gyro
  gyro bias (ZUPT update)   -      -      yes    -      yes
  PROX-LOC update           -      yes    -      -      yes
  LiDAR scan-match update   -      -      -      yes    yes
  ----------------------------------------------------------
  corridor guidance (PROX-NAV)  all configurations (shapes the path, never the pose)
  metric map (occupancy grid)   all configurations, LiDAR drawn at the ESTIMATED pose
  topological map (MazeMemory)  all configurations, junctions placed at the ESTIMATED pose
  GT                            evaluation and calibration only, never an input

Every run is done with a noise profile (sensors.noise_models): "ideal" (simulator
values, LiDAR systematic error calibrated out) or "realistic" (+ datasheet noise
of the real parts), so every configuration is compared with and without noise.
"""

from __future__ import annotations

from dataclasses import dataclass


@dataclass(frozen=True)
class Configuration:
    name: str
    description: str
    heading_source: str          # "encoders" | "gyro"
    gyro_zupt: bool              # gyro bias estimated whenever the encoders show standstill
    prox_loc_update: bool        # PROX-LOC (side / front walls) corrects the pose
    lidar_update: bool           # LiDAR scan matching corrects the pose
    # the same in every configuration (kept explicit for the report):
    distance_source: str = "encoders"
    prox_guidance: bool = True
    lidar_map: bool = True

    @property
    def uses_gyro(self) -> bool:
        return self.heading_source == "gyro"

    @property
    def estimator(self) -> str:
        """Odometry only, or an EKF with the listed updates."""
        return "odometry" if not (self.uses_gyro or self.prox_loc_update or self.lidar_update) else "ekf"

    def sensors(self) -> tuple[str, ...]:
        """Sensors whose data enter the POSE ESTIMATE."""
        s = ["encoders"]
        if self.uses_gyro:
            s.append("gyro")
        if self.prox_loc_update:
            s.append("proximity")
        if self.lidar_update:
            s.append("lidar")
        return tuple(s)


CONFIGS = {
    "A": Configuration("A", "ODOM: encoder odometry", "encoders", False, False, False),
    "B": Configuration("B", "ODOM + PROX-LOC", "encoders", False, True, False),
    "C": Configuration("C", "ODOM + IMU gyro (heading, bias by ZUPT)", "gyro", True, False, False),
    "D": Configuration("D", "ODOM + LiDAR scan matching", "encoders", False, False, True),
    "E": Configuration("E", "all: ODOM + gyro + PROX-LOC + LiDAR", "gyro", True, True, True),
}


def participation_table() -> str:
    """Markdown table of the matrix above (generated from CONFIGS, used in docs and the runner header)."""
    rows = [
        ("distance (prediction)", lambda c: c.distance_source),
        ("heading (prediction)", lambda c: c.heading_source),
        ("gyro bias (ZUPT)", lambda c: "yes" if c.gyro_zupt else "-"),
        ("PROX-LOC update", lambda c: "yes" if c.prox_loc_update else "-"),
        ("LiDAR update", lambda c: "yes" if c.lidar_update else "-"),
        ("corridor guidance (prox)", lambda c: "yes" if c.prox_guidance else "-"),
        ("metric map (LiDAR)", lambda c: "yes" if c.lidar_map else "-"),
        ("estimator", lambda c: c.estimator),
    ]
    names = list(CONFIGS)
    out = ["| role | " + " | ".join(names) + " |", "|---|" + "---|" * len(names)]
    for label, f in rows:
        out.append(f"| {label} | " + " | ".join(f(CONFIGS[n]) for n in names) + " |")
    return "\n".join(out)
