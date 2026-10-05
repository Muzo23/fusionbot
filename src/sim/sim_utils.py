"""Small Remote API helpers shared by live tools (step 5)."""

from __future__ import annotations

import time


def wait_until_stopped(sim, timeout_s: float = 15.0) -> None:
    deadline = time.time() + timeout_s
    while sim.getSimulationState() != sim.simulation_stopped:
        if time.time() >= deadline:
            raise RuntimeError("CoppeliaSim did not stop in time.")
        time.sleep(0.02)


def prepare_lockstep(sim) -> bool:
    """Simulation must be stopped; real-time off. Returns the old real-time flag."""
    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError("Stop the CoppeliaSim simulation first.")
    original = bool(sim.getBoolProperty(sim.handle_scene, "realtimeSimulation"))
    sim.setBoolProperty(sim.handle_scene, "realtimeSimulation", False)
    sim.setStepping(False)
    return original
