"""
Install the FusionBot bridge into the open CoppeliaSim scene (step 5).

Run from src/ with the simulation STOPPED:

    python -m sim.install_bridge            # lockstep (default)
    python -m sim.install_bridge --realtime # asynchronous, no waiting

Creates
    /FusionBot/FusionBotDirectZmqBridge                  bridge (sim/bridge_script.py)
    <left/right motor joint>/FusionBotMotor*500Hz        motor model (sim/motor_script.py)
and switches both wheel joints to dynamic CALLBACK control.

It checks (and never changes) dt = 10 ms, dynamics dt = 2 ms, MuJoCo engine.
Afterwards save the scene (Ctrl+S).
"""

from __future__ import annotations

import argparse
import json
from pathlib import Path

from coppeliasim_zmqremoteapi_client import RemoteAPIClient

from core.robot_config import (
    DYNAMICS_TIMESTEP_S,
    LEFT_MOTOR_JOINT_PATH,
    RIGHT_MOTOR_JOINT_PATH,
    ROBOT_PATH,
    SIMULATION_TIMESTEP_S,
)
from sim.bridge_script import bridge_script_source
from sim.motor_script import motor_script_source


BRIDGE_ALIAS = "FusionBotDirectZmqBridge"
LEFT_MOTOR_SCRIPT_ALIAS = "FusionBotMotorLeft500Hz"
RIGHT_MOTOR_SCRIPT_ALIAS = "FusionBotMotorRight500Hz"
BACKUP_JSON = "fusionbot_direct_zmq_bridge_backup.json"

EXPECTED_SIM_DT_S = SIMULATION_TIMESTEP_S
EXPECTED_DYNAMICS_DT_S = DYNAMICS_TIMESTEP_S


def _joint_mode_value(sim, joint: int) -> int:
    """
    Normalize sim.getJointMode() across CoppeliaSim/Remote API bindings.

    Some bindings expose only the joint mode integer, while others return
    (joint_mode, options). Only the first value is the actual joint mode.
    """
    result = sim.getJointMode(int(joint))

    if isinstance(result, (tuple, list)):
        if not result:
            raise RuntimeError("sim.getJointMode() returned an empty sequence.")
        return int(result[0])

    return int(result)


def _remove_object_if_present(sim, path: str) -> None:
    handle = sim.getObject(path, {"noError": True})
    if handle != -1:
        sim.removeObject(int(handle))


def _create_script(sim, alias: str, parent: int, source: str) -> int:
    handle = int(
        sim.createScript(
            sim.scripttype_simulation,
            source,
            0,
            "lua",
        )
    )
    sim.setObjectAlias(handle, alias)
    sim.setObjectParent(handle, int(parent), True)
    return handle


def main() -> None:
    parser = argparse.ArgumentParser(description="Install FusionBot2 bridge.")
    parser.add_argument(
        "--realtime",
        action="store_true",
        help="Asynchronous V1.2-style transport instead of lockstep.",
    )
    args = parser.parse_args()
    lockstep = not args.realtime

    client = RemoteAPIClient()
    sim = client.require("sim")

    if sim.getSimulationState() != sim.simulation_stopped:
        raise RuntimeError(
            "Stop the CoppeliaSim simulation before installing the bridge."
        )

    robot = int(sim.getObject(ROBOT_PATH))
    left_joint = int(sim.getObject(LEFT_MOTOR_JOINT_PATH))
    right_joint = int(sim.getObject(RIGHT_MOTOR_JOINT_PATH))

    sim_dt = float(sim.getFloatProperty(sim.handle_scene, "timeStep"))
    dyn_dt = float(
        sim.getFloatProperty(sim.handle_scene, "dynamicsStepSize")
    )
    steps_per_render = int(
        sim.getIntProperty(sim.handle_scene, "stepsPerRendering")
    )
    dynamics_engine = list(
        sim.getIntArrayProperty(sim.handle_scene, "dynamicsEngine")
    )

    print()
    print("FusionBot bridge installer, protocol 2.0, mode = " + ("LOCKSTEP" if lockstep else "REALTIME"))
    print("------------------------------------")
    print(f"simulation dt                 : {sim_dt:.6f} s")
    print(f"dynamics dt                   : {dyn_dt:.6f} s")
    print(f"steps per rendering           : {steps_per_render}")
    print(f"dynamics engine property      : {dynamics_engine}")

    if abs(sim_dt - EXPECTED_SIM_DT_S) > 1e-9:
        raise RuntimeError(
            f"Expected simulation dt={EXPECTED_SIM_DT_S:.3f} s, "
            f"got {sim_dt:.6f} s."
        )
    if abs(dyn_dt - EXPECTED_DYNAMICS_DT_S) > 1e-9:
        raise RuntimeError(
            f"Expected dynamics dt={EXPECTED_DYNAMICS_DT_S:.3f} s, "
            f"got {dyn_dt:.6f} s."
        )
    if dynamics_engine and int(dynamics_engine[0]) != int(sim.physics_mujoco):
        raise RuntimeError(
            "FusionBot direct-ZMQ bridge expects MuJoCo as the active dynamics engine."
        )

    left_mode = _joint_mode_value(sim, left_joint)
    right_mode = _joint_mode_value(sim, right_joint)
    if left_mode != int(sim.jointmode_dynamic):
        raise RuntimeError("Left motor joint is not in dynamic joint mode.")
    if right_mode != int(sim.jointmode_dynamic):
        raise RuntimeError("Right motor joint is not in dynamic joint mode.")

    left_dyn_ctrl = int(
        sim.getIntProperty(left_joint, "dynCtrlMode")
    )
    right_dyn_ctrl = int(
        sim.getIntProperty(right_joint, "dynCtrlMode")
    )

    backup_path = Path(BACKUP_JSON)
    if not backup_path.exists():
        backup_path.write_text(
            json.dumps(
                {
                    "left_joint_mode": left_mode,
                    "right_joint_mode": right_mode,
                    "left_dyn_ctrl_mode": left_dyn_ctrl,
                    "right_dyn_ctrl_mode": right_dyn_ctrl,
                    "simulation_dt_s": sim_dt,
                    "dynamics_dt_s": dyn_dt,
                    "steps_per_rendering": steps_per_render,
                },
                indent=2,
            ),
            encoding="utf-8",
        )
        print(f"backup                        : {backup_path.resolve()}")
    else:
        print(
            f"backup                        : {backup_path.resolve()} "
            "(existing file kept)"
        )

    bridge_path = f"{ROBOT_PATH}/{BRIDGE_ALIAS}"
    left_script_path = (
        f"{LEFT_MOTOR_JOINT_PATH}/{LEFT_MOTOR_SCRIPT_ALIAS}"
    )
    right_script_path = (
        f"{RIGHT_MOTOR_JOINT_PATH}/{RIGHT_MOTOR_SCRIPT_ALIAS}"
    )

    # Remove previous transport bridge variants so only one owns the robot I/O.
    for old_alias in (
        "FusionBotLowLevelBridge",
        "FusionBotMailboxBridge",
        "FusionBotDirectZmqBridge",
    ):
        _remove_object_if_present(
            sim,
            f"{ROBOT_PATH}/{old_alias}",
        )

    _remove_object_if_present(sim, left_script_path)
    _remove_object_if_present(sim, right_script_path)

    bridge_handle = _create_script(
        sim,
        BRIDGE_ALIAS,
        robot,
        bridge_script_source(lockstep),
    )
    left_script_handle = _create_script(
        sim,
        LEFT_MOTOR_SCRIPT_ALIAS,
        left_joint,
        motor_script_source(),
    )
    right_script_handle = _create_script(
        sim,
        RIGHT_MOTOR_SCRIPT_ALIAS,
        right_joint,
        motor_script_source(),
    )

    sim.setIntProperty(
        left_joint,
        "dynCtrlMode",
        int(sim.jointdynctrl_callback),
    )
    sim.setIntProperty(
        right_joint,
        "dynCtrlMode",
        int(sim.jointdynctrl_callback),
    )

    print()
    print(f"direct-ZMQ bridge script      : {bridge_handle} | {bridge_path}")
    print(
        f"left 500 Hz motor script      : "
        f"{left_script_handle} | {left_script_path}"
    )
    print(
        f"right 500 Hz motor script     : "
        f"{right_script_handle} | {right_script_path}"
    )
    print("joint dynamic control         : CALLBACK / CALLBACK")
    print()
    print("DIRECT-ZMQ INSTALLATION COMPLETE")
    print("Save the CoppeliaSim scene now (Ctrl+S).")
    print("Next: python -m checks.step05_live_drive")


if __name__ == "__main__":
    main()
