"""
Simulator-side 500 Hz motor script (step 1).

Generates the Lua joint-callback script that CoppeliaSim attaches to each
wheel joint. Constants are injected from core.robot_config, so Lua and the
Python reference (core.motor_model) cannot drift apart.

How the torque reaches MuJoCo
-----------------------------
CoppeliaSim calls sysCall_joint() at every dynamics step (2 ms) for a joint in
dynamic mode with "callback" control. The callback returns a target velocity
and a maximum force (torque). Returning

    vel = sign(tau) * omega_0,   force = |tau|

makes the joint push towards +-omega_0 with at most |tau|. The motor torque
falls to zero before the wheel can reach omega_0 (torque-speed line), so the
force limit is always active and the applied torque is exactly tau: a torque
source emulated through a velocity controller.

The wheel command is read from the joint property signal.fusionbotMotorCommand,
which the bridge (step 5) writes once per 10 ms pass.
"""

from __future__ import annotations

from core.robot_config import (
    COULOMB_FRICTION_NM,
    MOTOR_LAG_TAU_S,
    MOTOR_NO_LOAD_SPEED_RAD_S,
    MOTOR_STALL_TORQUE_NM,
    PWM_DEADZONE,
    VISCOUS_FRICTION_NM_S,
)

COMMAND_PROPERTY = "signal.fusionbotMotorCommand"
COUNT_PROPERTY = "signal.fusionbotMotorCallbackCount"
LAG_PROPERTY = "signal.fusionbotLaggedCommand"

_TEMPLATE = r"""
sim = require('sim')

local COMMAND_PROPERTY = '__COMMAND_PROPERTY__'
local COUNT_PROPERTY = '__COUNT_PROPERTY__'
local LAG_PROPERTY = '__LAG_PROPERTY__'

local OMEGA0 = __OMEGA0__
local TAU_STALL = __TAU_STALL__
local LAG_TAU = __LAG_TAU__
local DEADZONE = __DEADZONE__
local C_FRICTION = __C_FRICTION__
local B_FRICTION = __B_FRICTION__

local laggedCommand = 0.0
local callbackCount = 0

local function clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

local function sign(v)
    if v > 1e-12 then return 1.0 end
    if v < -1e-12 then return -1.0 end
    return 0.0
end

local function applyDeadzone(u)
    u = clamp(u, -1.0, 1.0)
    local m = math.abs(u)
    if m <= DEADZONE then return 0.0 end
    local r = (m - DEADZONE) / (1.0 - DEADZONE)
    if u < 0.0 then return -r end
    return r
end

-- Pure function: identical to core.motor_model.MotorModel.update()
function motorStep(command, omega, dt)
    local uDz = applyDeadzone(command)
    if LAG_TAU <= 0.0 then
        laggedCommand = uDz
    elseif dt > 0.0 then
        local a = 1.0 - math.exp(-dt / LAG_TAU)
        laggedCommand = clamp(laggedCommand + a * (uDz - laggedCommand), -1.0, 1.0)
    end
    local tauM = clamp(TAU_STALL * (laggedCommand - omega / OMEGA0), -TAU_STALL, TAU_STALL)
    local motion = sign(omega)
    if motion == 0.0 then motion = sign(laggedCommand) end
    local tau = tauM - C_FRICTION * motion - B_FRICTION * omega
    return clamp(tau, -TAU_STALL, TAU_STALL)
end

function sysCall_init()
    laggedCommand = 0.0
    callbackCount = 0
end

function sysCall_joint(inData)
    if inData.mode ~= sim.jointmode_dynamic then
        return nil
    end
    if inData.first then
        laggedCommand = 0.0
    end
    local command = sim.getFloatProperty(inData.handle, COMMAND_PROPERTY, {noError = true})
    if command == nil then command = 0.0 end
    local omega = inData.vel
    if omega == nil then omega = 0.0 end

    local tau = motorStep(command, omega, inData.dt)

    callbackCount = callbackCount + 1
    if inData.passCnt == inData.totalPasses then
        sim.setIntProperty(inData.handle, COUNT_PROPERTY, callbackCount)
        sim.setFloatProperty(inData.handle, LAG_PROPERTY, laggedCommand)
    end

    if math.abs(tau) < 1e-12 then
        return {vel = 0.0, force = 0.0}
    end
    local direction = 1.0
    if tau < 0.0 then direction = -1.0 end
    return {vel = direction * OMEGA0, force = math.abs(tau)}
end
"""


def motor_script_source() -> str:
    values = {
        "__COMMAND_PROPERTY__": COMMAND_PROPERTY,
        "__COUNT_PROPERTY__": COUNT_PROPERTY,
        "__LAG_PROPERTY__": LAG_PROPERTY,
        "__OMEGA0__": f"{MOTOR_NO_LOAD_SPEED_RAD_S:.17g}",
        "__TAU_STALL__": f"{MOTOR_STALL_TORQUE_NM:.17g}",
        "__LAG_TAU__": f"{MOTOR_LAG_TAU_S:.17g}",
        "__DEADZONE__": f"{PWM_DEADZONE:.17g}",
        "__C_FRICTION__": f"{COULOMB_FRICTION_NM:.17g}",
        "__B_FRICTION__": f"{VISCOUS_FRICTION_NM_S:.17g}",
    }
    src = _TEMPLATE
    for k, v in values.items():
        src = src.replace(k, v)
    return src
