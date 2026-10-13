"""
Simulator-side bridge script (step 5): Lua source generator.

One simulation script on /FusionBot that, every 10 ms pass:
  sysCall_actuation  writes the latest wheel commands to the joint properties
                     read by the 500 Hz motor scripts (step 1)
  sysCall_sensing    reads joint positions, the five proximity sensors
                     (environment-filtered), GT pose (evaluation only), sticky
                     physical + visual collision flags, publishes one state
                     packet (protocol 2.0, float64) and - in LOCKSTEP mode -
                     waits for the command computed for exactly this state.

Carried over from the live-validated FusionBot2 lockstep bridge (13a PASS:
two runs bit-identical, latency exactly 1 step). Contract details: see
sim/protocol.py and sim/transport.py.
"""

from __future__ import annotations

from core.robot_config import (
    BASE_PATH,
    BASE_YAW_TO_PHYSICAL_HEADING_RAD,
    LEFT_MOTOR_JOINT_PATH,
    PROXIMITY_SENSOR_PATHS,
    RIGHT_MOTOR_JOINT_PATH,
    ROBOT_PATH,
)


def _lua_string(value: str) -> str:
    s = str(value)
    s = s.replace("\\", "\\\\").replace("'", "\\'")
    return f"'{s}'"


def bridge_script_source(lockstep: bool = True) -> str:
    required = ("left", "front_left", "front", "front_right", "right")
    missing = [name for name in required if name not in PROXIMITY_SENSOR_PATHS]
    if missing:
        raise RuntimeError(
            "robot_config.PROXIMITY_SENSOR_PATHS is missing: "
            + ", ".join(missing)
        )

    sensor_paths_lua = ",\n    ".join(
        _lua_string(PROXIMITY_SENSOR_PATHS[name])
        for name in required
    )

    source = r"""
sim = require('sim')
simZMQ = require('simZMQ')

local MOTOR_COMMAND_PROPERTY = 'signal.fusionbotMotorCommand'
local COUNT_PROPERTY = 'signal.fusionbotMotorCallbackCount'
local LAG_PROPERTY = 'signal.fusionbotLaggedCommand'

local LEFT_JOINT_PATH = __LEFT_JOINT_PATH__
local RIGHT_JOINT_PATH = __RIGHT_JOINT_PATH__
local BASE_PATH = __BASE_PATH__
local ROBOT_PATH = __ROBOT_PATH__
local BASE_YAW_OFFSET = __BASE_YAW_OFFSET__

local STATE_ENDPOINT = 'tcp://127.0.0.1:23110'
local COMMAND_ENDPOINT = 'tcp://127.0.0.1:23111'
local PROTOCOL_VERSION = 2.0
local EVALUATION_DIVIDER = 1
local LOCKSTEP = __LOCKSTEP__
local LOCKSTEP_RECV_TIMEOUT_MS = 20
local LOCKSTEP_MAX_WAIT_S = 10.0
-- extension 2.1: IMU every step, every NEW LiDAR scan once (the scan script renders at 10 Hz)
local IMU_PATH = __IMU_PATH__
local LIDAR_PATH = __LIDAR_PATH__
local LIDAR_DIVIDER = 10
local lastStatePayload = nil
local lockstepRepublishCount = 0
local lockstepReleased = false
local gtYawRate = 0.0
local previousGtHeading = nil

local SENSOR_PATHS = {
    __SENSOR_PATHS__
}

local leftJoint = -1
local rightJoint = -1
local base = -1
local robot = -1
local sensors = {}
local originalSensorEntity = {}
local originalSensorExplicit = {}

local robotShapes = {}
local visualShapes = {}
local mazeWallSet = {}
local mazeWallCollisionCollection = -1
local environmentCollection = -1

local VISUAL_SHAPE_ALIASES = {
    chassis_visual = true,
    name = true,
    rightWheel = true,
    rightWheelRim = true,
    leftWheel = true,
    leftWheelRim = true,
}

local zmqContext = nil
local statePub = nil
local commandSub = nil

local stateSequence = 0
local sensingCount = 0
local lastAttachedLidarCount = 0
local evaluationSequence = 0

local appliedCommandSequence = 0
local appliedSourceStateSequence = 0
local appliedLeftCommand = 0.0
local appliedRightCommand = 0.0

local gtX = 0.0
local gtY = 0.0
local gtHeading = 0.0
local evaluationTime = 0.0
local leftCallbackCount = 0
local rightCallbackCount = 0
local leftLaggedCommand = 0.0
local rightLaggedCommand = 0.0

local stickyCollision = false
local stickyCollisionRobotHandle = -1
local stickyCollisionWallHandle = -1
local stickyCollisionTime = -1.0
local stickyContactPoint = {0.0, 0.0, 0.0}
local stickyContactForce = {0.0, 0.0, 0.0}
local stickyContactNormal = {0.0, 0.0, 0.0}

local stickyVisualCollision = false
local stickyVisualCollisionRobotHandle = -1
local stickyVisualCollisionWallHandle = -1
local stickyVisualCollisionTime = -1.0

local function clamp(v, lo, hi)
    if v < lo then return lo end
    if v > hi then return hi end
    return v
end

local function wrapToPi(a)
    local twoPi = 2.0 * math.pi
    return (a + math.pi) % twoPi - math.pi
end

local function buildEnvironmentCollection()
    local c = sim.createCollection(0)

    sim.addItemToCollection(
        c,
        sim.handle_all,
        -1,
        0
    )

    sim.addItemToCollection(
        c,
        sim.handle_tree,
        robot,
        1
    )

    local templates = sim.getObject(
        '/FusionBot_templates',
        {noError = true}
    )
    if templates ~= -1 then
        sim.addItemToCollection(
            c,
            sim.handle_tree,
            templates,
            1
        )
    end

    return c
end

local function buildCollisionLookup()
    robotShapes = sim.getObjectsInTree(
        robot,
        sim.object_shape_type,
        0
    )

    visualShapes = {}
    for i = 1, #robotShapes do
        local shape = robotShapes[i]
        local alias = sim.getObjectAlias(shape)
        local shortAlias = alias
        local p = string.match(alias, '([^/]+)$')
        if p ~= nil then
            shortAlias = p
        end

        if VISUAL_SHAPE_ALIASES[shortAlias] then
            table.insert(visualShapes, shape)
        end
    end

    mazeWallSet = {}

    if mazeWallCollisionCollection ~= nil
       and mazeWallCollisionCollection >= 0 then
        pcall(sim.destroyCollection, mazeWallCollisionCollection)
    end

    mazeWallCollisionCollection = sim.createCollection(1)

    local handles = sim.getObjectsInTree(
        sim.handle_scene,
        sim.handle_all,
        0
    )

    for i = 1, #handles do
        local alias = sim.getObjectAlias(handles[i])
        local shortAlias = alias
        local p = string.match(alias, '([^/]+)$')
        if p ~= nil then
            shortAlias = p
        end

        if string.sub(shortAlias, 1, 13) == 'FB_MAZE_wall_' then
            mazeWallSet[handles[i]] = true
            sim.addItemToCollection(
                mazeWallCollisionCollection,
                sim.handle_single,
                handles[i],
                0
            )
        end
    end
end

local function updateStickyCollision()
    if stickyCollision then
        return
    end

    for i = 1, #robotShapes do
        local shape = robotShapes[i]
        local index = 0

        while true do
            local pair, point, rForce, normal = sim.getContactInfo(
                sim.handle_all,
                shape,
                index
            )

            if pair == nil or #pair < 2 then
                break
            end

            local other = -1
            if pair[1] == shape then
                other = pair[2]
            elseif pair[2] == shape then
                other = pair[1]
            end

            if other ~= -1 and mazeWallSet[other] then
                stickyCollision = true
                stickyCollisionRobotHandle = shape
                stickyCollisionWallHandle = other
                stickyCollisionTime = sim.getSimulationTime()

                if point ~= nil and #point >= 3 then
                    stickyContactPoint = {point[1], point[2], point[3]}
                end
                if rForce ~= nil and #rForce >= 3 then
                    stickyContactForce = {rForce[1], rForce[2], rForce[3]}
                end
                if normal ~= nil and #normal >= 3 then
                    stickyContactNormal = {normal[1], normal[2], normal[3]}
                end
                return
            end

            index = index + 1
        end
    end
end

local function updateStickyVisualCollision()
    if stickyVisualCollision then
        return
    end

    if mazeWallCollisionCollection == nil
       or mazeWallCollisionCollection < 0 then
        return
    end

    for i = 1, #visualShapes do
        local shape = visualShapes[i]
        local result, pair = sim.checkCollision(
            shape,
            mazeWallCollisionCollection
        )

        if result ~= nil and result > 0 then
            stickyVisualCollision = true
            stickyVisualCollisionRobotHandle = shape
            stickyVisualCollisionTime = sim.getSimulationTime()

            local wall = -1
            if pair ~= nil and #pair >= 2 then
                if pair[1] == shape and mazeWallSet[pair[2]] then
                    wall = pair[2]
                elseif pair[2] == shape and mazeWallSet[pair[1]] then
                    wall = pair[1]
                end
            end

            stickyVisualCollisionWallHandle = wall
            return
        end
    end
end

local function updateEvaluation()
    evaluationTime = sim.getSimulationTime()

    local pl = sim.getObjectPosition(
        leftJoint,
        sim.handle_world
    )
    local pr = sim.getObjectPosition(
        rightJoint,
        sim.handle_world
    )

    gtX = 0.5 * (pl[1] + pr[1])
    gtY = 0.5 * (pl[2] + pr[2])

    -- Physical forward = base local +Y. Its world direction is the second
    -- column of the 3x4 row-major matrix: (m[2], m[6], m[10]).
    local m = sim.getObjectMatrix(base, sim.handle_world)
    gtHeading = math.atan2(m[6], m[2])

    local dtSim = sim.getSimulationTimeStep()
    if previousGtHeading ~= nil and dtSim > 0.0 then
        gtYawRate = wrapToPi(gtHeading - previousGtHeading) / dtSim
    else
        gtYawRate = 0.0
    end
    previousGtHeading = gtHeading

    local lc = sim.getIntProperty(
        leftJoint,
        COUNT_PROPERTY,
        {noError = true}
    )
    local rc = sim.getIntProperty(
        rightJoint,
        COUNT_PROPERTY,
        {noError = true}
    )
    local ll = sim.getFloatProperty(
        leftJoint,
        LAG_PROPERTY,
        {noError = true}
    )
    local rl = sim.getFloatProperty(
        rightJoint,
        LAG_PROPERTY,
        {noError = true}
    )

    if lc ~= nil then leftCallbackCount = lc end
    if rc ~= nil then rightCallbackCount = rc end
    if ll ~= nil then leftLaggedCommand = ll end
    if rl ~= nil then rightLaggedCommand = rl end
end

-- extension 2.2 (step 14b): optional obstacle poses appended to the command packet
--   [6] n, then n x (index, x, y, z): FB_MAZE_dyn_<index> moved to (x, y, z) in the world frame
local dynHandles = {}

local function applyObstacles(values)
    if values == nil or #values < 6 then
        return
    end
    local n = math.floor(values[6] + 0.5)
    for i = 0, n - 1 do
        local b = 7 + 4 * i
        if #values < b + 3 then
            return
        end
        local h = dynHandles[math.floor(values[b] + 0.5)]
        if h ~= nil and h ~= -1 then
            sim.setObjectPosition(h, {values[b + 1], values[b + 2], values[b + 3]}, sim.handle_world)
        end
    end
end

local function readLatestCommand()
    -- Drain all queued command messages. Only the newest command is applied.
    while true do
        local rc, data = simZMQ.recv(
            commandSub,
            simZMQ.DONTWAIT
        )

        if rc == nil or rc < 0 or data == nil then
            break
        end

        local values = sim.unpackDoubleTable(data)

        if values ~= nil and #values >= 5 then
            if math.abs(values[1] - PROTOCOL_VERSION) < 0.1 then
                appliedCommandSequence = math.floor(values[2] + 0.5)
                appliedSourceStateSequence =
                    math.floor(values[3] + 0.5)
                appliedLeftCommand = clamp(values[4], -1.0, 1.0)
                appliedRightCommand = clamp(values[5], -1.0, 1.0)
                applyObstacles(values)
            end
        end
    end
end

local function readProximityPacked()
    local out = {}

    -- The default CoppeliaSim main script already handles all non-explicit
    -- proximity sensors once per sensing pass, before simulation-script
    -- sysCall_sensing callbacks run. Read that result here instead of
    -- recomputing the same geometry with sim.checkProximitySensor.
    for i = 1, #sensors do
        local state, distance, point, objectHandle =
            sim.readProximitySensor(sensors[i])

        if state ~= nil and state > 0 then
            table.insert(out, 1.0)
            table.insert(out, distance)
            if objectHandle ~= nil then
                table.insert(out, objectHandle)
            else
                table.insert(out, -1.0)
            end
        else
            table.insert(out, 0.0)
            table.insert(out, -1.0)
            table.insert(out, -1.0)
        end
    end

    return out
end

local function publishState()
    local proximity = readProximityPacked()

    stateSequence = stateSequence + 1

    local state = {
        PROTOCOL_VERSION,
        stateSequence,
        sim.getSimulationTime(),
        sim.getJointPosition(leftJoint),
        sim.getJointPosition(rightJoint),
    }

    for i = 1, #proximity do
        table.insert(state, proximity[i])
    end

    table.insert(state, evaluationSequence)
    table.insert(state, gtX)
    table.insert(state, gtY)
    table.insert(state, gtHeading)

    if stickyCollision then
        table.insert(state, 1.0)
    else
        table.insert(state, 0.0)
    end

    table.insert(state, stickyCollisionRobotHandle)
    table.insert(state, stickyCollisionWallHandle)
    table.insert(state, leftCallbackCount)
    table.insert(state, rightCallbackCount)
    table.insert(state, leftLaggedCommand)
    table.insert(state, rightLaggedCommand)

    table.insert(state, appliedCommandSequence)
    table.insert(state, appliedSourceStateSequence)
    table.insert(state, appliedLeftCommand)
    table.insert(state, appliedRightCommand)

    table.insert(state, evaluationTime)
    table.insert(state, stickyCollisionTime)

    table.insert(state, stickyContactPoint[1])
    table.insert(state, stickyContactPoint[2])
    table.insert(state, stickyContactPoint[3])

    table.insert(state, stickyContactForce[1])
    table.insert(state, stickyContactForce[2])
    table.insert(state, stickyContactForce[3])

    table.insert(state, stickyContactNormal[1])
    table.insert(state, stickyContactNormal[2])
    table.insert(state, stickyContactNormal[3])

    if stickyVisualCollision then
        table.insert(state, 1.0)
    else
        table.insert(state, 0.0)
    end
    table.insert(state, stickyVisualCollisionRobotHandle)
    table.insert(state, stickyVisualCollisionWallHandle)
    table.insert(state, stickyVisualCollisionTime)
    table.insert(state, gtYawRate)
    table.insert(state, lockstepRepublishCount)

    -- extension 2.1
    table.insert(state, 1.0)
    local imuOk = false
    if imuHandle ~= -1 then
        local c = sim.getIntProperty(imuHandle, 'signal.fusionbotImuCount', {noError = true})
        local g = sim.getFloatArrayProperty(imuHandle, 'signal.fusionbotImuGyro', {noError = true})
        local a = sim.getFloatArrayProperty(imuHandle, 'signal.fusionbotImuAccel', {noError = true})
        if c ~= nil and g ~= nil and a ~= nil and #g == 3 and #a == 3 then
            table.insert(state, c)
            for k = 1, 3 do table.insert(state, g[k]) end
            for k = 1, 3 do table.insert(state, a[k]) end
            imuOk = true
        end
    end
    if not imuOk then
        table.insert(state, -1.0)
        for k = 1, 6 do table.insert(state, 0.0) end
    end
    local scan = nil
    local lidarCount = -1
    if lidarHandle ~= -1 then
        lidarCount = sim.getIntProperty(lidarHandle, 'signal.fusionbotLidarScanCount', {noError = true}) or -1
        -- attach every NEW scan once (the LiDAR script renders at 10 Hz, step 13b-7)
        if lidarCount > lastAttachedLidarCount then
            scan = sim.getFloatArrayProperty(lidarHandle, 'signal.fusionbotLidarXYZD', {noError = true})
            lastAttachedLidarCount = lidarCount
        end
    end
    table.insert(state, lidarCount)
    if scan ~= nil and #scan >= 4 then
        local n = math.floor(#scan / 4)
        table.insert(state, n)
        for k = 1, 4 * n do table.insert(state, scan[k]) end
    else
        table.insert(state, 0.0)
    end

    local payload = sim.packDoubleTable(state)
    lastStatePayload = payload

    -- PUB + DONTWAIT means simulator never waits for the external controller.
    simZMQ.send(
        statePub,
        payload,
        simZMQ.DONTWAIT
    )
end

local function applyCommandValues(values)
    appliedCommandSequence = math.floor(values[2] + 0.5)
    appliedSourceStateSequence = math.floor(values[3] + 0.5)
    appliedLeftCommand = clamp(values[4], -1.0, 1.0)
    appliedRightCommand = clamp(values[5], -1.0, 1.0)
    applyObstacles(values)
end

local function waitForLockstepCommand()
    if lockstepReleased then
        return
    end
    local t0 = sim.getSystemTime()
    local lastSend = t0
    while true do
        -- Non-blocking receive (same call as the validated V1.2 bridge),
        -- polled in a loop. No reliance on socket receive timeouts.
        local rc, data = simZMQ.recv(commandSub, simZMQ.DONTWAIT)
        local received = rc ~= nil and rc >= 0 and data ~= nil
        if received then
            local values = sim.unpackDoubleTable(data)
            if values ~= nil and #values >= 5
                and math.abs(values[1] - PROTOCOL_VERSION) < 0.1 then
                local src = math.floor(values[3] + 0.5)
                if values[2] < 0.0 then
                    -- RELEASE: controller finished; stop waiting so that the
                    -- Remote API stopSimulation request can be served.
                    lockstepReleased = true
                    appliedLeftCommand = 0.0
                    appliedRightCommand = 0.0
                    return
                end
                if src == stateSequence then
                    applyCommandValues(values)
                    return
                end
                -- older command: stale duplicate, discard and keep waiting
            end
        else
            local now = sim.getSystemTime()
            if now - t0 > LOCKSTEP_MAX_WAIT_S then
                error('FusionBot lockstep: no command for state '
                    .. tostring(stateSequence) .. ' within '
                    .. tostring(LOCKSTEP_MAX_WAIT_S) .. ' s')
            end
            if lastStatePayload ~= nil
                and (now - lastSend) * 1000.0 >= LOCKSTEP_RECV_TIMEOUT_MS then
                lockstepRepublishCount = lockstepRepublishCount + 1
                simZMQ.send(statePub, lastStatePayload, simZMQ.DONTWAIT)
                lastSend = now
            end
        end
    end
end

function sysCall_init()
    leftJoint = sim.getObject(LEFT_JOINT_PATH)
    rightJoint = sim.getObject(RIGHT_JOINT_PATH)
    base = sim.getObject(BASE_PATH)
    robot = sim.getObject(ROBOT_PATH)
    imuHandle = sim.getObject(IMU_PATH, {noError = true})
    lidarHandle = sim.getObject(LIDAR_PATH, {noError = true})
    dynHandles = {}
    for k = 0, 15 do
        local h = sim.getObject('/FB_MAZE_dyn_' .. k, {noError = true})
        if h ~= nil and h ~= -1 then dynHandles[k] = h end
    end
    if imuHandle == nil then imuHandle = -1 end
    if lidarHandle == nil then lidarHandle = -1 end

    sensors = {}
    for i = 1, #SENSOR_PATHS do
        sensors[i] = sim.getObject(SENSOR_PATHS[i])
    end

    environmentCollection = buildEnvironmentCollection()

    -- Preserve the previous sensor configuration, then make the default main
    -- script handle each sensor exactly once against our filtered environment.
    -- This keeps the old "scene minus FusionBot/templates" semantics without
    -- doing a second explicit detection in this bridge.
    originalSensorEntity = {}
    originalSensorExplicit = {}

    for i = 1, #sensors do
        originalSensorEntity[i] = sim.getObjectInt32Param(
            sensors[i],
            sim.proxintparam_entity_to_detect
        )
        originalSensorExplicit[i] = sim.getExplicitHandling(
            sensors[i]
        )

        sim.setObjectInt32Param(
            sensors[i],
            sim.proxintparam_entity_to_detect,
            environmentCollection
        )
        sim.setExplicitHandling(
            sensors[i],
            0
        )
    end

    buildCollisionLookup()

    stickyCollision = false
    stickyCollisionRobotHandle = -1
    stickyCollisionWallHandle = -1
    stickyCollisionTime = -1.0
    stickyContactPoint = {0.0, 0.0, 0.0}
    stickyContactForce = {0.0, 0.0, 0.0}
    stickyContactNormal = {0.0, 0.0, 0.0}

    stickyVisualCollision = false
    stickyVisualCollisionRobotHandle = -1
    stickyVisualCollisionWallHandle = -1
    stickyVisualCollisionTime = -1.0

    stateSequence = 0
    sensingCount = 0
    lastAttachedLidarCount = 0
    evaluationSequence = 0

    appliedCommandSequence = 0
    appliedSourceStateSequence = 0
    appliedLeftCommand = 0.0
    appliedRightCommand = 0.0

    sim.setFloatProperty(leftJoint, MOTOR_COMMAND_PROPERTY, 0.0)
    sim.setFloatProperty(rightJoint, MOTOR_COMMAND_PROPERTY, 0.0)

    updateEvaluation()

    zmqContext = simZMQ.ctx_new()

    statePub = simZMQ.socket(
        zmqContext,
        simZMQ.PUB
    )
    commandSub = simZMQ.socket(
        zmqContext,
        simZMQ.SUB
    )

    -- Ensure cleanup never waits for queued network data.
    simZMQ.setsockopt(
        statePub,
        simZMQ.LINGER,
        sim.packInt32Table({0})
    )
    simZMQ.setsockopt(
        commandSub,
        simZMQ.LINGER,
        sim.packInt32Table({0})
    )

    -- Subscribe to all command messages.
    simZMQ.setsockopt(
        commandSub,
        simZMQ.SUBSCRIBE,
        ''
    )

    local rcState = simZMQ.bind(
        statePub,
        STATE_ENDPOINT
    )
    if rcState ~= 0 then
        error(
            'FusionBot state PUB bind failed at '
            .. STATE_ENDPOINT
            .. ', rc='
            .. tostring(rcState)
        )
    end

    local rcCommand = simZMQ.bind(
        commandSub,
        COMMAND_ENDPOINT
    )
    if rcCommand ~= 0 then
        error(
            'FusionBot command SUB bind failed at '
            .. COMMAND_ENDPOINT
            .. ', rc='
            .. tostring(rcCommand)
        )
    end
end

function sysCall_actuation()
    if not LOCKSTEP then
        readLatestCommand()
    end

    sim.setFloatProperty(
        leftJoint,
        MOTOR_COMMAND_PROPERTY,
        appliedLeftCommand
    )
    sim.setFloatProperty(
        rightJoint,
        MOTOR_COMMAND_PROPERTY,
        appliedRightCommand
    )
end

function sysCall_sensing()
    sensingCount = sensingCount + 1

    updateStickyCollision()
    updateStickyVisualCollision()

    if sensingCount % EVALUATION_DIVIDER == 0 then
        evaluationSequence = evaluationSequence + 1
        updateEvaluation()
    end

    publishState()

    if LOCKSTEP then
        waitForLockstepCommand()
    end
end

function sysCall_cleanup()
    pcall(
        sim.setFloatProperty,
        leftJoint,
        MOTOR_COMMAND_PROPERTY,
        0.0
    )
    pcall(
        sim.setFloatProperty,
        rightJoint,
        MOTOR_COMMAND_PROPERTY,
        0.0
    )

    if statePub ~= nil then
        pcall(simZMQ.close, statePub)
        statePub = nil
    end
    if commandSub ~= nil then
        pcall(simZMQ.close, commandSub)
        commandSub = nil
    end
    if zmqContext ~= nil then
        pcall(simZMQ.ctx_term, zmqContext)
        zmqContext = nil
    end

    -- Restore original proximity-sensor handling and detection targets before
    -- destroying the temporary filtered environment collection.
    for i = 1, #sensors do
        if originalSensorEntity[i] ~= nil then
            pcall(
                sim.setObjectInt32Param,
                sensors[i],
                sim.proxintparam_entity_to_detect,
                originalSensorEntity[i]
            )
        end

        if originalSensorExplicit[i] ~= nil then
            pcall(
                sim.setExplicitHandling,
                sensors[i],
                originalSensorExplicit[i]
            )
        end
    end

    if environmentCollection ~= nil and environmentCollection >= 0 then
        pcall(sim.destroyCollection, environmentCollection)
        environmentCollection = -1
    end

    if mazeWallCollisionCollection ~= nil
       and mazeWallCollisionCollection >= 0 then
        pcall(sim.destroyCollection, mazeWallCollisionCollection)
        mazeWallCollisionCollection = -1
    end
end
"""

    replacements = {
        "__LEFT_JOINT_PATH__": _lua_string(LEFT_MOTOR_JOINT_PATH),
        "__RIGHT_JOINT_PATH__": _lua_string(RIGHT_MOTOR_JOINT_PATH),
        "__BASE_PATH__": _lua_string(BASE_PATH),
        "__ROBOT_PATH__": _lua_string(ROBOT_PATH),
        "__BASE_YAW_OFFSET__": f"{float(BASE_YAW_TO_PHYSICAL_HEADING_RAD):.17g}",
        "__SENSOR_PATHS__": sensor_paths_lua,
        "__LOCKSTEP__": "true" if lockstep else "false",
        "__IMU_PATH__": _lua_string(BASE_PATH + "/FusionBotIMU"),
        "__LIDAR_PATH__": _lua_string(BASE_PATH + "/fastHokuyo"),
    }

    for key, value in replacements.items():
        source = source.replace(key, value)

    return source
