--[[
Sea-Doo Sparktrixx IBR Auto Control Script
------------------------------------------
AUTO/GUIDED:
    - Reverse when throttle < 0
    - Forward when throttle > 0
    - Neutral when throttle = 0

MANUAL:
    - Passthrough

HOLD:
    - Allow autopilot to settle steering for HOLD_SETTLE_TIME ms
    - Then disarm stepper
    - Read rider throttle from pin 4 (0.5–2.97 V → 0–100)
    - Read rider brake   from pin 8 (0.5–2.97 V → 0–100)
    - Output through lua1/lua2

Geofence:
    - Trigger Relay 1 for 1 second on breach
    - Print state messages
]]

local SCRIPT_VERSION = 'v1.1'
------------------------------------------------------------
-- OBJECTS
------------------------------------------------------------
local AOUT = AnalogCommands()

-- Analog input channels for HOLD mode
local ain_throttle_fs = analog:channel()
local ain_throttle_hs = analog:channel()
local ain_brake_fs    = analog:channel()
local ain_brake_hs    = analog:channel()

------------------------------------------------------------
-- MODE CONSTANTS
------------------------------------------------------------
local MODES = {
    MANUAL = 0,
    ACRO = 1,
    STEERING = 3,
    HOLD = 4,
    LOITER = 5,
    FOLLOW = 6,
    SIMPLE = 7,
    DOCK = 8,
    CIRCLE = 9,
    AUTO = 10,
    RTL = 11,
    SMART_RTL = 12,
    GUIDED = 15,
    INITIALIZING = 16
}

------------------------------------------------------------
-- CONSTANTS AND PARAMS
------------------------------------------------------------
local RELAY_NUM = 0
local BREACH_RELAY_DURATION = 1000
local THROTTLE_MIN = 0.5
local THROTTLE_MAX = 2.97
local BRAKE_MIN    = 0.5/2
local BRAKE_MAX    = 2.97/2

local PARAM_TABLE_KEY = 0
assert(param:add_table(PARAM_TABLE_KEY, "SDSPK_", 10), 'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1,  'THR_CAP', 0.2), 'could not add param1')
assert(param:add_param(PARAM_TABLE_KEY, 2,  'LIM_MANUAL', 0), 'could not add param2')
assert(param:add_param(PARAM_TABLE_KEY, 3,  'THR_FS_PIN', 2), 'could not add param3')
assert(param:add_param(PARAM_TABLE_KEY, 4,  'THR_HS_PIN', 3), 'could not add param4')
assert(param:add_param(PARAM_TABLE_KEY, 5,  'BRK_FS_PIN', 4), 'could not add param5')
assert(param:add_param(PARAM_TABLE_KEY, 6,  'BRK_HS_PIN', 5), 'could not add param6')
assert(param:add_param(PARAM_TABLE_KEY, 7,  'THR_TRIM', 0), 'could not add param6')

local THROTTLE_CAP = Parameter()
THROTTLE_CAP:init('SDSPK_THR_CAP')

local MANUAL_THROTTLE_CAP = Parameter()
MANUAL_THROTTLE_CAP:init('SDSPK_LIM_MANUAL')

local PARAM_THR_FS_PIN = Parameter()
PARAM_THR_FS_PIN:init('SDSPK_THR_FS_PIN')

local PARAM_THR_HS_PIN = Parameter()
PARAM_THR_HS_PIN:init('SDSPK_THR_HS_PIN')

local PARAM_BRK_FS_PIN = Parameter()
PARAM_BRK_FS_PIN:init('SDSPK_BRK_FS_PIN')

local PARAM_BRK_HS_PIN = Parameter()
PARAM_BRK_HS_PIN:init('SDSPK_BRK_HS_PIN')

local PARAM_THROTTLE_TRIM = Parameter()
PARAM_THROTTLE_TRIM:init('SDSPK_THR_TRIM')

ain_throttle_fs:set_pin(math.floor(PARAM_THR_FS_PIN:get() or 2))
ain_throttle_hs:set_pin(math.floor(PARAM_THR_HS_PIN:get() or 3))
ain_brake_fs:set_pin(math.floor(PARAM_BRK_FS_PIN:get() or 4))
ain_brake_hs:set_pin(math.floor(PARAM_BRK_HS_PIN:get() or 5))
------------------------------------------------------------
-- STATE VARIABLES
------------------------------------------------------------
local breach_active = false
local relay_trigger_time = 0
local rider_in_control = false
local disarmed = 0


------------------------------------------------------------
-- HELPERS
------------------------------------------------------------
local function map(v, in_min, in_max, out_min, out_max)
    if v < in_min then v = in_min end
    if v > in_max then v = in_max end
    return (v - in_min) * (out_max - out_min) / (in_max - in_min) + out_min
end


local function check_user_override()
    local v_throttle = ain_throttle_fs:voltage_latest()
    local v_brake    = ain_brake_fs:voltage_latest()
    -- gcs:send_text(6, string.format("b: %f, t: %f", v_brake, v_throttle))
    -- local string = string.format("Throttle moved: %s, Brake moved: %s", v_throttle, v_brake)
    -- gcs:send_text(6, string)

    if rider_in_control then
        if not arming:is_armed() then
            disarmed = 1
        end

        if disarmed == 1 and arming:is_armed() then
            gcs:send_text(6, "Vehicle re-armed. Exiting rider control mode.")
            rider_in_control = false
            disarmed = 0
            stepper:relinquish_control()
        end
    else 
        -- Detect meaningful user input (threshold ~0.1 V above idle)
        local throttle_moved = (v_throttle > THROTTLE_MIN + 0.20)
        local brake_moved    = (v_brake    > BRAKE_MIN    + 0.20)
        
        if throttle_moved or brake_moved then
            gcs:send_text(6, "Rider input detected. Putting to manual and relinquishing control to rider.")
            -- gcs:send_text(6, string.format("b: %f, t: %f", v_brake, v_throttle))
            -- Switch to MANUAL mode
            vehicle:set_mode(MODES.MANUAL)
            stepper:disarm()
            -- ensure vehicle is armed for rider control throttle output.
            arming:arm();
            rider_in_control = true
        end
    end

end

local function rider_control_logic()
    local v_throttle = ain_throttle_fs:voltage_latest()
    local v_brake    = ain_brake_fs:voltage_latest()
    -- gcs:send_text(6, string.format("brake: %f, throttle: %f", v_brake, v_throttle))
    -- Detect meaningful user input (threshold ~0.1 V above idle)
    local throttle_out = 0
    local brake_out = 0
    
    throttle_out = map(v_throttle, THROTTLE_MIN, THROTTLE_MAX, 0, 100)
    brake_out    = map(v_brake,    BRAKE_MIN,    BRAKE_MAX,    0, 100)

    -- Only apply throttle cap if enabled for manual mode. This allows user to have full throttle range in manual if desired.
    if MANUAL_THROTTLE_CAP:get() == 1 then
        throttle_out = throttle_out * THROTTLE_CAP:get()
    end
    
    AOUT:lua1(throttle_out)
    AOUT:lua2(brake_out)
end

local function autopilot_control_logic()
    local mode = vehicle:get_mode()
    local throttle_in = AOUT:throttle()
    local brake_in = (AOUT:brake() / 4500 * 100)
    local throttle_out = 0
    local brake_out = 0
    --------------------------------------------------------
    -- AUTO / MANUAL / GUIDED → Normal IBR logic
    --------------------------------------------------------
    if mode == MODES.MANUAL then
        throttle_out = map(math.abs(throttle_in), PARAM_THROTTLE_TRIM:get(), 100, 0, 100)
        brake_out = brake_in

    elseif mode == MODES.AUTO or mode == MODES.GUIDED then

        if throttle_in < 0 then
            throttle_out =  map(math.abs(throttle_in), PARAM_THROTTLE_TRIM:get(), 100, 0, 100)
            brake_out = 100
        elseif throttle_in > 0 then
            throttle_out = map(math.abs(throttle_in), PARAM_THROTTLE_TRIM:get(), 100, 0, 100)
            brake_out = 0
        else
            throttle_out = 0
            brake_out = 100
        end
    end

    -- Always apply throttle cap for autopilot control to prevent excessive speed.
    AOUT:lua1(throttle_out * THROTTLE_CAP:get())
    AOUT:lua2(brake_out)
    
    -- gcs:send_text(6, string.format("IN: brake: %f, throttle: %f", PARAM_THROTTLE_TRIM:get(), throttle_in))
    -- gcs:send_text(6, string.format("OUT: brake: %f, throttle: %f", brake_out, throttle_out))
    -- Detect meaningful user input (threshold ~0.1 V above idle)
end

local function geofence_logic()
    --------------------------------------------------------
    -- GEOFENCE LOGIC (unchanged)
    --------------------------------------------------------
    local breaches = fence:get_breaches()
    local now  = millis()
    if breaches ~= 0 and not breach_active then
        breach_active = true
        relay_trigger_time = now
        relay:on(RELAY_NUM)
        gcs:send_text(3, string.format("Geofence BREACH detected! Type mask: %d", breaches))
    elseif breaches == 0 and breach_active then
        breach_active = false
        gcs:send_text(6, "Geofence breach cleared.")
    end

    if breach_active and (now - relay_trigger_time >= BREACH_RELAY_DURATION) then
        relay:off(RELAY_NUM)
        gcs:send_text(6, "Relay 1 OFF after geofence trigger")
        relay_trigger_time = 0
    end
end




------------------------------------------------------------
-- MAIN UPDATE LOOP
------------------------------------------------------------
function update()
    geofence_logic()

    -- rider_control_logic returns if rider is in control.
    check_user_override()
    if rider_in_control then
        rider_control_logic()
    else
        autopilot_control_logic()
    end
    return update, 10
end


gcs:send_text(6, "SeaDoo spark script " .. SCRIPT_VERSION .. " loaded.")
return update(), 10
