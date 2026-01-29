--[[
Sea-Doo Sparktrixx IBR Auto Control Script
This script manages the Intelligent Brake & Reverse (IBR) system for Sea-Doo in ArduPilot.
Manual mode passes throttle and brake inputs directly from the controller.
In Auto/Guided modes, it engages reverse when throttle is negative, passes forward throttle normally, 
and sets neutral when throttle is near zero. 

Additionally:
- When a geofence breach occurs, Relay 1 is triggered ON for 1 second.
- Status messages are printed to GCS for visibility.
]]

-- Create AOUT_COMMANDS object
local AOUT_COMMANDS = AnalogCommands()

-- Define all Rover modes
local MODES = {
    ROVER_MODE_MANUAL = 0,
    ROVER_MODE_ACRO = 1,
    ROVER_MODE_STEERING = 3,
    ROVER_MODE_HOLD = 4,
    ROVER_MODE_LOITER = 5,
    ROVER_MODE_FOLLOW = 6,
    ROVER_MODE_SIMPLE = 7,
    ROVER_MODE_DOCK = 8,
    ROVER_MODE_CIRCLE = 9,
    ROVER_MODE_AUTO = 10,
    ROVER_MODE_RTL = 11,
    ROVER_MODE_SMART_RTL = 12,
    ROVER_MODE_GUIDED = 15,
    ROVER_MODE_INITIALIZING = 16
}

-- Relay settings
local RELAY_NUM = 0           -- Relay 1 (index starts from 0)
local RELAY_DURATION = 1000   -- 1 second in ms

-- State for geofence relay logic
local breach_active = false
local relay_trigger_time = 0

gcs:send_text(6, "IBR + Geofence script loaded and running...")

function update()
    local now = millis()
    local mode = vehicle:get_mode()
    local throttle_in = AOUT_COMMANDS:throttle()  -- -100..100
    local throttle_out = 0
    local brake_out = 0

    -- === Throttle / Brake Control ===
    if mode == MODES.ROVER_MODE_MANUAL then
        -- Manual passthrough
        local brake_in = (AOUT_COMMANDS:brake() / 4500 * 100)  -- Convert 0..4500 to 0..100
        throttle_out = math.max(throttle_in, 0)  -- Passing only positive throttle
        brake_out = brake_in

    elseif mode == MODES.ROVER_MODE_AUTO or mode == MODES.ROVER_MODE_GUIDED then
        if throttle_in < 0 then
            -- Reverse/braking requested
            throttle_out = math.abs(throttle_in)
            brake_out = 100
        elseif throttle_in > 0 then
            -- Forward requested: passthrough
            throttle_out = throttle_in
            brake_out = 0
        else
            -- Neutral: throttle near zero
            throttle_out = 0
            brake_out = 100
        end
    end
    -- Apply analog outputs
    AOUT_COMMANDS:lua1(throttle_out)
    AOUT_COMMANDS:lua2(brake_out)

    -- === Geofence Breach Logic ===
    local breaches = fence:get_breaches()

    -- Handle geofence breach detection
    if breaches ~= 0 and not breach_active then
        breach_active = true
        relay_trigger_time = now
        relay:on(RELAY_NUM)
        gcs:send_text(3, string.format("Geofence BREACH detected! Type mask: %d", breaches))
    elseif breaches == 0 and breach_active then
        breach_active = false
        gcs:send_text(6, "Geofence breach cleared.")
    end

    -- Turn relay off after RELAY_DURATION
    if breach_active and (now - relay_trigger_time >= RELAY_DURATION) then
        relay:off(RELAY_NUM)
        gcs:send_text(6, "Relay 1 OFF after geofence trigger")
    end

    return update, 500  -- Run every 0.5 seconds
end

return update()
