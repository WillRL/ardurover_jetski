--[[
Sea-Doo Sparktrixx IBR Auto Control Script
This script manages the Intelligent Brake & Reverse (IBR) system for Sea-Doo in ArduPilot.
Manual mode passes throttle and brake inputs directly from the controller.
In Auto/Guided modes, it engages reverse when throttle is negative, passes forward throttle normally, 
and sets neutral when throttle is near zero. 
Placeholder code is included for future use with desired velocity-based braking.
]]


-- Create commands object
local commands = AnalogCommands()

-- Define all Rover modes
local Modes = {
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


function update()
    local mode = vehicle:get_mode()
    local throttle_in = commands:throttle()  -- -100..100
    local throttle_out = 0
    local brake_out = 0

    if mode == Modes.ROVER_MODE_MANUAL then
        -- Manual passthrough
        local brake_in = (commands:brake() / 4500 * 100)  -- Convert 0..4500 to 0..100
        throttle_out = math.max(throttle_in, 0) -- passing only positive throttle.
        brake_out = brake_in

    elseif mode == Modes.ROVER_MODE_AUTO or mode == Modes.ROVER_MODE_GUIDED then
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

    commands:lua1(throttle_out)
    commands:lua2(brake_out)

    return update, 10
end

return update()
