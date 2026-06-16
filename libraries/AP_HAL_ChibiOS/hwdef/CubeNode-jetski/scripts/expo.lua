--[[
Expo Endurance Test Script
Cycles between an active phase (steering left/right at full throttle) and a rest
phase (no steering, no throttle). Only runs when armed.

Parameters:
  EXPO_ACTIVE   - active phase duration in minutes (default 1)
  EXPO_REST     - rest phase duration in minutes   (default 5)
  EXPO_CTR_DLY  - center hold delay between direction changes in ms (default 500)
--]]

local PARAM_TABLE_KEY = 10
assert(param:add_table(PARAM_TABLE_KEY, "EXPO_", 5),              'could not add param table')
assert(param:add_param(PARAM_TABLE_KEY, 1, 'ACTIVE',      1),     'could not add ACTIVE')
assert(param:add_param(PARAM_TABLE_KEY, 2, 'REST',         5),    'could not add REST')
assert(param:add_param(PARAM_TABLE_KEY, 3, 'CTR_DLY',    500),   'could not add CTR_DLY')

local PARAM_ACTIVE  = Parameter()
PARAM_ACTIVE:init('EXPO_ACTIVE')

local PARAM_REST = Parameter()
PARAM_REST:init('EXPO_REST')

local PARAM_CTR_DLY = Parameter()
PARAM_CTR_DLY:init('EXPO_CTR_DLY')

local STEER_HOLD_MS = 800

-- RC channels
local CH_STEER    = rc:get_channel(1)
assert(CH_STEER,    'could not get steering channel (1)')
local CH_THROTTLE = rc:get_channel(3)
assert(CH_THROTTLE, 'could not get throttle channel (3)')

-- PWM values
local PWM_STEER_LEFT    = 1000
local PWM_STEER_RIGHT   = 2000
local PWM_STEER_CENTER  = 1500
local PWM_THROTTLE_FULL = 2000
local PWM_THROTTLE_OFF  = 1000

local state         = 'ACTIVE'
local phase_end_ms  = millis() + (PARAM_ACTIVE:get() or 1) * 60000
local last_steer_ms = millis()
local steer_left    = true
local centering     = false   -- true while holding center between direction changes

local function set_rc(steer_pwm, throttle_pwm)
    CH_STEER:set_override(steer_pwm)
    CH_THROTTLE:set_override(throttle_pwm)
end

function update()
    local now = millis()

    if not arming:is_armed() then
        state         = 'RESTING'
        phase_end_ms  = now
        last_steer_ms = now
        centering     = false
        set_rc(PWM_STEER_CENTER, PWM_THROTTLE_OFF)
        return update, 20
    end

    if state == 'ACTIVE' then
        if now >= phase_end_ms then
            state        = 'RESTING'
            phase_end_ms = now + (PARAM_REST:get() or 5) * 60000
            centering    = false
            set_rc(PWM_STEER_CENTER, PWM_THROTTLE_OFF)
            gcs:send_text(6, string.format("Expo: resting for %.1f min", (PARAM_REST:get() or 5)))
        elseif centering then
            set_rc(PWM_STEER_CENTER, PWM_THROTTLE_FULL)
            if now - last_steer_ms >= (PARAM_CTR_DLY:get() or 500) then
                centering     = false
                steer_left    = not steer_left
                last_steer_ms = now
            end
        else
            local steer_pwm = steer_left and PWM_STEER_LEFT or PWM_STEER_RIGHT
            set_rc(steer_pwm, PWM_THROTTLE_FULL)
            if now - last_steer_ms >= STEER_HOLD_MS then
                centering     = true
                last_steer_ms = now
            end
        end

    elseif state == 'RESTING' then
        set_rc(PWM_STEER_CENTER, PWM_THROTTLE_OFF)

        if now >= phase_end_ms then
            state         = 'ACTIVE'
            phase_end_ms  = now + (PARAM_ACTIVE:get() or 1) * 60000
            last_steer_ms = now
            centering     = false
            gcs:send_text(6, string.format("Expo: active for %.1f min", (PARAM_ACTIVE:get() or 1)))
        end
    end

    return update, 20
end

gcs:send_text(6, "Expo endurance test script loaded.")
return update, 20
