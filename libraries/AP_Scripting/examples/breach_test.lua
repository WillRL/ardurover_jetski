--[[
Relay Toggle Test Script
This script toggles Relay 1 ON and OFF every 2 seconds.
It prints the relay state to the GCS so you can confirm relay function.
]]

local RELAY_NUM = 0           -- Relay 1 (index starts from 0)
local TOGGLE_INTERVAL = 2000  -- 2 seconds
local last_toggle_time = 0
local relay_state = false

gcs:send_text(6, "Relay toggle test script started...")

function update()
    local now = millis()

    if now - last_toggle_time >= TOGGLE_INTERVAL then
        last_toggle_time = now
        relay_state = not relay_state

        if relay_state then
            relay:on(RELAY_NUM)
            gcs:send_text(6, string.format("Relay %d ON", RELAY_NUM + 1))
        else
            relay:off(RELAY_NUM)
            gcs:send_text(6, string.format("Relay %d OFF", RELAY_NUM + 1))
        end
    end

    return update, 100  -- run every 100 ms
end

return update()
