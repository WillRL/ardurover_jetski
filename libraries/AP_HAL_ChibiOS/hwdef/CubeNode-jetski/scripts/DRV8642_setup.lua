--[[
   DRV8462 stepper motor driver setup script for CubeNode-jetski.
   Configures the DRV8462 SPI stepper driver on startup.
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}


-- Mirror of C++ constants (KV sense resistor gain, IVREF internal reference)
local KV    = 0.33
local IVREF = 3.3

local STEP_FRQ_TOL = 2
local DECAY = 6
local drv = DRV8462SPI
-- Convert amps to 8-bit DAC value (0-255)
function drv8462_current_to_dac(I)
    local val = (I * KV) / IVREF * 255.0
    return math.floor(math.max(0.0, math.min(255.0, val)))
end

local function setup_drv8462()
    if not drv then
        gcs:send_text(MAV_SEVERITY.ERROR, "DRV8462: driver not available")
        return false
    end

    -- Step 1: clear faults before anything else
    local cfg = DRV8462SPIConfig()
    cfg:clr_flt(1)
    if not drv:write_config(cfg) then
        gcs:send_text(MAV_SEVERITY.ERROR, "DRV8462: failed to clear faults")
        return false
    end

    -- Step 2: main configuration
    cfg = DRV8462SPIConfig()

    -- CTRL1: smart-ripple decay, enable outputs
    cfg:en_out(1)

    -- CTRL13: use internal 3.3 V reference
    cfg:vref_int_en(1)

    -- filtering
    cfg:frq_chg(1)       -- enable step frequency checking
    cfg:step_frq_tol(STEP_FRQ_TOL)  -- set step frequency tolerance (2 = 4% filtering)
    cfg:decay(DECAY)     -- set decay mode to 6 (smart dynamic decay)

    -- -- CTRL12/13: standstill mode with gradual current fall
    -- cfg:en_stsl(1)
    -- cfg:tstsl_fall(3)   -- ~50 ms fall time
    -- cfg:tstsl_dly(10)   -- ~40 ms delay before entering standstill

    if not drv:write_config(cfg) then
        gcs:send_text(MAV_SEVERITY.ERROR, "DRV8462: failed to apply config")
        return false
    end

    gcs:send_text(MAV_SEVERITY.INFO, "DRV8462: setup complete")
    return true
end

function update()
    local cfg = DRV8462SPIConfig()
    cfg:step(1)
    gcs:send_text(MAV_SEVERITY.INFO, "step: " .. tostring(cfg:step()))
    gcs:send_text(MAV_SEVERITY.INFO, "en_out: " .. tostring(cfg:en_out()))
    return update(), 100
end

if not setup_drv8462() then
    gcs:send_text(MAV_SEVERITY.CRITICAL, "DRV8462: setup FAILED")
end



