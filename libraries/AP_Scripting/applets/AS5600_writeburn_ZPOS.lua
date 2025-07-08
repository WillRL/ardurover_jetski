-- This script scans for devices on the i2c bus

---@diagnostic disable: need-check-nil

local AS5600 = i2c:get_device(0, 0x36)
local ZMCO_REG = 0x00 -- ZPOS and MPOS burn count
local ZPOS_MSB_REG = 0x01 -- Start position MSB
local ZPOS_LSB_REG = 0x02 -- Start position LSB
local RAWANGLE_MSB_REG = 0x0C -- Raw angle output MSB 
local RAWANGLE_LSB_REG = 0x0D -- Raw angle output LSB
local AS5600_BURN_REG = 0xFF -- Burn register

AS5600:set_retries(10)

local function wrap_angle(ang)
    if ang < 0 then
        return 4096 + ang
    elseif ang > 4096 then
        return ang - 4096
    else
        return ang
    end
end

if not AS5600:read_registers(0) then
    gcs:send_text(0, "Unable to find AS5600 at 0x36")
    return
end
gcs:send_text(0, "Found AS5600 at 0x36")

local lsb = AS5600:read_registers(RAWANGLE_LSB_REG)
local msb = AS5600:read_registers(RAWANGLE_MSB_REG)

if not (lsb and msb) then
    gcs:send_text(0, "Failed to read raw angle")
    return
end

local new_value = wrap_angle((msb << 8 | lsb) - 4096 / 2)
msb = (new_value >> 8) & 0x0F
lsb = new_value & 0xFF

local success = AS5600:write_register(ZPOS_LSB_REG, lsb)
success = success and AS5600:write_register(ZPOS_MSB_REG, msb)

if not success then
    gcs:send_text(0, "Failed to write angle to ZPOS reg")
    return
end

local burn_count = AS5600:read_registers(ZMCO_REG)
if not burn_count or burn_count >= 3 then
    gcs:send_text(0, "Burn limit reached")
    return
end

success = AS5600:write_register(AS5600_BURN_REG, 0x80)
if not success then
    gcs:send_text(0, "Burn command failed")
    return
end

gcs:send_text(0, "Successfully burnt angle")
param:set_and_save("SCR_ENABLE", 0)
