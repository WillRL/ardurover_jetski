-- This script scans for devices on the i2c bus

---@diagnostic disable: need-check-nil


local AS5600 = i2c:get_device(0,0x36)
local ZPOS_MSB_REG = 0x01 -- Start position MSB
local ZPOS_LSB_REG = 0x02 -- Start position LSB
local RAWANGLE_MSB_REG = 0x0C -- Raw angle output (unscaled, unmodified) MSB 
local RAWANGLE_LSB_REG = 0x0D -- Raw angle output (unscaled, unmodified) LSB
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


local new_value = wrap_angle((msb << 8 | lsb) - 4096/2)
msb = (new_value >> 8) & 0x0F;
lsb = new_value & 0xFF;

local success = AS5600:write_register(ZPOS_LSB_REG, lsb)
success = success and AS5600:write_register(ZPOS_MSB_REG, msb)

if not success then
    gcs:send_text(0, "Failed to write angle to ZPOS reg")
    return
end

gcs:send_text(0, "Successfully wrote to ZPOS reg")

