-- GPIO toggle and read example for pins 50–55

---@diagnostic disable: need-check-nil

-- GPIO pins to use
local gpio_pins = {50, 51, 52, 53, 54, 55}

-- Configure pins as outputs
for _, pin in ipairs(gpio_pins) do
  gpio:pinMode(pin, 0) -- 1 = output
end

function update()
  local states = {}

  -- Read GPIO states
  for _, pin in ipairs(gpio_pins) do
    states[#states + 1] = string.format("%d=%s", pin, tostring(gpio:read(pin)))
  end

  -- Send GPIO states to GCS
  gcs:send_text(0, "GPIO: " .. table.concat(states, " "))
  return update, 1000 -- run every 1 second
end

return update()
