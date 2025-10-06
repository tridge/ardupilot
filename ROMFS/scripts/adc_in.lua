--[[
send ADC input pin as NAMED_VALUE_FLOAT
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local analog_in = analog:channel()
if not analog_in:set_pin(8) then -- ADC input pin
  gcs:send_text(0, "Invalid analog pin")
end

function update()
    local voltage = analog_in:voltage_average() * 0.5
    local press_PSI = (voltage - 0.6) * 39.37
    if press_PSI < 0.0 then
        press_PSI = 0.0
    end
    if press_PSI > 100.0 then
        press_PSI = 100.0
    end
    gcs:send_named_float("PRESS_PSI", press_PSI)

    return update, 500
end

gcs:send_text(MAV_SEVERITY.INFO, "Loaded ADC Pressure")

return update()
