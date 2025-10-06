--[[
custom telemetry for SA
--]]

function update()
    local as2_ms = airspeed:get_airspeed(1)
    local as2_mph = as2_ms * 2.23694

    gcs:send_named_float("AS2_MPH", as2_mph)

    return update,500
end

return update()

