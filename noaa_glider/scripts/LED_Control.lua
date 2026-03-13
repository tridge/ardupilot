-- Servo 6 HIGH when armed, LOW when disarmed

local PARAM_TABLE_KEY = 17
local PARAM_TABLE_PREFIX = "GLD_LED_"

-- setup package place specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

local GLD_LED_CH = bind_add_param('CHAN', 1, 5)
local GLD_LED_PWM_ON= bind_add_param('PWM_ON', 2, 1900)
local GLD_LED_PWM_OFF= bind_add_param('PWM_OFF', 3, 1100)

function update()
    if arming:is_armed() then
        SRV_Channels:set_output_pwm_chan(GLD_LED_CH:get(), GLD_LED_PWM_ON:get())
    else
        SRV_Channels:set_output_pwm_chan(GLD_LED_CH:get(), GLD_LED_PWM_OFF:get())
    end

    return update, 1000  -- run every 1000 ms
end

gcs:send_text(0, string.format("LUA: STROBES ON WHEN ARMED."))

return update(),1000