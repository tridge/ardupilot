
-- LED is switched on using motor function
local MOTOR_LED_FUNCTION = 94

-- Ensure that the out2_function min = 200 and max = 10000
local MOTOR_LED_PWM_VAL_TRIM  = 5000
local MOTOR_LED_PWM_VAL_MIN  = 200
local MOTOR_LED_PWM_VAL_MAX  = 10000
local MOTOR_LED_PWM_VAL_OFF  = 0

-- Arming check interval
local LED_CYCLE_TIME = 2000


local LED_servo = SRV_Channels:find_channel(MOTOR_LED_FUNCTION)


gcs:send_text(0, "LED -- BEGIN")


-- LED is switched ON using a PWM signal configured as MOTOR_LED_FUNCTION
-- This is done to overcome the design limitations on CPN
local function switch_LED(LED_val)

    if (LED_servo ~= null) then
        SRV_Channels:set_output_pwm_chan_timeout(LED_servo, LED_val, LED_CYCLE_TIME)
    end

    return check_arming, LED_CYCLE_TIME
end



-- Function to continuously check if the vehicle is armed or not
-- ARMED -> LED_ON
-- DISARMED -> LED_OFF 
local function check_arming()
    vehicle_armed = arming:is_armed()
    
    if vehicle_armed == true then
        switch_LED(MOTOR_LED_PWM_VAL_MAX)
    else
        switch_LED(MOTOR_LED_PWM_VAL_MIN)
    end

    return check_arming, LED_CYCLE_TIME
end

return check_arming(), 10000
