local roll_rc = rc:get_channel(2)
local pitch_rc = rc:get_channel(1)
local ntrl_pwm = 1500
local pos_step_pwm = 1700
local neg_step_pwm = 1300

local user_param1 = Parameter('SCR_USER1')
local user_param2 = Parameter('SCR_USER2')

local states = {
    IDLE        = 0,
    ROLL_HI     = 1,
    ROLL_LO     = 2,
    IDLE2       = 3,
    PITCH_HI    = 4,
    PITCH_LO    = 5,
 }

 local current_state = states.IDLE

function update()

    if current_state == states.IDLE or current_state == states.IDLE2 then
        gcs:send_text(0, "NTRL")
        roll_rc:set_override(ntrl_pwm)
        pitch_rc:set_override(ntrl_pwm)
    elseif current_state == states.ROLL_HI then
        gcs:send_text(0, "RHI")
        roll_rc:set_override(pos_step_pwm)
        pitch_rc:set_override(ntrl_pwm)
    elseif current_state == states.ROLL_LO then 
        gcs:send_text(0, "RLO")
        roll_rc:set_override(neg_step_pwm)
        pitch_rc:set_override(ntrl_pwm)
    elseif current_state == states.PITCH_HI then 
        gcs:send_text(0, "PHI")
        roll_rc:set_override(ntrl_pwm)
        pitch_rc:set_override(pos_step_pwm)
    elseif current_state == states.PITCH_LO then  
        gcs:send_text(0, "PLO")
        roll_rc:set_override(ntrl_pwm)
        pitch_rc:set_override(neg_step_pwm)
    end

    -- MAKE SURE FWD MOTOR IS 94 AND AFT IS 95
    local fwd_pwm = user_param1:get()
    local aft_pwm = user_param2:get()
    if not fwd_pwm then
        gcs:send_text(0, "NO FWD")
        SRV_Channels:set_output_pwm(94, 0)
    elseif fwd_pwm > 0 then
        gcs:send_text(0, "FWD")
        SRV_Channels:set_output_pwm(94, fwd_pwm)
    else
        gcs:send_text(0, "NO FWD")
        SRV_Channels:set_output_pwm(94, 0)
    end

    if not aft_pwm then
        gcs:send_text(0, "NO AFT")
        SRV_Channels:set_output_pwm(95, 0)
    elseif aft_pwm > 0 then
        gcs:send_text(0, "AFT")
        SRV_Channels:set_output_pwm(95, aft_pwm)
    else
        gcs:send_text(0, "NO AFT")
        SRV_Channels:set_output_pwm(95, 0)
    end
        
    current_state = (current_state + 1) % 6
    return update, 250
end

return update()


