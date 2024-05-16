--[[
    File Name: cx_built_in_test.lua
    Description: This script performs a continuous built-in test for various functionalities on Carbonix Aircrafts, focusing on ESC status check and fault detection.
    Owner: [Carbonix - Software Team]
]]

-- ******************* Macros *******************

local SCRIPT_NAME = 'CX_BIT'
local SCRIPT_VERSION = 1.0 -- Script Version


--------  MAVLINK/AUTOPILOT 'CONSTANTS'  --------
-- MAVLink Severity Levels
local MAV_SEVERITY_CRITICAL = 2
local MAV_SEVERITY_ERROR = 3
local MAV_SEVERITY_WARNING = 4
local MAV_SEVERITY_INFO = 6

-- Engine Types
local HIRTH_EFI_TYPE = 8

local ESC_WARMUP_TIME = 3000
local SERVO_OUT_THRESHOLD = 1010
local ESC_RPM_THRESHOLD = 10
-- ******************* Variables *******************

local number_of_esc = 5 --default value for Volanti

-- Add a new table to store the warm-up end times for each ESC
local esc_warmup_end_time = {}

local srv_number = {
    [1] = {"Motor1", 33},
    [2] = {"Motor2", 34},
    [3] = {"Motor3", 35},
    [4] = {"Motor4", 36},
    [5] = {"Motor5", 70},
    [6] = {"Motor6", 38},
    [7] = {"Elevator", 19},
    [8] = {"Rudder", 21},
    [9] = {"GPIO", -1},
    [10] = {"Script1", 94},
    [11] = {"Aileron", 4}
}

local srv_prv_telem_ms = {0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0, 0}
local srv_telem_in_err_status  = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}
local srv_rpm_in_err_status  = {false, false, false, false, false, false, false, false, false, false, false, false, false, false, false, false}

-- ******************* Objects *******************

local auth_id = arming:get_aux_auth_id()
assert(auth_id, SCRIPT_NAME .. ": could not get a prearm check auth id")

local params = {
    EFI_TYPE = Parameter()
}


-- ******************* Functions *******************

-- wrapper for gcs:send_text()
local function gcs_msg(severity, txt)
    if type(severity) == 'string' then
    -- allow just a string to be passed for simple/routine messages
        txt      = severity
        severity = MAV_SEVERITY_INFO
    end
    gcs:send_text(severity, string.format('%s: %s', SCRIPT_NAME, txt))
end

local function check_aircraft_type()
    if params.EFI_TYPE:get() == HIRTH_EFI_TYPE then
        number_of_esc = 4
    else
        number_of_esc = 5
    end
    return true
end

local function init_param()
    for param_name, param in pairs(params) do
        if not param:init(param_name) then
            return false
        end
    end
    return true
end

local function arming_check_init()
    -- check param fetch
    if not init_param() then
        arming:set_aux_auth_failed(auth_id, "Parameter Init Failed")
        gcs_msg(MAV_SEVERITY_WARNING, "Parameter Init Failed")
        return false
    end
    -- check aircraft Type
    if not check_aircraft_type() then
        arming:set_aux_auth_failed(auth_id, "Aircraft Type Check Failed")
        gcs_msg(MAV_SEVERITY_WARNING, "Aircraft Type Check Failed")
        return false
    end
    gcs_msg(MAV_SEVERITY_INFO, "Script Version " .. SCRIPT_VERSION .. " Initialized")
    arming:set_aux_auth_passed(auth_id)
    return true
end

-- Call this function whenever a motor starts running
local function esc_is_started(i)
    -- Set the warm-up end time for this ESC to 3 seconds from now
    esc_warmup_end_time[i] = millis() + ESC_WARMUP_TIME
end

-- Call this function whenever a motor stops running
local function esc_is_stopped(i)
    -- Clear the warm-up end time for this ESC
    esc_warmup_end_time[i] = nil
end

local function esc_check_loop()
    for i = 1, number_of_esc  do
        local esc_last_telem_data_ms = esc_telem:get_last_telem_data_ms(i-1):toint()
        -- Telem data timestamp check
        if not esc_last_telem_data_ms or esc_last_telem_data_ms == 0 or esc_last_telem_data_ms == srv_prv_telem_ms[i] then
            if srv_telem_in_err_status[i] == false then
                gcs_msg(MAV_SEVERITY_CRITICAL, "ESC " .. i .. " Telemetry Lost")
                srv_telem_in_err_status[i] = true
            end
        else
            if srv_telem_in_err_status[i] == true then
                gcs_msg(MAV_SEVERITY_INFO, "ESC " .. i .. " Telemetry Recovered")
                srv_telem_in_err_status[i] = false
            end
            if arming:is_armed() then
                local esc_rpm = esc_telem:get_rpm(i-1)
                local servo_out = SRV_Channels:get_output_pwm(srv_number[i][2])
                if esc_rpm and servo_out then
                    -- If the PWM is below the threshold, consider it a stopped motor situation
                    if servo_out < SERVO_OUT_THRESHOLD then
                        esc_is_stopped(i)
                    -- If the PWM is above the threshold consider it start the motor
                    elseif servo_out > SERVO_OUT_THRESHOLD and not esc_warmup_end_time[i]  then
                        esc_is_started(i)
                    -- If the motor is running and the PWM is above the threshold, check the RPM
                    elseif esc_warmup_end_time[i] and millis() > esc_warmup_end_time[i] then
                        if servo_out > SERVO_OUT_THRESHOLD and esc_rpm < ESC_RPM_THRESHOLD then
                            if srv_rpm_in_err_status[i] == false then
                                gcs_msg(MAV_SEVERITY_CRITICAL, "ESC " .. i .. " RPM Drop")
                                srv_rpm_in_err_status[i] = true
                            end
                        else
                            if srv_rpm_in_err_status[i] == true then
                                gcs_msg(MAV_SEVERITY_INFO, "ESC " .. i .. " RPM Recovered")
                                srv_rpm_in_err_status[i] = false
                            end
                        end
                    end
                else
                    gcs_msg(MAV_SEVERITY_CRITICAL, "ESC " .. i .. " Null Rcout or RPM")
                    srv_telem_in_err_status[i] = true
                end
            end
        end
        if esc_last_telem_data_ms and esc_last_telem_data_ms ~= 0 then
            srv_prv_telem_ms[i] = esc_last_telem_data_ms
        end
    end
end

local function arming_checks()
    -- check for status in srv_telem_in_err_status and also CX_SERVO_ERROR bit status
    local pre_arm_status = false-- check for status in srv_telem_in_err_status and also CX_SERVO_ERROR bit status
    arming:set_aux_auth_passed(auth_id)

    for i, status in ipairs(srv_telem_in_err_status) do
        if status == true then
            arming:set_aux_auth_failed(auth_id, "Actuator ".. i .. " Telemetry Error")
            return false
        end
    end
    if pre_arm_status == false then
        arming:set_aux_auth_passed(auth_id)
    end

end

local function update()
    esc_check_loop()
    arming_checks()
end

-- wrapper around update(). This calls update() and if update faults
-- then an error is displayed, but the script is not stopped
local function protected_wrapper()
    local success, err = pcall(update)
    if not success then
        gcs_msg(MAV_SEVERITY_ERROR, "Internal Error: " .. err)
        -- when we fault we run the update function again after 1s, slowing it
        -- down a bit so we don't flood the console with errors
        return protected_wrapper, 1000
    end
    return protected_wrapper, 200
end

local function script_exit()
    -- pre arm failure SCRIPT_NAME not Running
    arming:set_aux_auth_failed(auth_id, SCRIPT_NAME .. " Not Running")
    gcs_msg(MAV_SEVERITY_CRITICAL, "LUA SCRIPT EXIT   ... Need Reboot to Reinitialize")
end


-- ******************* Main *******************
if arming_check_init() then
    return protected_wrapper, 10000
end

script_exit()
