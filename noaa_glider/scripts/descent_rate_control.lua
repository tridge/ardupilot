--[[
    When Enabled, control the Descent rate of the aircraft using adjustments to 
    speed setoint. 
--]]

local PARAM_TABLE_KEY = 16
local PARAM_TABLE_PREFIX = "GLD_DC_"

-- setup package place specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), 'could not add param table')

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

local GLD_DC_ENABLE = bind_add_param('ENABLE', 1, 0)
local GLD_DC_DES_VZ = bind_add_param('DES_VZ', 2, 10)
local GLD_DC_MIS_CMD = bind_add_param('MIS_CMD', 3, 31)     -- default to Loiter to Alt
local GLD_DC_AVG_PER = bind_add_param('AVG_PER', 4, 20) 
local GLD_DC_SPD_GAIN = bind_add_param('SPD_GAIN', 5, 1)

local _avg_cnt = 0.0
local _avg_sum = 0.0
local _target_alt_cm = -1.0
local _avg_num = GLD_DC_AVG_PER:get() * 10
local _update_cnt = 300 
local LUA_msg = false 



-- local function update()
--     if GLD_DC_ENABLE:get() == 0 then
--         return
--     end

--     if not arming:is_armed() then
--         return
--     end

--     if not vehicle:get_mode() == 10 then
--         return
--     end

--     _update_cnt = _update_cnt + 1

--     --gcs:send_text(0, "LUA: mission: " .. tostring(mission:get_current_nav_id()) .. " Want: " .. tostring(GLD_DC_MIS_CMD:get())) -- for debugging      
--     if not (mission:get_current_nav_id() == GLD_DC_MIS_CMD:get()) then 
--         _target_alt_cm = -1.0
--         if _update_cnt >= 300  then 
--             gcs:send_text(0, "LUA: DC Waiting for cmd: " .. tostring(GLD_DC_MIS_CMD:get()) .. " (" .. tostring(mission:get_current_nav_id()) .. ")")
--             _update_cnt = 0
--         end


--         return
--     end

--     --local _vel = ahrs:get_velocity_NED()


--     if not LUA_msg then
--         gcs:send_text(0, "LUA: DR control active target VZ = " .. tostring(GLD_DC_DES_VZ:get()))
--         LUA_msg = true
--     end

--     local next_WP = vehicle:get_target_location()
--     local next_alt_cm = next_WP:alt()
--     local loc = ahrs:get_position()
--     local alt_cm = loc:alt()

--     -- first thing is check that the target alt has been set, if not, then set it to the current alt and then we can start controlling descent rate
--     if _target_alt_cm < 0.0 then 
--         _target_alt_cm = next_alt_cm
--         gcs:send_text(0, "LUA: Loiter target set: " .. tostring(_target_alt_cm/100))
--     end

--     if alt_cm < _target_alt_cm then 
--         gcs:send_text(0, "LUA: reached target alt, advancing.")
--         local i = mission:get_current_nav_index()
--         mission:set_current_cmd(i+1)
--         return
--     end


--     if math.abs(alt_cm - next_alt_cm) > 100000 then 
        
--         gcs:send_text(0, "LUA: Alt_T reset: " .. tostring(alt_cm/100) .. " / " .. tostring(next_alt_cm/100))
--         next_alt_cm = alt_cm - (500 * 100)
--         next_WP:alt(next_alt_cm)
--     else
--         next_WP:alt(next_alt_cm - (GLD_DC_DES_VZ:get() * 100)) -- add a little bit of lead to the altitude target to help with control
--     end

--     vehicle:set_target_location(next_WP) 

--     -- If this works, then we'll add an offset and a gain

--     if _update_cnt >= 60 then 
--         gcs:send_text(0, "LUA: Alt: " .. tostring(alt_cm * 0.01) .. " Target: " .. tostring(next_alt_cm * 0.01) .. " -> " .. tostring(_target_alt_cm * 0.01))
--         _update_cnt = 0
--     end


-- end



local function update()
    if GLD_DC_ENABLE:get() == 0 then
        return
    end

    if not arming:is_armed() then
        return
    end

    if not vehicle:get_mode() == 10 then
        return
    end

    --gcs:send_text(0, "LUA: mission: " .. tostring(mission:get_current_nav_id()) .. " Want: " .. tostring(GLD_DC_MIS_CMD:get())) -- for debugging      
    if not (mission:get_current_nav_id() == GLD_DC_MIS_CMD:get()) then 
        return
    end

    local _vel = ahrs:get_velocity_NED()


    if not LUA_msg then
        gcs:send_text(0, "LUA: DR control active target VZ = " .. tostring(GLD_DC_DES_VZ:get()))
        LUA_msg = true
    end

    

    -- averaging update
    _avg_sum = _avg_sum + _vel:z()
    _avg_cnt = _avg_cnt + 1.0

    if _avg_cnt > _avg_num then 
        _vza = _avg_sum / _avg_cnt
        _vz_sp = GLD_DC_DES_VZ:get()
        _vz_err = _vza - _vz_sp
        _avg_sum = 0.0
        _avg_cnt = 0.0
        --gcs:send_text(0, "LUA: avg VZ: " .. tostring(_vza) .. " Des: " .. tostring(_vz_sp) .. " Err" .. tostring(_vz_err) ) -- for debugging

        if math.abs(_vza) > 20.0 then -- assume we don't have the control yet 
            gcs:send_text(0, "LUA: VZ: " .. tostring(_vza) ..  " waiting for level.")
            return
        end


        if math.abs(_vz_err) > 0.35 then 
        --if _vz_err < 1.0 then
            local loc = ahrs:get_position()
            local alt = loc:alt() * 0.01
            
            local _as = vehicle:get_target_airspeed() --param:get('AIRSPEED_CRUISE')
            -- gcs:send_text(0, "LUA: SP_DEM: " .. tostring(_as))
            local _ts = _as + (-_vz_err * GLD_DC_SPD_GAIN:get() * 0.1)
            
            if _ts >= param:get('AIRSPEED_MIN') and _ts <= param:get('AIRSPEED_MAX') then 
                --gcs:send_text(0, "LUA: VZ/E: " .. tostring(_vza) .. "/" .. tostring(_vz_err) .. " ASP: " .. tostring(_ts))
                gcs:send_text(0, "LUA: VZ/E: " .. tostring(_vza) .. "/" .. tostring(_vz_err) .. ' SP_D:' .. tostring(_ts))
                --param:set('AIRSPEED_CRUISE', _ts)
                vehicle:do_change_airspeed(_ts)
            else
                gcs:send_text(0, "LUA: VZ/E: " .. tostring(_vza) .. "/" .. tostring(_vz_err) .. " ASP: " .. tostring(_ts) .. " out of range")
            end

        end
    end


end

    
local function loop()
    if GLD_DC_ENABLE:get() == 0 then
        return loop, 500
    end
    update()
    return loop, 100
end

gcs:send_text(0, "LUA: Loaded Descent Controller")
return loop,100