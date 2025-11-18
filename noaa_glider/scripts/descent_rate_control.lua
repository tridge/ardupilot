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

local _avg_num = GLD_DC_AVG_PER:get() * 10

local LUA_msg = false 

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
            return
        end


        if math.abs(_vz_err) > 0.35 then 
        --if _vz_err < 1.0 then
            local loc = ahrs:get_position()
            local alt = loc:alt() * 0.01
            -- local _sign = 1
            -- if _vz_err > 0 then  --vz is in m/s down
            --     _sign = -1
            -- end
            -- if alt < 10000 then
            --     -- placeholder, ideally this would be smarter 
            --     _sign = _sign * -1
            -- end
            -- gcs:send_text(0, "LUA: ALT: " .. tostring(alt) .. "S: " .. tostring(_sign))
            
            local _as = vehicle:get_target_airspeed() --param:get('AIRSPEED_CRUISE')
            -- gcs:send_text(0, "LUA: SP_DEM: " .. tostring(_as))
            local _ts = _as + (-_vz_err * GLD_DC_SPD_GAIN:get())
            gcs:send_text(0, "LUA: VZ/E: " .. tostring(_vza) .. "/" .. tostring(_vz_err) .. ' SP_D:' .. tostring(_as))
            if _ts >= param:get('AIRSPEED_MIN') and _ts <= param:get('AIRSPEED_MAX') then 
                --gcs:send_text(0, "LUA: VZ/E: " .. tostring(_vza) .. "/" .. tostring(_vz_err) .. " ASP: " .. tostring(_ts))
                --param:set('AIRSPEED_CRUISE', _ts)
                vehicle:do_change_airspeed(_ts)
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
return loop,1000