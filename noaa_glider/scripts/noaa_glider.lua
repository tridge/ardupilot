-- HORUS V2 LUA Script

local PARAM_TABLE_KEY = 18
local PARAM_TABLE_PREFIX = "GLD_AFS_"

-- setup package place specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 16), 'could not add param table')


-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return Parameter(PARAM_TABLE_PREFIX .. name)
end

local GLD_AFS_FENCE_FS_ENABLE = bind_add_param('FNC_EN', 1, 0)
local GLD_AFS_FENCE_MARGIN_KM = bind_add_param('FNC_MARG', 2, 30)
local GLD_AFS_CHUTE_MIN_FS_EN = bind_add_param('CHT_FSEN', 3, 0)
local GLD_AFS_CHUTE_MIN_MSL = bind_add_param('CHT_MMSL', 4, -1)

local MODE_AUTO = 10
local MODE_RTL = 11
local MISSION_WAIT_ALT_CMD = 83
local BALLOON_RELEASE_AUX_FUNC = 221 -- See https://github.com/ArduPilot/ardupilot/pull/30962/changes/46626d64a15e6b4293add4
local BALLOON_RELEASE_AUX_VALUE = 2 -- 0 low, 1 mid, 2 high
-- Conversions

local FEET_TO_METERS = 0.3048
local METERS_TO_FEET = 1.0/FEET_TO_METERS

local KNOTS_TO_MPS = 0.51444

-- altitude to force chute open if in AUTO and we've cut balloon free
-- overridden by
local CHUTE_OPEN_ALT_DEFAULT = 2700*FEET_TO_METERS

-- margin inside fence when armed to enable fence
local FENCE_MARGIN_DEFAULT = 50

local K_PARACHUTE = 27

local BALLOON_RELEASE_CHAN = 10

local last_mfs_state = 0
local last_mfs_alt = -9999
local prev_fence_margin = -999

-- chute checking is enabled when 50m above chute deploy alt
local chute_check_armed = false
local chute_check_margin = 50
local target_keas = 55
local max_alt_ft = 0.0






-- constrain a value between limits
function constrain(v, vmin, vmax)
   if v < vmin then
      v = vmin
   end
   if v > vmax then
      v = vmax
   end
   return v
end

local chute_triggered = false

function get_dist_home()
   local loc = ahrs:get_position()
   local home = ahrs:get_home()
   if not loc or not home then
      -- no position or home yet, can't do fence
      return 0
   end
   return loc:get_distance(home)
end

function balloon_has_released()
   if SRV_Channels:get_output_pwm_chan(BALLOON_RELEASE_CHAN-1) >= 1750 then
      return true
   end
   return false
end

function check_chute()
   local chute_min_en = GLD_AFS_CHUTE_MIN_FS_EN:get()
   if chute_min_en > 0 then

      if chute_alt >= 0 then
         local position = ahrs:get_position()
         local altitude_absolute = position:alt()
         local chute_alt = GLD_AFS_CHUTE_MIN_MSL:get() -- alt in meters

         -- Only arm the auto chute if you've exceeded the target alt already
         if not chute_check_armed then
            if alt > chute_alt + chute_check_margin then
               gcs:send_text(0, string.format("Armed chute check at %.0fft", alt*METERS_TO_FEET))
               chute_check_armed = true
            end
         end

         -- IF the auto chute is armed, the trigger the parachute if we go below that chute alt
         if chute_check_armed and alt < chute_alt then
            if not chute_triggered then
               chute_triggered = true
               gcs:send_text(0, string.format("Triggering chute at %.0fft", alt*METERS_TO_FEET))
               parachute:release()
            end
         end
      end
   end
end


function check_AFS()
   if arming:is_armed() then
      fence_margin = GLD_AFS_FENCE_MARGIN_KM:get() --param:get("SCR_USER2")
      if fence_margin <= 0 then
         fence_margin = FENCE_MARGIN_DEFAULT
      end

      if fence_margin ~= prev_fence_margin then
         prev_fence_margin = fence_margin
 
         gcs:send_text(0, string.format("LUA: Fence Margin AFS armed to: %.0f km",fence_margin))


      end

      --local margin = vehicle:fence_distance_inside()
      local margin = fence:get_breach_distance(4)
      

      margin = -margin

      -- only enable the fence if the ac is released and 
      -- we are inside the margin
      -- if (balloon_has_released() or margin >= fence_margin) then
      --    if not vehicle:fence_enabled() then
      --       if vehicle:enable_fence() then
      --          gcs:send_text(0, "LUA: Enabled fence")
      --       else
      --          gcs:send_text(0, "LUA: Fence enable FAILED")
      --       end
      --    end
      -- end

      -- This needs to be rewritten so that fence is enabled all the time, and we 
      -- cut the balloon if we breach before balloon release, and we RTL if we breach after balloon release.

      -- check if balloon not released, and in margin buffer, 
      -- advance mission waypoint to trigger balloon release.
      -- GLD_AFS_FENCE_FS_ENABLE enables Margin Failsafe
      if GLD_AFS_FENCE_FS_ENABLE:get() > 0 then
         if not balloon_has_released() then
            if last_mfs_state == 2 then
               local i = mission:get_current_nav_index()

               if mission:get_current_nav_id() == MISSION_WAIT_ALT_CMD then 
                  --mission:set_current_cmd(i+1)
                  --gcs:send_text(0, "Mission Advanced to Pullup")

                  -- Send AUX command to advance to pullup
                  gcs:send_text(0, "LUA Balloon MFS: Forcing Balloon Release!!!!")
                  rc:run_aux_function(BALLOON_RELEASE_AUX_FUNC,BALLOON_RELEASE_AUX_VALUE)
               end
               last_mfs_state = 3
            elseif margin < fence_margin and last_mfs_state < 2 then
               last_mfs_state = 2
               gcs:send_text(0, "!! Fence Margin Failsafe !!")

            elseif margin < 2*fence_margin and last_mfs_state < 1 then
                  gcs:send_text(0, string.format("LUA: MFS Near %.0f / %.0f", margin,fence_margin))
                  last_mfs_state = 1
            elseif margin >= 2*fence_margin and last_mfs_state > 0 then
               last_mfs_state = 0
               gcs:send_text(0, string.format("LUA: MFS Clear %.0f / %.0f", margin,fence_margin))
            end
         else
            -- balloon has released
            if fence:get_breaches() > 0 then
               local breach_time = millis() - fence:get_breach_time()
               if vehicle:get_mode() == MODE_AUTO and mission:get_current_nav_id() == MISSION_WAIT_ALT_CMD and breach_time < 10000 then 
                  gcs:send_text("Waiting to RTL %.0f",breach_time/1000.0)
               elseif vehicle:get_mode() ~= MODE_RTL then 
                  vehicle:set_mode(MODE_RTL) -- RTL
                  gcs:send_text(0, "LUA: Fence Breached, RTL mode enabled")
               end
            end
            last_mfs_state = 0 -- reset in case we come back into fence after balloon release
         end
      end


      local position = ahrs:get_position()
      local altitude_absolute = position:alt() / 100.0
      if (math.abs(altitude_absolute - last_mfs_alt) > 1000) then
         last_mfs_alt = altitude_absolute
         if fence:get_enabled_fences() & 4 == 0 then
            gcs:send_text(0, string.format("LUA: WARNING: Fence not enabled"))
         else
            gcs:send_text(0, string.format("LUA: Fence Dist %.0f / %.0f", margin,fence_margin))
         end
         
      end

      if AFS:should_crash_vehicle() and not balloon_has_released() then
         gcs:send_text(0, "LUA: AFS balloon release (crash)")
         --SRV_Channels:set_output_pwm_chan(BALLOON_RELEASE_CHAN-1, 2000)
         rc:run_aux_function(BALLOON_RELEASE_AUX_FUNC,BALLOON_RELEASE_AUX_VALUE)
      end

   end
end


function update()
   check_AFS()

   if arming:is_armed() and vehicle:get_mode() == MODE_AUTO then
      check_chute()
   end

   return update, 1000
end

gcs:send_text(0, string.format("Loader glider script"))

gcs:send_text(0, string.format("NOAA Glider LUA VTBD Date_TBD"))

return update, 1000