-- control SilentArrow drop

-- de-glitch lidar range for emergency landing
-- add default params in ROMFS

-- tests to run
-- check on loss of GPS, that we switch to pitot airspeed

done_init = false
local button_number = 1
local button_state = 0

local MODE_MANUAL = 0
local MODE_AUTO = 10

local LANDING_AMSL = 430.0
local RUNWAY_LENGTH = 1.0
local CHANGE_MARGIN = 100.0

local GLIDE_SLOPE = 6.0

local MAX_WIND = 10.0

local SMALL_WP_DIST = 500

-- expected height loss for a 180 degree turn, meters
local TURN_HEIGHT_LOSS = 180

local release_start_t = 0.0
local last_tick_t = 0.0
local last_mission_update_t = 0.0
local last_mission_setup_t = 0.0
local last_wp_change_t = 0.0
local release_t = 0

local ENABLE_LOGGING = false
local logging_init = false


function logit(txt)
   if not ENABLE_LOGGING then
      return
   end

   if (logging_init == false) then
      logfile = io.open("log.txt", "w")
      logging_init = true
   end

   logfile:write(txt .. "\n")
   logfile:flush()
end

DO_JUMP = 177
NAV_WAYPOINT = 16
NAV_LAND = 21
DO_LAND_START = 189

local APPROACH_DIST_MAX = 6400
local BASE_DIST = 1390
local STEP_RATIO = 0.5
-- see if we are running on SITL
local is_SITL = param:get('SIM_SPEEDUP')

local TARGET_AIRSPEED = param:get('AIRSPEED_CRUISE')

function get_glide_slope()
   GLIDE_SLOPE = param:get('SCR_USER2')
   if GLIDE_SLOPE <= 0 then
      GLIDE_SLOPE = 6.0
   end
end

-- fill in LANDING_AMSL, height of first landing point in mission
function get_landing_AMSL()
   local N = mission:num_commands()
   for i = 1, N-1 do
      local m = mission:get_item(i)
      if m:command() == NAV_LAND then
         local loc = get_location(i)
         ahrs:set_home(loc)
         LANDING_AMSL = m:z()
         return
      end
   end
end

-- return ground course in degrees
function ground_course()
   if gps:status(0) >= 3 and gps:ground_speed(0) > 20 then
      return gps:ground_course(0)
   end
   if gps:num_sensors() >= 2 and gps:status(1) >= 3 and gps:ground_speed(1) > 20 then
      return gps:ground_course(1)
   end
   -- use mission direction
   local cnum = mission:get_current_nav_index()
   local N = mission:num_commands()
   if cnum > N-1 then
      return 0.0
   end
   if mission:get_item(cnum):command() == NAV_WAYPOINT then
      local loc1 = get_position()
      local loc2 = get_location(cnum+1)
      return math.deg(loc1:get_bearing(loc2))
   end
   return 0.0
end

function feet(m)
   return m * 3.2808399
end

function wrap_180(angle)
   if angle > 180 then
      angle = angle - 360.0
   end
   return angle
end

function right_direction(cnum)
   local loc = get_position()
   if loc == nil then
      return false
   end
   local loc2 = get_location(cnum)
   local gcrs = wrap_180(ground_course())
   local gcrs2 = wrap_180(math.deg(loc:get_bearing(loc2)))
   local err = wrap_180(gcrs2 - gcrs)
   local dist = loc:get_distance(loc2)
   if dist < 100 then
      -- too close
      return false
   end
   if math.abs(err) > 60 then
      return false
   end
   return true
end

function resolve_jump(i)
   local m = mission:get_item(i)
   while m:command() == DO_JUMP do
      i = math.floor(m:param1())
      m = mission:get_item(i)
   end
   return i
end

-- return true if cnum is a candidate for wp selection
function is_candidate(cnum)
   local N = mission:num_commands()
   if cnum > N-3 then
      return false
   end
   m = mission:get_item(cnum)
   m2 = mission:get_item(cnum+1)
   m3 = mission:get_item(cnum+2)
   if m:command() == NAV_WAYPOINT and m2:command() == NAV_WAYPOINT and (m3:command() == DO_JUMP or m3:command() == NAV_WAYPOINT or m3:command() == NAV_LAND) then
      return true
   end
   return false
end


-- return true if cnum is a candidate for wp change
function is_change_candidate(cnum)
   logit(string.format('is_change_candidate(%d)', cnum))

   if is_candidate(cnum) then
      logit(string.format(' YES -> is_candidate'))
      return true
   end
   local loc = ahrs:get_position()
   local m1 = mission:get_item(cnum)
   if m1:command() ~= NAV_WAYPOINT then
      -- only change when navigating to a WP
      logit(string.format(' NO -> not WP'))
      return false
   end
   if loc:alt() * 0.01 - LANDING_AMSL > 1000 then
      -- if we have lots of height then we can change
      logit(string.format(' YES -> plenty of height'))
      return true
   end
   local loc2 = get_location(cnum)
   if loc:get_distance(loc2) < SMALL_WP_DIST then
      -- if we are within 1.5km then no change
      logit(string.format(' NO -> within %.0f', SMALL_WP_DIST))
      return false
   end
   if loc:get_distance(loc2) > 3500 then
      -- if a long way from target allow change
      logit(string.format(' YES -> long way'))
      return true
   end
   if cnum > mission:num_commands()-2 then
      logit(string.format(' NO -> num cmds'))
      return false
   end
   local m2 = mission:get_item(cnum+1)
   if m1:command() == NAV_WAYPOINT and m2:command() ~= NAV_LAND then
      -- allow change of cross WPs when more than 2km away
      logit(string.format(' YES -> is land'))
      return true
   end
   logit(string.format(' NO -> default'))
   return false
end

function get_location(i)
   local m = mission:get_item(i)
   local loc = Location()
   loc:lat(m:x())
   loc:lng(m:y())
   loc:relative_alt(false)
   loc:terrain_alt(false)
   loc:origin_alt(false)
   loc:alt(math.floor(m:z()*100))
   return loc
end

function get_position()
   local loc = ahrs:get_position()
   if not loc then
      loc = gps:location(0)
   end
   if not loc and gps:num_sensors() >= 2 then
      loc = gps:location(1)
   end
   if not loc then
      return nil
   end
   return loc
end

-- vector2 cross product
function vec2_cross(vec1, vec2)
   return vec1:x()*vec2:y() - vec1:y()*vec2:x()
end

-- vector2 dot product
function vec2_dot(vec1, vec2)
   return vec1:x()*vec2:x() + vec1:y()*vec2:y()
end

function constrain(v, minv, maxv)
   if v < minv then
      v = minv
   end
   if v > maxv then
      v = maxv
   end
   return v
end

-- calculate wind adjustment for a WP segment
function wind_adjustment(loc1, loc2)
   local distNE = loc1:get_distance_NE(loc2)
   local dist = distNE:length()
   if dist < 1 then
      return 0.0
   end
   distNE:normalize()
   local wind3d = ahrs:wind_estimate()

   -- get 2d wind
   local wind2d = Vector2f()
   wind2d:x(wind3d:x())
   wind2d:y(wind3d:y())

   -- dot product gives component along flight path
   local dot = vec2_dot(distNE, wind2d)
   dot = constrain(dot, -MAX_WIND, MAX_WIND)

   local change = -dot / TARGET_AIRSPEED
   return dist * change
end

function turn_adjustment(bearing_change_deg)
   local height_loss = TURN_HEIGHT_LOSS * math.abs(bearing_change_deg / 180.0)
   return height_loss * GLIDE_SLOPE
end

-- get distance to landing point, ignoring current location
function distance_to_land_nopos(cnum)
   local N = mission:num_commands()
   if cnum >= N then
      return -1
   end
   local distance = 0
   local i = cnum
   local last_bearing = 0
   while i < N do
      m = mission:get_item(i)
      if m:command() == NAV_LAND then
         break
      end
      local i2 = resolve_jump(i+1)
      local loc1 = get_location(i)
      local loc2 = get_location(i2)

      local d1 = loc1:get_distance(loc2)

      distance = distance + d1

      -- gcs:send_text(0, string.format("WP%u->WP%u d=%.0f dist=%.0f", i, i2, d1, distance))

      -- account for height lost in turns
      local bearing = wrap_180(math.deg(loc1:get_bearing(loc2)))
      if i == cnum then
         last_bearing = bearing
      end
      local bearing_change = math.abs(wrap_180(bearing - last_bearing))
      last_bearing = bearing
      distance = distance + turn_adjustment(bearing_change)

      -- account for wind
      distance = distance + wind_adjustment(loc1, loc2)

      i = i2
   end
   -- subtract half runway length, for ideal landing mid-runway
   return distance - RUNWAY_LENGTH * 0.5
end

-- get distance to landing point, with current location
function distance_to_land(cnum)
   local loc = get_position()
   local N = mission:num_commands()
   if cnum >= N then
      return -1
   end
   local loc1 = get_location(cnum)
   local distance = loc:get_distance(loc1)
   distance = distance + wind_adjustment(loc, loc1)

   local gcrs = wrap_180(ground_course())
   local wpcrs = wrap_180(math.deg(loc:get_bearing(loc1)))
   local bearing_error = math.abs(wrap_180(gcrs - wpcrs))

   distance = distance + turn_adjustment(bearing_error)

   return distance + distance_to_land_nopos(cnum)
end

-- see if we are on the optimal waypoint number for landing
-- given the assumed glide slope
function mission_update()
   logit("mission_update()")
   local cnum = mission:get_current_nav_index()
   if cnum <= 0 then
      logit("no mission")
      -- not flying mission yet
      return
   end
   local m = mission:get_item(cnum)
   if not m then
      logit("invalid mission")
      -- invalid mission?
      gcs:send_text(0, string.format("Invalid current cnum %u", cnum))
      return
   end
   if m:command() ~= NAV_WAYPOINT then
      -- only update when tracking to a waypoint (not LAND)
      logit("not NAV_WAYPOINT")
      return
   end
   local loc = get_position()
   if loc == nil then
      logit("no position")
      return
   end
   local current_distance = distance_to_land(cnum)
   local current_alt = loc:alt() * 0.01 - LANDING_AMSL
   local avail_dist = GLIDE_SLOPE * current_alt
   local current_slope = current_distance / current_alt
   local current_err = current_distance - avail_dist
   local original_err = current_err
   gcs:send_text(0, string.format("cnum=%u dist=%.0f alt=%.0f slope=%.2f err=%.0f",
                                  cnum, feet(current_distance), feet(current_alt), current_slope,
                                  feet(-current_err)))
   logit(string.format("cnum=%u dist=%.0f alt=%.0f slope=%.2f err=%.0f",
                       cnum, feet(current_distance), feet(current_alt), current_slope,
                       feet(-current_err)))

   local current_wp_loc = get_location(cnum)
   local current_wp_dist = loc:get_distance(current_wp_loc)
   if current_wp_dist < SMALL_WP_DIST then
      -- when within 1.5km of current wp consider moving to next WP
      if mission:get_item(cnum+1):command() == NAV_WAYPOINT then
         -- check if we should skip the current WP and move to the next WP
         local loc1 = get_position()
         local loc2 = get_location(cnum)
         local loc3 = get_location(cnum+1)
         local dist = loc1:get_distance(loc2)
         local dist_change = loc2:get_distance(loc3)
         local target_bearing = math.deg(loc1:get_bearing(loc2))
         local target_bearing2 = math.deg(loc1:get_bearing(loc3))
         local ang_change = math.abs(target_bearing - target_bearing2)
         if ang_change < 10 and dist < 1000 and dist_change < 1000 then
            gcs:send_text(0, string.format("Skip NEW WP %u ang=%.0f dc=%.0f", cnum+1, ang_change, dist_change))
            mission:set_current_cmd(cnum+1)
            return true
         end
      end
      logit(string.format("wp dist %.2f", current_wp_dist))
      return false
   end

   if not is_change_candidate(cnum) then
      -- only change if on a candidate now
      logit("not candidate")
      return false
   end

   local t = 0.001 * millis():tofloat()
   if t - last_wp_change_t < 5 then
      -- don't change too rapidly
      return
   end

   -- look for alternatives
   local N = mission:num_commands()
   local best = cnum
   for i = 1, N-3 do
      if is_candidate(i) and right_direction(i) then
         -- this is an alternative path
         local dist = distance_to_land(i)
         local err = dist - avail_dist
         local diff = math.abs(current_err) - math.abs(err)
         logit(string.format("  i=%u dist=%.0f err=%.0f current_err=%.0f diff=%.0f", i, dist, err, current_err, diff))
         if diff > CHANGE_MARGIN then
            best = i
            current_err = err
         end
      end
   end
   improvement = math.abs(current_err) - math.abs(original_err)
   if cnum ~= best then
      gcs:send_text(0, string.format("NEW WP %u err=%.0f imp=%.0f", best, current_err, improvement))
      logit(string.format("NEW WP %u err=%.0f imp=%.0f", best, current_err, improvement))
      mission:set_current_cmd(best)
      last_wp_change_t = t
   end
end

function release_trigger()
   gcs:send_text(0, string.format("release trigger"))
   vehicle:set_mode(MODE_AUTO)
   arming:arm_force()
   if mission:num_commands() > 2 then
      mission:set_current_cmd(2) -- ?
   end
   notify:handle_rgb(0,255,0,0)
end

function set_standby()
   local mode = vehicle:get_mode()
   if mode ~= MODE_MANUAL then
      gcs:send_text(0, string.format("forcing standby MANUAL"))
      vehicle:set_mode(MODE_MANUAL)
      arming:disarm()
      mission:set_current_cmd(1)
      -- maybe LOITER mode if no GPS lock?
   end
   local gps_status = gps:status(0)
   if gps_status == 0 and gps:num_sensors() >= 2 then
      gps_status = gps:status(1)
   end
   if gps_status == 0 then
      -- red flash slow for no GPS
      notify:handle_rgb(255,0,0,2)
   elseif mission:num_commands() <= 2 then
      -- red flash fast means no mission
      notify:handle_rgb(255,0,0,10)
   elseif gps_status >= 3 then
      -- green blinking 3D lock, manual
      notify:handle_rgb(0,255,0,2)
   else
      -- flashing blue for no 3D lock
      notify:handle_rgb(0,0,255,2)
   end
   local t = 0.001 * millis():tofloat()
   if t - last_tick_t > 10 then
      last_tick_t = t
      gcs:send_text(0, string.format("Drop: MANUAL idle "))
   end
end

function fix_WP_heights()
   gcs:send_text(0, string.format("Fixing WP heights"))
   local N = mission:num_commands()
   for i = 1, N-1 do
      local m = mission:get_item(i)
      if m:command() == NAV_WAYPOINT then
         local dist = distance_to_land_nopos(i)
         local current_alt = m:z()
         local new_alt = LANDING_AMSL + dist / GLIDE_SLOPE
         if math.abs(current_alt - new_alt) > -1 or m:frame() ~= 0 then
            gcs:send_text(0, string.format("Fixing WP[%u] d=%.0f alt %.1f->%.1f", i, dist, current_alt, new_alt))
            m:z(new_alt)
            m:frame(0)
            mission:set_item(i, m)
         end
      end
   end
end
-- see if mission is setup for auto-creation
function create_mission_check()
   if mission:num_commands() ~= 4 then
      return false
   end
   if mission:get_item(1):command() ~= NAV_LAND or mission:get_item(2):command() ~= NAV_LAND then
      return false
   end
   if mission:get_item(3):command() ~= DO_LAND_START then
      return false
   end
   return true
end

function loc_copy(loc)
   local loc2 = Location()
   loc2:lat(loc:lat())
   loc2:lng(loc:lng())
   loc2:alt(loc:alt())
   return loc2
end

-- set wp location
function wp_setloc(wp, loc)
   wp:x(loc:lat())
   wp:y(loc:lng())
   wp:z(loc:alt()*0.01)
end

-- get wp location
function wp_getloc(wp)
   local loc = Location()
   loc:lat(wp:x())
   loc:lng(wp:y())
   loc:alt(math.floor(wp:z()*100))
   return loc
end

-- shift a wp in polar coordinates
function wp_offset(wp, ang1, dist1)
   local loc = wp_getloc(wp)
   loc:offset_bearing(ang1,dist1)
   wp:x(loc:lat())
   wp:y(loc:lng())
end

-- copy and shift a location
function loc_shift(loc,angle,dist)
   local loc2 = Location()
   loc2:lat(loc:lat())
   loc2:lng(loc:lng())
   loc2:alt(loc:alt())
   loc2:offset_bearing(angle,dist)
   return loc2
end

-- add a waypoint to the end
function wp_add(loc,ctype,param1,param2)
   local wp = mavlink_mission_item_int_t()
   wp_setloc(wp,loc)
   wp:command(ctype)
   local seq = mission:num_commands()
   wp:seq(seq)
   wp:param1(param1)
   wp:param2(param2)
   mission:set_item(seq,wp)
end

-- adjust approach distance based on glide slope and location of LAND1
-- and DO_LAND_START
function adjust_approach_dist(land1_loc, dls_loc, angle)
   local alt_change = (dls_loc:alt() - land1_loc:alt()) * 0.01
   local target_dist = 1.3 * GLIDE_SLOPE * alt_change
   gcs:send_text(0, string.format("alt_change=%.2f target_dist=%.2f", alt_change, target_dist))

   -- converge on correct approach distance
   local dist
   for i = 1, 10 do
      local loc2 = loc_shift(land1_loc, angle, APPROACH_DIST_MAX)
      dist = land1_loc:get_distance(loc2) + loc2:get_distance(dls_loc)
      APPROACH_DIST_MAX = APPROACH_DIST_MAX * target_dist / dist
   end
   gcs:send_text(0, string.format("app_dist=%.2f dist=%.2f target_dist=%.2f", APPROACH_DIST_MAX, dist, target_dist))
end

function create_pattern(wp, basepos, angle, base_angle, runway_length)

   local jump_target = mission:num_commands() + 3

   wp:command(NAV_WAYPOINT)
   local loc = loc_copy(basepos)
   loc:offset_bearing(angle,APPROACH_DIST_MAX)
   loc:offset_bearing(base_angle,BASE_DIST)
   wp_add(loc,NAV_WAYPOINT,0,0)

   loc = loc_copy(basepos)
   loc:offset_bearing(angle,APPROACH_DIST_MAX)
   wp_add(loc,NAV_WAYPOINT,0,0)

   loc = loc_copy(basepos)
   loc:offset_bearing(angle,runway_length*1.2)
   wp_add(loc,NAV_WAYPOINT,0,0)

   loc = loc_copy(basepos)
   wp_add(loc,NAV_LAND,0,0)

   local step = runway_length*STEP_RATIO
   NUM_ALTERNATES = math.min(70,math.floor((APPROACH_DIST_MAX - runway_length*1.5) / step))

   gcs:send_text(0, string.format("NUM_ALTERNATES=%u", NUM_ALTERNATES))

   for i = 1, NUM_ALTERNATES do
      loc = Location()
      wp_add(loc,DO_JUMP,jump_target,-1)

      loc = loc_copy(basepos)
      loc:offset_bearing(angle,APPROACH_DIST_MAX - step*i)
      loc:offset_bearing(base_angle,BASE_DIST)
      wp_add(loc,NAV_WAYPOINT,0,0)

      loc = loc_copy(basepos)
      loc:offset_bearing(angle,APPROACH_DIST_MAX - step*i)
      wp_add(loc,NAV_WAYPOINT,0,0)
   end

   loc = Location()
   wp_add(loc,DO_JUMP,jump_target,-1)
end

-- auto-create mission
function create_mission()
   gcs:send_text(0, string.format("creating mission"))
   local land1_loc = get_location(1)
   local land2_loc = get_location(2)
   local dls = mission:get_item(3)
   local dls_loc = get_location(3)
   local runway_length = land1_loc:get_distance(land2_loc)

   local alt_agl = (dls_loc:alt() - land1_loc:alt()) * 0.01
   if alt_agl <= 0 then
      gcs:send_text(0, string.format("invalid altitudes"))
      return
   end

   -- refresh glide slope
   get_glide_slope()

   local flight_dist1 = dls_loc:get_distance(land1_loc)
   local flight_dist2 = dls_loc:get_distance(land2_loc)
   local slope1 = flight_dist1 / alt_agl
   local slope2 = flight_dist2 / alt_agl

   if slope1 > GLIDE_SLOPE then
      gcs:send_text(0, string.format("bad glide slope1 %.2f", slope1))
      return
   end
   if slope2 > GLIDE_SLOPE then
      gcs:send_text(0, string.format("bad glide slope2 %.2f", slope2))
      return
   end
   

   local bearing12 = math.deg(land1_loc:get_bearing(land2_loc))
   local bearing21 = math.deg(land2_loc:get_bearing(land1_loc))
   local base_angle1 = bearing12 + 90
   local base_angle2 = bearing12 - 90

   -- work out which base angle brings us closer to the DLS
   local base_dist1 = loc_shift(land1_loc,base_angle1,10):get_distance(dls_loc)
   local base_dist2 = loc_shift(land1_loc,base_angle2,10):get_distance(dls_loc)
   if base_dist1 < base_dist2 then
      base_angle = base_angle1
   else
      base_angle = base_angle2
   end

   adjust_approach_dist(land1_loc, dls_loc, bearing12)
   APPROACH_DIST_MAX = math.max(APPROACH_DIST_MAX, 3*runway_length)

   local wp = mission:get_item(1)
   wp:command(NAV_WAYPOINT)

   mission:clear()

   -- setup home
   wp_add(dls_loc,NAV_WAYPOINT,0,0)

   -- setup DLS1
   wp_add(dls_loc,DO_LAND_START,0,0)

   -- first pattern
   create_pattern(wp, land1_loc, bearing12, base_angle, runway_length)

   adjust_approach_dist(land2_loc, dls_loc, bearing21)
   APPROACH_DIST_MAX = math.max(APPROACH_DIST_MAX, 3*runway_length)
   
   -- setup DLS2
   wp_add(dls_loc,DO_LAND_START,0,0)

   -- second pattern
   create_pattern(wp, land2_loc, bearing21, base_angle, runway_length)

   fix_WP_heights()
end

-- init system
function init()
   button_state = button:get_button_state(button_number)
   get_landing_AMSL()
   get_glide_slope()

   gcs:send_text(0, string.format("LANDING_AMSL %.1f GLIDE_SLOPE %.1f", LANDING_AMSL, GLIDE_SLOPE))

   fix_WP_heights()
   done_init = true
end

--[[
   airspeed schedule in 30s steps, stops on final airspeed
--]]
local AIRSPEED_SCHEDULE_MPH = {
   140,
   135,
   130,
   125,
   120,
   130
}

function airspeed_update(dt)
   local stage = math.floor(dt / 30.0)
   stage = math.min(stage, #AIRSPEED_SCHEDULE_MPH-1)
   local spd_mph = AIRSPEED_SCHEDULE_MPH[stage+1]
   local spd_mps = spd_mph * 0.44704
   if TARGET_AIRSPEED ~= spd_mps then
      gcs:send_text(0, string.format("Target airspeed %.1f mph", spd_mph))
      param:set('AIRSPEED_CRUISE', spd_mps)
      TARGET_AIRSPEED = spd_mps
   end
end

function update()
   if not done_init then
      init()
   end
   if not is_SITL and rc:has_valid_input() and rc:get_pwm(8) > 1800 then
      -- disable automation
      notify:handle_rgb(255,255,255,10)
      return
   end
   if not arming:is_armed() and create_mission_check() then
      create_mission()
   end
   local t = 0.001 * millis():tofloat()

   local state
   if is_SITL then
      -- use armed state for button in SITL
      state = arming:is_armed()
   else
      state = button:get_button_state(button_number)
   end

   local state = button:get_button_state(button_number)
   if state ~= button_state then
      gcs:send_text(0, string.format("release: " .. tostring(state)))
      button_state = state
      if button_state then
         release_start_t = t
      end
   end

   if button_state and release_start_t > 0 and (t - release_start_t) > param:get('SCR_USER1') then
      release_trigger()
      release_start_t = 0.0
      release_t = t
   end

   if t - last_mission_setup_t >= 1.0 then
      last_mission_setup_t = t
      get_glide_slope()
      get_landing_AMSL()
   end

   if not button_state then
      set_standby()
   elseif mission:num_commands() < 2 then
      -- no mission, red fast
      notify:handle_rgb(255,0,0,10)
   elseif t - last_mission_update_t >= 1.0 then
      last_mission_update_t = t
      notify:handle_rgb(0,255,0,0)
      mission_update()
      if release_t > 0 then
         airspeed_update(t-release_t)
      end
   end
end

function set_fault_led()
   -- setup LED for lua fault condition
   notify:handle_rgb(255,255,0,2)
end

-- wrapper around update(). This calls update() at 10Hz
-- and if update faults then an error is displayed, but the script is not stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     local success, err = pcall(set_fault_led)
     return protected_wrapper, 1000
  end
  -- otherwise run at 10Hz
  return protected_wrapper, 100
end

-- start running update loop
return protected_wrapper()