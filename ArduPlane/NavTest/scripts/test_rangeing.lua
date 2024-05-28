--[[
   test injecting range information from a radio
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

local PARAM_TABLE_KEY = 49
local PARAM_TABLE_PREFIX = "RRNG_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   local p = Parameter(PARAM_TABLE_PREFIX .. name)
   assert(p, string.format("count not find parameter %s", name))
   return p
end

-- setup script specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 5), 'could not add param table')

--[[
  // @Param: RRNG_FREQ
  // @DisplayName: radio range frequency
  // @Description: frequency of radio range information
  // @Units: Hz
  // @User: Standard
--]]
local RRNG_FREQ = bind_add_param('FREQ', 1, 1)

--[[
  // @Param: RRNG_ACC
  // @DisplayName: radio range accuracy
  // @Description: Radio range 1-Sigma accuracy. Must be >= RRNG_ROUND
  // @Units: m
  // @User: Standard
--]]
local RRNG_ACC = bind_add_param('ACC', 2, 30)

--[[
  // @Param: RRNG_ROUND
  // @DisplayName: radio range rounding
  // @Description: radio range rounding
  // @Units: m
  // @User: Standard
--]]
local RRNG_ROUND = bind_add_param('ROUND', 3, 30)

--[[
  // @Param: RRNG_OFS_N
  // @DisplayName: base station offset North
  // @Description: The North position of the radio base station wrt home.
  // @Units: m
  // @User: Standard
--]]
local RRNG_OFS_N = bind_add_param('OFS_N', 4, 0)

--[[
  // @Param: RRNG_OFS_E
  // @DisplayName: base station offset East
  // @Description: The East position of the radio base station wrt home.
  // @Units: m
  // @User: Standard
--]]
local RRNG_OFS_E = bind_add_param('OFS_E', 5, 0)

local MIN_DIST = 100.0
local MAX_DIST = 100000.0

local last_send = 0

local function provide_range()
   if RRNG_FREQ:get() <= 0 then
      return
   end
   local now_ms = millis()
   local period_ms = 1000 / RRNG_FREQ:get()
   if now_ms - last_send < period_ms then
      return
   end
   last_send = now_ms

   local loc = gps:location(0)
   if not loc then
      gcs:send_text(MAV_SEVERITY.ERROR, "No loc")
      return
   end
   local home = ahrs:get_home()
   home:offset(RRNG_OFS_N:get(),RRNG_OFS_E:get())
   local dist = home:get_distance(loc)
   if dist < MIN_DIST or dist > MAX_DIST then
      return
   end
   dist = math.floor(dist / RRNG_ROUND:get()) * RRNG_ROUND:get()
   ahrs:writeRangeToLocation(dist, RRNG_ACC:get(), home, now_ms)
end

local function update()
   provide_range()
   return update,100
end

gcs:send_text(MAV_SEVERITY.INFO, "Loaded radio range simulator")

return update()

