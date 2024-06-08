--[[
   simulate a silvus radio providing time of flight range
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

PARAM_TABLE_KEY = 53
PARAM_TABLE_PREFIX = "SSIM_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 20), 'could not add param table')

--[[
  // @Param: SSIM_ENABLE
  // @DisplayName: enable Silvus simulator
  // @Description: Enable Silvus simulator
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SSIM_ENABLE = bind_add_param('ENABLE',  1, 0)
if SSIM_ENABLE:get() == 0 then
   return
end

--[[
  // @Param: SSIM_NODEID1
  // @DisplayName: Silvus node ID1
  // @Description: Silvus node ID1
  // @User: Standard
--]]
local SSIM_NODEID1 = bind_add_param('NODEID1', 2, 40001)

--[[
  // @Param: SSIM_NODEID2
  // @DisplayName: Silvus node ID2
  // @Description: Silvus node ID2
  // @User: Standard
--]]
local SSIM_NODEID2 = bind_add_param('NODEID2', 3, 40002)

--[[
  // @Param: SSIM_NODE1_OFS_N
  // @DisplayName: silvus node1 offset N
  // @Description: silvus node1 offset N from home
  // @Units: m
  // @User: Standard
--]]
local SSIM_NODE1_OFS_N = bind_add_param('NODE1_OFS_N', 4, 0)

--[[
  // @Param: SSIM_NODE1_OFS_E
  // @DisplayName: silvus node1 offset E
  // @Description: silvus node1 offset E from home
  // @Units: m
  // @User: Standard
--]]
local SSIM_NODE1_OFS_E = bind_add_param('NODE1_OFS_E', 5, 0)

--[[
  // @Param: SSIM_NODE2_OFS_N
  // @DisplayName: silvus node2 offset N
  // @Description: silvus node2 offset N from home
  // @Units: m
  // @User: Standard
--]]
local SSIM_NODE2_OFS_N = bind_add_param('NODE2_OFS_N', 6, 500)

--[[
  // @Param: SSIM_NODE2_OFS_E
  // @DisplayName: silvus node2 offset E
  // @Description: silvus node2 offset E from home
  // @Units: m
  // @User: Standard
--]]
local SSIM_NODE2_OFS_E = bind_add_param('NODE2_OFS_E', 7, 200)

--[[
  // @Param: SSIM_DIST_OFS
  // @DisplayName: Silvus distance offset
  // @Description: Silvus distance offset
  // @Units: m
  // @User: Standard
--]]
local SSIM_DIST_OFS = bind_add_param('DIST_OFS', 8, 400)

--[[
  // @Param: SSIM_DIST_MUL
  // @DisplayName: Silvus distance multiplier
  // @Description: Silvus distance multiplier
  // @Units: m
  // @User: Standard
--]]
local SSIM_DIST_MUL = bind_add_param('DIST_MUL', 9, 30)

--[[
  // @Param: SSIM_LISTEN_PORT
  // @DisplayName: Silvus listen port
  // @Description: Silvus listen port
  // @User: Standard
--]]
local SSIM_LISTEN_PORT = bind_add_param('LISTEN_PORT', 10, 8003)

local listen_sock = nil

gcs:send_text(MAV_SEVERITY.INFO, "SilvusSim: starting")

--[[
   get range in TOF microseconds, given offset of beacon from home
--]]
local function get_range(ofs_N, ofs_E)
   local gps_loc = gps:location(0)
   if not gps_loc then
      return nil
   end
   local loc = ahrs:get_home()
   if not loc then
      return nil
   end
   loc:offset(ofs_N, ofs_E)
   local range_2D_m = gps_loc:get_distance(loc)
   local alt_diff = math.abs(loc:alt()*0.01 - gps_loc:alt()*0.01)
   local range_3D_m = math.sqrt(range_2D_m^2 + alt_diff^2)
   if range_3D_m < SSIM_DIST_OFS:get() then
      return 0
   end
   local range_us = math.floor((range_3D_m - SSIM_DIST_OFS:get()) / SSIM_DIST_MUL:get())
   return range_us
end

--[[
   check for new connections
--]]
local function update()
   if SSIM_ENABLE:get() <= 0 then
      return
   end
   if not listen_sock then
      listen_sock = Socket(0)
      if not listen_sock then
         return
      end
      assert(listen_sock:bind("0.0.0.0", SSIM_LISTEN_PORT:get()))
      assert(listen_sock:listen(2))
      listen_sock:reuseaddress()
   end
   local sock = listen_sock:accept()
   if not sock then
      return
   end
   local range_str = [[""]]
   local range1 = get_range(SSIM_NODE1_OFS_N:get(), SSIM_NODE1_OFS_E:get())
   local range2 = get_range(SSIM_NODE2_OFS_N:get(), SSIM_NODE2_OFS_E:get())
   if range1 and range2 then
      range_str = string.format([["%u","%u","10","%u","%u","10"]], SSIM_NODEID1:get(), range1, SSIM_NODEID2:get(), range2)
   end
   local json = string.format([[{"result" : [%s],"id" : "sbkb5u0c", "jsonrpc" : "2.0"}]], range_str)
   local msg = string.format([[Content-Type: application/json
Cache-Control: no-cache
Content-Length: %u
Server: silvus sim

%s]], #json, json)
   msg = string.gsub(msg, "\n", "\r\n")
   sock:send(msg, #msg)
   sock:close()
end

-- wrapper around update(). This calls update() at 20Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     return protected_wrapper, 1000
  end
  return protected_wrapper, 50
end

return protected_wrapper,100
