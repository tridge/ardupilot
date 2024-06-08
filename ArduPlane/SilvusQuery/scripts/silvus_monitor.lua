--[[
   monitor silvus radio link

   192.168.0.1: gnd silvus radio
   192.168.0.2: air silvus radio
--]]

local MAV_SEVERITY = {EMERGENCY=0, ALERT=1, CRITICAL=2, ERROR=3, WARNING=4, NOTICE=5, INFO=6, DEBUG=7}

PARAM_TABLE_KEY = 46
PARAM_TABLE_PREFIX = "SLV_"

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
    assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
    return Parameter(PARAM_TABLE_PREFIX .. name)
end

-- Setup Parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 30), 'could not add param table')

--[[
  // @Param: SLV_ENABLE
  // @DisplayName: enable Silvus monitor
  // @Description: Enable Silvus monitor
  // @Values: 0:Disabled,1:Enabled
  // @User: Standard
--]]
local SLV_ENABLE = bind_add_param('ENABLE',  1, 0)
if SLV_ENABLE:get() == 0 then
   return
end

local SLV_IP = { bind_add_param('IP0', 2, 192),
                 bind_add_param('IP1', 3, 168),
                 bind_add_param('IP2', 4, 0),
                 bind_add_param('IP3', 5, 2) }

--[[
  // @Param: SLV_RATE
  // @DisplayName: request rate
  // @Description: request rate
  // @Units: Hz
  // @User: Standard
--]]
local SLV_RATE = bind_add_param('RATE', 6, 2)

--[[
  // @Param: SLV_NODEID
  // @DisplayName: Silvus node ID
  // @Description: Silvus node ID for ranging. Zero means auto-allocated
  // @User: Standard
--]]
local SLV_NODEID = bind_add_param('NODEID', 7, 0)

--[[
  // @Param: SLV_DIST_OFS
  // @DisplayName: Silvus distance offset
  // @Description: Silvus distance offset. A value of 0 for tof_request is assumed to be less than or equal to this distance
  // @Units: m
  // @User: Standard
--]]
local SLV_DIST_OFS = bind_add_param('DIST_OFS', 8, 400)

--[[
  // @Param: SLV_DIST_ACC
  // @DisplayName: Silvus distance accuracy
  // @Description: Silvus distance accuracy
  // @Units: m
  // @User: Standard
--]]
local SLV_DIST_ACC = bind_add_param('DIST_ACC', 9, 30)

--[[
  // @Param: SLV_DIST_MUL
  // @DisplayName: Silvus distance multiplier
  // @Description: Silvus distance multiplier
  // @Units: m
  // @User: Standard
--]]
local SLV_DIST_MUL = bind_add_param('DIST_MUL', 10, 30)

--[[
  // @Param: SLV_GND_LAT
  // @DisplayName: Silvus ground radio latitude
  // @Description: Silvus ground radio latitude. Use home lat if all zero
  // @Units: deg
  // @User: Standard
--]]
local SLV_GND_LAT = bind_add_param('GND_LAT', 11, 0)

--[[
  // @Param: SLV_GND_LON
  // @DisplayName: Silvus ground radio longitude
  // @Description: Silvus ground radio longitude. Use home lat if all zero
  // @Units: deg
  // @User: Standard
--]]
local SLV_GND_LON = bind_add_param('GND_LON', 12, 0)

--[[
  // @Param: SLV_GND_ALT
  // @DisplayName: Silvus ground radio altitude AMSL
  // @Description: Silvus ground radio altitude AMSL
  // @Units: m
  // @User: Standard
--]]
local SLV_GND_ALT = bind_add_param('GND_ALT', 13, 0)

--[[
  // @Param: SLV_HTTP_PORT
  // @DisplayName: Silvus HTTP port
  // @Description: Silvus HTTP port
  // @Units: m
  // @User: Standard
--]]
local SLV_HTTP_PORT = bind_add_param('HTTP_PORT', 14, 80)

local radio_nodeid = SLV_NODEID:get()

gcs:send_text(MAV_SEVERITY.INFO, "Silvus: starting")

local function silvus_ip()
   return string.format("%u.%u.%u.%u", SLV_IP[1]:get(), SLV_IP[2]:get(), SLV_IP[3]:get(), SLV_IP[4]:get())
end

local function save_to_file(fname, data)
   local fh = io.open(fname,'wb')
   fh:write(data)
   fh:close()
end

local sock = nil
local http_reply = nil
local reply_start = nil
local REQUEST_TIMEOUT = 250
local last_request_ms = nil
local json = require("json")

--[[
   make a silvus API request
--]]
local function http_request(api)
   if sock then
      sock:close()
      sock = nil
   end
   sock = Socket(0)
   local node_ip = silvus_ip()
   if not sock:connect(node_ip, SLV_HTTP_PORT:get()) then
      gcs:send_text(MAV_SEVERITY.ERROR, string.format("Silvus: failed to connect", name))
      return nil
   end
   local json = string.format([[{"jsonrpc":"2.0","method":"%s","id":"sbkb5u0c"}]], api)
   local cmd = string.format([[POST /streamscape_api HTTP/1.1
Host: %s
User-Agent: lua
Connection: close
Content-Length: %u

]], node_ip, #json)
   cmd = string.gsub(cmd,"\n","\r\n")
   local full_cmd = cmd .. json
   --save_to_file("json_req.txt", full_cmd)
   sock:set_blocking(false)
   sock:send(cmd, #cmd)
   sock:send(json, #json)
   http_reply = ''
   reply_start = millis()
end

--[[
   get ground radio location
--]]
local function get_radio_location()
   local loc = ahrs:get_home()
   if not loc then
      gcs:send_text(0,"no home")
      return nil
   end
   local lat = SLV_GND_LAT:get()
   local lon = SLV_GND_LON:get()
   local alt = SLV_GND_ALT:get()
   if lat ~= 0 or lon ~= 0 then
      loc:lat(math.floor(lat*1.0e7))
      loc:lng(math.floor(lon*1.0e7))
      loc:alt(math.floor(alt*1.0e2))
   end
   return loc
end

--[[
   handle a distance measurement
--]]
local function handle_TOF(node_id, distance_ticks, age_ms)
   if radio_nodeid == 0 then
      radio_nodeid = node_id
   end
   if radio_nodeid ~= node_id then
      -- not for us
      return
   end
   local distance_m = 0
   if distance_ticks > 0 then
      distance_m = SLV_DIST_OFS:get() + distance_ticks * SLV_DIST_MUL:get()
   end
   gcs:send_named_float("SlvRange", distance_m)
   if distance_m > 0 then
      local accuracy = SLV_DIST_ACC:get()
      if accuracy < SLV_DIST_MUL:get() then
         accuracy = SLV_DIST_MUL:get()
      end
      local now_ms = millis()
      local radio_loc = get_radio_location()
      if not radio_loc then
         return
      end
      ahrs:writeRangeToLocation(distance_m, accuracy, radio_loc, now_ms - age_ms)
   end
end

--[[
   parse JSON reply from remote radio
--]]
local function parse_reply()
   sock:close()
   sock = nil
   lines = {}
   --save_to_file("json_rep.txt", http_reply)
   for s in http_reply:gmatch("[^\r\n]+") do
      table.insert(lines, s)
   end
   local success, req = pcall(json.parse, lines[#lines])
   if not success then
      return
   end
   local result = req['result']
   if not result then
      -- badly formatted
      return
   end
   local num_nodes = math.floor(#result / 3)
   for i = 1, num_nodes do
      local node_id = tonumber(result[1+(i-1)*3])
      local distance_ticks = tonumber(result[2+(i-1)*3])
      local age_ms = tonumber(result[3+(i-1)*3])
      --gcs:send_text(MAV_SEVERITY.INFO, string.format("node:%u dist:%u age_ms:%u", node_id, distance_ticks, age_ms))
      logger:write("STOF","Node,Dist,Age", "III", node_id, distance_ticks, age_ms)
      if age_ms < 500 then
         handle_TOF(node_id, distance_ticks, age_ms)
      end
   end
end

local function check_reply()
   if not sock then
      return
   end
   local now = millis()
   if reply_start and now - reply_start > REQUEST_TIMEOUT then
      parse_reply()
      return
   end
   local r = sock:recv(1024)
   if r then
      http_reply = http_reply .. r
   else
      parse_reply()
   end
end


local function update()
   if sock then
      check_reply()
      return
   end
   local now = millis()
   local period_ms = 1000.0 / SLV_RATE:get()
   if not last_request_ms or now - last_request_ms >= period_ms then
      last_request_ms = now
      http_request("current_tof")
   end
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
