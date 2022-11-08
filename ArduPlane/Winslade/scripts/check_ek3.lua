local PARAM_TABLE_KEY = 37
local PARAM_TABLE_PREFIX = "TNAV_"

local UPDATE_RATE_HZ = 10

-- bind a parameter to a variable
function bind_param(name)
   local p = Parameter()
   assert(p:init(name), string.format('could not find %s parameter', name))
   return p
end

-- add a parameter and bind it to a variable
function bind_add_param(name, idx, default_value)
   assert(param:add_param(PARAM_TABLE_KEY, idx, name, default_value), string.format('could not add param %s', name))
   return bind_param(PARAM_TABLE_PREFIX .. name)
end

-- setup quicktune specific parameters
assert(param:add_table(PARAM_TABLE_KEY, PARAM_TABLE_PREFIX, 2), 'could not add param table')

local TNAV_MAX_DIST    = bind_add_param('MAX_DIST',       1, 150)
local TNAV_MAX_HB_MS   = bind_add_param('MAX_HB_MS',      2, 4000)
local AHRS_EKF_TYPE    = bind_param('AHRS_EKF_TYPE')

-- main update function
function update()
   if AHRS_EKF_TYPE:get() ~= 3 then
      return
   end
   local gps_loc = gps:location(0)
   if gps_loc ~= nil then
      local ahrs_loc = ahrs:get_location()
      local dist = gps_loc:get_distance(ahrs_loc)
      if dist > TNAV_MAX_DIST:get() then
         AHRS_EKF_TYPE:set_and_save(2)
         rc:run_aux_function(65, 0)
         gcs:send_text(0,string.format("Forcing EKF2 dist=%.1f", dist))
      end
   end
   if millis() - gcs:sysid_myggcs_last_seen_time_ms() > TNAV_MAX_HB_MS:get() then
      AHRS_EKF_TYPE:set_and_save(2)
      rc:run_aux_function(65, 0)
      gcs:send_text(0,string.format("Forcing EKF2 GCS"))
   end
end

-- wrapper around update(). This calls update() at 10Hz,
-- and if update faults then an error is displayed, but the script is not
-- stopped
function protected_wrapper()
  local success, err = pcall(update)
  if not success then
     gcs:send_text(0, "Internal Error: " .. err)
     -- when we fault we run the update function again after 1s, slowing it
     -- down a bit so we don't flood the console with errors
     --return protected_wrapper, 1000
     return
  end
  return protected_wrapper, 1000/UPDATE_RATE_HZ
end

-- start running update loop
return protected_wrapper()
