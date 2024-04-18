--[[
   script to facilitate testing of external AHRS with focus on dead-reckoning

   Two key functions:
   - ability to switch between EKF3, EKF2 and EAHRS on a 3 position switch
   - ability to disable use of GPS by EKF3 and EAHRS using a 2 position switch
--]]

local AHRS_EKF_TYPE = Parameter('AHRS_EKF_TYPE')
local EAHRS_OPTIONS = Parameter('EAHRS_OPTIONS')

local AHRS_SWITCH_FN = 300
local GPS_SWITCH_FN = 301

local last_ahrs_sw = -1
local last_gps_sw = -1

function update()

   local ahrs_sw_pos = rc:get_aux_cached(AHRS_SWITCH_FN)
   if ahrs_sw_pos ~= last_ahrs_sw then
      if ahrs_sw_pos == 0 then
         AHRS_EKF_TYPE:set(2)
      elseif ahrs_sw_pos == 1 then
         AHRS_EKF_TYPE:set(3)
      else
         AHRS_EKF_TYPE:set(11)
      end
      last_ahrs_sw = ahrs_sw_pos
   end

   local gps_sw_pos = rc:get_aux_cached(GPS_SWITCH_FN)
   if gps_sw_pos ~= last_gps_sw then
      if gps_sw_pos == 2 then
         gcs:send_text(0, "Disable GPS")
         ahrs:set_posvelyaw_source_set(1)
         EAHRS_OPTIONS:set(EAHRS_OPTIONS:get() | 4)
      else
         gcs:send_text(0, "Enable GPS")
         ahrs:set_posvelyaw_source_set(0)
         EAHRS_OPTIONS:set(EAHRS_OPTIONS:get() & ~4)
      end
      last_gps_sw = gps_sw_pos
   end

   return update, 100
end

gcs:send_text(0, "EAHRS Testing loaded")

return update()
