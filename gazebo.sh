#!/bin/bash

# /usr/share/gazebo-9
# All of the stuff the ardupilot_gazebo plugin uses is in the above directory
#
# param set DENY_ARM 0
# mode guided
# arm throttle
# takeoff 5

# AHRS_GPS_USE 0
# BATT_FS_LOW_ACT 0
# FS_THR_ENABLE 0
# FENCE_ENABLE 0
#
# param set DENY_ARM 0
# param set throw_type 1 (drop)
# param set throw_nextmode 4 (guided)
# mode throw
# arm throttle
#
#
#
gazebo --verbose worlds/iris_arducopter_runway.world
