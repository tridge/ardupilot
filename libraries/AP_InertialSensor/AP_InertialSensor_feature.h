#pragma once

#include <AP_Vehicle/AP_Vehicle_Type.h>
#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_INERTIALSENSOR_RATE_LOOP_WINDOW_ENABLED
#define AP_INERTIALSENSOR_RATE_LOOP_WINDOW_ENABLED APM_BUILD_TYPE(APM_BUILD_ArduCopter)
#endif
