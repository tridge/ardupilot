#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

// Only relocation harnesses may load containers without runtime identity.
#ifndef AP_MODULE_LEGACY_PROBE_ENABLED
#define AP_MODULE_LEGACY_PROBE_ENABLED 0
#endif
