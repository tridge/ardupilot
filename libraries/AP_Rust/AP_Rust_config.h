#pragma once

#include <AP_HAL/AP_HAL_Boards.h>

#ifndef AP_SCRIPTING_RUST_ENABLED
#define AP_SCRIPTING_RUST_ENABLED 0
#endif

// Only relocation harnesses may load containers without runtime identity.
#ifndef AP_RUST_LEGACY_PROBE_ENABLED
#define AP_RUST_LEGACY_PROBE_ENABLED 0
#endif
