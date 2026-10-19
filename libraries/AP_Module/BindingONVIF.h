// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Module_config.h"

#if AP_MODULE_ENABLED
#include <stdint.h>

namespace AP_Module
{
int32_t binding_onvif_start(void *context, const uint8_t *input, uint32_t input_size,
                            uint8_t *output, uint32_t output_size);

// Called by the manager only after the module worker has joined. The firmware
// ONVIF singleton has no teardown API, so only its Rust ownership is released.
void binding_onvif_cleanup(void *context);
}
#endif
