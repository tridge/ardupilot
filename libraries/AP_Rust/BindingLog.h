// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once
#include "AP_Rust_config.h"
#include <stdint.h>

#if AP_SCRIPTING_RUST_ENABLED
namespace AP_Rust
{
int32_t binding_log_write(const uint8_t *input, uint32_t input_size,
                          uint8_t *output, uint32_t output_size);
}
#endif
