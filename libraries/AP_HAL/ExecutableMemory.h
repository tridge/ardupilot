// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include <AP_Common/AP_Common.h>
#include "AP_HAL_Boards.h"

#if HAL_EXECUTABLE_MEMORY_ENABLED
namespace AP_HAL
{
// Owned executable-capable storage. The caller must join every user of the
// image before deletion. This interface does not isolate untrusted native code.
class ExecutableMemory
{
public:
    enum class Target : uint8_t {
        Unsupported,
        ThumbV7EMHardFloat,
        X86_64Baseline,
    };

    CLASS_NO_COPY(ExecutableMemory);
    virtual ~ExecutableMemory() = default;
    virtual void *data() const = 0;
    virtual uint32_t size() const = 0;
    // Known reserved bytes, including this handle and backend rounding.
    // Excludes opaque OS and general-purpose allocator bookkeeping.
    virtual uint32_t allocation_size() const = 0;

    // Publish once after loading/relocation, before executing. code_size is
    // the page-aligned RX prefix; the rest is mutable data. On STM32 this
    // performs cache maintenance, not per-image MPU permission enforcement.
    // Never modify the code or call it after a failed publication.
    virtual bool publish(uint32_t code_size) = 0;

protected:
    ExecutableMemory() = default;
};
}
#endif
