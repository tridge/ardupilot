// SPDX-License-Identifier: GPL-3.0-or-later
#include "Util.h"

#if HAL_EXECUTABLE_MEMORY_ENABLED
#include <AP_HAL/utility/ExecutableMemory_posix.h>

AP_HAL::ExecutableMemory::Target HALSITL::Util::executable_memory_target() const
{
#if defined(__x86_64__) && !defined(__ILP32__)
    return AP_HAL::ExecutableMemory::Target::X86_64Baseline;
#else
    return AP_HAL::ExecutableMemory::Target::Unsupported;
#endif
}

AP_HAL::ExecutableMemory *HALSITL::Util::allocate_executable(uint32_t size)
{
    return ExecutableMemoryPosix::create(size);
}
#endif
