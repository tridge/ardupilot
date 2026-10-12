// SPDX-License-Identifier: GPL-3.0-or-later
#pragma once

#include "AP_Rust_config.h"

#if AP_SCRIPTING_RUST_ENABLED
#include <AP_HAL/Semaphores.h>
#include <AP_MultiHeap/AP_MultiHeap.h>

namespace AP_Rust
{

// One module owns each heap. The owner must join its thread before destruction.
class Heap
{
public:
    Heap() = default;
    CLASS_NO_COPY(Heap);
    ~Heap();

    bool create(uint32_t size);
    void *allocate(uint32_t size, uint32_t alignment);
    bool deallocate(void *pointer);
    void clear();

    struct Stats {
        uint32_t used;
        uint32_t peak;
        uint32_t allocations;
    };
    Stats stats();

private:
    struct Allocation {
        Allocation *next;
        void *pointer;
        uint32_t charge;
    };

    MultiHeap _heap{};
    HAL_Semaphore _sem;
    Allocation *_allocations{};
    Stats _stats{};
};

}
#endif // AP_SCRIPTING_RUST_ENABLED
