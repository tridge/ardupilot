// SPDX-License-Identifier: GPL-3.0-or-later
#include "Heap.h"

#if AP_SCRIPTING_RUST_ENABLED

namespace AP_Rust
{

Heap::~Heap()
{
    clear();
    _heap.destroy();
}

bool Heap::create(uint32_t size)
{
    WITH_SEMAPHORE(_sem);
    return size >= 256 && size <= 16U * 1024 * 1024 && _heap.create(size, 8, false, 0);
}

void *Heap::allocate(uint32_t size, uint32_t alignment)
{
    if (size == 0 || alignment == 0 || alignment > 4096 || (alignment & (alignment - 1)) != 0) {
        return nullptr;
    }
    const uint32_t overhead = sizeof(Allocation) + alignment - 1;
    if (size > UINT32_MAX - overhead) {
        return nullptr;
    }
    const uint32_t charge = size + overhead;
    WITH_SEMAPHORE(_sem);
    auto *allocation = static_cast<Allocation *>(_heap.allocate(charge));
    if (allocation == nullptr) {
        return nullptr;
    }
    const uintptr_t start = reinterpret_cast<uintptr_t>(allocation + 1);
    allocation->pointer = reinterpret_cast<void *>((start + alignment - 1) & ~uintptr_t(alignment - 1));
    allocation->charge = charge;
    allocation->next = _allocations;
    _allocations = allocation;
    _stats.used += charge;
    if (_stats.used > _stats.peak) {
        _stats.peak = _stats.used;
    }
    _stats.allocations++;
    return allocation->pointer;
}

bool Heap::deallocate(void *pointer)
{
    if (pointer == nullptr) {
        return true;
    }
    WITH_SEMAPHORE(_sem);
    for (Allocation **link = &_allocations; *link != nullptr; link = &(*link)->next) {
        Allocation *allocation = *link;
        if (allocation->pointer == pointer) {
            *link = allocation->next;
            _stats.used -= allocation->charge;
            _stats.allocations--;
            _heap.deallocate(allocation);
            return true;
        }
    }
    return false;
}

void Heap::clear()
{
    WITH_SEMAPHORE(_sem);
    while (_allocations != nullptr) {
        Allocation *allocation = _allocations;
        _allocations = allocation->next;
        _heap.deallocate(allocation);
    }
    _stats.used = 0;
    _stats.allocations = 0;
}

Heap::Stats Heap::stats()
{
    WITH_SEMAPHORE(_sem);
    return _stats;
}

}
#endif // AP_SCRIPTING_RUST_ENABLED
