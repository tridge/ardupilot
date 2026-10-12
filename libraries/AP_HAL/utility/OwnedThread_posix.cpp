// SPDX-License-Identifier: GPL-3.0-or-later
#include "OwnedThread_posix.h"

#if HAL_OWNED_THREADS_ENABLED && (CONFIG_HAL_BOARD == HAL_BOARD_SITL || CONFIG_HAL_BOARD == HAL_BOARD_LINUX)
#include <sys/mman.h>
#include <unistd.h>

OwnedThreadPosix::~OwnedThreadPosix()
{
    if (_mapping != nullptr) {
        munmap(_mapping, _mapping_size);
    }
}

AP_HAL::Scheduler::OwnedThread *OwnedThreadPosix::create(AP_HAL::MemberProc proc, const char *name,
        uint32_t stack_size, int policy, int priority)
{
    const long page = sysconf(_SC_PAGESIZE);
    // Bound arithmetic and reject nonsensical requests before allocating.
    if (!proc || stack_size == 0 || stack_size > 16U * 1024 * 1024 || page <= 0 || page > 65536) {
        return nullptr;
    }
    if (stack_size < size_t(PTHREAD_STACK_MIN)) {
        stack_size = PTHREAD_STACK_MIN;
    }
    stack_size = ((stack_size + page - 1) / page) * page;
    auto *thread = NEW_NOTHROW OwnedThreadPosix(proc);
    if (thread == nullptr) {
        return nullptr;
    }
    thread->_stack_size = stack_size;
    thread->_mapping_size = stack_size + 2 * page;
    thread->_mapping = mmap(nullptr, thread->_mapping_size, PROT_NONE, MAP_PRIVATE | MAP_ANONYMOUS, -1, 0);
    if (thread->_mapping == MAP_FAILED) {
        thread->_mapping = nullptr;
        delete thread;
        return nullptr;
    }
    void *stack = static_cast<uint8_t *>(thread->_mapping) + page;
    if (mprotect(stack, stack_size, PROT_READ | PROT_WRITE) != 0) {
        delete thread;
        return nullptr;
    }
    pthread_attr_t attr;
    if (pthread_attr_init(&attr) != 0) {
        delete thread;
        return nullptr;
    }
    const sched_param param{priority};
    // Explicit scheduling prevents inheriting the creator's realtime priority.
    const bool attributes_ok = pthread_attr_setstack(&attr, stack, stack_size) == 0 &&
                               pthread_attr_setguardsize(&attr, 0) == 0 &&
                               pthread_attr_setinheritsched(&attr, PTHREAD_EXPLICIT_SCHED) == 0 &&
                               pthread_attr_setschedpolicy(&attr, policy) == 0 &&
                               pthread_attr_setschedparam(&attr, &param) == 0;
    const int result = attributes_ok ? pthread_create(&thread->_thread, &attr, trampoline, thread) : -1;
    pthread_attr_destroy(&attr);
    if (result != 0) {
        delete thread;
        return nullptr;
    }
#if !defined(__APPLE__) && !defined(__OpenBSD__) && !defined(__EMSCRIPTEN__)
    if (name != nullptr) {
        pthread_setname_np(thread->_thread, name);
    }
#endif
    return thread;
}

void *OwnedThreadPosix::trampoline(void *context)
{
    auto *thread = static_cast<OwnedThreadPosix *>(context);
    thread->_proc();
    thread->_finished.store(true, std::memory_order_release);
    return nullptr;
}

bool OwnedThreadPosix::join_if_finished()
{
    if (!_finished.load(std::memory_order_acquire) || pthread_equal(pthread_self(), _thread)) {
        return false;
    }
    return pthread_join(_thread, nullptr) == 0;
}
#endif
