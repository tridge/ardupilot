#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include "AP_HAL_SITL_Namespace.h"
#include <pthread.h>

class HALSITL::Semaphore : public AP_HAL::Semaphore {
public:
    friend class HALSITL::CountingSemaphore;
    Semaphore();
    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;

    void check_owner() const;  // asserts that current thread owns semaphore

protected:
    pthread_mutex_t _lock;
    pthread_t owner;

    // keep track the recursion level to ensure we only disown the
    // semaphore once we're done with it
    uint8_t take_count;
};


#define AP_COUNTSEM_NATIVE_SEM_T !defined(__APPLE__)

#if AP_COUNTSEM_NATIVE_SEM_T
#include <semaphore.h>
#endif

class HALSITL::CountingSemaphore : public AP_HAL::CountingSemaphore {
public:
    CountingSemaphore(uint8_t initial_count=0);

    bool wait(uint32_t timeout_us) override;
    bool wait_blocking(void) override;
    void signal(void) override;
    uint8_t get_count(void) override;

private:
#if AP_COUNTSEM_NATIVE_SEM_T
    sem_t _sem;
#else
    HALSITL::Semaphore mtx;
    pthread_cond_t cond;
    uint8_t count;
#endif
};
