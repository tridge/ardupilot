#include <AP_HAL/AP_HAL.h>

#include "Semaphores.h"

extern const AP_HAL::HAL& hal;

using namespace QURT;

// construct a semaphore
Semaphore::Semaphore()
{
    qurt_rmutex_init(&_lock);
}

bool Semaphore::give()
{
    qurt_rmutex_unlock(&_lock);
    return true;
}

bool Semaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        qurt_rmutex_lock(&_lock);
        return true;
    }
    return qurt_rmutex_lock_timed(&_lock, timeout_ms*1000UL);
}

bool Semaphore::take_nonblocking()
{
    return qurt_rmutex_try_lock(&_lock) == 0;
}

/*
  binary semaphore using condition variables
 */

BinarySemaphore::BinarySemaphore(bool initial_state) :
    AP_HAL::BinarySemaphore(initial_state)
{
    qurt_cond_init(&cond);
    pending = initial_state;
}

bool BinarySemaphore::wait(uint32_t timeout_us)
{
    /*
      not available!
     */
    AP_HAL::panic("no timed BinarySemaphore");
    return true;
}

bool BinarySemaphore::wait_blocking(void)
{
    WITH_SEMAPHORE(mtx);
    if (!pending) {
        qurt_cond_wait(&cond, &mtx._lock);
    }
    pending = false;
    return true;
}

void BinarySemaphore::signal(void)
{
    WITH_SEMAPHORE(mtx);
    if (!pending) {
        pending = true;
        qurt_cond_signal(&cond);
    }
}
