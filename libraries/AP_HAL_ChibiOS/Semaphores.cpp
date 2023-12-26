/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 *
 * Code by Andrew Tridgell and Siddharth Bharat Purohit
 */

#include <AP_HAL/AP_HAL.h>
#include "Semaphores.h"
#include <hal.h>
#include <chsem.h>
#include "AP_HAL_ChibiOS.h"
#include <AP_Math/AP_Math.h>

#if CH_CFG_USE_MUTEXES == TRUE

extern const AP_HAL::HAL& hal;

using namespace ChibiOS;

// constructor
Semaphore::Semaphore()
{
    static_assert(sizeof(_lock) >= sizeof(mutex_t), "invalid mutex size");
    mutex_t *mtx = (mutex_t *)_lock;
    chMtxObjectInit(mtx);
}

bool Semaphore::give()
{
    mutex_t *mtx = (mutex_t *)_lock;
    chMtxUnlock(mtx);
    return true;
}

bool Semaphore::take(uint32_t timeout_ms)
{
    mutex_t *mtx = (mutex_t *)_lock;
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        chMtxLock(mtx);
        return true;
    }
    if (take_nonblocking()) {
        return true;
    }
    uint64_t start = AP_HAL::micros64();
    do {
        hal.scheduler->delay_microseconds(200);
        if (take_nonblocking()) {
            return true;
        }
    } while ((AP_HAL::micros64() - start) < timeout_ms*1000);
    return false;
}

bool Semaphore::take_nonblocking()
{
    mutex_t *mtx = (mutex_t *)_lock;
    return chMtxTryLock(mtx);
}

bool Semaphore::check_owner(void)
{
    mutex_t *mtx = (mutex_t *)_lock;
    return mtx->owner == chThdGetSelfX();
}

void Semaphore::assert_owner(void)
{
    osalDbgAssert(check_owner(), "owner");
}

#if CH_CFG_USE_SEMAPHORES == TRUE
CountingSemaphore::CountingSemaphore(uint8_t initial_count) :
    AP_HAL::CountingSemaphore(initial_count)
{
    static_assert(sizeof(_lock) >= sizeof(semaphore_t), "invalid semaphore_t size");
    semaphore_t *sem = (semaphore_t *)_lock;
    chSemObjectInit(sem, initial_count);
}

bool CountingSemaphore::wait(uint32_t timeout_us)
{
    semaphore_t *sem = (semaphore_t *)_lock;
    if (timeout_us == 0) {
        return chSemWaitTimeout(sem, TIME_IMMEDIATE) == MSG_OK;
    }
    // loop waiting for 60ms at a time. This ensures we can wait for
    // any amount of time even on systems with a 16 bit timer
    while (timeout_us > 0) {
        const uint32_t us = MIN(timeout_us, 60000U);
        if (chSemWaitTimeout(sem, TIME_US2I(us)) == MSG_OK) {
            return true;
        }
        timeout_us -= us;
    }
    return false;
}

bool CountingSemaphore::wait_blocking(void)
{
    semaphore_t *sem = (semaphore_t *)_lock;
    return chSemWait(sem) == MSG_OK;
}

void CountingSemaphore::signal(void)
{
    semaphore_t *sem = (semaphore_t *)_lock;
    chSemSignal(sem);
}

void CountingSemaphore::signal_ISR(void)
{
    semaphore_t *sem = (semaphore_t *)_lock;
    chSysLockFromISR();
    chSemSignalI(sem);
    chSysUnlockFromISR();
}

uint8_t CountingSemaphore::get_count(void)
{
    semaphore_t *sem = (semaphore_t *)_lock;
    chSysLock();
    uint8_t ret = chSemGetCounterI(sem);
    chSysUnlock();
    return ret;
}
#endif // CH_CFG_USE_SEMAPHORES == TRUE

#endif // CH_CFG_USE_MUTEXES
