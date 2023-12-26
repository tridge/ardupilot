#include <AP_HAL/AP_HAL.h>

#include "Semaphores.h"

extern const AP_HAL::HAL& hal;

using namespace Linux;

// construct a semaphore
Semaphore::Semaphore()
{
    pthread_mutexattr_t attr;
    pthread_mutexattr_init(&attr);
    pthread_mutexattr_setprotocol(&attr, PTHREAD_PRIO_INHERIT);
    pthread_mutexattr_settype(&attr, PTHREAD_MUTEX_RECURSIVE);
    pthread_mutex_init(&_lock, &attr);
}

bool Semaphore::give()
{
    return pthread_mutex_unlock(&_lock) == 0;
}

bool Semaphore::take(uint32_t timeout_ms)
{
    if (timeout_ms == HAL_SEMAPHORE_BLOCK_FOREVER) {
        return pthread_mutex_lock(&_lock) == 0;
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
    return pthread_mutex_trylock(&_lock) == 0;
}

/*
  counting semaphores
 */
CountingSemaphore::CountingSemaphore(uint8_t initial_count) :
    AP_HAL::CountingSemaphore(initial_count)
{
    sem_init(&_sem, 0, initial_count);
}

bool CountingSemaphore::wait(uint32_t timeout_us)
{
    if (timeout_us == 0) {
        return sem_trywait(&_sem) == 0;
    }
    struct timespec ts;
    if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
        return false;
    }
    ts.tv_sec += timeout_us/1000000UL;
    ts.tv_nsec += (timeout_us % 1000000U) * 1000UL;
    if (ts.tv_nsec >= 1000000000L) {
        ts.tv_sec++;
        ts.tv_nsec -= 1000000000L;
    }
    return sem_timedwait(&_sem, &ts) == 0;
}

bool CountingSemaphore::wait_blocking(void)
{
    return sem_wait(&_sem) == 0;
}

void CountingSemaphore::signal(void)
{
    sem_post(&_sem);
}

uint8_t CountingSemaphore::get_count(void)
{
    int v = 0;
    sem_getvalue(&_sem, &v);
    if (v < 0) {
        return 0;
    }
    if (v > 255) {
        return 255;
    }
    return v;
}
