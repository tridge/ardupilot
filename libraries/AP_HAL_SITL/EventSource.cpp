#include "EventSource.h"
#include <sys/time.h>

using namespace HALSITL;

EventSource::EventSource()
{
    pthread_condattr_init(&condattr);
    pthread_condattr_setclock(&condattr, CLOCK_REALTIME);
    pthread_cond_init(&cond, &condattr);
    pthread_mutex_init(&mtx, nullptr);
}

EventSource::~EventSource()
{
    pthread_cond_destroy(&cond);
    pthread_condattr_destroy(&condattr);
}

bool EventSource::wait(uint16_t duration_us, AP_HAL::EventHandle *evt_handle)
{
    pthread_mutex_lock(&mtx);
    while (true) {
        const uint32_t evts = events & evt_handle->get_evt_mask();
        if (evts != 0) {
            events &= ~evts;
            pthread_mutex_unlock(&mtx);
            return true;
        }
        int ret;
        if (duration_us != 0) {
            struct timespec ts;
            if (clock_gettime(CLOCK_REALTIME, &ts) != 0) {
                return false;
            }
            ts.tv_sec += duration_us/1000000UL;
            ts.tv_nsec += ((duration_us % 1000000UL))*1000ULL;
            if (ts.tv_nsec >= 1000000000L) {
                ts.tv_sec++;
                ts.tv_nsec -= 1000000000L;
            }
            ret = pthread_cond_timedwait(&cond, &mtx, &ts);
        } else {
            ret = pthread_cond_wait(&cond, &mtx);
        }
        if (ret != 0) {
            const uint32_t evts2 = events & evt_handle->get_evt_mask();
            if (evts2 != 0) {
                events &= ~evts2;
                pthread_mutex_unlock(&mtx);
                return true;
            }
            break;
        }
    }
    // timed out
    pthread_mutex_unlock(&mtx);
    return false;
}

void EventSource::signal(uint32_t evt_mask)
{
    pthread_mutex_lock(&mtx);
    events |= evt_mask;
    pthread_cond_broadcast(&cond);
    pthread_mutex_unlock(&mtx);
}
