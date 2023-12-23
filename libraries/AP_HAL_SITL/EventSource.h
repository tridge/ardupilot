#pragma once

#include <AP_HAL/EventHandle.h>
#include <pthread.h>

class HALSITL::EventSource : public AP_HAL::EventSource {
public:
    EventSource();
    ~EventSource() override;

    // generate event from thread context
    void signal(uint32_t evt_mask) override;

    // Wait on an Event handle, method for internal use by EventHandle
    bool wait(uint16_t duration_us, AP_HAL::EventHandle* evt_handle) override;

private:
    pthread_condattr_t condattr;
    pthread_cond_t cond;
    pthread_mutex_t mtx;
    volatile uint32_t events;
};
