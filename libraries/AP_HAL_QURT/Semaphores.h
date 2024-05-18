#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <AP_HAL/Semaphores.h>

namespace QURT {

class Semaphore : public AP_HAL::Semaphore {
public:
    friend class BinarySemaphore;
    Semaphore();
    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;

protected:
    int dummy_lock;
};


class BinarySemaphore : public AP_HAL::BinarySemaphore {
public:
    BinarySemaphore(bool initial_state=false);

    bool wait(uint32_t timeout_us) override;
    bool wait_blocking(void) override;
    void signal(void) override;

protected:
    Semaphore mtx;
    bool pending;
};
    
}
