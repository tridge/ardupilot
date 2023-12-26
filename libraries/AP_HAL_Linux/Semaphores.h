#pragma once

#include <AP_HAL/AP_HAL_Boards.h>
#include <stdint.h>
#include <AP_HAL/AP_HAL_Macros.h>
#include <AP_HAL/Semaphores.h>
#include <pthread.h>
#include <semaphore.h>

namespace Linux {

class Semaphore : public AP_HAL::Semaphore {
public:
    Semaphore();
    bool give() override;
    bool take(uint32_t timeout_ms) override;
    bool take_nonblocking() override;
protected:
    pthread_mutex_t _lock;
};


class CountingSemaphore : public AP_HAL::CountingSemaphore {
public:
    CountingSemaphore(uint8_t initial_count=0);

    bool wait(uint32_t timeout_ms) override;
    bool wait_blocking(void) override;
    void signal(void) override;
    uint8_t get_count(void) override;

protected:
    sem_t _sem;
};
    
}
