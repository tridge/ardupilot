#pragma once

#ifdef HAL_PERIPH_ENABLE_NETWORKING

class Networking_Periph {
public:
    friend class AP_Periph_FW;
    friend class AP_Netwokring;
    Networking_Periph(void);

    static const struct AP_Param::GroupInfo var_info[];

    // update uart pass-thru
    void update_passthru();

private:
    AP_Networking networking;

    struct {
        bool enabled;
        bool timer_installed;
        AP_HAL::UARTDriver *port1;
        AP_HAL::UARTDriver *port2;
        uint32_t start_ms;
        uint32_t last_ms;
        uint32_t last_port1_data_ms;
        uint32_t baud1;
        uint32_t baud2;
        uint8_t timeout_s;
        HAL_Semaphore sem;
    } _passthru;

    // timer called to implement pass-thru
    void passthru_timer();

};

#endif // HAL_PERIPH_ENABLE_BATTERY_BALANCE

