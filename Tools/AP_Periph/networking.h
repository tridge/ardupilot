#pragma once

#ifdef HAL_PERIPH_ENABLE_NETWORKING

#define HAL_PERIPH_NETWORK_NUM_PASSTHRU 2

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
        AP_Int8 ep1;
        AP_Int8 ep2;
        AP_HAL::UARTDriver *port1;
        AP_HAL::UARTDriver *port2;
    } passthru[HAL_PERIPH_NETWORK_NUM_PASSTHRU];
};

#endif // HAL_PERIPH_ENABLE_BATTERY_BALANCE

