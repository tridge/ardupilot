/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.
   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */
/*
  suppport for serial connected AHRS systems
 */

#pragma once

#include "AP_ExternalAHRS_backend.h"

#if HAL_EXTERNAL_AHRS_ENABLED

#include <GCS_MAVLink/GCS_MAVLink.h>

class AP_ExternalAHRS_LORD: public AP_ExternalAHRS_backend {
public:
    AP_ExternalAHRS_LORD(AP_ExternalAHRS *frontend, AP_ExternalAHRS::state_t &state);

    // get serial port number, -1 for not enabled
    int8_t get_port(void) const override;

    // accessors for AP_AHRS
    bool healthy(void) const override;
    bool initialised(void) const override;
    bool pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const override;
    void get_filter_status(nav_filter_status &status) const override;
    void send_status_report(mavlink_channel_t chan) const override;

    // check for new data
    void update() override {

    };

private:
    typedef struct {
        Vector3f accel;
        Vector3f gyro;
    } LORDpacketData_t;

    AP_HAL::UARTDriver *uart;
    int8_t port_num;
    uint32_t baudrate;

    LORDpacketData_t processLORDPacket(const uint8_t*);
    LORDpacketData_t insData(const uint8_t*);
    Vector3f populateVector3f(const uint8_t*,uint8_t);
    uint64_t get8ByteField(const uint8_t*,uint8_t);
    uint32_t get4ByteField(const uint8_t*,uint8_t);
    uint16_t get2ByteField(const uint8_t*,uint8_t);

};


#endif // HAL_EXTERNAL_AHRS_ENABLED