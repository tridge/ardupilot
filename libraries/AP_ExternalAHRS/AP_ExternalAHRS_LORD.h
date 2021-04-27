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

    void update_thread();

    AP_HAL::UARTDriver *uart;
    uint32_t baudrate;

    struct LORD_Packet {
        uint8_t header[4];
        uint8_t payload[512];
        uint8_t checksum[2];
    };

    //shared ring buffer
    static const uint32_t bufferSize = 1024;
    ByteBuffer buffer{bufferSize};
    uint8_t tempData[bufferSize];

    //packet building state variables
    struct LORD_Packet currPacket;
    enum SearchPhase { sync, payloadSize, payloadAndChecksum };
    SearchPhase currPhase = sync;
    int searchBytes = 1;
    //sync bytes phase
    const uint8_t syncByte1 = 0x75;
    const uint8_t syncByte2 = 0x65;
    uint8_t nextSyncByte = syncByte1;

    //variables for final data to be output
    bool portOpened = false;
    bool packetReady = false;
    Vector3f accelNew;
    Vector3f gyroNew;
    Vector3f magNew;
    float pressureNew;
    Quaternion quatNew;
    uint16_t GPSweek;
    double GPSTOW;
    

    void readIMU();
    void buildPacket();
    bool validPacket();

    void parsePacket();
    void parseIMU();
    void parseGNSS();
    void parseEFD();
    Vector3f populateVector3f(const uint8_t*,uint8_t,float);
    Quaternion populateQuaternion(const uint8_t*,uint8_t);
    uint64_t get8ByteField(const uint8_t*,uint8_t);
    uint32_t get4ByteField(const uint8_t*,uint8_t);
    uint16_t get2ByteField(const uint8_t*,uint8_t);
};


#endif // HAL_EXTERNAL_AHRS_ENABLED