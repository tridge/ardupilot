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

#define ALLOW_DOUBLE_MATH_FUNCTIONS

#include "AP_ExternalAHRS_LORD.h"
#include <AP_Math/AP_Math.h>
#include <AP_Math/crc.h>
#include <AP_Baro/AP_Baro.h>
#include <AP_Compass/AP_Compass.h>
#include <AP_GPS/AP_GPS.h>
#include <AP_InertialSensor/AP_InertialSensor.h>
#include <GCS_MAVLink/GCS.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/NMEA.h>
#include <stdio.h>

#if HAL_EXTERNAL_AHRS_ENABLED

extern const AP_HAL::HAL &hal;

AP_ExternalAHRS_LORD::AP_ExternalAHRS_LORD(AP_ExternalAHRS *_frontend,
                                                     AP_ExternalAHRS::state_t &_state) :
    AP_ExternalAHRS_backend(_frontend, _state)
{
    auto &sm = AP::serialmanager();
    uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    // uart = hal.serial(1);

    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        hal.console->printf("LORD IS NOT CONNECTED ANYMORE\n");
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LORD ExternalAHRS initialised");

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_LORD::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }

    baudrate = 115200;
}

void AP_ExternalAHRS_LORD::update_thread()
{
    if(!portOpened) {
        portOpened = true;
        uart->begin(baudrate);
        hal.scheduler->delay(1000);
    }

    while(true) {
        if(IMUPacketReady) {
            handleIMUPacket();
        }
        if(GNSSPacketReady) {
            handleGNSSPacket();
        }
        if(EFDPacketReady) {
            handleEFDPacket();
        }

        readIMU();
        buildPacket();
        hal.scheduler->delay(1);
    }
}


//read all available bytes into ring buffer.
void AP_ExternalAHRS_LORD::readIMU() {
    uint32_t amountRead = uart -> read(tempData, bufferSize);
    buffer.write(tempData, amountRead);
}

//use all available bytes to continue building packets where we left off last loop
void AP_ExternalAHRS_LORD::buildPacket() {
    while(buffer.available() >= (uint32_t)searchBytes) {
        switch (currPhase) {
            case sync: {
                bool good = buffer.read_byte(tempData);
                if(!good) break;
                if (tempData[0] == nextSyncByte) {
                    if (nextSyncByte == syncByte2) {
                        nextSyncByte = syncByte1;
                        currPacket.header[0] = 0x75;
                        currPacket.header[1] = 0x65;
                        currPhase = payloadSize;
                        searchBytes = 2;
                    } else {
                        nextSyncByte = syncByte2;
                    }
                } else {
                    nextSyncByte = syncByte1;
                }
            }
                break;
            case payloadSize: {
                buffer.peekbytes(tempData, searchBytes);
                currPacket.header[2] = tempData[0];
                currPacket.header[3] = tempData[1];
                searchBytes = tempData[1] + 4; //next time we need to peek the second half of the header (which we already peeked) + payload + checksum
                currPhase = payloadAndChecksum;
            }
                break;
            case payloadAndChecksum: {
                buffer.peekbytes(tempData, searchBytes);
                //copy in the payload and checksum, skip second half of header
                for (int i = 2; i < searchBytes - 2; i++) {
                    currPacket.payload[i - 2] = tempData[i];
                }
                currPacket.checksum[0] = tempData[searchBytes - 2];
                currPacket.checksum[1] = tempData[searchBytes - 1];
                //if checksum is good we can move read pointer, otherwise we leave all those bytes and start after the last sync bytes
                if (validPacket()) {
                    parsePacket();
                    buffer.read(tempData, searchBytes);
                }
                currPhase = sync;
                searchBytes = 1;
            }
                break;
        }
    }
}

//gets checksum and compares it to curr packet
bool AP_ExternalAHRS_LORD::validPacket() {
    uint8_t checksumByte1 = 0;
    uint8_t checksumByte2 = 0;

    for (int i = 0; i < 4; i++) {
        checksumByte1 += currPacket.header[i];
        checksumByte2 += checksumByte1;
    }

    for (int i = 0; i < currPacket.header[3]; i++) {
        checksumByte1 += currPacket.payload[i];
        checksumByte2 += checksumByte1;
    }

    return (currPacket.checksum[0] == checksumByte1 && currPacket.checksum[1] == checksumByte2);
}

// NEW PACKET PARSING CODE
void AP_ExternalAHRS_LORD::parsePacket() {
    uint8_t dataSet = currPacket.header[2];
    switch (dataSet) {
        case 0x80:
            parseIMU();
            break;
        case 0x81:
            parseGNSS();
            break;
        case 0x82:
            parseEFD();
            break;
    }
}

void AP_ExternalAHRS_LORD::parseIMU() {
    IMUPacketReady = true;

    uint8_t payloadLen = currPacket.header[3];
    for (uint8_t i = 0; i < payloadLen; i += currPacket.payload[i]) {
        uint8_t fieldDesc = currPacket.payload[i+1];
        switch (fieldDesc) {
            case 0x04: {
                accelNew = populateVector3f(currPacket.payload, i, 9.8);
                }break;
            case 0x05: {
                gyroNew = populateVector3f(currPacket.payload, i, 1);
                }break;
            case 0x06: {
                magNew = populateVector3f(currPacket.payload, i, 1000);
                }break;
            case 0x0A: { // Quat
                quatNew = populateQuaternion(currPacket.payload, i);
                }break;
            case 0x0C: { // Euler

                }break;
            case 0x12: {
                // TOW & GPSWeek
                uint16_t timestamp_flags = get4ByteField(currPacket.payload, i+14);
                if (timestamp_flags >= 4) {
                    auto temp = get8ByteField(currPacket.payload, i + 2);
                    GPSTOW = *reinterpret_cast<double *>(&temp);
                    GPSweek = get2ByteField(currPacket.payload, i + 10);
                }
                }break;
            case 0x17: {
                uint32_t tmp = get4ByteField(currPacket.payload, i + 2);
                pressureNew = *reinterpret_cast<float *>(&tmp);
                pressureNew *= 100;
                }break;
        }
    }
}

void AP_ExternalAHRS_LORD::parseGNSS() {
    GNSSPacketReady = true;

}

void AP_ExternalAHRS_LORD::parseEFD() {
    EFDPacketReady = true;

}

void AP_ExternalAHRS_LORD::handleIMUPacket() {
    IMUPacketReady = false;

    {
        WITH_SEMAPHORE(state.sem);
        state.accel = accelNew;
        state.gyro = gyroNew;
    }

    {
        AP_ExternalAHRS::ins_data_message_t ins;

        ins.accel = accelNew;
        ins.gyro = gyroNew;

        AP::ins().handle_external(ins);
    }

    {
        AP_ExternalAHRS::mag_data_message_t mag;
        mag.field = magNew;

        AP::compass().handle_external(mag);
    }

    {
        AP_ExternalAHRS::baro_data_message_t baro;
        baro.pressure_pa = pressureNew;
        AP::baro().handle_external(baro);
    }

    {
        AP_ExternalAHRS::gps_data_message_t gps;
        gps.gps_week = GPSweek;
        gps.ms_tow = (uint32_t)(GPSTOW * 1000);

        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "GPSweek: %d", gps.gps_week);
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ms_tow: %d", (int)gps.ms_tow);
        AP::gps().handle_external(gps);
    }
}

void AP_ExternalAHRS_LORD::handleGNSSPacket() {
    GNSSPacketReady = false;
}

void AP_ExternalAHRS_LORD::handleEFDPacket() {
    EFDPacketReady = false;
}

    int8_t AP_ExternalAHRS_LORD::get_port(void) const
{
    return 4;
};

bool AP_ExternalAHRS_LORD::healthy(void) const
{
    return true;
}

bool AP_ExternalAHRS_LORD::initialised(void) const
{
    return true;
}

bool AP_ExternalAHRS_LORD::pre_arm_check(char *failure_msg, uint8_t failure_msg_len) const
{
    return true;
}

void AP_ExternalAHRS_LORD::get_filter_status(nav_filter_status &status) const
{
    return;
}

void AP_ExternalAHRS_LORD::send_status_report(mavlink_channel_t chan) const
{
    return;
}

Vector3f AP_ExternalAHRS_LORD::populateVector3f(const uint8_t* pkt, uint8_t offset, float multiplier) {
    Vector3f data;
    uint32_t tmp[3];
    for (uint8_t j = 0; j < 3; j++) {
        tmp[j] = get4ByteField(pkt, offset + j * 4 + 2);
    }
    data.x = *reinterpret_cast<float*>( &tmp[0] );
    data.y = *reinterpret_cast<float*>( &tmp[1] );
    data.z = *reinterpret_cast<float*>( &tmp[2] );
    return data * multiplier;
}

Quaternion AP_ExternalAHRS_LORD::populateQuaternion(const uint8_t* pkt, uint8_t offset) {
    // uint32_t tmp[4];
    // for (uint8_t j = 0; j < 3; j++) {
    //     tmp[j] = get4ByteField(pkt, offset + j * 4 + 2);
    // }
    Quaternion x;
    x.initialise();
    return x;
}

uint64_t AP_ExternalAHRS_LORD::get8ByteField(const uint8_t* pkt, uint8_t offset) {
    uint64_t res = 0;
    // for (int i = 0; i < 2; i++)
    //     res = res << 32 | get4ByteField(pkt,offset + 4 * i);
    memmove(&res, pkt+offset, 8);
    if (char(1) == 1)
        res = ((res & 0xff) << 56) | ((res & 0xff00000000000000) >> 56) | ((res & 0xff00) << 40) | ((res & 0xff000000000000) >> 40) | ((res & 0xff0000) << 24) | ((res & 0xff0000000000) >> 24) | ((res & 0xff000000) << 8) | ((res & 0xff00000000) >> 8);
    return res;
}

uint32_t AP_ExternalAHRS_LORD::get4ByteField(const uint8_t* pkt, uint8_t offset) {
    uint32_t res = 0;
    // for (int i = 0; i < 2; i++)
    //     res = res << 16 | get2ByteField(pkt, offset + 2 * i);
    memmove(&res, pkt+offset, 4);
    if (char(1) == 1) // Is the device little endian (need to convert big endian packet field to little endian)
        res = ((res & 0xff) << 24) | ((res & 0xff000000) >> 24) | ((res & 0xff00) << 8) | ((res & 0xff0000) >> 8);
    return res;
}

uint16_t AP_ExternalAHRS_LORD::get2ByteField(const uint8_t* pkt, uint8_t offset) {
    uint16_t res = 0;
    // for (int i = 0; i < 2; i++)
    //     res = res << 8 | pkt[offset + i];
    memmove(&res, pkt+offset, 2);
    if (char(1) == 1)
        res = ((res & 0xff) << 8) | ((res & 0xff00) >> 8);
    return res;
}



#endif  // HAL_EXTERNAL_AHRS_ENABLED