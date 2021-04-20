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
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LORD about to try to use serial manager!!!");
    //uart = sm.find_serial(AP_SerialManager::SerialProtocol_AHRS, 0);
    uart = hal.serial(4);
    if (!uart) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "ExternalAHRS no UART");
        hal.console->printf("LORD IS NOT CONNECTED ANYMORE\n");
        return;
    }
    GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LORD ExternalAHRS initialised");

    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_ExternalAHRS_LORD::update_thread, void), "AHRS", 2048, AP_HAL::Scheduler::PRIORITY_SPI, 0)) {
        AP_HAL::panic("Failed to start ExternalAHRS update thread");
    }

    baudrate = sm.find_baudrate(AP_SerialManager::SerialProtocol_AHRS, 0);
    port_num = sm.find_portnum(AP_SerialManager::SerialProtocol_AHRS, 0);
    //uart->begin(baudrate);
    //GCS_SEND_TEXT(MAV_SEVERITY_INFO, "LORD ExternalAHRS with baud: %lu, on port %d", baudrate, port_num);

}

void AP_ExternalAHRS_LORD::update_thread()
{
    if(!portOpened) {
        portOpened = true;
        uart->begin(115200);
        hal.scheduler->delay(1000);
    }

    while(true) {
        if(packetReady) {
            packetReady = false;
             AP_ExternalAHRS::ins_data_message_t ins;

            ins.accel = accelNew;
            ins.gyro = gyroNew;

            AP::ins().handle_external(ins);
        }

        readIMU();
        buildPacket();
        hal.scheduler->delay(1);
    }
}

//LORD METHODS


//read all available bytes into ring buffer.
void AP_ExternalAHRS_LORD::readIMU() {
    uint32_t amountRead = uart -> read(tempData, bufferSize);
    buffer.write(tempData, amountRead);
}

//use all available bytes to continue building packets where we left off last loop
void AP_ExternalAHRS_LORD::buildPacket() {
    while(buffer.available() >= searchBytes) {
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
                    getCurrPacket();
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


void AP_ExternalAHRS_LORD::accelGyroData(uint8_t * fieldData, float arr[]) {
    //vector<float> xyzData;
    uint32_t midx = 0;
    for (uint8_t i = 0; i < 4; i++ ) {
        midx = (midx << 8) + static_cast<uint32_t>(fieldData[i]);
    }
    uint32_t midy = 0;
    for (uint8_t i = 4; i < 8; i++ ) {
        midy = (midy << 8) + static_cast<uint32_t>(fieldData[i]);
    }
    uint32_t midz = 0;
    for (uint8_t i = 8; i < 12; i++ ) {
        midz = (midz << 8) + static_cast<uint32_t>(fieldData[i]);
    }
    arr[0] = ( (* (float *) &midx) ); // Reinterpret_cast does not work for this so
    arr[1] = ( (* (float *) &midy) );
    arr[2] = ( (* (float *) &midz) );
    //xyzData.push_back(* reinterpret_cast<float*>(&midx)); // Works, but its doing the same thing as above, if you do it without address' it doesn't work
    //return xyzData;
}

void AP_ExternalAHRS_LORD::getCurrPacket() {
    /*for(int i = 0; i < 4; i++) {
        console->printf("0x%x ", currPacket.header[i]);
    }
    for(int i = 0; i < currPacket.header[3]; i++) {
        console->printf("0x%x ", currPacket.payload[i]);
    }
    for(int i = 0; i < 2; i++) {
        console->printf("0x%x ", currPacket.checksum[i]); 75 65 80 A3
    }
    console -> printf("\n");*/
    uint8_t payloadLength = currPacket.header[3];
// length descriptor, 12bit payload
    for (uint8_t i = 0; i < payloadLength; i += currPacket.payload[i]) {
        //uint8_t fieldLength = currPacket.payload[i]; //Each field in the payload has its own length property as the first byte of the field
        uint8_t fieldDescriptor = currPacket.payload[i+1]; // Second byte is the field descriptor. Whether it is accel or gyro, etc.
        //ctor<float> xyz; // Maybe call accelGyroData() here instead to reduce reuse of code but could be a waste if we include other data in the field like GPS data
        float retarr[3];
        accelGyroData(currPacket.payload + (i+2),retarr); // This will probably break if the packet is wrong
        switch (fieldDescriptor) {  // Switch to relevant course for accel or gyro or etc. data
            case 4:
                accelNew = Vector3f{retarr[0], retarr[1], retarr[2]};
                //console->printf("Accel - X: %f,\tY: %f,\tZ: %f\n",retarr[0], retarr[1], retarr[2]);
                break;
            case 5:
                gyroNew = Vector3f{retarr[0], retarr[1], retarr[2]};
                //console->printf("Gyro  - X: %f,\tY: %f,\tZ: %f\n",retarr[0], retarr[1], retarr[2]);
                // cout << "Gyro:\nX: " << xyz[0] << "\nY: " << xyz[1] << "\nZ: " << xyz[2] << '\n';
                // firstg = (firstg + 1) % BUFFSIZE; // Before we push on data increment the index so that firstg will equal the most recent data
                // gyroBuffer[firstg] = xyz; // Push vector with gyro xyz onto the buffer
                break;
            default:
                //cout << "Don't want anything but Gyro and Accel\n";
                break;
        }
    }
    packetReady = true;
}

//END LORD METHODS

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
/*
LORDpacketData_t AP_ExternalAHRS_LORD::processLORDPacket(const uint8_t*) {
    uint8_t pktDesc = pkt[2];
    switch (pktDesc) {
        case 0x80:
            return insData(pkt);
        default:
            LORDpacketData_t nullPacket = { -999, -999, -999, -999, -999, -999 };
            return nullPacket;
    }
}

LORDpacketData_t AP_ExternalAHRS_LORD::insData(const uint8_t*) {
    LORDpacketData_t data;
    uint8_t payloadLen = pkt[3];
    for (uint8_t i = 4; i < payloadLen; i += pkt[i]) {
        uint8_t fieldDesc = pkt[i+1];
        switch (fieldDesc) {
            case 04:
                data.accel = populateVector3f(pkt, i);
                break;
            case 05:
                data.gyro = populateVector3f(pkt, i);
                break;
        }
    }
    return data;
}

Vector3f AP_ExternalAHRS_LORD::populateVector3f(const uint8_t*,uint8_t) {
    Vector3f data;
    uint32_t tmp[3];
    for (uint8_t j = 0; j < 3; j++) {
        tmp[j] = get4ByteField(pkt, offset + j * 4 + 2);
    }
    data.x = *reinterpret_cast<float*>( &tmp[0] );
    data.y = *reinterpret_cast<float*>( &tmp[1] );
    data.z = *reinterpret_cast<float*>( &tmp[2] );
    return data;
}

uint64_t AP_ExternalAHRS_LORD::get8ByteField(const uint8_t*,uint8_t) {
    uint64_t res = 0;
    for (int i = 0; i < 2; i++)
        res = res << 32 | get4ByteField(pkt,offset + 4 * i);
    return res;
}

uint32_t AP_ExternalAHRS_LORD::get4ByteField(const uint8_t*,uint8_t) {
    uint32_t res = 0;
    for (int i = 0; i < 2; i++)
        res = res << 16 | get2ByteField(pkt, offset + 2 * i);
    return res;
}

uint16_t AP_ExternalAHRS_LORD::get2ByteField(const uint8_t*,uint8_t) {
    uint16_t res = 0;
    for (int i = 0; i < 2; i++)
        res = res << 8 | pkt[offset + i];
    return res;
}
*/

#endif  // HAL_EXTERNAL_AHRS_ENABLED