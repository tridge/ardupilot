/*
 * This file is free software: you can redistribute it and/or modify it
 * under the terms of the GNU General Public License as published by the
 * Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This file is distributed in the hope that it will be useful, but
 * WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
 * See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along
 * with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>

#if defined(HAL_IMUF_RESET_PIN) && defined(HAL_IMUF_READY_PIN)
#include "AP_InertialSensor_IMUF.h"
#include <stdio.h>
#include <utility>



#define IMUF_COMMAND_NONE          0
#define IMUF_COMMAND_CALIBRATE    99
#define IMUF_COMMAND_LISTENING   108
#define IMUF_COMMAND_REPORT_INFO 121
#define IMUF_COMMAND_SETUP       122
#define IMUF_COMMAND_SETPOINT    126

extern const AP_HAL::HAL& hal;

AP_InertialSensor_IMUF::AP_InertialSensor_IMUF(AP_InertialSensor &imu,
                                                   AP_HAL::OwnPtr<AP_HAL::SPIDevice> _dev,
                                                   enum Rotation _rotation)
    : AP_InertialSensor_Backend(imu)
    , dev(std::move(_dev))
    , rotation(_rotation)
{
}

AP_InertialSensor_Backend *
AP_InertialSensor_IMUF::probe(AP_InertialSensor &imu,
                                AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                enum Rotation rotation)
{
    if (!dev) {
        return nullptr;
    }
    auto sensor = new AP_InertialSensor_IMUF(imu, std::move(dev), rotation);

    if (!sensor) {
        return nullptr;
    }

    if (!sensor->init()) {
        delete sensor;
        return nullptr;
    }

    return sensor;
}

void AP_InertialSensor_IMUF::start()
{
    printf("IMUF start\n");
    accel_instance = _imu.register_accel(1000, dev->get_bus_id_devtype(DEVTYPE_INS_IMUF));
    gyro_instance = _imu.register_gyro(1000,   dev->get_bus_id_devtype(DEVTYPE_INS_IMUF));

    // setup sensor rotations from probe()
    set_gyro_orientation(gyro_instance, rotation);
    set_accel_orientation(accel_instance, rotation);
    
    // setup callback
    dev->register_periodic_callback(100000,
                                    FUNCTOR_BIND_MEMBER(&AP_InertialSensor_IMUF::read_sensor, void));
    printf("IMUF start done\n");
}

/*
  calculate a 32 bit CRC using the STM32 hardware CRC support. Uses
  the default polynomial of 0x04C11DB7
 */
static uint32_t crc_block(const uint32_t *data, uint8_t len)
{
    CRC->CR = CRC_CR_RESET;
    while (len--) {
        CRC->DR = *data++;
    }
    return CRC->DR;
}

struct PACKED IMUFCommand {
   uint32_t command;
   uint32_t param[10];
   uint32_t crc;
   uint32_t tail;
};

bool AP_InertialSensor_IMUF::init()
{
    rccEnableCRC(FALSE);

    uint8_t xx[4] = { 50, 10, 243, 147 };

    printf("xx_crc=0x%08x\n", crc_block((const uint32_t *)xx, 1));

    // reset IMUF
    hal.gpio->write(HAL_IMUF_RESET_PIN, 0);
    hal.scheduler->delay(550);
    
    // set reset high to start IMUF
    hal.gpio->write(HAL_IMUF_RESET_PIN, 1);
    hal.scheduler->delay(1000);

    uint32_t start_ms = AP_HAL::millis();
    while (AP_HAL::millis() - start_ms < 2000) {
        if (hal.gpio->read(HAL_IMUF_READY_PIN)) {
            break;
        }
    }
    if (!hal.gpio->read(HAL_IMUF_READY_PIN)) {
        printf("IMUF not ready\n");
        dev->get_semaphore()->give();
        return false;
    }

    printf("IMUF sending setup\n");
    dev->get_semaphore()->take_blocking();
    struct IMUFCommand cmd {}, reply {};

    cmd.command = IMUF_COMMAND_SETUP;
    cmd.param[0] = 32; // gyro+accel+temp+crc
    cmd.param[1] = (6<<16) | 300; // 1kHz, 300 window
    cmd.param[2] = (3000<<16) | 3000; // RollQ, PitchQ
    cmd.param[3] = (3000<<16) | 100; // YawQ, RollGyroLPF
    cmd.param[4] = (100<<16) | 100; // pitchGyroLPF, YawGyroLPF
    cmd.param[5] = 0; // unused
    cmd.param[6] = 0; // unused
    cmd.param[7] = 0; // RollOrient, Orient
    cmd.param[8] = 0; // YawOrient, PitchOrient
    cmd.param[9] = 0; // unknown
    cmd.crc = crc_block((const uint32_t *)&cmd, 11);

    const uint16_t tries = 100;
    bool ret = false;
    for (uint16_t i=0; i<tries; i++) {
        ret = dev->transfer((const uint8_t *)&cmd, sizeof(cmd), (uint8_t *)&reply, sizeof(reply));
        if (!ret) {
            break;
        }
        uint32_t crc2 = crc_block((const uint32_t *)&reply, 11);
        printf("crcs: 0x%08x 0x%08x\n", reply.crc, crc2);
        if (reply.crc == crc2) {
            break;
        }
        ret = false;
        hal.scheduler->delay(10);
    }
    dev->get_semaphore()->give();
    printf("IMUF init done: %u 0x%08x 0x%x 0x%x\n", ret, cmd.crc, reply.command, reply.crc);
    return ret;
}

/*
  read accel fifo
 */
void AP_InertialSensor_IMUF::read_sensor(void)
{
    struct PACKED {
        float gyro[3];
        float accel[3];
        float temp;
        uint32_t crc;
    } data;
    if (!dev->transfer(nullptr, 0, (uint8_t *)&data, sizeof(data))) {
        return;
    }
    uint32_t crc2 = crc_block((uint32_t *)&data, 7);
    if (crc2 != data.crc) {
        printf("crc2 0x%08x crc 0x%08x\n", crc2, data.crc);
        return;
    }
    Vector3f gyro(data.gyro[0], data.gyro[1], data.gyro[2]);
    _rotate_and_correct_gyro(gyro_instance, gyro);
    _notify_new_gyro_raw_sample(gyro_instance, gyro);

    Vector3f accel(data.accel[0], data.accel[1], data.accel[2]);
    _rotate_and_correct_accel(accel_instance, accel);
    _notify_new_accel_raw_sample(accel_instance, accel);

    _publish_temperature(accel_instance, data.temp);
}

bool AP_InertialSensor_IMUF::update()
{
    update_accel(accel_instance);
    update_gyro(gyro_instance);
    return true;
}

#endif
