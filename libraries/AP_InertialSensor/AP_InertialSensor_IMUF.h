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
/*
  The IMUF is the STM32F3 based IMU on the HelioSpring
 */
#pragma once

#include <AP_HAL/AP_HAL.h>

#if defined(HAL_IMUF_RESET_PIN) && defined(HAL_IMUF_READY_PIN)
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

class AP_InertialSensor_IMUF : public AP_InertialSensor_Backend {
public:
    static AP_InertialSensor_Backend *probe(AP_InertialSensor &imu,
                                            AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                                            enum Rotation rotation = ROTATION_NONE);

    /**
     * Configure the sensors and start reading routine.
     */
    void start() override;
    bool update() override;

private:
    AP_InertialSensor_IMUF(AP_InertialSensor &imu,
                           AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev,
                           enum Rotation rotation = ROTATION_NONE);

    /*
      initialise driver
     */
    bool init();
    bool imuf_send_receive(uint32_t imufCommand);
    void read_sensor();
    bool wait_ready(uint32_t timeout_ms);
    void reset();

    AP_HAL::OwnPtr<AP_HAL::SPIDevice> dev;

    uint8_t accel_instance;
    uint8_t gyro_instance;
    enum Rotation rotation;
};
#endif
