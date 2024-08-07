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

#include <AP_AHRS/AP_AHRS.h>
#include "AP_InertialSensor_rate_config.h"
#include "AP_InertialSensor.h"
#include "AP_InertialSensor_Backend.h"

#if AP_INERTIALSENSOR_RATE_LOOP_WINDOW_ENABLED
#include "FastRateBuffer.h"
#include <stdio.h>

extern const AP_HAL::HAL& hal;

#if CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
// hal.console can be accessed from bus threads on ChibiOS
#define debug(fmt, args ...)  do {hal.console->printf("IMU: " fmt "\n", ## args); } while(0)
#else
#define debug(fmt, args ...)  do {printf("IMU: " fmt "\n", ## args); } while(0)
#endif

void AP_InertialSensor::enable_fast_rate_buffer()
{
    fast_rate_buffer = NEW_NOTHROW FastRateBuffer();
}

void AP_InertialSensor::disable_fast_rate_buffer()
{
    delete fast_rate_buffer;
    fast_rate_buffer = nullptr;
}

uint32_t AP_InertialSensor::get_num_gyro_samples()
{
    return fast_rate_buffer->get_num_gyro_samples();
}

void AP_InertialSensor::set_rate_decimation(uint8_t rdec)
{
    fast_rate_buffer->set_rate_decimation(rdec);
}

// are gyro samples being sourced from the rate loop buffer
bool AP_InertialSensor::use_rate_loop_gyro_samples() const
{
    return fast_rate_buffer != nullptr;
}

// whether or not to push the current gyro sample
bool AP_InertialSensor::push_rate_loop_gyro(uint8_t instance) const
{
    return use_rate_loop_gyro_samples() && fast_rate_buffer->use_rate_loop_gyro_samples() && instance == AP::ahrs().get_primary_gyro_index();
}

bool AP_InertialSensor::get_next_gyro_sample(Vector3f& gyro)
{
    if (!use_rate_loop_gyro_samples()) {
        return false;
    }

    return fast_rate_buffer->get_next_gyro_sample(gyro);
}

bool FastRateBuffer::get_next_gyro_sample(Vector3f& gyro)
{
    if (!use_rate_loop_gyro_samples()) {
        return false;
    }

    _cmutex.lock_and_wait(FUNCTOR_BIND_MEMBER(&FastRateBuffer::gyro_samples_available, bool));
    bool ret = _rate_loop_gyro_window.pop(gyro);
    _cmutex.unlock();

    return ret;
}

void AP_InertialSensor::push_next_gyro_sample(uint8_t instance)
{
    if (push_rate_loop_gyro(instance)) {
        /*
          tell the rate thread we have a new sample
        */
        if (++fast_rate_buffer->rate_decimation_count >= fast_rate_buffer->rate_decimation) {
            fast_rate_buffer->_cmutex.lock_and_signal();
            if (!fast_rate_buffer->_rate_loop_gyro_window.push(_gyro_filtered[instance])) {
                debug("dropped rate loop sample");
            }
            fast_rate_buffer->rate_decimation_count = 0;
            // semaphore is already held so we can directly publish the gyro data
            _gyro[instance] = _gyro_filtered[instance];
#if HAL_GYROFFT_ENABLED
            // copy the gyro samples from the backend to the frontend window for FFTs sampling at less than IMU rate
            _gyro_for_fft[instance] = _last_gyro_for_fft[instance];
#endif
            fast_rate_buffer->_cmutex.unlock();
        }
    }
}

void AP_InertialSensor::update_backend_filters()
{
    for (uint8_t i=0; i<_backend_count; i++) {
        _backends[i]->update_filters();
    }
}

void AP_InertialSensor_Backend::update_filters()
{
    WITH_SEMAPHORE(_sem);

    update_accel_filters(accel_instance);
    update_gyro_filters(gyro_instance);
}

#endif // AP_INERTIALSENSOR_RATE_LOOP_WINDOW_ENABLED