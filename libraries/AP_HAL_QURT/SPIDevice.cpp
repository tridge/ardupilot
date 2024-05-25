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

#include "SPIDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL/utility/OwnPtr.h>
#include "Scheduler.h"
#include "Semaphores.h"
#include <stdio.h>

using namespace QURT;

#define MHZ (1000U*1000U)
#define KHZ (1000U)

const char *device_names[] = {"INV1", "INV2", "INV3"};

extern qurt_func_ptrs_t qurt_func_ptrs;

static SPIBus *spi_bus;

SPIBus::SPIBus(void):
    DeviceBus(Scheduler::PRIORITY_SPI)
{
    fd = qurt_func_ptrs._config_spi_bus_func_t();
    HAP_PRINTF("Created SPI bus -> %d", fd);
}

SPIDevice::SPIDevice(const char *name, SPIBus &_bus)
    : bus(_bus)
{
    set_device_bus(0);
    set_device_address(1);
    set_speed(AP_HAL::Device::SPEED_LOW);

    pname = name;
}

SPIDevice::~SPIDevice()
{
}

bool SPIDevice::set_speed(AP_HAL::Device::Speed _speed)
{
    return true;
}

bool SPIDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    //HAP_PRINTF("SPI transfer %p %u %p %u [%02x %02x]", send, send_len, recv, recv_len, send?send[0]:1, recv?recv[0]:1);
    if (!send || !recv) {
        // simplest cases
        return transfer_fullduplex(send, recv, recv_len?recv_len:send_len);
    }
    uint8_t buf[send_len+recv_len];
    if (send_len > 0) {
        memcpy(buf, send, send_len);
    }
    if (recv_len > 0) {
        memset(&buf[send_len], 0, recv_len);
    }
    transfer_fullduplex(buf, buf, send_len+recv_len);
    if (recv_len > 0) {
        memcpy(recv, &buf[send_len], recv_len);
    }
    return true;
}

bool SPIDevice::transfer_fullduplex(const uint8_t *send, uint8_t *recv, uint32_t len)
{
    uint8_t dummy[len];
    memset(dummy, 0xFF, sizeof(dummy));
    if (send == nullptr) {
        send = dummy;
    }
    if (recv == nullptr) {
        recv = dummy;
    }
    int ret = qurt_func_ptrs._spi_transfer_func_t(bus.fd, send, recv, len);
    return ret == 0;
}

void SPIDevice::acquire_bus(bool accuire)
{
}

AP_HAL::Semaphore *SPIDevice::get_semaphore()
{
    return &bus.semaphore;
}

AP_HAL::Device::PeriodicHandle SPIDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bus.register_periodic_callback(period_usec, cb, this);
}

bool SPIDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

AP_HAL::OwnPtr<AP_HAL::SPIDevice>
SPIDeviceManager::get_device(const char *name)
{
    uint8_t i;
    for (i = 0; i<ARRAY_SIZE(device_names); i++) {
        if (strcmp(device_names[i], name) == 0) {
            break;
        }
    }
    if (i == ARRAY_SIZE(device_names)) {
        HAP_PRINTF("*&*&*&*&* Unknown SPI name %s", name);
        return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(nullptr);
    }

    if (spi_bus == nullptr) {
        spi_bus = new SPIBus();
    }

    HAP_PRINTF("*&*&*&*&* got SPI name %s", name);

    return AP_HAL::OwnPtr<AP_HAL::SPIDevice>(new SPIDevice(name, *spi_bus));
}

