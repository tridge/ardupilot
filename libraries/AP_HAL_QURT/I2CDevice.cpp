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
#include "I2CDevice.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include <AP_HAL_QURT/Semaphores.h>
#include "Scheduler.h"

extern qurt_func_ptrs_t qurt_func_ptrs;

using namespace QURT;

/*
  4 I2C buses

  bus1: mag
  bus2: power manager
  bus4: external spare bus
  bus5: barometer (internal)
*/
static uint8_t i2c_bus_ids[] = { 1, 2, 4, 5 };

static uint32_t i2c_internal_mask = (1U<<3);

I2CBus I2CDeviceManager::businfo[ARRAY_SIZE(i2c_bus_ids)];

I2CDeviceManager::I2CDeviceManager(void)
{
    /*
      open all the buses with a default speed
     */
    HAP_PRINTF("In I2CDeviceManager constructor");
}

I2CDevice::I2CDevice(uint8_t busnum, uint8_t address, uint32_t bus_clock, bool use_smbus, uint32_t timeout_ms) :
    _retries(10),
    _address(address),
    bus(I2CDeviceManager::businfo[busnum])
{
    HAP_PRINTF("Constructing I2CDevice %u 0x%02x %u", unsigned(busnum), unsigned(address), unsigned(bus_clock));
    qurt_timer_sleep(5000);

    if (bus.fd == -2) {
        HAP_PRINTF("Calling _config_i2c_bus_func_t %u",
                   unsigned(i2c_bus_ids[busnum])); qurt_timer_sleep(5000);
        bus.fd = qurt_func_ptrs._config_i2c_bus_func_t(i2c_bus_ids[busnum], address, bus_clock/1000);
        HAP_PRINTF("Opened I2C bus %u -> %d", unsigned(busnum), bus.fd);
    }
    set_device_bus(busnum);
    set_device_address(address);
    asprintf(&pname, "I2C:%u:%02x",
             (unsigned)busnum, (unsigned)address);
}

I2CDevice::~I2CDevice()
{
    free(pname);
}

bool I2CDevice::transfer(const uint8_t *send, uint32_t send_len,
                         uint8_t *recv, uint32_t recv_len)
{
    if (bus.fd <= 0) {
        return false;
    }
    if (!bus.semaphore.check_owner()) {
        HAP_PRINTF("I2C: not owner of 0x%x", (unsigned)get_bus_id());
        return false;
    }
    if (bus.last_address != _address) {
        HAP_PRINTF("I2C: changing address for fd %d to 0x%02x", bus.fd, _address);
        qurt_func_ptrs._set_i2c_address_func_t(bus.fd, _address);
        bus.last_address = _address;
    }
    return qurt_func_ptrs._i2c_transfer_func_t(bus.fd, send, send_len, recv, recv_len) == 0;
}

/*
  register a periodic callback
*/
AP_HAL::Device::PeriodicHandle I2CDevice::register_periodic_callback(uint32_t period_usec, AP_HAL::Device::PeriodicCb cb)
{
    return bus.register_periodic_callback(period_usec, cb, this);
}


/*
  adjust a periodic callback
*/
bool I2CDevice::adjust_periodic_callback(AP_HAL::Device::PeriodicHandle h, uint32_t period_usec)
{
    return bus.adjust_timer(h, period_usec);
}

AP_HAL::OwnPtr<AP_HAL::I2CDevice>
I2CDeviceManager::get_device(uint8_t bus, uint8_t address,
                             uint32_t bus_clock,
                             bool use_smbus,
                             uint32_t timeout_ms)
{
    if (bus >= ARRAY_SIZE(i2c_bus_ids)) {
        return AP_HAL::OwnPtr<AP_HAL::I2CDevice>(nullptr);
    }
    auto dev = AP_HAL::OwnPtr<AP_HAL::I2CDevice>(new I2CDevice(bus, address, bus_clock, use_smbus, timeout_ms));
    return dev;
}

/*
  get mask of bus numbers for all configured I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask(void) const
{
    return ((1U << ARRAY_SIZE(i2c_bus_ids)) - 1);
}

/*
  get mask of bus numbers for all configured internal I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_internal(void) const
{
    return i2c_internal_mask;
}

/*
  get mask of bus numbers for all configured external I2C buses
*/
uint32_t I2CDeviceManager::get_bus_mask_external(void) const
{
    return get_bus_mask() & ~get_bus_mask_internal();
}
