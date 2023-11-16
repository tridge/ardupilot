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

#include "AP_Periph.h"

#ifdef HAL_PERIPH_ENABLE_NETWORKING

extern const AP_HAL::HAL &hal;

const AP_Param::GroupInfo Networking_Periph::var_info[] {
    // @Group: _
    // @Path: ../AP_Networking/AP_Networking.cpp
    AP_SUBGROUPINFO(networking, "_", 1, Networking_Periph, AP_Networking),

    /*
      the NET_Pn_ parameters need to be bere as otherwise we
      are too deep in the parameter tree
     */

#if AP_NETWORKING_NUM_PORTS > 0
    // @Group: _P1_
    // @Path: ../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking.ports[0], "_P1_", 2, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 1
    // @Group: _P2_
    // @Path: ../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking.ports[1], "_P2_", 3, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 2
    // @Group: _P3_
    // @Path: ../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking.ports[2], "_P3_", 4, Networking_Periph, AP_Networking::Port),
#endif

#if AP_NETWORKING_NUM_PORTS > 3
    // @Group: _P4_
    // @Path: ../libraries/AP_Networking/AP_Networking_port.cpp
    AP_SUBGROUPINFO(networking.ports[3], "_P4_", 5, Networking_Periph, AP_Networking::Port),
#endif

    AP_GROUPEND
};

Networking_Periph::Networking_Periph(void)
{
    AP_Param::setup_object_defaults(this, var_info);
}


/*
  update UART pass-thru, if enabled
 */
void Networking_Periph::update_passthru(void)
{
    WITH_SEMAPHORE(_passthru.sem);
    uint32_t now = AP_HAL::millis();
    uint32_t baud1, baud2;
    bool enabled = AP::serialmanager().get_passthru(_passthru.port1, _passthru.port2, _passthru.timeout_s,
                                                    baud1, baud2);
    if (enabled && !_passthru.enabled) {
        _passthru.start_ms = now;
        _passthru.last_ms = 0;
        _passthru.enabled = true;
        _passthru.last_port1_data_ms = now;
        _passthru.baud1 = baud1;
        _passthru.baud2 = baud2;
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Passthru enabled");
        if (!_passthru.timer_installed) {
            _passthru.timer_installed = true;
            hal.scheduler->register_timer_process(FUNCTOR_BIND_MEMBER(&Networking_Periph::passthru_timer, void));
        }
    } else if (!enabled && _passthru.enabled) {
        _passthru.enabled = false;
        _passthru.port1->lock_port(0, 0);
        _passthru.port2->lock_port(0, 0);
        // Restore original baudrates
        if (_passthru.baud1 != baud1) {
            _passthru.port1->end();
            _passthru.port1->begin(baud1);
        }
        if (_passthru.baud2 != baud2) {
            _passthru.port2->end();
            _passthru.port2->begin(baud2);
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Passthru disabled");
    } else if (enabled &&
               _passthru.timeout_s &&
               now - _passthru.last_port1_data_ms > uint32_t(_passthru.timeout_s)*1000U) {
        // timed out, disable
        _passthru.enabled = false;
        _passthru.port1->lock_port(0, 0);
        _passthru.port2->lock_port(0, 0);
        AP::serialmanager().disable_passthru();
        // Restore original baudrates
        if (_passthru.baud1 != baud1) {
            _passthru.port1->end();
            _passthru.port1->begin(baud1);
        }
        if (_passthru.baud2 != baud2) {
            _passthru.port2->end();
            _passthru.port2->begin(baud2);
        }
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "Passthru timed out");
    }
}

/*
  called at 1kHz to handle pass-thru between SERIA0_PASSTHRU port and hal.console
 */
void Networking_Periph::passthru_timer(void)
{
    WITH_SEMAPHORE(_passthru.sem);

    if (!_passthru.enabled) {
        // it has been disabled after starting
        return;
    }
    if (_passthru.start_ms != 0) {
        uint32_t now = AP_HAL::millis();
        if (now - _passthru.start_ms < 1000) {
            // delay for 1s so the reply for the SERIAL0_PASSTHRU param set can be seen by GCS
            return;
        }
        _passthru.start_ms = 0;
        _passthru.port1->begin(_passthru.baud1);
        _passthru.port2->begin(_passthru.baud2);
    }

    // while pass-thru is enabled lock both ports. They remain
    // locked until disabled again, or reboot
    const uint32_t lock_key = 0x3256AB9F;
    _passthru.port1->lock_port(lock_key, lock_key);
    _passthru.port2->lock_port(lock_key, lock_key);

    // Check for requested Baud rates over USB
    uint32_t baud = _passthru.port1->get_usb_baud();
    if (_passthru.baud2 != baud && baud != 0) {
        _passthru.baud2 = baud;
        _passthru.port2->end();
        _passthru.port2->begin_locked(baud, 0, 0, lock_key);
    }

    baud = _passthru.port2->get_usb_baud();
    if (_passthru.baud1 != baud && baud != 0) {
        _passthru.baud1 = baud;
        _passthru.port1->end();
        _passthru.port1->begin_locked(baud, 0, 0, lock_key);
    }

    uint8_t buf[64];

    // read from port1, and write to port2
    int16_t nbytes = _passthru.port1->read_locked(buf, sizeof(buf), lock_key);
    if (nbytes > 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "data1 %d", nbytes);
        _passthru.last_port1_data_ms = AP_HAL::millis();
        _passthru.port2->write_locked(buf, nbytes, lock_key);
    }

    // read from port2, and write to port1
    nbytes = _passthru.port2->read_locked(buf, sizeof(buf), lock_key);
    if (nbytes > 0) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "data2 %d", nbytes);
        _passthru.port1->write_locked(buf, nbytes, lock_key);
    }
}
#endif  // HAL_PERIPH_ENABLE_NETWORKING

