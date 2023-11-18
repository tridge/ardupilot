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

#include <AP_SerialManager/AP_SerialManager.h>

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

    AP_GROUPINFO("PASS1_EP1", 6,  Networking_Periph, passthru[0].ep1,   -1),
    AP_GROUPINFO("PASS1_EP2", 7,  Networking_Periph, passthru[0].ep2,   -1),

    AP_GROUPINFO("PASS2_EP1", 8,  Networking_Periph, passthru[1].ep1,   -1),
    AP_GROUPINFO("PASS2_EP2", 9,  Networking_Periph, passthru[1].ep2,   -1),

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
    /*
      see if we need to open a new port
     */
    auto &serial_manager = AP::serialmanager();

    for (auto &p : passthru) {
        if (p.port1 == nullptr && p.port2 == nullptr &&
            p.ep1 != -1 && p.ep2 != -1 && p.ep1 != p.ep2) {
            p.port1 = serial_manager.get_serial_by_id(p.ep1);
            p.port2 = serial_manager.get_serial_by_id(p.ep2);
            if (p.port1 != nullptr && p.port2 != nullptr) {
                p.port1->begin(115200);
                p.port2->begin(115200);
            }
        }
    }

    for (auto &p : passthru) {
        uint8_t buf[64];
        if (p.port1 == nullptr || p.port2 == nullptr) {
            continue;
        }
        // read from port1, and write to port2
        auto avail = p.port1->available();
        if (avail > 0) {
            auto space = p.port2->txspace();
            const uint32_t n = MIN(space, sizeof(buf));
            const auto nbytes = p.port1->read(buf, n);
            if (nbytes > 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "data1 %d", unsigned(nbytes));
                p.port2->write(buf, nbytes);
            }
        }

        // read from port2, and write to port1
        avail = p.port2->available();
        if (avail > 0) {
            auto space = p.port1->txspace();
            const uint32_t n = MIN(space, sizeof(buf));
            const auto nbytes = p.port2->read(buf, n);
            if (nbytes > 0) {
                GCS_SEND_TEXT(MAV_SEVERITY_INFO, "data2 %d", unsigned(nbytes));
                p.port1->write(buf, nbytes);
            }
        }
    }
}
#endif  // HAL_PERIPH_ENABLE_NETWORKING

