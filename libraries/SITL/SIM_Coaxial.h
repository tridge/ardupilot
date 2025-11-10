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
  single copter simulator class
*/

#pragma once

#include "SIM_Aircraft.h"
#include <AP_Motors/AP_Motors.h>

namespace SITL {

/*
  Spirit coaxial simulator
 */
class Coaxial : public Aircraft {
public:

    friend class Parameters;

    Coaxial(const char *frame_str);

    /* update model by one time step */
    void update(const struct sitl_input &input) override;

    /* static object creator */
    static Aircraft *create(const char *frame_str) {
      return new Coaxial(frame_str);
    }

private:
    const float aero_damping_constant = 0.2f;
    const float cda = 0.006f;
    const float radius_m = 0.1f;
    const float max_voltage_V = 50.4;
    const float avionics_draw_A = 0.06;

    float last_param_batt_voltage;
    float last_param_batt_number;
    float last_param_batt_capacity;
    float last_param_batt_resistance;
};

} // namespace SITL
