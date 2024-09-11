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
  SA_GD2000 simulator class
*/

#include "SIM_SA_GD2000.h"

#if AP_SIM_SA_GD2000_ENABLED

#include <AP_Logger/AP_Logger.h>

#include <stdio.h>

using namespace SITL;

extern const AP_HAL::HAL& hal;

// SITL SA_GD2000 parameters
const AP_Param::GroupInfo SA_GD2000::var_info[] = {
    // @Param: MASS
    // @DisplayName: mass
    // @Description: mass of SA_GD2000
    // @Units: kg
    AP_GROUPINFO("MASS",     1, SA_GD2000,  params.mass, 5), // 907kg = 2000lbs

    // @Param: ALT
    // @DisplayName: launch alt MSL
    // @Description: launch alt MSL
    // @Units: m
    AP_GROUPINFO("ALT",     2, SA_GD2000,  params.launch_alt, 3810), // 3810m == 125000 ft

    AP_GROUPEND
};

SA_GD2000::SA_GD2000(const char *frame_str) :
    Plane(frame_str)
{
    AP_Param::load_defaults_file("@ROMFS/models/sa_gd2000.parm", false);
    AP_Param::load_defaults_file("@ROMFS/models/sa_gd2000_SITL.parm", false);
    AP::sitl()->models.sa_gd2000_ptr = this;
    AP_Param::setup_object_defaults(this, var_info);


    mass = params.mass.get();
    thrust_scale = 0;

    coefficient.c_drag_p = 0.05;

}

/*
  update the vehicle simulation by one time step
 */
void SA_GD2000::update(const struct sitl_input &input)
{
    if (!has_launched) {
        // we're in a pre-launch state cruising in the launch vehicle (like a C-130)
        position.z = -1 * params.launch_alt; // 3810 == 12500ft 
        velocity_ef.x = 10;

        if (hal.util->get_soft_armed()) {
            has_launched = true;
        }
    }

    Plane::update(input);

    // constrain accelerations
    accel_body.x = constrain_float(accel_body.x, -16*GRAVITY_MSS, 16*GRAVITY_MSS);
    accel_body.y = constrain_float(accel_body.y, -16*GRAVITY_MSS, 16*GRAVITY_MSS);
    accel_body.z = constrain_float(accel_body.z, -16*GRAVITY_MSS, 16*GRAVITY_MSS);
}


#endif // AP_SIM_SA_GD2000_ENABLED
