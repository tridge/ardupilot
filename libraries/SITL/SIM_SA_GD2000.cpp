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
    AP_GROUPINFO("MASS",     1, SA_GD2000,  params.mass, 907), // 907kg = 2000lbs

    // @Param: ALT
    // @DisplayName: launch alt MSL
    // @Description: launch alt MSL
    // @Units: m
    AP_GROUPINFO("ALT",     2, SA_GD2000,  params.launch_alt, 3810), // 3810m == 12500 ft

    // @Param: VEL
    // @DisplayName: launch velocity
    // @Description: launch velocity
    // @Units: m/s
    AP_GROUPINFO("VEL",     3, SA_GD2000,  params.launch_vel, 10),

    // @Param: DIR
    // @DisplayName: launch direction
    // @Description: launch direction
    // @Units: degrees
    AP_GROUPINFO("DIR",     4, SA_GD2000,  params.launch_dir, 0),

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
    dspoilers = true;

    coefficient.c_drag_p = 0.05;
}

/*
  map inputs to mixer for AETR
*/
void SA_GD2000::input_mixer(const struct sitl_input &input, float &aileron, float &elevator, float &throttle, float &rudder)
{
    /*
      For all 4 surfaces, an increase in input causes right roll
       for channels 3 and 4 an increase in input causes pitch up
       for channels 1 and 2 an increase in input causes pitch down
    */
    const float surface_front_right  = filtered_servo_angle(input, 0);
    const float surface_rear_left  = filtered_servo_angle(input, 1);
    const float surface_front_left  = filtered_servo_angle(input, 2);
    const float surface_rear_right  = filtered_servo_angle(input, 3);
    aileron = (surface_front_right + surface_rear_left + surface_front_left + surface_rear_right) * 0.25;
    elevator = (surface_front_left + surface_rear_right - (surface_front_right + surface_rear_left)) * 0.25;
    rudder = 0;
    throttle = 0;
}


/*
  update the vehicle simulation by one time step
 */
void SA_GD2000::update(const struct sitl_input &input)
{
    if (!has_launched) {
        // we're in a pre-launch state cruising in the launch vehicle (like a C-130)
        position.z = -1 * params.launch_alt; // 3810 == 12500ft 
        velocity_ef.x = cos(radians(params.launch_dir)) * params.launch_vel;
        velocity_ef.y = sin(radians(params.launch_dir)) * params.launch_vel;

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
