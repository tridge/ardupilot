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
  simple quadplane simulator class
*/

#include "SIM_QuadPlane.h"

#include <stdio.h>

using namespace SITL;

QuadPlane::QuadPlane(const char *frame_str) :
    Plane(frame_str)
{
    // default to X frame
    const char *frame_type = "x";
    uint8_t motor_offset = 4;

    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;

    if (strstr(frame_str, "-octa-quad-cor")) {
        frame_type = "octa-quad-cor";
    } else if (strstr(frame_str, "-octa-quad-cw-cor")) {
        frame_type = "octa-quad-cw-cor";
    } else if (strstr(frame_str, "-octa-quad")) {
        frame_type = "octa-quad";
    } else if (strstr(frame_str, "-octaquad")) {
        frame_type = "octa-quad";
    } else if (strstr(frame_str, "-octa")) {
        frame_type = "octa";
    } else if (strstr(frame_str, "-hexax")) {
        frame_type = "hexax";
    } else if (strstr(frame_str, "-hexa")) {
        frame_type = "hexa";
    } else if (strstr(frame_str, "-plus")) {
        frame_type = "+";
    } else if (strstr(frame_str, "-y6")) {
        frame_type = "y6";
    } else if (strstr(frame_str, "-tri")) {
        frame_type = "tri";
    } else if (strstr(frame_str, "-tilttrivec")) {
        frame_type = "tilttrivec";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "-tilthvec")) {
        frame_type = "tilthvec";
    } else if (strstr(frame_str, "-tilttri")) {
        frame_type = "tilttri";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "firefly")) {
        frame_type = "firefly";
        // elevon style surfaces
        elevons = true;
        // fwd motor gives zero thrust
        thrust_scale = 0;
        // vtol motors start at 2
        motor_offset = 2;
    } else if (strstr(frame_str, "-tilt")) {
        frame_type = "tilt";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "cl84")) {
        frame_type = "tilttri";
        // fwd motor gives zero thrust
        thrust_scale = 0;
    } else if (strstr(frame_str, "-copter_tailsitter")) {
        frame_type = "+";
        copter_tailsitter = true;
        ground_behavior = GROUND_BEHAVIOR_TAILSITTER;
        thrust_scale *= 1.5;
    }
    frame = Frame::create_frame(frame_type);
    if (frame == nullptr) {
        printf("Failed to find frame '%s' or insufficient memory\n", frame_type);
        exit(1);
    }

    if (strstr(frame_str, "cl84")) {
        // setup retract servos at front
        frame->motors[0].servo_type = Motor::SERVO_RETRACT;
        frame->motors[0].servo_rate = 7*60.0/90; // 7 seconds to change
        frame->motors[1].servo_type = Motor::SERVO_RETRACT;
        frame->motors[1].servo_rate = 7*60.0/90; // 7 seconds to change
    }
    
    // leave first 4 servos free for plane
    frame->motor_offset = motor_offset;

    // we use zero terminal velocity to let the plane model handle the drag
    frame->init(frame_str, &battery);

    // increase mass for plane components
    mass = frame->get_mass() * 1.5;
    frame->set_mass(mass);


    lock_step_scheduled = true;
}

/*
  update the quadplane simulation by one time step
 */
void QuadPlane::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    // first plane forces
    Vector3f rot_accel;
    calculate_forces(input, rot_accel);

    // now quad forces
    Vector3f quad_rot_accel;
    Vector3f quad_accel_body;

    motor_mask |= ((1U<<frame->num_motors)-1U) << frame->motor_offset;
    frame->calculate_forces(*this, input, quad_rot_accel, quad_accel_body, rpm, false);

    // rotate frames for copter tailsitters
    if (copter_tailsitter) {
        quad_rot_accel.rotate(ROTATION_PITCH_270);
        quad_accel_body.rotate(ROTATION_PITCH_270);
    }

    /*
      forward motor electrical model:
        I = SIM_FWD_THR_A * throttle^SIM_FWD_THR_EXP + SIM_FWD_I_FIXED

      The old model was a hard-coded linear "20A at full throttle" with no
      constant term. That is wrong in two ways that matter when integrating
      energy over a long mission: a real prop is strongly non-linear in
      throttle, and a real aircraft has a fixed avionics/payload load that never
      goes away. Defaults keep the old behaviour; see PhoenixSITL.parm for a
      set fitted to a real aircraft.
     */
    float fwd_current = 0;
    if (!is_zero(thrust_scale)) {
        // frames with thrust_scale 0 (firefly, tilt variants, cl84) have no
        // separate forward motor - servo 3 there is a lift motor, already
        // accounted for by the frame, so it must not be charged twice
        float throttle;
        if (reverse_thrust) {
            throttle = filtered_servo_angle(input, 2);
        } else {
            throttle = filtered_servo_range(input, 2);
        }
        const float fwd_exp = MAX(0.1f, sitl->fwd_thr_exp.get());
        fwd_current = sitl->fwd_thr_A * powf(fabsf(throttle), fwd_exp) + sitl->fwd_i_fixed;
    }

    /*
      Total current must be known BEFORE the battery is updated, otherwise the
      forward motor neither discharges the pack nor causes any voltage sag -
      which in fixed-wing cruise (lift motors off) meant essentially no
      discharge at all.
     */
    frame->current_and_voltage(battery_voltage, battery_current);
    battery_current += fwd_current;
    battery.set_current(battery_current);
    battery_voltage = battery.get_voltage();
    
    rot_accel += quad_rot_accel;
    accel_body += quad_accel_body;

    update_dynamics(rot_accel);
    update_external_payload(input);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}
