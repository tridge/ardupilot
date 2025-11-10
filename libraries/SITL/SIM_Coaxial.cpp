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
  Coaxial simulator class
*/

#include "SIM_Coaxial.h"
#include <SITL/SITL.h>
#include <stdio.h>

using namespace SITL;

Coaxial::Coaxial(const char *frame_str) :
    Aircraft(frame_str)
{
    frame_height = 0.1;
    ground_behavior = GROUND_BEHAVIOR_NO_MOVEMENT;
    battery.setup(AP::sitl()->batt_capacity_ah*AP::sitl()->battery_number, AP::sitl()->batt_res_ohm/AP::sitl()->battery_number, 50.4f);
    battery.init_voltage(AP::sitl()->batt_voltage);
}

/*
  update the copter simulation by one time step
 */
void Coaxial::update(const struct sitl_input &input)
{
    // get wind vector setup
    update_wind(input);

    // Non-dimensional between -1 and 1
    const float roll_non_dim = constrain_float((input.servos[0]-1500) / 500.0f, -1, 1);
    const float pitch_non_dim = constrain_float((input.servos[1]-1500) / 500.0f, -1, 1);

    // Conversion to RPM and store in member variable
    rpm[0] = constrain_float(input.servos[3]-1090,0,1000)*6.3; // From ATP test
    rpm[1] = constrain_float(input.servos[2]-1090,0,1000)*6.3; // From ATP test

    // Non-dimensional
    const float motor1_non_dim = rpm[0]/5000.0f;
    const float motor2_non_dim = rpm[1]/5000.0f;

    // Forces and torques, ATP measures them in Z-up, Y-left
    const float thrust_N = -38.0f*motor1_non_dim*motor1_non_dim - 65.0f*motor2_non_dim*motor2_non_dim;
    const float yaw_torque_Nm = 2.0f*motor1_non_dim*motor1_non_dim - 2.0f*motor2_non_dim*motor2_non_dim;
    const float roll_torque_Nm = -14*motor2_non_dim*motor2_non_dim*roll_non_dim;
    const float pitch_torque_Nm = 14*motor2_non_dim*motor2_non_dim*pitch_non_dim;
    const float power_W = 900.0f*motor1_non_dim*motor1_non_dim*motor1_non_dim 
                          + 1350.0f*motor2_non_dim*motor2_non_dim*motor2_non_dim;

    // Aerodynamic forces
    Vector3f aero_torque(roll_torque_Nm,
                         pitch_torque_Nm,
                         yaw_torque_Nm); 

    aero_torque.x -= gyro.x * aero_damping_constant;
    aero_torque.y -= gyro.y * aero_damping_constant;
    aero_torque.z -= gyro.z * aero_damping_constant;

    float mass_kg = 5.0;
    float length_m = 1.0f;
    if (AP::sitl() != nullptr) {
      mass_kg = AP::sitl()->core_mass_kg + 1.42*AP::sitl()->battery_number + AP::sitl()->payload_weight_kg;
      length_m = 0.30f + AP::sitl()->battery_number*0.12f; // core + batteries
    } 

    const float Izz = 0.5f*mass_kg*radius_m*radius_m + 2*1.04e-3; // Uniformly distributed
    const float Ixx =  (1.0f/4.0f)*mass_kg*radius_m*radius_m + (1.0f/12.0f)*mass_kg*length_m*length_m; // Uniformly distributed

    // Non-inertial effects
    const Vector3f IOmega(Ixx*gyro.x,
                          Ixx*gyro.y,
                          Izz*gyro.z);

    const Vector3f omega_cross_Iomega = gyro % IOmega;

    // Angular accelerations
    const Vector3f rot_accel( (aero_torque.x - omega_cross_Iomega.x)/Ixx,
                              (aero_torque.y - omega_cross_Iomega.y)/Ixx,
                              (aero_torque.z - omega_cross_Iomega.z)/Izz );

    // Drag
    const Vector3f air_resistance = -velocity_air_bf * velocity_air_bf.length() * cda;

    accel_body = Vector3f(0, 0, thrust_N / mass_kg); // ATP thrust/torque is backwards from flight dynamics convention
    accel_body += air_resistance;

    // Initialize battery if changed
    const float param_batt_number = AP::sitl()->battery_number;
    const float param_batt_capacity = AP::sitl()->batt_capacity_ah;
    const float param_batt_voltage = AP::sitl()->batt_voltage;
    const float param_batt_resistance = AP::sitl()->batt_res_ohm;

    if (!is_equal(last_param_batt_number,param_batt_number) || 
        !is_equal(last_param_batt_capacity,param_batt_capacity) ||
        !is_equal(last_param_batt_voltage,param_batt_voltage) ||
        !is_equal(last_param_batt_resistance,param_batt_resistance)) {
          battery.setup(param_batt_capacity*param_batt_number, param_batt_resistance/param_batt_number, max_voltage_V);
          battery.init_voltage(param_batt_voltage);
          last_param_batt_number = param_batt_number;
          last_param_batt_voltage = param_batt_voltage;
          last_param_batt_capacity = param_batt_capacity;
          last_param_batt_resistance = param_batt_resistance;
    }

    const float current_A = power_W/battery.get_voltage() + avionics_draw_A;
    battery.set_current(current_A);

    update_dynamics(rot_accel);

    // update lat/lon/altitude
    update_position();
    time_advance();

    // update magnetic field
    update_mag_field_bf();
}