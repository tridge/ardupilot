/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

/*
  Silent Arrow CLS-200 glider simulator.

  This started with the simple aerodynamic model in SIM_Plane.cpp.  The
  dimensions, mass and drag polar are changed for the CLS-200, and the
  four canard/rear surfaces are decoded using the flight aircraft's
  servo directions.
*/

#include "SIM_SA.h"

#include <AP_HAL/AP_HAL.h>
#include <AP_Mission/AP_Mission.h>
#include <SRV_Channel/SRV_Channel.h>

using namespace SITL;

extern const AP_HAL::HAL& hal;

static const struct {
    const char *name;
    float value;
} sim_defaults[] = {
    { "SIM_SA_DROP_LAT", 0 },
    { "SIM_SA_DROP_LON", 0 },
    { "SIM_SA_DROP_ALT", 0 },
    { "SIM_SA_DROP_SPD", 63 },
    { "SIM_SA_DROP_HDG", 138 },
    { "SIM_SA_GLD_RATIO", 5 },
    { "AHRS_EKF_TYPE", 10 },
    { "EK2_ENABLE", 0 },
    { "EK3_ENABLE", 0 },
    { "ARMING_CHECK", 0 },
    { "ARMING_REQUIRE", 1 },
    { "ARSPD_TYPE", 2 },
    { "ARSPD_USE", 1 },
    { "ARSPD_OFFSET", 2013 },
    { "ARSPD_SKIP_CAL", 1 },
    { "ARSPD_FBW_MIN", 48 },
    { "ARSPD_FBW_MAX", 72 },
    { "TRIM_ARSPD_CM", 6300 },
    { "SCALING_SPEED", 63 },
    { "STALL_PREVENTION", 0 },
    { "TERRAIN_ENABLE", 0 },
    { "THR_MIN", 0 },
    { "THR_MAX", 1 },
    { "THR_FAILSAFE", 2 },
    { "TRIM_THROTTLE", 0 },
    { "MIXING_GAIN", 1 },
    { "KFF_RDDRMIX", 0.5 },
    { "DSPOILR_RUD_RATE", 100 },
    { "SERVO1_FUNCTION", 17 },
    { "SERVO1_MIN", 1200 },
    { "SERVO1_MAX", 1600 },
    { "SERVO1_TRIM", 1400 },
    { "SERVO1_REVERSED", 1 },
    { "SERVO2_FUNCTION", 86 },
    { "SERVO2_MIN", 1680 },
    { "SERVO2_MAX", 2160 },
    { "SERVO2_TRIM", 1880 },
    { "SERVO2_REVERSED", 1 },
    { "SERVO3_FUNCTION", 16 },
    { "SERVO3_MIN", 1620 },
    { "SERVO3_MAX", 2160 },
    { "SERVO3_TRIM", 1860 },
    { "SERVO3_REVERSED", 0 },
    { "SERVO4_FUNCTION", 87 },
    { "SERVO4_MIN", 1240 },
    { "SERVO4_MAX", 1660 },
    { "SERVO4_TRIM", 1440 },
    { "SERVO4_REVERSED", 0 },
    { "PTCH2SRV_D", 0.3 },
    { "PTCH2SRV_FF", 0.414 },
    { "PTCH2SRV_I", 0.4 },
    { "PTCH2SRV_P", 0 },
    { "PTCH2SRV_RLL", 0.5 },
    { "PTCH2SRV_RMAX_DN", 40 },
    { "PTCH2SRV_RMAX_UP", 40 },
    { "PTCH2SRV_TCONST", 1 },
    { "RLL2SRV_D", 0.5 },
    { "RLL2SRV_FF", 0.32 },
    { "RLL2SRV_I", 0.18 },
    { "RLL2SRV_P", 0 },
    { "RLL2SRV_RMAX", 40 },
    { "RLL2SRV_TCONST", 1.2 },
    { "LIM_ROLL_CD", 4500 },
    { "LIM_PITCH_MIN", -2000 },
    { "LIM_PITCH_MAX", 3000 },
    { "NAVL1_DAMPING", 0.75 },
    { "NAVL1_PERIOD", 20 },
    { "WP_RADIUS", 750 },
    { "WP_LOITER_RAD", 300 },
    { "TECS_CLMB_MAX", 5 },
    { "TECS_LAND_ARSPD", 52 },
    { "TECS_OPTIONS", 1 },
    { "TECS_PITCH_MAX", -1 },
    { "TECS_PITCH_MIN", -12 },
    { "TECS_PTCH_DAMP", 0.3 },
    { "TECS_SINK_MAX", 15 },
    { "TECS_SINK_MIN", 10 },
    { "TECS_SPDWEIGHT", 2 },
    { "TECS_TIME_CONST", 7 },
    { "BTN_ENABLE", 1 },
    { "BTN_PIN1", 5 },
    { "SIM_PIN_MASK", 0 },
    { "SCR_ENABLE", 1 },
    { "SCR_HEAP_SIZE", 100000 },
    { "SCR_VM_I_COUNT", 120000 },
    { "SCR_USER1", 0.5 },
    { "SCR_USER2", 5 },
    { "LOG_DISARMED", 1 },
};

const AP_Param::GroupInfo SA::var_info[] = {
    // @Param: DROP_LAT
    // @DisplayName: Drop latitude
    // @Description: Latitude at which the glider is held while disarmed. When both latitude and longitude are zero the first DO_LAND_START in the mission is used, falling back to SITL home
    // @Units: deg
    AP_GROUPINFO("DROP_LAT", 1, SA, drop_lat, 0.0f),

    // @Param: DROP_LON
    // @DisplayName: Drop longitude
    // @Description: Longitude at which the glider is held while disarmed. When both latitude and longitude are zero the first DO_LAND_START in the mission is used, falling back to SITL home
    // @Units: deg
    AP_GROUPINFO("DROP_LON", 2, SA, drop_lon, 0.0f),

    // @Param: DROP_ALT
    // @DisplayName: Drop altitude
    // @Description: AMSL altitude at which the glider is held while disarmed. Zero uses the mission DO_LAND_START altitude when available, otherwise SITL home altitude
    // @Units: m
    AP_GROUPINFO("DROP_ALT", 3, SA, drop_alt, 0.0f),

    // @Param: DROP_SPD
    // @DisplayName: Drop airspeed
    // @Description: Airspeed shown while the glider is held before release
    // @Units: m/s
    AP_GROUPINFO("DROP_SPD", 4, SA, drop_spd, 63.0f),

    // @Param: DROP_HDG
    // @DisplayName: Drop heading
    // @Description: True heading shown while the glider is held before release
    // @Units: deg
    AP_GROUPINFO("DROP_HDG", 5, SA, drop_hdg, 138.0f),

    // @Param: GLD_RATIO
    // @DisplayName: Best lift to drag ratio
    // @Description: Lift to drag ratio at the 63m/s best glide airspeed
    AP_GROUPINFO("GLD_RATIO", 6, SA, glide_ratio, 5.0f),

    AP_GROUPEND
};

namespace {

constexpr float cls200_mass = 907.0f;       // 2000 lb class
constexpr float wing_area = 10.0f;          // m^2
constexpr float wing_span = 8.0f;           // m
constexpr float mean_chord = 1.25f;         // m
constexpr float air_density = 1.225f;       // kg/m^3, sea-level ISA
constexpr float best_glide_speed = 63.0f;   // m/s
constexpr float stall_speed = 40.0f;        // m/s

// Approximate rigid body moments for the folded-wing cargo glider.
constexpr float roll_inertia = 2500.0f;     // kg m^2
constexpr float pitch_inertia = 3500.0f;    // kg m^2
constexpr float yaw_inertia = 5000.0f;      // kg m^2

// The glider lands on skids rather than rolling freely on wheels.
constexpr float ground_friction_accel = 8.0f;       // m/s^2
constexpr float ground_drag_rate = 0.5f;            // 1/s
constexpr float max_ground_decel = 3.0f * GRAVITY_MSS;

// SITL uses a pin representable in the 16-bit SIM_PIN_MASK.
constexpr uint8_t release_gpio = 5;

}

SA::SA(const char *frame_str) :
    Aircraft(frame_str),
    last_armed(false),
    ever_armed(false),
    late_defaults_applied(false),
    mission_drop_valid(false),
    last_mission_check_ms(0)
{
    AP_Param::setup_object_defaults(this, var_info);
    apply_default_parameters();
    sitl->sa_ptr = this;

    mass = cls200_mass;
    frame_height = 0.5f;
    num_motors = 1;
    ground_behavior = GROUND_BEHAVIOR_FWD_ONLY;

    // Carrier interface holds the flight release input low before drop.
    sitl->pin_mask.set(sitl->pin_mask.get() & ~(1U << release_gpio));
}

void SA::apply_default_parameters()
{
    for (uint8_t i = 0; i < ARRAY_SIZE(sim_defaults); i++) {
        AP_Param::set_default_by_name(sim_defaults[i].name,
                                      sim_defaults[i].value);
    }
}

/*
  The carrier release input is wired to BTN_PIN1 on the real aircraft.
  Treat an external SITL arming transition as the carrier release event,
  and drive the same GPIO so droptest.lua follows its flight-hardware path.
*/
void SA::update_release_gpio(bool armed)
{
    const uint16_t mask = static_cast<uint16_t>(sitl->pin_mask.get());
    const uint16_t release_mask = 1U << release_gpio;
    const uint16_t new_mask = armed ? mask | release_mask : mask & ~release_mask;

    // AP_Button periodically reapplies its input pull-up, so enforce the
    // carrier-driven state as well as responding to an arming transition.
    if (armed != last_armed || new_mask != mask) {
        sitl->pin_mask.set_and_notify(new_mask);
        last_armed = armed;
    }
}

/*
  When no explicit horizontal drop position is configured, use the first
  DO_LAND_START in the loaded mission. This lets a GCS completely configure
  the simulated carrier location by loading a mission.
*/
void SA::update_drop_from_mission()
{
    if (!is_zero(drop_lat.get()) || !is_zero(drop_lon.get())) {
        mission_drop_valid = false;
        return;
    }

    const uint32_t now = AP_HAL::millis();
    if (now - last_mission_check_ms < 1000) {
        return;
    }
    last_mission_check_ms = now;
    mission_drop_valid = false;

    AP_Mission *mission = AP::mission();
    if (mission == nullptr) {
        return;
    }

    AP_Mission::Mission_Command cmd;
    uint16_t index;
    if (!mission->find_command(MAV_CMD_DO_LAND_START, 0, index, cmd)) {
        return;
    }

    Location drop = cmd.content.location;
    if (!drop.change_alt_frame(Location::AltFrame::ABSOLUTE)) {
        return;
    }
    mission_drop = drop;
    mission_drop_valid = true;
}

/*
  Convert a PWM output to physical surface travel.  The ranges and
  direction arguments below mirror the flight SERVO1..SERVO4 setup.
  Keeping this in the model makes a mistaken reversal visible in SITL.
*/
float SA::surface_input(const struct sitl_input &input, uint8_t idx,
                        uint16_t minimum, uint16_t trim,
                        uint16_t maximum, float direction)
{
    const float pwm = input.servos[idx];
    float travel;
    if (pwm >= trim) {
        travel = (pwm - trim) / (maximum - trim);
    } else {
        travel = (pwm - trim) / (trim - minimum);
    }
    travel = direction * constrain_float(travel, -1.0f, 1.0f);
    return filtered_idx(travel, idx);
}

float SA::lift_coeff(float alpha) const
{
    const float dynamic_pressure = 0.5f * air_density * sq(best_glide_speed);
    const float cl_best = cls200_mass * GRAVITY_MSS /
                          (dynamic_pressure * wing_area);
    const float cl_max = cl_best * sq(best_glide_speed / stall_speed);
    const float linear_lift = cl_best + 4.5f * alpha;

    // The maximum coefficient gives a 40m/s one-g stall speed.
    return constrain_float(linear_lift, -cl_max, cl_max);
}

float SA::drag_coeff(float lift) const
{
    const float dynamic_pressure = 0.5f * air_density * sq(best_glide_speed);
    const float cl_best = cls200_mass * GRAVITY_MSS /
                          (dynamic_pressure * wing_area);
    const float ld = MAX(glide_ratio.get(), 1.0f);

    // A parabolic polar whose maximum L/D is GLD_RATIO at CL=cl_best.
    const float cd0 = cl_best / (2.0f * ld);
    const float induced_drag = 1.0f / (2.0f * ld * cl_best);
    return cd0 + induced_drag * sq(lift);
}

Vector3f SA::get_force(float aileron, float elevator) const
{
    if (is_zero(airspeed)) {
        return Vector3f();
    }

    const float lift = lift_coeff(angle_of_attack);
    const float drag = drag_coeff(lift) + 0.02f * fabsf(elevator);
    const float cx = -drag * cosf(angle_of_attack) +
                     lift * sinf(angle_of_attack);
    const float cz = -drag * sinf(angle_of_attack) -
                     lift * cosf(angle_of_attack);

    const float p = gyro.x;
    const float r = gyro.z;
    const float cy = -0.98f * beta +
                     0.10f * aileron -
                     0.20f * wing_span * r / (2.0f * airspeed) +
                     0.05f * wing_span * p / (2.0f * airspeed);
    const float qbar_area = 0.5f * air_density * sq(airspeed) * wing_area;
    return Vector3f(cx, cy, cz) * qbar_area;
}

Vector3f SA::get_rot_accel(float aileron, float elevator) const
{
    if (is_zero(airspeed)) {
        return Vector3f();
    }

    const float p = gyro.x;
    const float q = gyro.y;
    const float r = gyro.z;
    const float qbar_area = 0.5f * air_density * sq(airspeed) * wing_area;

    const float roll_moment = qbar_area * wing_span *
        (-0.12f * beta -
         1.0f * wing_span * p / (2.0f * airspeed) +
         0.14f * wing_span * r / (2.0f * airspeed) +
         0.08f * aileron);
    const float pitch_moment = qbar_area * mean_chord *
        (-0.70f * angle_of_attack -
         20.0f * mean_chord * q / (2.0f * airspeed) +
         elevator);
    const float yaw_moment = qbar_area * wing_span *
        (0.25f * beta +
         0.022f * wing_span * p / (2.0f * airspeed) -
         1.0f * wing_span * r / (2.0f * airspeed) +
         0.02f * aileron);

    return Vector3f(roll_moment / roll_inertia,
                    pitch_moment / pitch_inertia,
                    yaw_moment / yaw_inertia);
}

void SA::calculate_forces(const struct sitl_input &input,
                          Vector3f &rot_accel,
                          Vector3f &body_accel)
{
    // Flight functions: S1/S3 are the front right/left canards and
    // S2/S4 are the rear left/right surfaces.
    const float front_right = surface_input(input, 0, 1200, 1400, 1600, -1.0f);
    const float rear_left   = surface_input(input, 1, 1680, 1880, 2160, -1.0f);
    const float front_left  = surface_input(input, 2, 1620, 1860, 2160, 1.0f);
    const float rear_right  = surface_input(input, 3, 1240, 1440, 1660, 1.0f);

    const float elevator = 0.25f *
        (front_right + rear_left + front_left + rear_right);
    const float aileron = 0.25f *
        (front_left + rear_right - front_right - rear_left);

    angle_of_attack = atan2f(velocity_air_bf.z, velocity_air_bf.x);
    beta = atan2f(velocity_air_bf.y, velocity_air_bf.x);

    body_accel = get_force(aileron, elevator) / mass;
    rot_accel = get_rot_accel(aileron, elevator);

    if (on_ground()) {
        const Vector3f velocity_body = dcm.transposed() * velocity_ef;
        const float forward_speed = MAX(velocity_body.x, 0.0f);
        if (forward_speed > 0.0f) {
            const float delta_time = frame_time_us * 1.0e-6f;
            const float friction = MIN(max_ground_decel,
                                       ground_friction_accel +
                                       ground_drag_rate * forward_speed);
            // Limit the final step so friction alone cannot reverse the
            // aircraft as it comes to rest.
            body_accel.x -= MIN(friction, forward_speed / delta_time);
        }
    }

    rpm[0] = 0.0f;
    battery_voltage = sitl->batt_voltage;
    battery_current = 0.0f;
}

void SA::set_drop_state(const struct sitl_input &input)
{
    update_drop_from_mission();

    Location drop = mission_drop_valid ? mission_drop : home;
    if (!is_zero(drop_lat.get())) {
        drop.lat = drop_lat.get() * 1.0e7f;
    }
    if (!is_zero(drop_lon.get())) {
        drop.lng = drop_lon.get() * 1.0e7f;
    }
    if (!is_zero(drop_alt.get())) {
        drop.alt = drop_alt.get() * 100.0f;
    }

    const float heading = radians(wrap_360(drop_hdg.get()));
    const float speed = MAX(drop_spd.get(), 0.0f);

    location = drop;
    position = home.get_distance_NED(location);
    dcm.from_euler(0.0f, 0.0f, heading);
    gyro.zero();
    velocity_ef = Vector3f(speed * cosf(heading), speed * sinf(heading), 0.0f);

    update_wind(input);
    velocity_air_ef = velocity_ef + wind_ef;
    velocity_air_bf = dcm.transposed() * velocity_air_ef;
    airspeed = velocity_air_ef.length();
    airspeed_pitot = MAX(velocity_air_bf.x, 0.0f);
    accel_body = Vector3f(0.0f, 0.0f, -GRAVITY_MSS);
    rpm[0] = 0.0f;
    battery_voltage = sitl->batt_voltage;
    battery_current = 0.0f;
}

void SA::update(const struct sitl_input &input)
{
    const uint32_t now = AP_HAL::millis();
    if (!late_defaults_applied && now >= 1000) {
        // Some Plane defaults are applied after the simulator is constructed
        // in this old tree. Apply the model defaults once more after setup;
        // configured user values remain authoritative.
        apply_default_parameters();

        // Plane initially configures channel 3 as a throttle output. On a
        // wiped first boot that leaves its cached output type as a 0..100
        // range even after SERVO3_FUNCTION becomes DspoilerLeft1. Restore
        // angular scaling for all four surfaces and rebuild the function map.
        SRV_Channels::set_angle(SRV_Channel::k_dspoilerLeft1, 4500);
        SRV_Channels::set_angle(SRV_Channel::k_dspoilerLeft2, 4500);
        SRV_Channels::set_angle(SRV_Channel::k_dspoilerRight1, 4500);
        SRV_Channels::set_angle(SRV_Channel::k_dspoilerRight2, 4500);
        SRV_Channels::update_aux_servo_function();
        late_defaults_applied = true;
    }

    const bool armed = hal.util->get_soft_armed();
    update_release_gpio(armed);
    if (armed) {
        ever_armed = true;
    }

    if (!armed && !ever_armed) {
        // Report carrier speed and heading, but do not integrate position.
        set_drop_state(input);
        update_external_payload(input);
        time_advance();
        update_mag_field_bf();
        return;
    }

    update_wind(input);

    Vector3f rot_accel;
    calculate_forces(input, rot_accel, accel_body);
    update_dynamics(rot_accel);
    update_external_payload(input);
    update_position();
    time_advance();
    update_mag_field_bf();
}
