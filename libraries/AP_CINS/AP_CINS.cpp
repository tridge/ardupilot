/*
  CINS state estimator for AP_AHRS
 */

#include "AP_CINS.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_DAL/AP_DAL.h>
#include <AP_Logger/AP_Logger.h>

/*
  initialise the filter
 */
void AP_CINS::init(void)
{
    state.rotation_matrix.from_euler(0,0,0);
}

/*
  update function, called at loop rate
 */
void AP_CINS::update(void)
{
    auto &dal = AP::dal();

    // get delta angle
    const uint8_t gyro_index = dal.ins().get_primary_gyro();
    Vector3f delta_angle;
    float dangle_dt;
    if (!dal.ins().get_delta_angle(gyro_index, delta_angle, dangle_dt) || dangle_dt <= 0) {
        // can't update, no delta angle
        return;
    }
    // turn delta angle into a gyro in radians/sec
    Vector3f gyro = delta_angle / dangle_dt;

    // get delta velocity
    Vector3f delta_velocity;
    float dvel_dt;
    if (!dal.ins().get_delta_velocity(gyro_index, delta_velocity, dvel_dt) || dvel_dt <= 0) {
        // can't update, no delta velocity
        return;
    }
    // turn delta velocity into a accel vector
    const Vector3f accel = delta_velocity / dvel_dt;

    // see if we have new GPS data
    const auto &gps = dal.gps();
    if (gps.status() >= AP_DAL_GPS::GPS_OK_FIX_3D) {
        const uint32_t last_gps_fix_ms = gps.last_message_time_ms(0);
        if (last_gps_update_ms != last_gps_fix_ms) {
            last_gps_update_ms = last_gps_fix_ms;
            const auto &loc = gps.location();
            if (!state.have_origin) {
                state.have_origin = true;
                state.origin = loc;
            }
            const Vector3f pos = state.origin.get_distance_NED(loc);
            update_gps(pos);
        }
    }

    update_imu(gyro, accel, dangle_dt);

    if (state.have_origin) {
        // fill in location
        state.location = state.origin;
        state.location.offset(state.position.x, state.position.y);
        state.location.alt -= state.position.z * 100;
    }

    // @LoggerMessage: CINS
    // @Description: CINS state
    // @Field: TimeUS: Time since system startup
    // @Field: Roll: euler roll
    // @Field: Pitch: euler pitch
    // @Field: Yaw: euler yaw
    // @Field: VN: velocity north
    // @Field: VE: velocity east
    // @Field: VD: velocity down
    // @Field: PN: position north
    // @Field: PE: position east
    // @Field: PD: position down
    // @Field: Lat: latitude
    // @Field: Lon: longitude
    // @Field: Alt: altitude AMSL
    float roll_rad, pitch_rad, yaw_rad;
    state.rotation_matrix.to_euler(&roll_rad, &pitch_rad, &yaw_rad);

    AP::logger().WriteStreaming("CINS", "TimeUS,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD,Lat,Lon,Alt",
                                "sdddnnnmmmDUm",
                                "F000000000GG0",
                                "QfffffffffLLf",
                                AP_HAL::micros64(),
                                degrees(roll_rad),
                                degrees(pitch_rad),
                                degrees(yaw_rad),
                                state.velocity_NED.x,
                                state.velocity_NED.y,
                                state.velocity_NED.z,
                                state.position.x,
                                state.position.y,
                                state.position.z,
                                state.location.lat,
                                state.location.lng,
                                state.location.alt*0.01);


}

/*
  update on new GPS sample
 */
void AP_CINS::update_gps(const Vector3f &pos)
{
    /*
      REPLACE: update position estimate properly
     */
    state.position = pos;
}


/*
  update from IMU data
 */
void AP_CINS::update_imu(const Vector3f &gyro_rads, const Vector3f &accel_mss, const float dt)
{
    // rotate state estimate
    state.rotation_matrix.rotate(gyro_rads * dt);

    // rotate accel vector into earth frame
    Vector3f accel_ef = state.rotation_matrix * accel_mss;
    accel_ef.z += GRAVITY_MSS;

    state.velocity_NED += accel_ef * dt;
    state.position += state.velocity_NED * dt;
}
