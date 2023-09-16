/*
  CINS state estimator for AP_AHRS
 */

#include "AP_CINS.h"
#include <AP_AHRS/AP_AHRS.h>
#include <AP_DAL/AP_DAL.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Declination/AP_Declination.h>
#include <GCS_MAVLink/GCS.h>

// gains tested for 5Hz GPS
#define CINS_GAIN_P (-0.01*5)
#define CINS_GAIN_L1 (0.99*5)
#define CINS_GAIN_L2 (0.8*5)

#define CINS_MAG_GAIN 0.1

#define CINS_INITIAL_ROLL 0
#define CINS_INITIAL_PITCH 0
#define CINS_INITIAL_YAW 0


/*
  initialise the filter
 */
void AP_CINS::init(void)
{
    //Initialise XHat and ZHat as stationary at the origin
    state.XHat = SE23f(Matrix3f(1,0,0,0,1,0,0,0,1),Vector3f(0,0,0),Vector3f(0,0,0),0.0f);
    state.ZHat = SE23f(Matrix3f(1,0,0,0,1,0,0,0,1),Vector3f(0,0,0),Vector3f(0,0,0),0.0f);

    state.rotation_matrix.from_euler(radians(CINS_INITIAL_ROLL),radians(CINS_INITIAL_PITCH),radians(CINS_INITIAL_YAW));
    state.XHat.init(state.rotation_matrix);
    
    correction_terms.gain_p = CINS_GAIN_P;
    correction_terms.gain_l1 = CINS_GAIN_L1;
    correction_terms.gain_l2 = CINS_GAIN_L2;
}

/*
  update function, called at loop rate
 */
void AP_CINS::update(void)
{
    auto &dal = AP::dal();

    if (!done_yaw_init) {
        done_yaw_init = init_yaw();
    }

    const auto &ins = dal.ins();

    // get delta angle
    const uint8_t gyro_index = ins.get_primary_gyro();
    Vector3f delta_angle;
    float dangle_dt;
    if (!ins.get_delta_angle(gyro_index, delta_angle, dangle_dt) || dangle_dt <= 0) {
        // can't update, no delta angle
        return;
    }
    // turn delta angle into a gyro in radians/sec
    Vector3f gyro = delta_angle / dangle_dt;

    // get delta velocity
    Vector3f delta_velocity;
    float dvel_dt;
    if (!ins.get_delta_velocity(gyro_index, delta_velocity, dvel_dt) || dvel_dt <= 0) {
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
            // don't allow for large gain if we lose and regain GPS
            const float gps_dt = MIN((last_gps_fix_ms - last_gps_update_ms)*0.001, 1);
            last_gps_update_ms = last_gps_fix_ms;
            const auto &loc = gps.location();
            if (!state.have_origin) {
                state.have_origin = true;
                state.origin = loc;
            }
            const Vector3f pos = state.origin.get_distance_NED(loc);
            update_gps(pos, gps_dt);
        }
    }

    update_imu(gyro, accel, dangle_dt);

    update_yaw_from_compass();
    
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
                                wrap_360(degrees(yaw_rad)),
                                state.velocity_NED.x,
                                state.velocity_NED.y,
                                state.velocity_NED.z,
                                state.position.x,
                                state.position.y,
                                state.position.z,
                                state.location.lat,
                                state.location.lng,
                                state.location.alt*0.01);

    state.accel = accel;
    state.gyro = gyro;
}


/*
  update on new GPS sample
 */
void AP_CINS::update_gps(const Vector3f &pos, const float gps_dt)
{
    //Jump set update 
    //compute correction terms 
    update_correction_terms(pos, gps_dt);

    //Update XHat and ZHat using correciton terms
    state.XHat = correction_terms.Delta * state.XHat;
    state.ZHat = state.ZHat * correction_terms.Gamma;

    //Update position estimate
    state.rotation_matrix = state.XHat.rot();
    state.position = state.XHat.x();
    state.velocity_NED = state.XHat.w();

    // use AHRS3 for debugging
    float roll_rad, pitch_rad, yaw_rad;
    const auto rot = state.rotation_matrix * AP::ahrs().get_rotation_vehicle_body_to_autopilot_body();
    rot.to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    const mavlink_ahrs3_t pkt {
      roll : roll_rad,
      pitch : pitch_rad,
      yaw : yaw_rad,
      altitude : -state.position.z,
      lat: state.location.lat,
      lng: state.location.lng,
      v1 : 0,
      v2 : 0,
      v3 : 0,
      v4 : 0
    };
    gcs().send_to_active_channels(MAVLINK_MSG_ID_AHRS3, (const char *)&pkt);
}


/*
  update from IMU data
 */
void AP_CINS::update_imu(const Vector3f &gyro_rads, const Vector3f &accel_mss, const float dt)
{
    /*
    Normal time update, based of the CINS Disc alg developed by Dr Pieter Van goor and Pat Wiltshire  
    */
    //Integrate Dynamics using the Matrix exponential 
    //Create Zero vector 
    Vector3f zero_vector;
    zero_vector.zero();
    Vector3f gravity_vector;
    gravity_vector = Vector3f(0,0,GRAVITY_MSS);

    //Update XHat (Observer Dynamics)
    state.XHat = (SE23::exponential(zero_vector, zero_vector, gravity_vector*dt, -dt)*state.XHat) * SE23::exponential(gyro_rads*dt, zero_vector, accel_mss*dt, dt);
    //Update ZHat (Auxilary Dynamics)
    state.ZHat = SE23::exponential(zero_vector, zero_vector, gravity_vector*dt, -dt) * state.ZHat;

    //Return states 
    state.rotation_matrix = state.XHat.rot(); 
    state.velocity_NED = state.XHat.w();
    state.position = state.XHat.x();
}


/*
Compute correction terms for the jump set. This will be called in update GPS. It will generate a 
Delta and Gamma which is then applied to the dynamics. This allows the update_gps to output, rotation_matrix
velocity_NED and position. 
*/

void AP_CINS::update_correction_terms(const Vector3f &pos, const float gps_dt)
{
    //Collect position estimate from XHat and ZHat
    Vector3f pos_est = state.XHat.x();
    Vector3f pos_z = state.ZHat.x();
    //Create Components of correction terms 
    Vector3f pos_tz = pos - pos_z;
    Vector3f pos_ez = pos_est - pos_z;
    Vector3f pos_te = pos - pos_est;

    //Gamma 
    Matrix3f R_gamma;
    R_gamma.identity();
    Vector3f V_gamma_1 = pos_tz * correction_terms.gain_l1*gps_dt;
    Vector3f V_gamma_2 = pos_tz * correction_terms.gain_l2*gps_dt;
    float alpha_gamma = -1*state.ZHat.alpha();
    correction_terms.Gamma = SE23f(R_gamma, V_gamma_1, V_gamma_2, alpha_gamma);

    //Delta
    //R_delta is incorrect, why? TODO
    Matrix3f R_delta;
    Vector3f maucross_mauhat = Matrix3f::skew_symmetric(pos_te) * pos_ez;
    float mautrans_mauhat = pos_tz * pos_ez; //Dotproduct 
    float norm_maucross_mauhat = maucross_mauhat.length();
    //Check is not too small as would lead to too large of a correction, if so dont apply correction 
    if (fabsF(norm_maucross_mauhat) > 0.00001f){
        float psi = atan2F(norm_maucross_mauhat, mautrans_mauhat);
        R_delta = Matrix3f::from_angular_velocity(maucross_mauhat * ((correction_terms.gain_p*psi*gps_dt )/norm_maucross_mauhat));
    }
    else {
        R_delta.identity();
    }
    //Create V_delta
    //prelimaries
    Vector3f Q = R_delta * pos_ez;
    Vector3f Q_1 = Q * correction_terms.gain_l1*gps_dt;
    Vector3f Q_2 = Q * correction_terms.gain_l2*gps_dt;
    //Calculate components of V_delta
    Vector3f V_delta_1 = V_gamma_1 - Q_1 -(V_gamma_2 - Q_2)*alpha_gamma;
    Vector3f V_delta_2 = V_gamma_2 - Q_2;
    //Assign Delta
    SE23f Delta = SE23f(R_delta, V_delta_1, V_delta_2, 0.0f);
    //Rebase delta
    correction_terms.Delta = (state.ZHat * Delta) * SE23::inverse_ZHat(state.ZHat);
}

/*
  initialise yaw from compass, if available
 */
bool AP_CINS::init_yaw(void)
{
    float mag_yaw, dt;
    if (!get_compass_yaw(mag_yaw, dt)) {
        return false;
    }
    float roll_rad, pitch_rad, yaw_rad;
    state.rotation_matrix.to_euler(&roll_rad, &pitch_rad, &yaw_rad);

    state.rotation_matrix.from_euler(roll_rad,pitch_rad,mag_yaw);
    state.XHat.init(state.rotation_matrix);

    return true;
}

/*
  get yaw from compass
 */
bool AP_CINS::get_compass_yaw(float &yaw_rad, float &dt)
{
    auto &dal = AP::dal();
    const auto &compass = dal.compass();
    if (compass.get_num_enabled() == 0) {
        return false;
    }
    const uint8_t mag_idx = compass.get_first_usable();
    if (!compass.healthy(mag_idx)) {
        return false;
    }
    if (!state.have_origin) {
        return false;
    }
    const auto &field = compass.get_field(mag_idx);
    const uint32_t last_us = compass.last_update_usec(mag_idx);
    if (last_us == last_mag_us) {
        // no new data
        return false;
    }
    dt = (last_us - last_mag_us) * 1.0e-6;
    last_mag_us = last_us;

    const float declination_deg = AP_Declination::get_declination(state.location.lat*1.0e-7, state.location.lng*1.0e-7);
    if (is_zero(declination_deg)) {
        // wait for declination
        return false;
    }

    const float cos_pitch_sq = 1.0f-(state.rotation_matrix.c.x*state.rotation_matrix.c.x);
    const float headY = field.y * state.rotation_matrix.c.z - field.z * state.rotation_matrix.c.y;

    // Tilt compensated magnetic field X component:
    const float headX = field.x * cos_pitch_sq - state.rotation_matrix.c.x * (field.y * state.rotation_matrix.c.y + field.z * state.rotation_matrix.c.z);

    // return magnetic yaw
    yaw_rad = wrap_PI(atan2f(-headY,headX) + radians(declination_deg));

    return true;
}

/*
  update yaw from compass
 */
void AP_CINS::update_yaw_from_compass(void)
{
    float mag_yaw, dt;
    if (!get_compass_yaw(mag_yaw, dt)) {
        return;
    }
    float roll_rad, pitch_rad, yaw_rad;
    state.rotation_matrix.to_euler(&roll_rad, &pitch_rad, &yaw_rad);

    const float yaw_err = wrap_PI(mag_yaw - yaw_rad);
    const Vector3f yaw_rot_ef{0,0,yaw_err*dt*CINS_MAG_GAIN};
    const Vector3f yaw_rot_bf = state.rotation_matrix.mul_transpose(yaw_rot_ef);

    state.rotation_matrix.rotate(yaw_rot_bf);
    state.XHat.init(state.rotation_matrix);
}
