/*
  CINS state estimator for AP_AHRS, devloped by Mr Patrick Wiltshire and Dr Pieter Van Goor 
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

// Gains for new CINS method
#define CINS_GAIN_GPS_POS (1.0)
#define CINS_GAIN_MAG (1.0)
#define CINS_GAIN_Q11 (0.1)
#define CINS_GAIN_Q22 (0.02)

#define CINS_INITIAL_ROLL 0
#define CINS_INITIAL_PITCH 0
#define CINS_INITIAL_YAW 0


/*
  initialise the filter
 */
void AP_CINS::init(void)
{
    //Initialise XHat and ZHat as stationary at the origin
    state.XHat = Gal3F(Matrix3F(1,0,0,0,1,0,0,0,1),Vector3F(0,0,0),Vector3F(0,0,0),0.0f);
    state.ZHat = SIM23::identity();

    state.XHat.rot().from_euler(radians(CINS_INITIAL_ROLL),radians(CINS_INITIAL_PITCH),radians(CINS_INITIAL_YAW));
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
    Vector3F gyro = (delta_angle / dangle_dt).toftype();

    // get delta velocity
    Vector3f delta_velocity;
    float dvel_dt;
    if (!ins.get_delta_velocity(gyro_index, delta_velocity, dvel_dt) || dvel_dt <= 0) {
        // can't update, no delta velocity
        return;
    }
    // turn delta velocity into a accel vector
    const Vector3F accel = (delta_velocity / dvel_dt).toftype();

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
            const auto & vel = gps.velocity();
            const Vector3d pos = state.origin.get_distance_NED_double(loc);
            update_gps(pos.toftype(), vel.toftype(), gps_dt);
        }
    }

    update_imu(gyro, accel, dangle_dt);

    // update_yaw_from_compass();
    update_attitude_from_compass();

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
    ftype roll_rad, pitch_rad, yaw_rad;
    state.XHat.rot().to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    const Location loc = get_location();

    AP::logger().WriteStreaming("CINS", "TimeUS,Roll,Pitch,Yaw,VN,VE,VD,PN,PE,PD,Lat,Lon,Alt",
                                "sdddnnnmmmDUm",
                                "F000000000GG0",
                                "QfffffffffLLf",
                                AP_HAL::micros64(),
                                degrees(roll_rad),
                                degrees(pitch_rad),
                                wrap_360(degrees(yaw_rad)),
                                state.XHat.vel().x,
                                state.XHat.vel().y,
                                state.XHat.vel().z,
                                state.XHat.pos().x,
                                state.XHat.pos().y,
                                state.XHat.pos().z,
                                loc.lat,
                                loc.lng,
                                loc.alt*0.01);
}


/*
  update on new GPS sample
 */
void AP_CINS::update_gps(const Vector3F &pos, const Vector3F &vel, const ftype gps_dt)
{
    //compute correction terms 
    update_correction_terms_GPS(pos, vel, gps_dt);

    //Update XHat and ZHat using correciton terms
    state.XHat = correction_terms.Delta * state.XHat;
    state.ZHat = state.ZHat * correction_terms.Gamma;

    // use AHRS3 for debugging
    ftype roll_rad, pitch_rad, yaw_rad;
    const auto rot = state.XHat.rot() * AP::ahrs().get_rotation_vehicle_body_to_autopilot_body().toftype();
    rot.to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    const Location loc = get_location();
    const mavlink_ahrs3_t pkt {
    roll : float(roll_rad),
    pitch : float(pitch_rad),
    yaw : float(yaw_rad),
    altitude : float(-state.XHat.pos().z),
    lat: loc.lat,
    lng: loc.lng,
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
void AP_CINS::update_imu(const Vector3F &gyro_rads, const Vector3F &accel_mss, const ftype dt)
{
    //Integrate Dynamics using the Matrix exponential 
    //Create Zero vector 
    Vector3F zero_vector;
    zero_vector.zero();
    Vector3F gravity_vector;
    gravity_vector = Vector3F(0,0,GRAVITY_MSS);

    const Gal3 leftMat = Gal3::exponential(zero_vector, zero_vector, gravity_vector*dt, -dt);
    //Update XHat (Observer Dynamics)
    state.XHat = leftMat * state.XHat * Gal3::exponential(gyro_rads*dt, zero_vector, accel_mss*dt, dt);
    //Update ZHat (Auxilary Dynamics)
    SIM23 rightMatZ = SIM23::identity();
    rightMatZ.A() =  GL2::exponential((-dt*0.5) * state.ZHat.A().transposed() * GL2(CINS_GAIN_Q11, 0., 0., CINS_GAIN_Q22) * state.ZHat.A()); 

    state.ZHat = SIM23(leftMat) * state.ZHat * rightMatZ;
}


/*
Compute correction terms when a GPS signal is received. This code generates corrections terms
Delta and Gamma which are then applied to the estimated state XHat and auxiliary state ZHat, respectively.
This allows the update_gps to output corrected estimates for attitude, velocity (NED) and position. 
*/
void AP_CINS::update_correction_terms_GPS(const Vector3F &pos_tru, const Vector3F &vel_tru, const ftype gps_dt)
{
    const SIM23 ZInv = state.ZHat.inverse();
    const Vector3F pos_est = state.XHat.pos();
    const Vector3F pos_ZInv = ZInv.W2();

    // Gamma: Correction term to apply to the auxiliary state ZHat
    const Vector2F C = Vector2F(0,1); // (0, 1) for position measurements.
    const GL2 CCT = GL2(0,0,0,1.);
    const Vector2F gains_Gamma = CINS_GAIN_GPS_POS * ZInv.A() * C;

    Matrix3F R_gamma;
    R_gamma.identity();
    const GL2 A_Gamma = GL2::identity() + 0.5 * CINS_GAIN_GPS_POS * ZInv.A() * CCT * ZInv.A().transposed();
    const Vector3F V_Gamma_1 = (pos_tru + pos_ZInv) * gains_Gamma.x;
    const Vector3F V_Gamma_2 = (pos_tru + pos_ZInv) * gains_Gamma.y;

    correction_terms.Gamma = SIM23(R_gamma, V_Gamma_1, V_Gamma_2, A_Gamma);

    // Delta: Correction term to apply to the estimated state XHat
    Matrix3F R_delta;
    const Vector3F maucross_mauhat = Matrix3F::skew_symmetric(pos_tru + pos_ZInv) * (pos_est + pos_ZInv);
    const ftype mautrans_mauhat = (pos_tru + pos_ZInv) * (pos_est + pos_ZInv); //Dotproduct 
    const ftype norm_maucross_mauhat = maucross_mauhat.length();    
    if (fabsF(norm_maucross_mauhat) > 0.00001f){
        ftype psi = atan2F(norm_maucross_mauhat, mautrans_mauhat);
        R_delta = Matrix3F::from_angular_velocity(maucross_mauhat * ((CINS_GAIN_P*psi*gps_dt )/norm_maucross_mauhat));
    } else {
        R_delta.identity();
    }

    const Vector3F pre_V_Delta = (pos_tru + pos_ZInv) - R_delta * (pos_est + pos_ZInv);
    const Vector2F gains_Delta = A_Gamma.inverse().transposed() * gains_Gamma;
    const Vector3F V_Delta_1 = pre_V_Delta * gains_Delta.x;
    const Vector3F V_Delta_2 = pre_V_Delta * gains_Delta.y;

    SIM23 Delta = Gal3F(R_delta, V_Delta_2, V_Delta_1, 0.0f);
    Delta = state.ZHat * Delta * state.ZHat.inverse();
    correction_terms.Delta = Gal3F(Delta.R(), Delta.W2(), Delta.W1(), 0.);
}

/*
  initialise yaw from compass, if available
 */
bool AP_CINS::init_yaw(void)
{
    ftype mag_yaw, dt;
    if (!get_compass_yaw(mag_yaw, dt)) {
        return false;
    }
    ftype roll_rad, pitch_rad, yaw_rad;
    state.XHat.rot().to_euler(&roll_rad, &pitch_rad, &yaw_rad);
    state.XHat.rot().from_euler(roll_rad, pitch_rad, mag_yaw);
    
    return true;
}

/*
  get yaw from compass
 */
bool AP_CINS::get_compass_yaw(ftype &yaw_rad, ftype &dt)
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

    const Location loc = get_location();
    const float declination_deg = AP_Declination::get_declination(loc.lat*1.0e-7, loc.lng*1.0e-7);
    if (is_zero(declination_deg)) {
        // wait for declination
        return false;
    }

    const float cos_pitch_sq = 1.0f-(state.XHat.rot().c.x*state.XHat.rot().c.x);
    const float headY = field.y * state.XHat.rot().c.z - field.z * state.XHat.rot().c.y;

    // Tilt compensated magnetic field X component:
    const float headX = field.x * cos_pitch_sq - state.XHat.rot().c.x * (field.y * state.XHat.rot().c.y + field.z * state.XHat.rot().c.z);

    // return magnetic yaw
    yaw_rad = wrap_PI(atan2f(-headY,headX) + radians(declination_deg));

    return true;
}

/*
  update yaw from compass
 */
void AP_CINS::update_yaw_from_compass(void)
{
    ftype mag_yaw, dt;
    if (!get_compass_yaw(mag_yaw, dt)) {
        return;
    }
    ftype roll_rad, pitch_rad, yaw_rad;
    state.XHat.rot().to_euler(&roll_rad, &pitch_rad, &yaw_rad);

    const ftype yaw_err = wrap_PI(mag_yaw - yaw_rad);
    const Vector3F yaw_rot_ef{0,0,yaw_err*dt*CINS_GAIN_MAG};
    const Vector3F yaw_rot_bf = state.XHat.rot().mul_transpose(yaw_rot_ef);

    
    state.XHat.rot().rotate(yaw_rot_bf);
}


bool AP_CINS::get_compass_vector(Vector3F &mag_vec, Vector3F &mag_ref, ftype &dt)
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
    mag_vec = compass.get_field(mag_idx).toftype();
    const uint32_t last_us = compass.last_update_usec(mag_idx);
    if (last_us == last_mag_us) {
        // no new data
        return false;
    }
    dt = (last_us - last_mag_us) * 1.0e-6;
    last_mag_us = last_us;

    const Location loc = get_location();
    mag_ref = AP_Declination::get_earth_field_ga(loc).toftype();
    if (mag_ref.is_zero()) {
        // wait for declination
        return false;
    }

    return true;
}

void AP_CINS::update_attitude_from_compass() {
    ftype dt;
    Vector3F mag_vec, mag_ref;
    if (!get_compass_vector(mag_vec, mag_ref, dt)) {
        return;
    }
    // Convert mag measurement from milliGauss to Gauss
    mag_vec *= 1.e-3;
    Vector3F omega_Delta = - (Matrix3F::skew_symmetric(mag_ref) * state.XHat.rot() * mag_vec) * CINS_GAIN_MAG;
    Matrix3F R_Delta = Matrix3F::from_angular_velocity(omega_Delta*dt);

    SIM23 Delta = Gal3F(R_Delta, Vector3F(), Vector3F(), 0.0f);
    Delta = state.ZHat * Delta * state.ZHat.inverse();

    state.XHat = Gal3(Delta.R(), Delta.W2(), Delta.W1(), 0.) * state.XHat;
}