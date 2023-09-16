/*
  CINS state estimator for AP_AHRS
 */

#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Math/SE23.h>
#include <AP_Common/Location.h>

class AP_CINS {
public:
    void init(void);
    void update();

    Vector3f get_accel() const {
        return state.accel.tofloat();
    }
    Vector3f get_gyro() const {
        return state.gyro.tofloat();
    }
    Quaternion get_quat() const {
        QuaternionF quat;
        quat.from_rotation_matrix(state.rotation_matrix);
        return quat.tofloat();
    }
    Location get_location() const {
        return state.location;
    }
    Vector3f get_velocity() const {
        return state.velocity_NED.tofloat();
    }
    bool healthy(void) const {
        return state.have_origin;
    }
    bool get_origin(Location &loc) {
        loc = state.origin;
        return state.have_origin;
    }

private:
    void update_gps(const Vector3F &pos, const ftype gps_dt);
    void update_imu(const Vector3F &gyro_rads, const Vector3F &accel_mss, const ftype dt);
    void update_correction_terms(const Vector3F &pos, const ftype dt);
    bool init_yaw(void);
    bool get_compass_yaw(ftype &yaw_rad, ftype &dt);
    void update_yaw_from_compass();

    struct {
        Vector3F accel;
        Vector3F gyro;
        Matrix3F rotation_matrix;
        Vector3F accel_ef;
        Location origin;
        bool have_origin;
        Location location;
        Vector3F velocity_NED;
        //XHat and ZHat for CINS
        SE23F XHat;
        SE23F ZHat;
        // NED position from the origin
        Vector3F position;
    } state;
    //Struct for correction terms 
    struct{
        SE23F Gamma;
        SE23F Delta;
        //Gains for correction terms
        ftype gain_p;
        ftype gain_l1;
        ftype gain_l2;
    } correction_terms;

    uint32_t last_gps_update_ms;
    bool done_yaw_init;
    uint32_t last_mag_us;
};
