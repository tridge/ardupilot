/*
  CINS state estimator for AP_AHRS
 */

#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>

class AP_CINS {
public:
    void init(void);
    void update();

    Vector3f get_accel() const {
        return state.accel;
    }
    Vector3f get_gyro() const {
        return state.gyro;
    }
    Quaternion get_quat() const {
        Quaternion quat;
        quat.from_rotation_matrix(state.rotation_matrix);
        return quat;
    }
    Location get_location() const {
        return state.location;
    }
    Vector3f get_velocity() const {
        return state.velocity_NED;
    }
    bool healthy(void) const {
        return state.have_origin;
    }
    bool get_origin(Location &loc) {
        loc = state.origin;
        return state.have_origin;
    }

private:
    void update_gps(const Vector3f &pos);
    void update_imu(const Vector3f &gyro_rads, const Vector3f &accel_mss, const float dt);

    struct {
        Vector3f accel;
        Vector3f gyro;
        Matrix3f rotation_matrix;
        Vector3f accel_ef;
        Location origin;
        bool have_origin;
        Location location;
        Vector3f velocity_NED;

        // NED position from the origin
        Vector3f position;
    } state;

    uint32_t last_update_us;
    uint32_t last_gps_update_ms;
};
