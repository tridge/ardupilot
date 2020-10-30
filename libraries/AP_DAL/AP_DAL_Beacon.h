#pragma once

#include <AP_Logger/LogStructure.h>

#include <AP_Beacon/AP_Beacon.h>

#include <AP_Vehicle/AP_Vehicle_Type.h>

class AP_DAL_Beacon {
public:

    // Beacon-like methods:
    bool count() const {
        return _RBCH.count;
    }

    bool get_origin(Location &loc) const {
        loc = _origin;
        return _RBCH.get_origin_returncode;
    }

    // return beacon health
    bool beacon_healthy(uint8_t i) const {
        return _RBCI[i].healthy;
    }

    // return last update time from beacon in milliseconds
    uint32_t beacon_last_update_ms(uint8_t i) const {
        return _RBCI[i].last_update_ms;
    }

    // return distance to beacon in meters
    float beacon_distance(uint8_t i) const {
        return _RBCI[i].distance;
    }

    // return NED position of beacon in meters relative to the beacon systems origin
    const Vector3f &beacon_position(uint8_t i) const {
        return *(Vector3f *)_RBCI[i].position;
    }

    // return vehicle position in NED from position estimate system's origin in meters
    bool get_vehicle_position_ned(Vector3f& pos, float& accuracy_estimate) const {
        pos = *((Vector3f *)&_RBCH.vehicle_position_ned);
        accuracy_estimate = _RBCH.accuracy_estimate;
        return _RBCH.get_vehicle_position_ned_returncode;
    }

    // AP_DAL methods:
    AP_DAL_Beacon();

    AP_DAL_Beacon *beacon() {
        if (_RBCH.ptr_is_nullptr) {
            return nullptr;
        }
        return this;
    }

    void start_frame(const uint64_t time_us);

#if APM_BUILD_TYPE(APM_BUILD_Replay)
    void handle_message(const log_RBCH &msg) {
        _RBCH = msg;
        _origin.lat = _RBCH.lat;
        _origin.lng = _RBCH.lng;
        _origin.alt = _RBCH.alt;
   }
    void handle_message(const log_RBCI &msg) {
        _RBCI[msg.instance] = msg;
    }
#endif

private:

    struct log_RBCH _RBCH;
    struct log_RBCI _RBCI[AP_BEACON_MAX_BEACONS];

    uint32_t _last_logged_update_ms[AP_BEACON_MAX_BEACONS];

    Location _origin;
};