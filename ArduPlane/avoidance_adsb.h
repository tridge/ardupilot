#pragma once

#include <AP_Avoidance/AP_Avoidance.h>

// Provide Plane-specific implementation of avoidance.  While most of
// the logic for doing the actual avoidance is present in
// AP_Avoidance, this class allows Plane to override base
// functionality - for example, not doing anything while landed.
class AP_Avoidance_Plane : public AP_Avoidance {
public:
    AP_Avoidance_Plane(AP_AHRS &ahrs, class AP_ADSB &adsb)
        : AP_Avoidance(ahrs, adsb)
    {
    }

    /* Do not allow copies */
    AP_Avoidance_Plane(const AP_Avoidance_Plane &other) = delete;
    AP_Avoidance_Plane &operator=(const AP_Avoidance_Plane&) = delete;

    bool mission_avoidance(const Location &loc, Location &target_loc, float groundspeed);
    
protected:
    // override avoidance handler
    MAV_COLLISION_ACTION handle_avoidance(const AP_Avoidance::Obstacle *obstacle, MAV_COLLISION_ACTION requested_action) override;

    // override recovery handler
    void handle_recovery(uint8_t recovery_action) override;

    // check flight mode is avoid_adsb
    bool check_flightmode(bool allow_mode_change);

    // vertical avoidance handler
    bool handle_avoidance_vertical(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    // horizontal avoidance handler
    bool handle_avoidance_horizontal(const AP_Avoidance::Obstacle *obstacle, bool allow_mode_change);

    // control mode before avoidance began
    FlightMode prev_control_mode = RTL;

private:
    bool mission_avoids_collisions(const Location &our_loc, const Vector2f &our_velocity, float avoid_sec);
    bool mission_avoid_exclusions(const Location &current_loc, const Location &loc_test);
    bool mission_avoid_fence(const Location &loc_test);
    void load_exclusion_zones(void);
    void unload_exclusion_zones(void);
    void load_fence_boundary(void);
    float get_avoidance_radius(const class Obstacle &obstacle) const;
    bool within_avoidance_height(const class Obstacle &obstacle) const;
    bool have_collided(const Location &loc);

    uint8_t num_exclusion_zones;
    struct exclusion_zone {
        // closed polygon, with points[0] == points[n-1]
        uint8_t num_points;
        Vector2f *points;
    } *exclusion_zones;
    uint32_t mission_change_ms;

    // georefence, with a inner margin (shrunk from real geofence)
    uint8_t num_fence_points;
    Vector2l *fence_points;
    uint32_t last_fence_change_ms;

    // grow a polygon by the given number of meters. Used to add a
    // margin to exclusion zones and the fence. The position change in
    // points is relative to the average point
    void grow_polygonf(Vector2f *points, uint8_t num_points, float change_m);
    void grow_polygonl(Vector2l *points, uint8_t num_points, float change_m);
    
    // number of meters of padding around exlusion zones and inside the fence
    const float exclusion_margin = 50;
    const float fence_margin = 100;
};
