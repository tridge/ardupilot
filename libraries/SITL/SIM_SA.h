/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.
*/

/*
  Silent Arrow CLS-200 glider simulator
*/

#pragma once

#include "SIM_Aircraft.h"

namespace SITL {

class SA : public Aircraft {
public:
    SA(const char *frame_str);

    void update(const struct sitl_input &input) override;

    static Aircraft *create(const char *frame_str)
    {
        return new SA(frame_str);
    }

    static const struct AP_Param::GroupInfo var_info[];

private:
    AP_Float drop_lat;
    AP_Float drop_lon;
    AP_Float drop_alt;
    AP_Float drop_spd;
    AP_Float drop_hdg;
    AP_Float glide_ratio;

    float angle_of_attack;
    float beta;
    bool last_armed;
    bool ever_armed;
    bool late_defaults_applied;
    Location mission_drop;
    bool mission_drop_valid;
    uint32_t last_mission_check_ms;

    void update_release_gpio(bool armed);
    void apply_default_parameters();
    void update_drop_from_mission();
    void set_drop_state(const struct sitl_input &input);
    void calculate_forces(const struct sitl_input &input,
                          Vector3f &rot_accel,
                          Vector3f &body_accel);
    Vector3f get_force(float aileron, float elevator) const;
    Vector3f get_rot_accel(float aileron, float elevator) const;
    float lift_coeff(float alpha) const;
    float drag_coeff(float lift) const;
    float surface_input(const struct sitl_input &input, uint8_t idx,
                        uint16_t minimum, uint16_t trim,
                        uint16_t maximum, float direction);
};

} // namespace SITL
