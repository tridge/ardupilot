#pragma once

#include <AP_Param/AP_Param.h>

#define RPM_HOVER_DEFAULT 3500.0f
#define NEUTRAL_CAM_ANGLE_DEFAULT 20.0f

class AscentParameters {

public:
    AscentParameters();
    static const struct AP_Param::GroupInfo var_info[];
    
    // Put accessors to your parameter variables here
    // UserCode usage example: g2.user_parameters.get_int8Param()

    AP_Float get_rpm_hover() const { return rpm_hover; }
    void set_rpm_hover(float rpm) { rpm_hover.set(rpm); }

    AP_Int8 get_battery_number() const { return battery_number; }

    AP_Int32 get_single_pack_batt_cap_mah() const { return single_pack_capacity_mah; }

    AP_Int8 get_single_pack_parallel() const { return single_pack_parallel; }

    AP_Int8 get_single_pack_series() const { return single_pack_series; }

    AP_Float get_bad_cell_res_mohm() const { return bad_cell_res_mohm; }

    AP_Float get_payload_weight() const { return payload_weight; }

    AP_Int8 get_camera_type() const { return spirit_camera_type; }

    AP_Int8 get_en_topple_sense() const { return en_topple_sense; }

    AP_Float get_total_vehicle_wt() const { return total_vehicle_wt; }
    void set_total_vehicle_wt(float wt) { total_vehicle_wt.set(wt);}

    AP_Int8 get_auto_config_enabled() const { return auto_config_enabled; }

    AP_Float get_blade_hung_up_current() const { return blade_hung_up_current; }

    AP_Int8 get_gcs_type() const { return gcs_type; }

    AP_Float get_neutral_cam_angle() const { return neutral_cam_angle; }

    AP_Int8 get_top_mount() const { return top_mount; }

    AP_Int16 get_killswitch_delay_ms() const { return killswitch_delay_ms; }

    AP_Int8 get_follow_camera() const { return follow_camera; }
    void set_follow_camera(bool value) { follow_camera.set(value);}

    AP_Int8 get_follow_camera_on_reboot() const { return follow_camera_on_reboot; }

    AP_Int8 get_stow_camera() const { return stow_camera; }
    
    AP_Int16 get_stow_camera_height_cm() const { return stow_camera_height_cm; }

    AP_Int8 get_ekf_source() const { return ekf_source; }
    void set_ekf_source(uint8_t source) { ekf_source.set(source); };

    AP_Int8 get_vehicle_type() const { return vehicle_type; }

    AP_Int8 get_tube_launch() const { return tube_launch; }
    
    AP_Int8 get_auto_mot_offs() const { return auto_mot_offs; }

    /*
        PID RELATED PARAMS
    */
    AP_Float get_ang_kp_slope()             const { return ang_kp_slope; }
    AP_Float get_ang_kp_intercept()         const { return ang_kp_intercept; }
    AP_Float get_ang_kp_constraint_lo()     const { return ang_kp_constraint_lo; }
    AP_Float get_ang_kp_constraint_hi()     const { return ang_kp_constraint_hi; }

    AP_Float get_rate_kp_slope()            const { return rate_kp_slope; }
    AP_Float get_rate_kp_intercept()        const { return rate_kp_intercept; }
    AP_Float get_rate_kp_constraint_lo()    const { return rate_kp_constraint_lo; }
    AP_Float get_rate_kp_constraint_hi()    const { return rate_kp_constraint_hi; }

    AP_Float get_rate_ki_slope()            const { return rate_ki_slope; }
    AP_Float get_rate_ki_intercept()        const { return rate_ki_intercept; }
    AP_Float get_rate_ki_constraint_lo()    const { return rate_ki_constraint_lo; }
    AP_Float get_rate_ki_constraint_hi()    const { return rate_ki_constraint_hi; }

    AP_Float get_rate_kd_slope()            const { return rate_kd_slope; }
    AP_Float get_rate_kd_intercept()        const { return rate_kd_intercept; }
    AP_Float get_rate_kd_constraint_lo()    const { return rate_kd_constraint_lo; }
    AP_Float get_rate_kd_constraint_hi()    const { return rate_kd_constraint_hi; }

    AP_Float get_rpmh_slope()            const { return rpmh_slope; }
    AP_Float get_rpmh_intercept()        const { return rpmh_intercept; }
    AP_Float get_rpmh_constraint_lo()    const { return rpmh_constraint_lo; }
    AP_Float get_rpmh_constraint_hi()    const { return rpmh_constraint_hi; }

    AP_Float get_yaw_p()                        const { return yaw_p; }
    AP_Float get_yaw_i()                        const { return yaw_i; }
    AP_Float get_yaw_p_ll()                     const { return yaw_p_ll; }
    AP_Float get_yaw_i_ll()                     const { return yaw_i_ll; }

private:
    // Put your parameter variable definitions here

    AP_Float    rpm_hover;
    AP_Int8     battery_number;
    AP_Int32    single_pack_capacity_mah;
    AP_Int8     single_pack_series;
    AP_Int8     single_pack_parallel;
    AP_Float    bad_cell_res_mohm;
    AP_Float    payload_weight;
    AP_Int8     spirit_camera_type;
    AP_Int8     en_topple_sense;
    AP_Float    total_vehicle_wt;
    AP_Int8     auto_config_enabled;
    AP_Float    blade_hung_up_current;
    AP_Int8     gcs_type;
    AP_Float    neutral_cam_angle;
    AP_Int8     top_mount;
    AP_Int16    killswitch_delay_ms;
    AP_Int8     follow_camera;
    AP_Int8     follow_camera_on_reboot;
    AP_Int8     stow_camera;
    AP_Int16    stow_camera_height_cm;
    AP_Int8     ekf_source; // 0 (GPS) or 1 (VisOdom)
    AP_Int8     vehicle_type;
    AP_Int8     tube_launch;
    AP_Int8     auto_mot_offs;

    /*
        PID RELATED PARAMS
    */
    AP_Float                ang_kp_slope;
    AP_Float                ang_kp_intercept;
    AP_Float                ang_kp_constraint_lo;
    AP_Float                ang_kp_constraint_hi;

    AP_Float                rate_kp_slope;
    AP_Float                rate_kp_intercept;
    AP_Float                rate_kp_constraint_lo;
    AP_Float                rate_kp_constraint_hi;

    AP_Float                rate_ki_slope;
    AP_Float                rate_ki_intercept;
    AP_Float                rate_ki_constraint_lo;
    AP_Float                rate_ki_constraint_hi;

    AP_Float                rate_kd_slope;
    AP_Float                rate_kd_intercept;
    AP_Float                rate_kd_constraint_lo;
    AP_Float                rate_kd_constraint_hi;

    AP_Float                rpmh_slope;
    AP_Float                rpmh_intercept;
    AP_Float                rpmh_constraint_lo;
    AP_Float                rpmh_constraint_hi;

    AP_Float                yaw_p;
    AP_Float                yaw_i;
    AP_Float                yaw_p_ll;
    AP_Float                yaw_i_ll;

};