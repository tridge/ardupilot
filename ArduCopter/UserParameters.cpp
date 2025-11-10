#include "UserParameters.h"
#include "Copter.h"

// "AA" + 14 chars remaining for param name 
const AP_Param::GroupInfo AscentParameters::var_info[] = {
    
    // Put your parameters definition here
    // Note the maximum length of parameter name is 10 chars
    AP_GROUPINFO("_RPM_HOVER",  1,      AscentParameters, rpm_hover,                 RPM_HOVER_DEFAULT),
    AP_GROUPINFO("_BATT_NUM",   2,      AscentParameters, battery_number,            1),
    AP_GROUPINFO("_BATT_CAP",   3,      AscentParameters, single_pack_capacity_mah,  6000),
    AP_GROUPINFO("_BATT_S",     4,      AscentParameters, single_pack_series,        12),
    AP_GROUPINFO("_BATT_P",     5,      AscentParameters, single_pack_parallel,      2),
    AP_GROUPINFO("_BATT_R_HI",  6,      AscentParameters, bad_cell_res_mohm,         0.021),
    AP_GROUPINFO("_PAYLD_WT",   7,      AscentParameters, payload_weight,            0.0),
    AP_GROUPINFO("_CAM_TYPE",   8,      AscentParameters, spirit_camera_type,        0),
    AP_GROUPINFO("_TPLE_EN",    9,      AscentParameters, en_topple_sense,           3),
    AP_GROUPINFO("_TOTAL_WT",   10,     AscentParameters, total_vehicle_wt,          4.0),
    AP_GROUPINFO("_AUTO_CONF",  11,     AscentParameters, auto_config_enabled,       1),
    AP_GROUPINFO("_BLD_HUNG",   12,     AscentParameters, blade_hung_up_current,     5.0),
    AP_GROUPINFO("_GCS_TYPE",   13,     AscentParameters, gcs_type,                  3),
    AP_GROUPINFO("_NTRL_ANGL",  14,     AscentParameters, neutral_cam_angle,         NEUTRAL_CAM_ANGLE_DEFAULT),
    AP_GROUPINFO("_TOP_MNT",    15,     AscentParameters, top_mount,                 0),
    AP_GROUPINFO("_KS_MS",      16,     AscentParameters, killswitch_delay_ms,       200),

    AP_GROUPINFO("_ANG_KP_SL",  17,     AscentParameters, ang_kp_slope,             -0.23),
    AP_GROUPINFO("_ANG_KP_IN",  18,     AscentParameters, ang_kp_intercept,          7.0),
    AP_GROUPINFO("_ANG_KP_LO",  19,     AscentParameters, ang_kp_constraint_lo,      4.0),
    AP_GROUPINFO("_ANG_KP_HI",  20,     AscentParameters, ang_kp_constraint_hi,      5.25),

    AP_GROUPINFO("_RAT_KP_SL",  21,     AscentParameters, rate_kp_slope,             0.0193),
    AP_GROUPINFO("_RAT_KP_IN",  22,     AscentParameters, rate_kp_intercept,         0.106),
    AP_GROUPINFO("_RAT_KP_LO",  23,     AscentParameters, rate_kp_constraint_lo,     0.28),
    AP_GROUPINFO("_RAT_KP_HI",  24,     AscentParameters, rate_kp_constraint_hi,     0.412),

    AP_GROUPINFO("_RAT_KI_SL",  25,     AscentParameters, rate_ki_slope,             0.0227),
    AP_GROUPINFO("_RAT_KI_IN",  26,     AscentParameters, rate_ki_intercept,         0.106),
    AP_GROUPINFO("_RAT_KI_LO",  27,     AscentParameters, rate_ki_constraint_lo,     0.28),
    AP_GROUPINFO("_RAT_KI_HI",  28,     AscentParameters, rate_ki_constraint_hi,     0.412),

    AP_GROUPINFO("_RAT_KD_SL",  29,     AscentParameters, rate_kd_slope,             0.0012),
    AP_GROUPINFO("_RAT_KD_IN",  30,     AscentParameters, rate_kd_intercept,         -0.00199),
    AP_GROUPINFO("_RAT_KD_LO",  31,     AscentParameters, rate_kd_constraint_lo,     0.008),
    AP_GROUPINFO("_RAT_KD_HI",  32,     AscentParameters, rate_kd_constraint_hi,     0.01556),

    AP_GROUPINFO("_RPMH_SL",     33,     AscentParameters, rpmh_slope,             156),
    AP_GROUPINFO("_RPMH_SL_IN",  34,     AscentParameters, rpmh_intercept,         1553),
    AP_GROUPINFO("_RPMH_SL_LO",  35,     AscentParameters, rpmh_constraint_lo,     2700.0),
    AP_GROUPINFO("_RPMH_SL_HI",  36,     AscentParameters, rpmh_constraint_hi,     4000.0),

    AP_GROUPINFO("_FTC_EN",     37,     AscentParameters, follow_camera,           0),
    AP_GROUPINFO("_FTC_ON_RBT", 38,     AscentParameters, follow_camera_on_reboot,    0),
    AP_GROUPINFO("_STW_LND",    39,     AscentParameters, stow_camera,             1),
    AP_GROUPINFO("_STW_LND_H",  40,     AscentParameters, stow_camera_height_cm,   200),
    AP_GROUPINFO("_VIS_SRC",    42,     AscentParameters, ekf_source, 0),
    AP_GROUPINFO("_VEHICLE",    43,     AscentParameters, vehicle_type, 1),
    AP_GROUPINFO("_TUBE_LAUNCH", 44,     AscentParameters, tube_launch, 0),
    AP_GROUPINFO("_RAT_YAW_P",          45,    AscentParameters, yaw_p,         0.09),
    AP_GROUPINFO("_RAT_YAW_I",          46,    AscentParameters, yaw_i,         0.015),
    AP_GROUPINFO("_RAT_YAW_P_LL",       47,    AscentParameters, yaw_p_ll,      0.04),
    AP_GROUPINFO("_RAT_YAW_I_LL",       48,    AscentParameters, yaw_i_ll,      0.007),
    AP_GROUPINFO("_AUTO_MOT",   49,     AscentParameters, auto_mot_offs, 1),

    AP_GROUPEND
};

AscentParameters::AscentParameters() {
    AP_Param::setup_object_defaults(this, var_info);
}