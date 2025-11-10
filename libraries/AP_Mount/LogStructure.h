#pragma once

#include <AP_Logger/LogStructure.h>

#define LOG_IDS_FROM_MOUNT \
    LOG_VP_GIMBAL,  \
    LOG_NV_GIMBAL,  \
    LOG_GA_GIMBAL,  \
    LOG_GREMSY_GIMBAL,  \
    LOG_MOUNT_MSG

// @LoggerMessage: MNT
// @Description: Mount's desired and actual roll, pitch and yaw angles
// @Field: TimeUS: Time since system startup
// @Field: I: Instance number
// @Field: DRoll: Desired roll
// @Field: Roll: Actual roll
// @Field: DPitch: Desired pitch
// @Field: Pitch: Actual pitch
// @Field: DYawB: Desired yaw in body frame
// @Field: YawB: Actual yaw in body frame
// @Field: DYawE: Desired yaw in earth frame
// @Field: YawE: Actual yaw in earth frame
// @Field: Dist: Rangefinder distance

struct PACKED log_Mount {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t  instance;
    float    desired_roll;
    float    actual_roll;
    float    desired_pitch;
    float    actual_pitch;
    float    desired_yaw_bf;
    float    actual_yaw_bf;
    float    desired_yaw_ef;
    float    actual_yaw_ef;
    float    rangefinder_dist;
};
struct PACKED log_vp_gimbal {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float pitch;
    float pan;
    float pan_rel;
    float speed_pitch;
    float speed_pan;
};

struct PACKED log_nv_gimbal {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float tilt;
    float pan;
    float speedTilt;
    float speedPan;
    bool internal_error;
    bool eo_error;
    bool ir_error;
    bool camera_overheat;
    bool camera_comms_error;
    float camera_temperature;
    float trip_temperature;
};

struct PACKED log_ga_gimbal {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    uint8_t geotagging_mode;
    uint8_t geotagging_session;
    uint16_t session_pictures;
    float tilt;
    uint8_t geotagging_progress;
    float pan;
};

struct PACKED log_gremsy_gimbal {
    LOG_PACKET_HEADER;
    uint64_t time_us;
    float pan;
    float tilt;
    float roll;
};

#define LOG_STRUCTURE_FROM_MOUNT \
    { LOG_MOUNT_MSG, sizeof(log_Mount), \
      "MNT", "QBfffffffff","TimeUS,I,DRoll,Roll,DPitch,Pitch,DYawB,YawB,DYawE,YawE,Dist", "s#ddddddddm", "F---------0" }, \
    { LOG_VP_GIMBAL, sizeof(log_vp_gimbal), \
     "VP", "Qfffff","TimeUS,tilt,pan,panR,spdT,spdP", "s-----", "F-----" }, \
    { LOG_NV_GIMBAL, sizeof(log_nv_gimbal), \
     "NV", "QffffBBBBBff","TimeUS,tilt,pan,spdT,spdP,intE,eoEr,irEr,cmOHT,cmCom,cmTmp,tpTmp", "s-----------", "F-----------" }, \
    { LOG_GA_GIMBAL, sizeof(log_ga_gimbal), \
     "GA", "QBBHfBf","TimeUS,geotagMode,geotagSession,sessionPics,tilt,progress,pan", "s------", "F------" }, \
    { LOG_GREMSY_GIMBAL, sizeof(log_gremsy_gimbal), \
     "GMSY", "Qfff","TimeUS,pan,tilt,roll", "s---", "F---" }, 

