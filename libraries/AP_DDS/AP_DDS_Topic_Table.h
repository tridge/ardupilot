#include "builtin_interfaces/msg/Time.h"
#include "sensor_msgs/msg/NavSatFix.h"
#include "tf2_msgs/msg/TFMessage.h"
#include "sensor_msgs/msg/BatteryState.h"
#include "geographic_msgs/msg/GeoPoseStamped.h"
#if AP_DDS_IMU_PUB_ENABLED
#include "sensor_msgs/msg/Imu.h"
#endif //AP_DDS_IMU_PUB_ENABLED

#include "uxr/client/client.h"

// Code generated table based on the enabled topics.
// Mavgen is using python, loops are not readable.
// Can use jinja to template (like Flask)

enum class TopicIndex: uint8_t {
#if AP_DDS_TIME_PUB_ENABLED
    TIME_PUB,
#endif // AP_DDS_TIME_PUB_ENABLED
#if AP_DDS_NAVSATFIX_PUB_ENABLED
    NAV_SAT_FIX_PUB,
#endif // AP_DDS_NAVSATFIX_PUB_ENABLED
#if AP_DDS_STATIC_TF_PUB_ENABLED
    STATIC_TRANSFORMS_PUB,
#endif // AP_DDS_STATIC_TF_PUB_ENABLED
#if AP_DDS_BATTERY_STATE_PUB_ENABLED
    BATTERY_STATE_PUB,
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED
#if AP_DDS_IMU_PUB_ENABLED
    IMU_PUB,
#endif //AP_DDS_IMU_PUB_ENABLED
#if AP_DDS_LOCAL_POSE_PUB_ENABLED
    LOCAL_POSE_PUB,
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED
#if AP_DDS_LOCAL_VEL_PUB_ENABLED
    LOCAL_VELOCITY_PUB,
#endif // AP_DDS_LOCAL_VEL_PUB_ENABLED
#if AP_DDS_GEOPOSE_PUB_ENABLED
    GEOPOSE_PUB,
#endif // AP_DDS_GEOPOSE_PUB_ENABLED
#if AP_DDS_CLOCK_PUB_ENABLED
    CLOCK_PUB,
#endif // AP_DDS_CLOCK_PUB_ENABLED
#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
    GPS_GLOBAL_ORIGIN_PUB,
#endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
#if AP_DDS_JOY_SUB_ENABLED
    JOY_SUB,
#endif // AP_DDS_JOY_SUB_ENABLED
#if AP_DDS_DYNAMIC_TF_SUB
    DYNAMIC_TRANSFORMS_SUB,
#endif // AP_DDS_DYNAMIC_TF_SUB
#if AP_DDS_VEL_CTRL_ENABLED
    VELOCITY_CONTROL_SUB,
#endif // AP_DDS_VEL_CTRL_ENABLED
#if AP_DDS_GLOBAL_POS_CTRL_ENABLED
    GLOBAL_POSITION_SUB,
#endif // AP_DDS_GLOBAL_POS_CTRL_ENABLED
};

static inline constexpr uint8_t to_underlying(const TopicIndex index)
{
    static_assert(sizeof(index) == sizeof(uint8_t));
    return static_cast<uint8_t>(index);
}


constexpr struct AP_DDS_Client::Topic_table AP_DDS_Client::topics[] = {
#if AP_DDS_TIME_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::TIME_PUB),
        .pub_id = to_underlying(TopicIndex::TIME_PUB),
        .sub_id = to_underlying(TopicIndex::TIME_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::TIME_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::TIME_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "time__t",
        .dw_profile_label = "time__dw",
        .dr_profile_label = "",
    },
#endif // AP_DDS_TIME_PUB_ENABLED
#if AP_DDS_NAVSATFIX_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::NAV_SAT_FIX_PUB),
        .pub_id = to_underlying(TopicIndex::NAV_SAT_FIX_PUB),
        .sub_id = to_underlying(TopicIndex::NAV_SAT_FIX_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::NAV_SAT_FIX_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::NAV_SAT_FIX_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "navsatfix0__t",
        .dw_profile_label = "navsatfix0__dw",
        .dr_profile_label = "",
    },
#endif // AP_DDS_NAVSATFIX_PUB_ENABLED
#if AP_DDS_STATIC_TF_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB),
        .pub_id = to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB),
        .sub_id = to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::STATIC_TRANSFORMS_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "statictransforms__t",
        .dw_profile_label = "statictransforms__dw",
        .dr_profile_label = "",
    },
#endif // AP_DDS_STATIC_TF_PUB_ENABLED
#if AP_DDS_BATTERY_STATE_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::BATTERY_STATE_PUB),
        .pub_id = to_underlying(TopicIndex::BATTERY_STATE_PUB),
        .sub_id = to_underlying(TopicIndex::BATTERY_STATE_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::BATTERY_STATE_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::BATTERY_STATE_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "batterystate0__t",
        .dw_profile_label = "batterystate0__dw",
        .dr_profile_label = "",
    },
#endif // AP_DDS_BATTERY_STATE_PUB_ENABLED
#if AP_DDS_IMU_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::IMU_PUB),
        .pub_id = to_underlying(TopicIndex::IMU_PUB),
        .sub_id = to_underlying(TopicIndex::IMU_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::IMU_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::IMU_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "imu__t",
        .dw_profile_label = "imu__dw",
        .dr_profile_label = "",
    },
#endif //AP_DDS_IMU_PUB_ENABLED
#if AP_DDS_LOCAL_POSE_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::LOCAL_POSE_PUB),
        .pub_id = to_underlying(TopicIndex::LOCAL_POSE_PUB),
        .sub_id = to_underlying(TopicIndex::LOCAL_POSE_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_POSE_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_POSE_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "localpose__t",
        .dw_profile_label = "localpose__dw",
        .dr_profile_label = "",
    },
#endif // AP_DDS_LOCAL_POSE_PUB_ENABLED
#if AP_DDS_LOCAL_VEL_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::LOCAL_VELOCITY_PUB),
        .pub_id = to_underlying(TopicIndex::LOCAL_VELOCITY_PUB),
        .sub_id = to_underlying(TopicIndex::LOCAL_VELOCITY_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_VELOCITY_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::LOCAL_VELOCITY_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "localvelocity__t",
        .dw_profile_label = "localvelocity__dw",
        .dr_profile_label = "",
    },
#endif // AP_DDS_LOCAL_VEL_PUB_ENABLED
#if AP_DDS_GEOPOSE_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::GEOPOSE_PUB),
        .pub_id = to_underlying(TopicIndex::GEOPOSE_PUB),
        .sub_id = to_underlying(TopicIndex::GEOPOSE_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::GEOPOSE_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::GEOPOSE_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "geopose__t",
        .dw_profile_label = "geopose__dw",
        .dr_profile_label = "",
    },
#endif // AP_DDS_GEOPOSE_PUB_ENABLED
#if AP_DDS_CLOCK_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::CLOCK_PUB),
        .pub_id = to_underlying(TopicIndex::CLOCK_PUB),
        .sub_id = to_underlying(TopicIndex::CLOCK_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::CLOCK_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::CLOCK_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "clock__t",
        .dw_profile_label = "clock__dw",
        .dr_profile_label = "",
    },
#endif // AP_DDS_CLOCK_PUB_ENABLED
#if AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::GPS_GLOBAL_ORIGIN_PUB),
        .pub_id = to_underlying(TopicIndex::GPS_GLOBAL_ORIGIN_PUB),
        .sub_id = to_underlying(TopicIndex::GPS_GLOBAL_ORIGIN_PUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::GPS_GLOBAL_ORIGIN_PUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::GPS_GLOBAL_ORIGIN_PUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "gps_global_origin__t",
        .dw_profile_label = "gps_global_origin__dw",
        .dr_profile_label = "",
    },
#endif // AP_DDS_GPS_GLOBAL_ORIGIN_PUB_ENABLED
#if AP_DDS_JOY_SUB_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::JOY_SUB),
        .pub_id = to_underlying(TopicIndex::JOY_SUB),
        .sub_id = to_underlying(TopicIndex::JOY_SUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::JOY_SUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::JOY_SUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "joy__t",
        .dw_profile_label = "",
        .dr_profile_label = "joy__dr",
    },
#endif // AP_DDS_JOY_SUB_ENABLED
#if AP_DDS_DYNAMIC_TF_SUB
    {
        .topic_id = to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB),
        .pub_id = to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB),
        .sub_id = to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::DYNAMIC_TRANSFORMS_SUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "dynamictf__t",
        .dw_profile_label = "",
        .dr_profile_label = "dynamictf__dr",
    },
#endif // AP_DDS_DYNAMIC_TF_SUB
#if AP_DDS_VEL_CTRL_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::VELOCITY_CONTROL_SUB),
        .pub_id = to_underlying(TopicIndex::VELOCITY_CONTROL_SUB),
        .sub_id = to_underlying(TopicIndex::VELOCITY_CONTROL_SUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::VELOCITY_CONTROL_SUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::VELOCITY_CONTROL_SUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "velocitycontrol__t",
        .dw_profile_label = "",
        .dr_profile_label = "velocitycontrol__dr",
    },
#endif // AP_DDS_VEL_CTRL_ENABLED
#if AP_DDS_GLOBAL_POS_CTRL_ENABLED
    {
        .topic_id = to_underlying(TopicIndex::GLOBAL_POSITION_SUB),
        .pub_id = to_underlying(TopicIndex::GLOBAL_POSITION_SUB),
        .sub_id = to_underlying(TopicIndex::GLOBAL_POSITION_SUB),
        .dw_id = uxrObjectId{.id=to_underlying(TopicIndex::GLOBAL_POSITION_SUB), .type=UXR_DATAWRITER_ID},
        .dr_id = uxrObjectId{.id=to_underlying(TopicIndex::GLOBAL_POSITION_SUB), .type=UXR_DATAREADER_ID},
        .topic_profile_label = "globalposcontrol__t",
        .dw_profile_label = "",
        .dr_profile_label = "globalposcontrol__dr",
    },
#endif // AP_DDS_GLOBAL_POS_CTRL_ENABLED
};
