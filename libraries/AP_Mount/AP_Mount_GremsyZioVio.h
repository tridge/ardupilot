#pragma once

#include "AP_Mount_Backend.h"


#include <AP_HAL/AP_HAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Math/AP_Math.h>
#include <AP_Common/AP_Common.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Logger/AP_Logger.h>

class AP_Mount_GremsyZioVio : public AP_Mount_Backend
{

public:
    AP_Mount_GremsyZioVio(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance);
    // init
    void init() override {}

    // update mount position
    void update() override;

    // return true if healthy
    bool healthy() const override;

    // has_pan_control
    bool has_pan_control() const override { return yaw_range_valid(); }

    // handle GIMBAL_DEVICE_INFORMATION message
    void handle_gimbal_device_information(const mavlink_message_t &msg) override;

    // handle GIMBAL_DEVICE_ATTITUDE_STATUS message
    void handle_gimbal_device_attitude_status(const mavlink_message_t &msg) override;

protected:
    // get attitude as a quaternion.  returns true on success
    bool get_attitude_quaternion(Quaternion& att_quat) override;

private:
    void set_zoom_rate(float zoom_speed) override;
    void set_gimbal_rates(float pan_speed, float tilt_speed)override;
    void set_angles(float pan_input, float tilt_input)override;
    bool center_camera(bool follow_camera)override;
    void look_down()override;
    void stow()override;

    void payload_take_picture()override;
    void start_record()override;
    void stop_record()override;
    void eo_full_screen()override;
    void ir_full_screen()override;
    void eo_ir()override;
    void ir_eo()override;
    void next_color_pallette()override;
    void fusion()override;

    void set_osd_mode(int arg)override;
    void set_record_source(int arg)override;
    void set_zoom_mode(int arg)override;
    void ir_zoom(int arg)override;
    void set_detection(int arg)override;
    void start_tracking( int x, int y)override;
    void stop_tracking()override;
    
    void request_payload_info()override;
    void request_param_ext_list();
    void handle_param_ext_value(const mavlink_message_t &msg) override;
    void send_param_set(const char *id, int val);
    void send_cmd_long(int sys, int comp, int cmd, int p1, int p2, int p3, int p4, int p5, int p6, int p7);

    // search for gimbal in GCS_MAVLink routing table
    void find_gimbal();

    void write_gimbal_log();

    // request GIMBAL_DEVICE_INFORMATION from gimbal (holds vendor and model name, max lean angles)
    void request_gimbal_device_information() const;

    // start sending ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE to gimbal
    // returns true on success, false on failure to start sending
    bool start_sending_attitude_to_gimbal();

    // send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to command gimbal to retract (aka relax)
    void send_gimbal_device_retract() const;

    // send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control rate
    // earth_frame should be true if yaw_rads target is an earth frame rate, false if body_frame
    void send_gimbal_device_set_rate(float roll_rads, float pitch_rads, float yaw_rads, bool earth_frame) const;

    // send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control attitude
    // earth_frame should be true if yaw_rad target is an earth frame angle, false if body_frame
    void send_gimbal_device_set_attitude(float roll_rad, float pitch_rad, float yaw_rad, bool earth_frame) const;


    // internal variables
    bool _got_device_info;          // true once gimbal has provided device info
    bool _initialised;              // true once the gimbal has provided a GIMBAL_DEVICE_INFORMATION
    uint32_t _last_devinfo_req_ms;  // system time that GIMBAL_DEVICE_INFORMATION was last requested (used to throttle requests)
    class GCS_MAVLINK *_link;       // link we have found gimbal on; nullptr if not seen yet
    uint8_t _sysid;                 // sysid of gimbal
    uint8_t _compid;                // component id of gimbal
    mavlink_channel_t _chan;
    mavlink_gimbal_device_attitude_status_t _gimbal_device_attitude_status;  // copy of most recently received gimbal status
    uint32_t _last_attitude_status_ms;  // system time last attitude status was received (used for health reporting)
    uint32_t _sent_gimbal_device_attitude_status_ms;    // time_boot_ms field of gimbal_device_status message last forwarded to the GCS (used to prevent sending duplicates)
    uint8_t  _current_zoom_state;
    uint8_t  _prev_zoom_state = ZOOM_STOP;
    const char *c_source_id = "C_SOURCE";
    const char *c_palette_id = "C_T_PALETTE";
    const char *track_mode_id = "TRACK_MODE";
    const char *osd_mode_id = "OSD_MODE";
    const char *rec_src_id = "C_V_REC";
    const char *ir_zoom_id = "C_T_ZOOM";
    const char *zoom_mode_id = "C_V_ZM_MODE";
    const char *gb_mode_id = "GB_MODE";

    int _zoom_mode = -1;
    int _rec_source = -1;
    int _osd_state = -1;
    int _obj_detect = -1;

    enum zoom_state{
        ZOOM_STOP,
        ZOOM_IN,
        ZOOM_OUT = -1,
    };

    enum tracking_cmd_t{
	TRACK_IDLE = 0,
	TRACK_ACT = 1
};

    MAVPACKED(
    typedef struct {
        union {
            float       param_float;
            double      param_double;
            int64_t     param_int64;
            uint64_t    param_uint64;
            int32_t     param_int32;
            uint32_t    param_uint32;
            int16_t     param_int16;
            uint16_t    param_uint16;
            int8_t      param_int8;
            uint8_t     param_uint8;
            uint8_t     bytes[MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_LEN];
        };
        uint8_t type;
    }) param_ext_union_t;

};