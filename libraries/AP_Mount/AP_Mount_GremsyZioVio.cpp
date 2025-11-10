#include "AP_Mount_GremsyZioVio.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_GREMSY_SEARCH_MS  60000    // search for gimbal for 1 minute after startup
#define AP_MOUNT_GREMSY_ATTITUDE_INTERVAL_US    20000  // send ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE at 50hz
#define VIO_RATE_MAX_DEGS   60 //Max angular rate. Spec is 180 deg/s but that is too fast
#define VIO_CAM_SRC_COMPID 191 //Messages need to originate from 191
/*

    mavlink_param_ext_set_t p;
    memset(&p, 0, sizeof(mavlink_param_ext_set_t));
    param_ext_union_t   union_value;
    mavlink_message_t   msg;

    union_value.param_uint32 = 0;
    memcpy(&p.param_value[0], &union_value.bytes[0], MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_LEN);
    const char *pname = "OSD_MODE";
    strncpy(p.param_id, pname, MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_ID_LEN);
    p.target_system = 1;
    p.target_component = 101;
    p.param_type = 5;

    static uint8_t mavlink_seq;
    mavlink_status_t *chan_status = mavlink_get_channel_status(MAVLINK_COMM_2);
    uint8_t saved_seq = chan_status->current_tx_seq;
    chan_status->current_tx_seq = mavlink_seq++;
    mavlink_msg_param_ext_set_pack(
                        1,
                        191,
                        &msg,
                        1,
                        101,
                        p.param_id,
                        p.param_value,
                        p.param_type);
    chan_status->current_tx_seq = saved_seq;
    _mavlink_resend_uart(MAVLINK_COMM_2, &msg);


*/
AP_Mount_GremsyZioVio::AP_Mount_GremsyZioVio(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance) :
    AP_Mount_Backend(frontend, params, instance)
{}

// update mount position
void AP_Mount_GremsyZioVio::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_gimbal();
        return;
    }
    if(_zoom_mode == -1 || _rec_source == -1 || _osd_state == -1 || _obj_detect == -1){
        request_param_ext_list();
    }
    write_gimbal_log();
}

void AP_Mount_GremsyZioVio::write_gimbal_log(){
    AP_Logger *logger = AP_Logger::get_singleton();
    logger->write_gremsy_log(_camera_pan_angle, _camera_tilt_angle, _camera_roll_angle);
}

void AP_Mount_GremsyZioVio::payload_take_picture()
{
    send_cmd_long(1, 101, MAV_CMD_IMAGE_START_CAPTURE, 0, 0, 0, 0, 0, 0, 0);
}

void AP_Mount_GremsyZioVio::start_record()
{
    send_cmd_long(1, 101, MAV_CMD_VIDEO_START_CAPTURE, 0, 0, 0, 0, 0, 0, 0);
}

void AP_Mount_GremsyZioVio::stop_record()
{
    send_cmd_long(1, 101, MAV_CMD_VIDEO_STOP_CAPTURE, 0, 0, 0, 0, 0, 0, 0);
}

void AP_Mount_GremsyZioVio::eo_full_screen()
{
    send_param_set(c_source_id, 1);
}

void AP_Mount_GremsyZioVio::ir_full_screen()
{
    send_param_set(c_source_id, 2);
}

void AP_Mount_GremsyZioVio::eo_ir()
{
    send_param_set(c_source_id, 0);
}

void AP_Mount_GremsyZioVio::ir_eo()
{
    send_param_set(c_source_id, 3);
}

void AP_Mount_GremsyZioVio::next_color_pallette()
{
    //use function
    static int current_color = 0;
    current_color = ((current_color + 1) % 10);
    send_param_set(c_palette_id, current_color);
}

void AP_Mount_GremsyZioVio::fusion(){
    send_param_set(c_source_id, 4);
}

void AP_Mount_GremsyZioVio::set_osd_mode(int arg){
    send_param_set(osd_mode_id, arg);
}
void AP_Mount_GremsyZioVio::set_record_source(int arg){
    send_param_set(rec_src_id, arg);
}
void AP_Mount_GremsyZioVio::set_zoom_mode(int arg){
    send_param_set(zoom_mode_id, arg);
}
void AP_Mount_GremsyZioVio::ir_zoom(int arg){
    
    send_param_set(ir_zoom_id, arg);
}
void AP_Mount_GremsyZioVio::set_detection(int arg){
    send_param_set(track_mode_id, arg);
}

void AP_Mount_GremsyZioVio::set_zoom_rate(float zoom_speed) {
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }
    if (_link == nullptr) {
        return;
    }
    static float prev_zoom_speed = 0.0;
    if(zoom_speed < 0){_current_zoom_state = ZOOM_IN;}
	else if(zoom_speed > 0){_current_zoom_state = ZOOM_OUT;}
	else{_current_zoom_state = ZOOM_STOP;}

	if(is_equal(zoom_speed, prev_zoom_speed)) return;
	prev_zoom_speed = zoom_speed;
    mavlink_message_t   msg;
    static uint8_t mavlink_seq;
    mavlink_status_t *chan_status = mavlink_get_channel_status(MAVLINK_COMM_2);
    uint8_t saved_seq = chan_status->current_tx_seq;
    chan_status->current_tx_seq = mavlink_seq++;
    mavlink_msg_command_long_pack(
                        1,
                        191,
                        &msg,
                        1,
                        101,
                        531,
                        0,
                        1,
                        _current_zoom_state,
                        0,
                        0,
                        0,
                        0,
                        0);
    chan_status->current_tx_seq = saved_seq;
    _mavlink_resend_uart(MAVLINK_COMM_2, &msg);
}

void AP_Mount_GremsyZioVio::set_gimbal_rates(float pan_speed, float tilt_speed)  {
    static bool flag = true;
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    static float prev_pan_input = 0.0;
	static float prev_tilt_input = 0.0;
    if(is_equal(prev_pan_input, pan_speed) && //Pan speed is same as last time
	   is_equal(prev_tilt_input, tilt_speed) && //Tilt speed is same as last time
	  (is_equal(pan_speed, 0.0f) && is_equal(tilt_speed, 0.0f))){ //The repeat speed is 0
        if(!flag){
            stop_tracking();
            flag = true;
        }
		return;
	}
    if(flag){
        flag = false;
    }
    prev_pan_input = pan_speed;
	prev_tilt_input = tilt_speed;
    
    send_gimbal_device_set_rate(0, (tilt_speed * radians(VIO_RATE_MAX_DEGS)), (pan_speed * radians(VIO_RATE_MAX_DEGS)), true);
}

bool AP_Mount_GremsyZioVio::center_camera(bool follow_camera) {
    stop_tracking();
    send_param_set(gb_mode_id, 4);
    return true;
}

void AP_Mount_GremsyZioVio::set_angles(float pan_input, float tilt_input)
{
    stop_tracking();
    send_gimbal_device_set_attitude(0, radians(tilt_input), radians(pan_input), false);
}

void AP_Mount_GremsyZioVio::look_down()
{
    stop_tracking();
    send_param_set(gb_mode_id, 3);
}

void AP_Mount_GremsyZioVio::stow()
{
    stop_tracking();
    const Vector3f &angle_bf_target = _params.neutral_angles.get();
    send_gimbal_device_set_attitude(ToRad(angle_bf_target.x), ToRad(_params.pitch_angle_max.get()), ToRad(angle_bf_target.z), false);
}

void AP_Mount_GremsyZioVio::send_param_set(const char *id, int val)
{
    if (!_initialised) {
        return;
    }
    if (_link == nullptr) {
        return;
    }

    mavlink_param_ext_set_t p;
    memset(&p, 0, sizeof(mavlink_param_ext_set_t));
    param_ext_union_t   union_value;
    mavlink_message_t   msg;
    union_value.param_uint32 = val;
    memcpy(&p.param_value[0], &union_value.bytes[0], MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_LEN);
    const char *pname = id;
    strncpy(p.param_id, pname, MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_ID_LEN);
    p.param_id[MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_ID_LEN - 1] = '\0';
    p.target_system = 1;
    p.target_component = 101;
    p.param_type = 5;
    static uint8_t mavlink_seq;
    mavlink_status_t *chan_status = mavlink_get_channel_status(MAVLINK_COMM_2);
    uint8_t saved_seq = chan_status->current_tx_seq;
    chan_status->current_tx_seq = mavlink_seq++;
    mavlink_msg_param_ext_set_pack(
                        1,
                        191,
                        &msg,
                        1,
                        101,
                        p.param_id,
                        p.param_value,
                        p.param_type);
    chan_status->current_tx_seq = saved_seq;
    _mavlink_resend_uart(MAVLINK_COMM_2, &msg);
}

void AP_Mount_GremsyZioVio::send_cmd_long(int sys, int comp, int cmd, int p1, int p2, int p3, int p4, int p5, int p6, int p7) {
    if (!_initialised) {
        return;
    }
    if (_link == nullptr) {
        return;
    }
    mavlink_message_t msg;
    static uint8_t mavlink_seq;
    mavlink_status_t *chan_status = mavlink_get_channel_status(MAVLINK_COMM_2);
    uint8_t saved_seq = chan_status->current_tx_seq;
    chan_status->current_tx_seq = mavlink_seq++;
    mavlink_msg_command_long_pack(1, 191, &msg, sys, comp, cmd, 0, p1, p2, p3, p4, p5, p6, p7);
    chan_status->current_tx_seq = saved_seq;
    _mavlink_resend_uart(MAVLINK_COMM_2, &msg);
}

void AP_Mount_GremsyZioVio::request_param_ext_list() {
    if (!_initialised) {
        return;
    }
    if (_link == nullptr) {
        return;
    }
    mavlink_message_t msg;
    static uint8_t mavlink_seq;
    mavlink_status_t *chan_status = mavlink_get_channel_status(MAVLINK_COMM_2);
    uint8_t saved_seq = chan_status->current_tx_seq;
    chan_status->current_tx_seq = mavlink_seq++;
    mavlink_msg_param_ext_request_list_pack(1, 191, &msg, 1, 101);
    chan_status->current_tx_seq = saved_seq;
    _mavlink_resend_uart(MAVLINK_COMM_2, &msg);
}

void AP_Mount_GremsyZioVio::request_payload_info() {
    if (!_initialised) {
        return;
    }
    if (_link == nullptr) {
        return;
    }
    if(_osd_state != -1) {
        //Send Data to GCS
        mavlink_command_long_t cmd;
        cmd.command = MAV_CMD_ASCENT_PAYLOAD;
        cmd.target_system = 255;
        cmd.target_component = 0;
        cmd.param1 = 1; // Gremsy Vio message back to the GCS
        cmd.param2 = 3;  //Param
        cmd.param3 = _osd_state; //Value
        gcs().send_to_active_channels(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&cmd);
    }
    if(_rec_source != -1) {
        //Send Data to GCS
        mavlink_command_long_t cmd;
        cmd.command = MAV_CMD_ASCENT_PAYLOAD;
        cmd.target_system = 255;
        cmd.target_component = 0;
        cmd.param1 = 1; // Gremsy Vio message back to the GCS
        cmd.param2 = 2;  //Param
        cmd.param3 = _rec_source; //Value
        gcs().send_to_active_channels(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&cmd);
    }
    if(_zoom_mode != -1) {
        //Send Data to GCS
        mavlink_command_long_t cmd;
        cmd.command = MAV_CMD_ASCENT_PAYLOAD;
        cmd.target_system = 255;
        cmd.target_component = 0;
        cmd.param1 = 1; // Gremsy Vio message back to the GCS
        cmd.param2 = 1;  //Param
        cmd.param3 = _zoom_mode; //Value
        gcs().send_to_active_channels(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&cmd);
    }
    if(_obj_detect != -1) {
        //Send Data to GCS
        mavlink_command_long_t cmd;
        cmd.command = MAV_CMD_ASCENT_PAYLOAD;
        cmd.target_system = 255;
        cmd.target_component = 0;
        cmd.param1 = 1; // Gremsy Vio message back to the GCS
        cmd.param2 = 0;  //Param
        cmd.param3 = _obj_detect; //Value
        gcs().send_to_active_channels(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&cmd);
    }
}

void AP_Mount_GremsyZioVio::handle_param_ext_value(const mavlink_message_t &msg) {
    mavlink_param_ext_value_t p;
    mavlink_msg_param_ext_value_decode(&msg, &p);

    param_ext_union_t   union_value;
    memcpy(&union_value.bytes[0], &p.param_value[0], MAVLINK_MSG_PARAM_EXT_SET_FIELD_PARAM_VALUE_LEN);

    if (strcmp(p.param_id, "OSD_MODE") == 0) {
        if(_osd_state != union_value.param_uint32) {
            _osd_state = union_value.param_uint32;
            //Send Data to GCS
            mavlink_command_long_t cmd;
		    cmd.command = MAV_CMD_ASCENT_PAYLOAD;
            cmd.target_system = 255;
		    cmd.target_component = 0;
            cmd.param1 = 1; // Gremsy Vio message back to the GCS
            cmd.param2 = 3;  //Param
            cmd.param3 = _osd_state; //Value
            gcs().send_to_active_channels(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&cmd);
        }
    }
    if (strcmp(p.param_id, "C_V_REC") == 0) {
        if(_rec_source != union_value.param_uint32) {
            _rec_source = union_value.param_uint32;
            //Send Data to GCS
            mavlink_command_long_t cmd;
		    cmd.command = MAV_CMD_ASCENT_PAYLOAD;
            cmd.target_system = 255;
		    cmd.target_component = 0;
            cmd.param1 = 1; // Gremsy Vio message back to the GCS
            cmd.param2 = 2;  //Param
            cmd.param3 = _rec_source; //Value
            gcs().send_to_active_channels(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&cmd);
        }
    }
    if (strcmp(p.param_id, "C_V_ZM_MODE") == 0) {
        if(_zoom_mode != union_value.param_uint32) {
            _zoom_mode = union_value.param_uint32;
            //Send Data to GCS
            mavlink_command_long_t cmd;
            cmd.command = MAV_CMD_ASCENT_PAYLOAD;
            cmd.target_system = 255;
            cmd.target_component = 0;
            cmd.param1 = 1; // Gremsy Vio message back to the GCS
            cmd.param2 = 1;  //Param
            cmd.param3 = _zoom_mode; //Value
            gcs().send_to_active_channels(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&cmd);
        }
    }
    if (strcmp(p.param_id, "TRACK_MODE") == 0) {
        if(_obj_detect != union_value.param_uint32) {
            _obj_detect = union_value.param_uint32;
            //Send Data to GCS
            mavlink_command_long_t cmd;
            cmd.command = MAV_CMD_ASCENT_PAYLOAD;
            cmd.target_system = 255;
            cmd.target_component = 0;
            cmd.param1 = 1; // Gremsy Vio message back to the GCS
            cmd.param2 = 0;  //Param
            cmd.param3 = _obj_detect; //Value
            gcs().send_to_active_channels(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&cmd);
        }
    }
}

void AP_Mount_GremsyZioVio::start_tracking( int x, int y) {
    send_cmd_long(1, MAV_COMP_ID_USER2, MAV_CMD_USER_4, 4, 0, TRACK_ACT, x, y, 0, 0);
}
void AP_Mount_GremsyZioVio::stop_tracking() {
    send_cmd_long(1, MAV_COMP_ID_USER2, MAV_CMD_USER_4, 4, 0, TRACK_IDLE, 0, 0, 0, 0);
}

/////////////////////////////////////////
//       STOCK ARDUPILOT METHODS
/////////////////////////////////////////

// return true if healthy
bool AP_Mount_GremsyZioVio::healthy() const
{
    // unhealthy until gimbal has been found and replied with device info
    if (_link == nullptr || !_got_device_info) {
        return false;
    }

    // unhealthy if attitude information NOT received within the last second
    if (AP_HAL::millis() - _last_attitude_status_ms > 1000) {
        return false;
    }

    // check failure flags
    uint32_t critical_failure_flags = GIMBAL_DEVICE_ERROR_FLAGS_ENCODER_ERROR |
                                      GIMBAL_DEVICE_ERROR_FLAGS_POWER_ERROR |
                                      GIMBAL_DEVICE_ERROR_FLAGS_MOTOR_ERROR |
                                      GIMBAL_DEVICE_ERROR_FLAGS_SOFTWARE_ERROR |
                                      GIMBAL_DEVICE_ERROR_FLAGS_COMMS_ERROR;

    if ((_gimbal_device_attitude_status.failure_flags & critical_failure_flags) > 0) {
        return false;
    }

    // if we get this far return mount is healthy
    return true;
}

// get attitude as a quaternion.  returns true on success
bool AP_Mount_GremsyZioVio::get_attitude_quaternion(Quaternion& att_quat)
{
    att_quat = _gimbal_device_attitude_status.q;
    return true;
}

// search for gimbal in GCS_MAVLink routing table
void AP_Mount_GremsyZioVio::find_gimbal()
{
    // do not look for gimbal for first 10 seconds so user may see banner
    uint32_t now_ms = AP_HAL::millis();
    if (now_ms < 10000) {
        return;
    }

    // search for gimbal for 60 seconds or until armed
    if ((now_ms > AP_MOUNT_GREMSY_SEARCH_MS) && hal.util->get_soft_armed()) {
        return;
    }

    // search for a mavlink enabled gimbal
    if (_link == nullptr) {
        // we expect that instance 0 has compid = MAV_COMP_ID_GIMBAL, instance 1 has compid = MAV_COMP_ID_GIMBAL2, etc
        uint8_t compid = (_instance == 0) ? MAV_COMP_ID_GIMBAL : MAV_COMP_ID_GIMBAL2 + (_instance - 1);
        _link = GCS_MAVLINK::find_by_mavtype_and_compid(MAV_TYPE_GIMBAL, compid, _sysid);
        if (_link == nullptr) {
            // have not yet found a gimbal so return
            return;
        }

        _compid = compid;
    }

    // request GIMBAL_DEVICE_INFORMATION
    if (!_got_device_info) {
        if (now_ms - _last_devinfo_req_ms > 1000) {
            _last_devinfo_req_ms = now_ms;
            request_gimbal_device_information();
        }
        return;
    }

    // start sending autopilot attitude to gimbal
    if (start_sending_attitude_to_gimbal()) {
        _initialised = true;
    }
}

// handle GIMBAL_DEVICE_INFORMATION message
void AP_Mount_GremsyZioVio::handle_gimbal_device_information(const mavlink_message_t &msg)
{
    // exit immediately if this is not our message
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    mavlink_gimbal_device_information_t info;
    mavlink_msg_gimbal_device_information_decode(&msg, &info);

    // set parameter defaults from gimbal information
    _params.roll_angle_min.set_default(degrees(info.roll_min));
    _params.roll_angle_max.set_default(degrees(info.roll_max));
    _params.pitch_angle_min.set_default(degrees(info.pitch_min));
    _params.pitch_angle_max.set_default(degrees(info.pitch_max));
    _params.yaw_angle_min.set_default(degrees(info.yaw_min));
    _params.yaw_angle_max.set_default(degrees(info.yaw_max));

    const uint8_t fw_ver_major = info.firmware_version & 0x000000FF;
    const uint8_t fw_ver_minor = (info.firmware_version & 0x0000FF00) >> 8;
    const uint8_t fw_ver_revision = (info.firmware_version & 0x00FF0000) >> 16;
    const uint8_t fw_ver_build = (info.firmware_version & 0xFF000000) >> 24;

    // display gimbal info to user
    gcs().send_text(MAV_SEVERITY_INFO, "Mount: %s %s fw:%u.%u.%u.%u",
            info.vendor_name,
            info.model_name,
            (unsigned)fw_ver_major,
            (unsigned)fw_ver_minor,
            (unsigned)fw_ver_revision,
            (unsigned)fw_ver_build);

    _got_device_info = true;
}

// handle GIMBAL_DEVICE_ATTITUDE_STATUS message
void AP_Mount_GremsyZioVio::handle_gimbal_device_attitude_status(const mavlink_message_t &msg)
{
    // exit immediately if this is not our message
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    // take copy of message so it can be forwarded onto GCS later
    mavlink_msg_gimbal_device_attitude_status_decode(&msg, &_gimbal_device_attitude_status);
    _last_attitude_status_ms = AP_HAL::millis();
}

// request GIMBAL_DEVICE_INFORMATION message
void AP_Mount_GremsyZioVio::request_gimbal_device_information() const
{
    if (_link == nullptr) {
        return;
    }

    const mavlink_command_long_t pkt {
        MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION,  // param1
        0,  // param2
        0,  // param3
        0,  // param4
        0,  // param5
        0,  // param6
        0,  // param7
        MAV_CMD_REQUEST_MESSAGE,
        _sysid,
        _compid,
        0  // confirmation
    };

    _link->send_message(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&pkt);
}

// start sending ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE to gimbal
bool AP_Mount_GremsyZioVio::start_sending_attitude_to_gimbal()
{
    // better safe than sorry:
    if (_link == nullptr) {
        return false;
    }
    // send AUTOPILOT_STATE_FOR_GIMBAL_DEVICE
    const MAV_RESULT res = _link->set_message_interval(MAVLINK_MSG_ID_AUTOPILOT_STATE_FOR_GIMBAL_DEVICE, AP_MOUNT_GREMSY_ATTITUDE_INTERVAL_US);

    // return true on success
    return (res == MAV_RESULT_ACCEPTED);
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to command gimbal to retract (aka relax)
void AP_Mount_GremsyZioVio::send_gimbal_device_retract() const
{
    const mavlink_gimbal_device_set_attitude_t pkt {
        {NAN, NAN, NAN, NAN},  // attitude
        0,   // angular velocity x
        0,  // angular velocity y
        0,    // angular velocity z
        GIMBAL_DEVICE_FLAGS_RETRACT,  // flags
        _sysid,
        _compid
    };

    _link->send_message(MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE, (const char*)&pkt);
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control rate
// earth_frame should be true if yaw_rads target is an earth frame rate, false if body_frame
void AP_Mount_GremsyZioVio::send_gimbal_device_set_rate(float roll_rads, float pitch_rads, float yaw_rads, bool earth_frame) const
{
    // prepare flags
    const uint16_t flags = earth_frame ? (GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_YAW_LOCK) : 0;

    const mavlink_gimbal_device_set_attitude_t pkt {
        {NAN, NAN, NAN, NAN},  // attitude
        roll_rads,   // angular velocity x
        pitch_rads,  // angular velocity y
        yaw_rads,    // angular velocity z
        flags,
        _sysid,
        _compid
    };

    _link->send_message(MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE, (const char*)&pkt);
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control attitude
// earth_frame should be true if yaw_rad target is in earth frame angle, false if body_frame
void AP_Mount_GremsyZioVio::send_gimbal_device_set_attitude(float roll_rad, float pitch_rad, float yaw_rad, bool earth_frame) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // prepare flags
    const uint16_t flags = earth_frame ? (GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_YAW_LOCK) : 0;

    // convert euler angles to quaternion
    Quaternion q;
    q.from_euler(roll_rad, pitch_rad, yaw_rad);

    const mavlink_gimbal_device_set_attitude_t pkt {
        {q.q1, q.q2, q.q3, q.q4},
        NAN,  // angular velocity x
        NAN,  // angular velocity y
        NAN,  // angular velocity z
        flags,
        _sysid,
        _compid
    };

    _link->send_message(MAVLINK_MSG_ID_GIMBAL_DEVICE_SET_ATTITUDE, (const char*)&pkt);
}