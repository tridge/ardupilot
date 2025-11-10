#include "AP_Mount_GremsyAirpixel.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

#define AP_MOUNT_GREMSY_RESEND_MS  1000     // resend angle targets to gimbal at least once per second
#define AP_MOUNT_GREMSY_SEARCH_MS  60000    // search for gimbal for 1 minute after startup
#define AP_MOUNT_GREMSY_ATTITUDE_INTERVAL_US    20000  // send ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE at 50hz
#define PIXY_LR_RATE_MAX_DEGS   90 //Max angular rate. Spec is 180 deg/s but that is too fast

AP_Mount_GremsyAirpixel::AP_Mount_GremsyAirpixel(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance) :
    AP_Mount_Backend(frontend, params, instance)
{}

// update mount position
void AP_Mount_GremsyAirpixel::update()
{
    // exit immediately if not initialised
    if (!_initialised) {
        find_gimbal();
        return;
    }

    MountTarget angle_target_rad {};
	if (get_mode()==MAV_MOUNT_MODE_GPS_POINT){
		if (get_angle_target_to_roi(angle_target_rad)) {
            //When sending "unlocked" angles, the frame is relative to bootup and not earth...
            send_gimbal_device_set_attitude(angle_target_rad.roll, angle_target_rad.pitch, (angle_target_rad.yaw - _initial_quat.get_euler_yaw()), angle_target_rad.yaw_is_ef); 
        }
    }
    write_gimbal_log();
}


void AP_Mount_GremsyAirpixel::handle_data64_extension(const mavlink_message_t &msg)
{
    //Retrieve Data
    uint8_t buf[64];
    memcpy(&buf, _MAV_PAYLOAD(&msg), msg.len);
    memcpy(&geotagging_mode, &buf[6],1);
    memcpy(&geotagging_session, &buf[7],1);
    //Send Data to GCS
    mavlink_msg_command_long_send(MAVLINK_COMM_1, // Assumes Telemetry is on COM1
                                  255,
                                  0,
                                  MAV_CMD_ASCENT_PAYLOAD,
                                  0, // confirmation of zero means this is the first time this message has been sent
                                  0, // Gremsy message back to the GCS
                                  geotagging_mode, geotagging_session, session_pictures, 0, 0, 0); 
}

void AP_Mount_GremsyAirpixel::handle_data16_extension(const mavlink_message_t &msg)
{
    uint8_t photos_l;
    uint8_t photos_h;
    uint8_t buf[64];
    memcpy(&buf, _MAV_PAYLOAD(&msg), msg.len);
    memcpy(&geotagging_progress, &buf[5],1);
    memcpy(&photos_l, &buf[6],1);
    memcpy(&photos_h, &buf[7],1);
    session_pictures = (photos_h << 8) | photos_l;
}

void AP_Mount_GremsyAirpixel::write_gimbal_log(){
	AP_Logger *logger = AP_Logger::get_singleton();
	logger->write_ga_log(geotagging_mode, geotagging_session, session_pictures, _camera_tilt_angle, geotagging_progress, _camera_pan_angle);
}

void AP_Mount_GremsyAirpixel::set_gimbal_rates(float pan_speed, float tilt_speed)  {
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    static float prev_pan_input = 0.0;
	static float prev_tilt_input = 0.0;
    if(is_equal(prev_pan_input, pan_speed) && //Pan speed is same as last time
	   is_equal(prev_tilt_input, tilt_speed) && //Tilt speed is same as last time
	  (is_equal(pan_speed, 0.0f) && is_equal(tilt_speed, 0.0f))){ //The repeat speed is 0
		return;
	}
    prev_pan_input = pan_speed;
	prev_tilt_input = tilt_speed;

    if ( (get_mode() == MAV_MOUNT_MODE_GPS_POINT) && (!is_zero(pan_speed) || !is_zero(tilt_speed))){
		clear_roi_target(); // Break ROI targetting
	}

    send_gimbal_device_set_rate(0, (tilt_speed * radians(PIXY_LR_RATE_MAX_DEGS)), (pan_speed * radians(PIXY_LR_RATE_MAX_DEGS)), true);
}

void AP_Mount_GremsyAirpixel::set_angles(float pan_input, float tilt_input)
{
    send_gimbal_device_set_attitude(0, radians(tilt_input), radians(pan_input), false);
    send_gimbal_device_set_attitude(0, radians(tilt_input), radians(pan_input), false);// Need to send twice for some reason. If camera is too far from neutral position, it will return to center but never look down...
}

bool AP_Mount_GremsyAirpixel::center_camera(bool follow_camera) {
    clear_roi_target();
    const Vector3f &angle_bf_target = _params.neutral_angles.get();
    send_gimbal_device_set_attitude(ToRad(angle_bf_target.x), ToRad(angle_bf_target.y), ToRad(angle_bf_target.z), false);
    return true;
}

void AP_Mount_GremsyAirpixel::look_down()
{
    clear_roi_target();
    const Vector3f &angle_bf_target = _params.neutral_angles.get();
    send_gimbal_device_set_attitude(ToRad(angle_bf_target.x), ToRad(-90), ToRad(angle_bf_target.z), false);
    send_gimbal_device_set_attitude(ToRad(angle_bf_target.x), ToRad(-90), ToRad(angle_bf_target.z), false);// Need to send twice for some reason. If camera is too far from neutral position, it will return to center but never look down...
}

void AP_Mount_GremsyAirpixel::stow()
{
    clear_roi_target();
    const Vector3f &angle_bf_target = _params.neutral_angles.get();
    send_gimbal_device_set_attitude(ToRad(angle_bf_target.x), ToRad(_params.pitch_angle_max.get()), ToRad(angle_bf_target.z), false);
}

void AP_Mount_GremsyAirpixel::send_digicam_control(float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
    if (_link == nullptr) {
        return;
    }
    const mavlink_channel_t chan = _link->get_chan();

    if (!HAVE_PAYLOAD_SPACE(chan, MAVLINK_MSG_ID_COMMAND_LONG)) {
        return;
    }
    mavlink_msg_command_long_send(chan, _sysid, 105,
                                    MAV_CMD_DO_DIGICAM_CONTROL, 0,
                                    p1,p2,p3,p4,p5,p6,p7);
}

void AP_Mount_GremsyAirpixel::send_digicam_configure(float p1, float p2, float p3, float p4, float p5, float p6, float p7)
{
    if (_link == nullptr) {
        return;
    }
    const mavlink_channel_t chan = _link->get_chan();

    if (!HAVE_PAYLOAD_SPACE(chan, MAVLINK_MSG_ID_COMMAND_LONG)) {
        return;
    }
    mavlink_msg_command_long_send(chan, _sysid, 105,
                                    MAV_CMD_DO_DIGICAM_CONFIGURE, 0,
                                    p1,p2,p3,p4,p5,p6,p7);
}

void AP_Mount_GremsyAirpixel::payload_take_picture()
{
    send_digicam_control(0,0,0,0,1,0,0);
}

//Start and stop send the same command because the command is a toggle, but the Ascent payload API
//Requires a start and stop function to exist for AscentQ
void AP_Mount_GremsyAirpixel::start_record()
{
    if(!_recording){
        _recording = true;
        send_digicam_configure(106,0,0,0,0,0,0);
    }
}

void AP_Mount_GremsyAirpixel::stop_record()
{
    if(_recording){
        _recording = false;
        send_digicam_configure(106,0,0,0,0,0,0);
    }
}

void AP_Mount_GremsyAirpixel::trigger_menu_control(AP_Mount::GREMSY_MENU_CONTROL ctrl)
{
    send_digicam_configure((float)ctrl,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_display()
{
    send_digicam_configure(134,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_power()
{
    send_digicam_configure(1004,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_af()
{
    send_digicam_configure(135,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_c1()
{
    send_digicam_configure(1001,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_c2()
{
    send_digicam_configure(1002,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_c3()
{
    send_digicam_configure(1003,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_mf_up()
{
    send_digicam_configure(113,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_mf_down()
{
    send_digicam_configure(114,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_wb_up()
{
    send_digicam_configure(109,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_wb_down()
{
    send_digicam_configure(110,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_speed_up()
{
    send_digicam_configure(0,0,0,101,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_speed_down()
{
    send_digicam_configure(0,0,0,99,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_aperture_up()
{
    send_digicam_configure(0,101,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_aperture_down()
{
    send_digicam_configure(0,99,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_iso_up()
{
    send_digicam_configure(0,0,101,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_iso_down()
{
    send_digicam_configure(0,0,99,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_expcorr_up()
{
    send_digicam_configure(122,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_expcorr_down()
{
    send_digicam_configure(123,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_zoom_up()
{
    send_digicam_control(0,101,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_zoom_down()
{
    send_digicam_control(0,99,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_focus_up()
{
    send_digicam_configure(111,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::trigger_focus_down()
{
    send_digicam_configure(112,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::start_geotagging()
{
    gcs().send_text(MAV_SEVERITY_INFO, "Manually Starting Geotagging"); //For logging
    send_digicam_configure(132,0,0,0,0,0,0);
}

void AP_Mount_GremsyAirpixel::stop_geotagging()
{
    gcs().send_text(MAV_SEVERITY_INFO, "Manually Stopping Geotagging"); //For logging
    send_digicam_configure(133,0,0,0,0,0,0);
}






/////////////////////////////////////////
//       STOCK ARDUPILOT METHODS
/////////////////////////////////////////

// return true if healthy
bool AP_Mount_GremsyAirpixel::healthy() const
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
bool AP_Mount_GremsyAirpixel::get_attitude_quaternion(Quaternion& att_quat)
{
    // check we have received an updated message
    if (_gimbal_device_attitude_status.time_boot_ms == _sent_gimbal_device_attitude_status_ms) {
        return false;
    }
    _sent_gimbal_device_attitude_status_ms = _gimbal_device_attitude_status.time_boot_ms;

    att_quat.from_euler(radians(0), radians(_camera_tilt_angle), radians(_camera_pan_angle));
    
    return true;
}

// search for gimbal in GCS_MAVLink routing table
void AP_Mount_GremsyAirpixel::find_gimbal()
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
        if (!AP::ahrs().get_quaternion(_initial_quat)) {
            return;
        }
    }
}

// handle GIMBAL_DEVICE_INFORMATION message
void AP_Mount_GremsyAirpixel::handle_gimbal_device_information(const mavlink_message_t &msg)
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
void AP_Mount_GremsyAirpixel::handle_gimbal_device_attitude_status(const mavlink_message_t &msg)
{
    // exit immediately if this is not our message
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    // take copy of message so it can be forwarded onto GCS later
    mavlink_msg_gimbal_device_attitude_status_decode(&msg, &_gimbal_device_attitude_status);
    _last_attitude_status_ms = AP_HAL::millis();
}

// handle MOUNT_ORIENTATION message
void AP_Mount_GremsyAirpixel::handle_mount_orientation(const mavlink_message_t &msg)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // exit immediately if this is not our message
    if (msg.sysid != _sysid || msg.compid != _compid) {
        return;
    }

    mavlink_mount_orientation_t _mount_orientation;

    // take copy of message so it can be forwarded onto GCS later
    mavlink_msg_mount_orientation_decode(&msg, &_mount_orientation);

    _camera_pan_angle = _mount_orientation.yaw;
    _camera_tilt_angle = _mount_orientation.pitch;
}

// request GIMBAL_DEVICE_INFORMATION message
void AP_Mount_GremsyAirpixel::request_gimbal_device_information() const
{
    if (_link == nullptr) {
        return;
    }
    const mavlink_channel_t chan = _link->get_chan();

    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(chan, COMMAND_LONG)) {
        return;
    }

    mavlink_msg_command_long_send(
        chan,
        _sysid,
        _compid,
        MAV_CMD_REQUEST_MESSAGE,
        0, MAVLINK_MSG_ID_GIMBAL_DEVICE_INFORMATION, 0, 0, 0, 0, 0, 0);
}

// start sending ATTITUDE and AUTOPILOT_STATE_FOR_GIMBAL_DEVICE to gimbal
bool AP_Mount_GremsyAirpixel::start_sending_attitude_to_gimbal()
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
void AP_Mount_GremsyAirpixel::send_gimbal_device_retract() const
{
    if (_link == nullptr) {
        return;
    }
    const mavlink_channel_t chan = _link->get_chan();

    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(chan, GIMBAL_DEVICE_SET_ATTITUDE)) {
        return;
    }

    // send command_long command containing a do_mount_control command
    const float quat_array[4] = {NAN, NAN, NAN, NAN};
    mavlink_msg_gimbal_device_set_attitude_send(chan,
                                                _sysid,     // target system
                                                _compid,    // target component
                                                GIMBAL_DEVICE_FLAGS_RETRACT,    // gimbal device flags
                                                quat_array, // attitude as a quaternion
                                                0, 0, 0);   // angular velocities
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control rate
// earth_frame should be true if yaw_rads target is an earth frame rate, false if body_frame
void AP_Mount_GremsyAirpixel::send_gimbal_device_set_rate(float roll_rads, float pitch_rads, float yaw_rads, bool earth_frame) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }
    if (_link == nullptr) {
        return;
    }
    const mavlink_channel_t chan = _link->get_chan();
    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(chan, GIMBAL_DEVICE_SET_ATTITUDE)) {
        return;
    }

    // prepare flags
    const uint16_t flags = earth_frame ? (GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_YAW_LOCK) : 0;
    const float quat_array[4] = {NAN, NAN, NAN, NAN};

    // send command_long command containing a do_mount_control command
    mavlink_msg_gimbal_device_set_attitude_send(chan,
                                                _sysid,     // target system
                                                _compid,    // target component
                                                flags,      // gimbal device flags
                                                quat_array, // attitude as a quaternion
                                                roll_rads, pitch_rads, yaw_rads);   // angular velocities
}

// send GIMBAL_DEVICE_SET_ATTITUDE to gimbal to control attitude
// earth_frame should be true if yaw_rad target is in earth frame angle, false if body_frame
void AP_Mount_GremsyAirpixel::send_gimbal_device_set_attitude(float roll_rad, float pitch_rad, float yaw_rad, bool earth_frame) const
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }
    if (_link == nullptr) {
        return;
    }
    const mavlink_channel_t chan = _link->get_chan();
    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(chan, GIMBAL_DEVICE_SET_ATTITUDE)) {
        return;
    }

    // prepare flags
    const uint16_t flags = earth_frame ? (GIMBAL_DEVICE_FLAGS_ROLL_LOCK | GIMBAL_DEVICE_FLAGS_PITCH_LOCK | GIMBAL_DEVICE_FLAGS_YAW_LOCK) : 0;

    // convert euler angles to quaternion
    Quaternion q;
    q.from_euler(roll_rad, pitch_rad, yaw_rad);
    const float quat_array[4] = {q.q1, q.q2, q.q3, q.q4};

    // send command_long command containing a do_mount_control command
    mavlink_msg_gimbal_device_set_attitude_send(chan,
                                                _sysid,     // target system
                                                _compid,    // target component
                                                flags,      // gimbal device flags
                                                quat_array, // attitude as a quaternion
                                                NAN, NAN, NAN);   // angular velocities
}