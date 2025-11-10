#include <GCS_MAVLink/GCS.h>

#include "AP_Mount_NextVision.h"
#include <AP_HAL/AP_HAL.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <GCS_MAVLink/include/mavlink/v2.0/checksum.h>
#include <GCS_MAVLink/include/mavlink/v2.0/common/common.h>	//
#include <AP_GPS/AP_GPS.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <AP_Logger/AP_Logger.h>

#define CMD_PRINTF 1  // Comment out to turn printf off

//Speeds at which the gimbal will move in deg/s
#define CAM_SPD_MAX 25.0 //zoomed out  dac - need to verify
#define CAM_SPD_MIN 2.0 // zoomed in    "      "

extern const AP_HAL::HAL& hal;

AP_Mount_NextVision::AP_Mount_NextVision(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance) :
    AP_Mount_Backend(frontend, params, instance),
    _chan(MAVLINK_COMM_2)
{
}

// init - performs any required initialisation for this instance
void AP_Mount_NextVision::init()
{
    const AP_SerialManager& serial_manager = AP::serialmanager();
    if(_instance > 0){
    	_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MAVLink2, 1);
    }else{
    	_port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_MAVLink2, 0);
    }
    if (_port) {
		set_observation_mode();  // for debug
		set_cam_stabilization((bool) 1);
		find_gimbal();

		// Set the Joystick mode to normal - 
		mavlink_msg_command_long_send(_chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									NV_SET_NORMAL_JOYSTICK,
									1, 0, 0, 0, 
									0, 0);  // param2 ~ param7 unused
		set_derotation(true);
    }

	_current_zoom_state = ZOOM_STOP;
    _is_recording = false;
}

void AP_Mount_NextVision::measure_freq(char name[5])
{
static uint32_t timer_diff[20];
static uint8_t  timer_idx = 0;
static uint32_t last_tick = 0;
uint32_t current_tick;
uint32_t sum = 0;


current_tick = AP_HAL::millis();

		timer_diff[timer_idx++] = current_tick - last_tick;
		last_tick = current_tick;

		if ((timer_idx % 20) == 0)
		{
			name[4] = 0;

			for (uint8_t i = 0; i < 20; i++)
			{
				sum = sum + timer_diff[i]; 
			}

		}

		if(timer_idx >= 20)
		{
			timer_idx = 0;
		} 

}

// update mount position - seems to be called at about 50 Hz
//
void AP_Mount_NextVision::update()
{
	MountTarget angle_target_rad {};
	if (get_mode()==MAV_MOUNT_MODE_GPS_POINT){
		if (get_angle_target_to_roi(angle_target_rad)) {
			mavlink_msg_command_long_send(  _chan,
										_sysid,
										_compid,
										MAV_CMD_DO_DIGICAM_CONTROL,
										0,        // confirmation of zero means this is the first time this message has been sent
										NV_SET_SYSTEM_MODE,
										5,  //Global Position
										degrees(angle_target_rad.pitch),        
										degrees(angle_target_rad.yaw),       
										0, 
										0, 0);  // param6 ~ param7 unused
			_camera_mode = NV_SET_GLOBAL_MODE;
		}
	}

    write_gimbal_log();
}

bool AP_Mount_NextVision::get_attitude_quaternion(Quaternion& att_quat) {
    // construct quaternion
    att_quat.from_euler(radians(0), radians(_camera_pan_angle), radians(_camera_tilt_angle));
    return true;
}

void AP_Mount_NextVision::write_gimbal_log(){
	AP_Logger *logger = AP_Logger::get_singleton();
	logger->write_nv_log(_camera_tilt_angle, _camera_pan_angle, 0, _camera_pan_rate, bit_report.internal_error, bit_report.day_sensor, bit_report.ir_sensor, bit_report.camera_overheat, bit_report.camera_communication, camera_temperature, trip_temperature);
}

// search for camera in GCS_MAVLink routing table
void AP_Mount_NextVision::find_gimbal()
{
    // return immediately if initialised
    if (_initialised) {
        return;
    }

    // return if search time has has passed
    if (AP_HAL::millis() > AP_MOUNT_NEXTVISION_SEARCH_MS) {
        return;
    }

    if (GCS_MAVLINK::find_by_mavtype(MAV_TYPE_CAMERA, _sysid, _compid, _chan)) {
        _initialised = true;
    }
}

//Runs as 10 Hz per UserCode
bool AP_Mount_NextVision::healthy() const { //10hz
	if(AP_HAL::millis() - _feedback_timeout > PAYLOAD_TIMEOUT_MS) {
		gcs().send_text(MAV_SEVERITY_INFO,"GIMBAL FEEDBACK TIMEOUT");
		return false;
	}
	if(fabs(_camera_pan_angle) > MAX_PAN_ANGLE_THRESHOLD) {
		gcs().send_text(MAV_SEVERITY_INFO,"GIMBAL ANGLE THRESHOLD EXCEEDED");
		return false;
	}
	if(fabs(_delta_camera_pan_angle) > DELTA_PAN_ANGLE_TH) {
		gcs().send_text(MAV_SEVERITY_INFO,"GIMBAL DELTA ANGLE EXCEEDED");
		return false;
	}
	if(bit_report.internal_error || bit_report.camera_overheat || bit_report.camera_communication) {
		gcs().send_text(MAV_SEVERITY_INFO,"GIMBAL ERROR");
		return false;
	}
	return true;
}

void AP_Mount_NextVision::handle_v2_extension(const mavlink_message_t &msg){
	uint16_t report_type;
	uint8_t buf[256];
    memcpy(&buf, _MAV_PAYLOAD(&msg), msg.len);
	memcpy(&report_type, &buf[0],2);
	if(!_init_reports){
		_init_reports = true;
		set_report_frequency(report_type, 0);
		set_report_frequency(0, 5);
		set_report_frequency(1, 2);
		set_report_frequency(2, 5);

		mavlink_msg_command_long_send(_chan,
					_sysid,
					_compid,
					MAV_CMD_DO_DIGICAM_CONTROL,
					0,        // confirmation of zero means this is the first time this message has been sent
					NV_SET_NORMAL_JOYSTICK,
					1, 0, 0, 0, 
					0, 0);  // param2 ~ param7 unused
	}
	else if (report_type == 0){ //5hz
		memcpy(&_camera_pan_angle, &buf[2], 4);
		memcpy(&_camera_tilt_angle, &buf[6], 4);
		if(_topMount) _camera_pan_angle = _camera_pan_angle * -1;
		uint32_t _current_time = AP_HAL::millis();
		_delta_camera_pan_angle = _camera_pan_angle - _prev_camera_pan_angle;
		_camera_pan_rate = (_delta_camera_pan_angle / ((_current_time - _feedback_timeout) * 1000));
		_prev_camera_pan_angle = _camera_pan_angle;
		_feedback_timeout = _current_time;

		uint16_t br;
		memcpy(&br, &buf[41], 2);
		bit_report.internal_error = (br & 1);
		bit_report.roll_motor = (br & (1 << 1)); //Not Dynamic
		bit_report.pitch_motor = (br & (1 << 2)); //Not Dynamic
		bit_report.day_sensor = (br & (1 << 4));
		bit_report.ir_sensor = (br & (1 << 5));
		bit_report.camera_overheat = (br & (1 << 7));
		bit_report.camera_communication = (br & (1 << 15));

		memcpy(&trip_temperature, &buf[29], 4);
		memcpy(&camera_temperature, &buf[53], 4);
	}
}

// has_pan_control - returns true if this mount can control it's pan (required for multicopters)
bool AP_Mount_NextVision::has_pan_control() const
{
    // we do not have yaw control
    return true;
}

void AP_Mount_NextVision::set_cam_stabilization(bool en){
    mavlink_msg_command_long_send(_chan,
                                  _sysid,
                                  _compid,
                                  MAV_CMD_DO_DIGICAM_CONTROL,
								  0,        // confirmation of zero means this is the first time this message has been sent
                                  NV_SET_CAMERA_STABILIZATION,
                                  (float)en,
								  0, 0, 0, 0, 0);  // param3 ~ param7 unused


}

void AP_Mount_NextVision::set_derotation(bool en) {
	mavlink_msg_command_long_send(_chan,
                                  _sysid,
                                  _compid,
                                  MAV_CMD_DO_DIGICAM_CONTROL,
								  0,        // confirmation of zero means this is the first time this message has been sent
                                  18,
                                  en,
								  0, 0, 0, 0, 0);  // param3 ~ param7 unused
}

void AP_Mount_NextVision::enable_single_yaw() {
	mavlink_msg_command_long_send(  _chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									9,1,0,0,0,0,0);
}

void AP_Mount_NextVision::disable_single_yaw() {
	mavlink_msg_command_long_send(  _chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									9,0,0,0,0,0,0);
}

void AP_Mount_NextVision::set_observation_mode(void)
{
    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }
    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }
    // send command_long command containing 
    mavlink_msg_command_long_send(_chan,
                                  _sysid,
                                  _compid,
                                  MAV_CMD_DO_DIGICAM_CONTROL,
								  0,        // confirmation of zero means this is the first time this message has been sent
                                  (float)NV_SET_SYSTEM_MODE,
								  (float)NV_SET_OBSERVATION_MODE,
                                  0, 0, 0, 0, 0);  // param3 ~ param7 unused

	/* Set the mode in the MOUNT structure */
//	set_mode(MAV_MOUNT_MODE_OBSERVATION);

	/* Keep track of the mode in our local variables */
	_camera_mode = NV_SET_OBSERVATION_MODE;

    // store time of send
    _last_send = AP_HAL::millis();
}

//	Toggles camera through its three modes; Daylight, IR mode Black Hot; IR mode White Hot
//
//
void AP_Mount_NextVision::set_camera_mode(void)
{
	uint16_t cmd1 = NV_NO_CMD;
	uint16_t cmd2 = NV_NO_CMD; 
	uint16_t cmd3 = NV_NO_CMD; 
	float p1 = NV_NO_CMD;
	float p2 = NV_NO_CMD;
	float p3 = NV_NO_CMD;

    // exit immediately if not initialised
    if (!_initialised) {
        return;
    }

    // check we have space for the message
    if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }

	//  Determine commands needed to get us to the next mode.
	//
	if ( _sensor_mode == NV_CAMERA_DAY_MODE )
	{
		cmd1 = NV_SET_SENSOR;
		p1 = NV_SENSOR_DAY;

	}
	else if ( _sensor_mode == NV_CAMERA_IR_MODE_BLACK )
	{
		cmd1 = NV_SET_SENSOR;
		p1 = NV_SENSOR_IR;
		cmd2 = NV_SET_IR_POLARITY;
		p2 = NV_IR_POLARITY_BLACK;
		cmd3 = NV_SET_IR_COLOR;
		p3 = NV_IR_NO_COLOR;
	}else if ( _sensor_mode == NV_CAMERA_IR_MODE_WHITE )
	{
		cmd1 = NV_SET_SENSOR;
		p1 = NV_SENSOR_IR;
		cmd2 = NV_SET_IR_POLARITY;
		p2 = NV_IR_POLARITY_WHITE;
		cmd3 = NV_SET_IR_COLOR;
		p3 = NV_IR_NO_COLOR;
	}

	// Send the necessary commnands
	if ( cmd1 != NV_NO_CMD )
	{
    // send command_long command containing a do_mount_control command
    mavlink_msg_command_long_send(_chan,
                                  _sysid,
                                  _compid,
                                  MAV_CMD_DO_DIGICAM_CONTROL,
								  0,        // confirmation of zero means this is the first time this message has been sent
                                  cmd1,
                                  p1,
                                  0, 0, 0, 0, 0);  // param3 ~ param7 unused

	}


	if ( cmd2 != NV_NO_CMD )
	{
		mavlink_msg_command_long_send(_chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									cmd2,
									p2,
									0, 0, 0, 0, 0);  // param3 ~ param7 unused
	}


	if ( cmd3 != NV_NO_CMD )
	{
		mavlink_msg_command_long_send(_chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									cmd3,
									p3,
									0, 0, 0, 0, 0);  // param3 ~ param7 unused
	}

    // store time of send

    _last_send = AP_HAL::millis();

}

/*
 *  set_zoom_FOV
 *
 *  Set the specified FOV zoom level. 
 */
void AP_Mount_NextVision::set_zoom_FOV(float fov_deg)
{

	mavlink_msg_command_long_send(_chan,
								_sysid,
								_compid,
								MAV_CMD_DO_DIGICAM_CONTROL,
								0,        // confirmation of zero means this is the first time this message has been sent
								NV_SET_FOV_CMD,
								(float)fov_deg,
								0,
								0,
								0, 
								0, 0);  // param3 ~ param7 unused
}

void AP_Mount_NextVision::set_gimbal_rates(float pan_speed, float tilt_speed) {
	float amsl = 0;
	static uint32_t prev_time = AP_HAL::millis();
	static float prev_pan_input = 0;
	static float prev_tilt_input = 0;

	float pan_speed_i = pan_speed * -1; //Invert pan input

	if(is_equal(prev_pan_input, pan_speed) && //Pan speed is same as last time
	   is_equal(prev_tilt_input, tilt_speed) && //Tilt speed is same as last time
	  (is_equal(pan_speed, 0.0f) && is_equal(tilt_speed, 0.0f))){ //The repeat speed is 0
		return;
	  }

	if(is_equal(prev_pan_input, pan_speed) && //Pan speed is the same
		is_equal(prev_tilt_input, tilt_speed) && //Tilt speed is the same
		_current_zoom_state == _prev_zoom_state && //Zoom speed is the same
		(AP_HAL::millis() - prev_time < 1000 || (is_equal(pan_speed, 0.0f) && is_equal(tilt_speed, 0.0f)))){ //It has been 1 second since last speed cmd or pan and tilt are 0
		return;
	}

	//Update prev values for next check
	prev_time = AP_HAL::millis();
	prev_pan_input = pan_speed;
	prev_tilt_input = tilt_speed;
	_prev_zoom_state = _current_zoom_state;

	//Set _moving variable to false so the zoom functions knows the gimbal is not moving and it needs to send its own mavlink commands
	if(is_equal(pan_speed, 0.0f) && is_equal(tilt_speed, 0.0f)){ _moving = false;}
	else {_moving = true;}

	//Cease tracking if tracking
	if(_tracking){
		stop_tracking();
	}

	if ( (get_mode() == MAV_MOUNT_MODE_GPS_POINT) && (!is_zero(pan_speed) || !is_zero(tilt_speed))){
		clear_roi_target(); // Break ROI targetting
	}

	//Set camera to observation mode
	if(_camera_mode != NV_SET_OBSERVATION_MODE){
		mavlink_msg_command_long_send(  _chan,
                                    _sysid,
                                    _compid,
                                    MAV_CMD_DO_DIGICAM_CONTROL,
                                    0,        // confirmation of zero means this is the first time this message has been sent
                                    NV_SET_SYSTEM_MODE,
                                    3,  //Observation Mode
                                    0,              
                                    0,                   
                                    0, 
                                    0, 0); 
		_camera_mode = NV_SET_OBSERVATION_MODE;
	}

	//If panning, disable single yaw. 
    if(!is_zero(pan_speed)) disable_single_yaw();

	//Send rates
	mavlink_msg_command_long_send(_chan,
                                    _sysid,
                                    _compid,
                                    MAV_CMD_DO_DIGICAM_CONTROL,
                                    0,        // confirmation of zero means this is the first time this message has been sent
                                    NV_SET_GIMBAL_CMD,
                                    pan_speed_i,  //Swapping left and right here with (* -1)
                                    tilt_speed,                //otherwise they are reversed. Oddly, could
                                    _current_zoom_state,                     //not find a camera config param to handle it.
                                    amsl, 
                                    0, 0);  // param6 ~ param7 unused
}

void AP_Mount_NextVision::set_zoom_rate(float zoom_speed) {
	float amsl = 0;
	static float prev_zoom_speed = 0.0;

	if(zoom_speed < 0){_current_zoom_state = ZOOM_IN;}
	else if(zoom_speed > 0){_current_zoom_state = ZOOM_OUT;}
	else{_current_zoom_state = ZOOM_STOP;}

	if(is_equal(zoom_speed, prev_zoom_speed)) return;
	prev_zoom_speed = zoom_speed;

	if(!_moving){
		mavlink_msg_command_long_send(_chan,
								_sysid,
								_compid,
								MAV_CMD_DO_DIGICAM_CONTROL,
								0,        // confirmation of zero means this is the first time this message has been sent
								NV_SET_GIMBAL_CMD,
								0,  //Swapping left and right here with (* -1)
								0,                //otherwise they are reversed. Oddly, could
								_current_zoom_state,                     //not find a camera config param to handle it.
								amsl, 
								0, 0);  // param6 ~ param7 unused
	}
}

bool AP_Mount_NextVision::center_camera(bool follow_camera){
	static int state = 0;
	static bool flag = false;
	static uint32_t prev_time = AP_HAL::millis();
	const int command_delay = 500;

	clear_roi_target();

    switch(state) {
        case 0:
			if(follow_camera) state = 1; //Do not enable single yaw if in FTC mode
      else if(!flag){
        flag = true;
        prev_time = AP_HAL::millis();
				enable_single_yaw();
			}
			else if(AP_HAL::millis() - prev_time >= command_delay){
				state = 1;
				flag = false;
			}
			break;
		
		case 1:
			if(!flag){
				flag = true;
				prev_time = AP_HAL::millis();
				_camera_mode = NV_SET_LOCAL_MODE;

				mavlink_msg_command_long_send(  _chan, _sysid, _compid,
												MAV_CMD_DO_DIGICAM_CONTROL,
												0,        // confirmation of zero means this is the first time this message has been sent
												NV_SET_SYSTEM_MODE,
												4, _centered_angle, 0, 0, 0, 0);
			}
			else if(AP_HAL::millis() - prev_time >= command_delay){
				state = 2;
				flag = false;
			}
			break;
		
		case 2:
			if(!flag){
				flag = true;
				prev_time = AP_HAL::millis();
				set_zoom_FOV(NV_ZOOM_OUT_ALL);
			}
			else if(AP_HAL::millis() - prev_time >= command_delay){
				state = 0;
				flag = false;
				return true;
			}
			break;
	}
	return false;
}

void AP_Mount_NextVision::look_down(){

	clear_roi_target();

	mavlink_msg_command_long_send(  _chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									NV_SET_SYSTEM_MODE,
									4,  //Local Position
									_lookdown_angle,        
									0,       
									0, 
									0, 0);  // param6 ~ param7 unused
	_camera_mode = NV_SET_LOCAL_MODE;
}

void AP_Mount_NextVision::stow(){

	clear_roi_target();

	mavlink_msg_command_long_send(  _chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									NV_SET_SYSTEM_MODE,
									4,  //Local Position
									-90,        
									0,       
									0, 
									0, 0);  // param6 ~ param7 unused
	_camera_mode = NV_SET_LOCAL_MODE;
}

void AP_Mount_NextVision::payload_take_picture(){

	mavlink_msg_command_long_send(_chan,
								_sysid,
								_compid,
								MAV_CMD_DO_DIGICAM_CONTROL,
								0,        // confirmation of zero means this is the first time this message has been sent
								NV_SNAP_SHOT,
								NV_VIDEO_CH0,
								0,
								0,
								0, 
								0, 0);  // param3 ~ param7 unused

}

void AP_Mount_NextVision::start_record(){

	mavlink_msg_command_long_send(_chan,
								_sysid,
								_compid,
								MAV_CMD_DO_DIGICAM_CONTROL,
								0,        // confirmation of zero means this is the first time this message has been sent
								NV_VIDEO_RECORD,
								NV_ENABLED,
								NV_VIDEO_CH0,
								0,
								0, 
								0, 0);  // param3 ~ param7 unused
	_is_recording = true;

}

void AP_Mount_NextVision::stop_record(){

	mavlink_msg_command_long_send(_chan,
								_sysid,
								_compid,
								MAV_CMD_DO_DIGICAM_CONTROL,
								0,        // confirmation of zero means this is the first time this message has been sent
								NV_VIDEO_RECORD,
								NV_DISABLED,
								NV_VIDEO_CH0,
								0,
								0, 
								0, 0);  // param3 ~ param7 unused

	_is_recording = false;


}

/*
 *  Switch to Day mode. 
 */
void AP_Mount_NextVision::eo_full_screen(){
	mavlink_msg_command_long_send(_chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									57,
									0, //Set stream mode
									1, //EO 
									0, 0, 0, 0);  // param5 ~ param 7 unused

	mavlink_msg_command_long_send(_chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									3, //Set Sensor
									0, //EO
									0, 0, 0, 0, 0);  
}

void AP_Mount_NextVision::ir_full_screen(){
	mavlink_msg_command_long_send(_chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									57,
									0, //Set stream mode
									2, //IR 
									0, 0, 0, 0);  // param5 ~ param 7 unused
	mavlink_msg_command_long_send(_chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									3, //Set Sensor
									1, //EO
									0, 0, 0, 0, 0);  
}

void AP_Mount_NextVision::eo_ir(){
		_stream = PIP;
	    mavlink_msg_command_long_send(_chan,
									  _sysid,
									  _compid,
									  MAV_CMD_DO_DIGICAM_CONTROL,
									  0,        // confirmation of zero means this is the first time this message has been sent
									  57,
									  0, //Set stream mode
									  4, //PIP 
									  0,   // param2 ~ param 4 unused   
									  0, 0, 0);  // param5 ~ param 7 unused
		mavlink_msg_command_long_send(_chan,
									  _sysid,
									  _compid,
									  MAV_CMD_DO_DIGICAM_CONTROL,
									  0,        // confirmation of zero means this is the first time this message has been sent
									  57,
									  1, //Set pip mode
									  0, //EO Large
									  0,   // param2 ~ param 4 unused   
									  0, 0, 0);  // param5 ~ param 7 unused
}

void AP_Mount_NextVision::ir_eo(){
		_stream = PIP;
	    mavlink_msg_command_long_send(_chan,
									  _sysid,
									  _compid,
									  MAV_CMD_DO_DIGICAM_CONTROL,
									  0,        // confirmation of zero means this is the first time this message has been sent
									  57,
									  0, //Set stream mode
									  4, //PIP 
									  0,   // param2 ~ param 4 unused   
									  0, 0, 0);  // param5 ~ param 7 unused
		mavlink_msg_command_long_send(_chan,
									  _sysid,
									  _compid,
									  MAV_CMD_DO_DIGICAM_CONTROL,
									  0,        // confirmation of zero means this is the first time this message has been sent
									  57,
									  1, //Set pip mode
									  1, //IR Large
									  0,   // param2 ~ param 4 unused   
									  0, 0, 0);  // param5 ~ param 7 unused
}

void AP_Mount_NextVision::fusion(){
	mavlink_msg_command_long_send(_chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									57,
									0, //Set stream mode
									3, //Fusion 
									0,   // param2 ~ param 4 unused   
									0, 0, 0);  // param5 ~ param 7 unused
}

void AP_Mount_NextVision::toggle_detection(){
		_ai = !_ai;
	    mavlink_msg_command_long_send(_chan,
									  _sysid,
									  _compid,
									  MAV_CMD_DO_DIGICAM_CONTROL,
									  0,
									  54, //Detection Control
									  0, //Detector Enable/Disable
									  (int)_ai,
									  0,
									  0, 0, 0);
}

void AP_Mount_NextVision::set_detection(int arg){
	    mavlink_msg_command_long_send(_chan,
									  _sysid,
									  _compid,
									  MAV_CMD_DO_DIGICAM_CONTROL,
									  0,
									  54, //Detection Control
									  1,  //Detector Select
									  arg,
									  0,
									  0, 0, 0);
}

void AP_Mount_NextVision::next_color_pallette(){
	static int color = 0;
	color = (color + 1)%2; //Only two colors according to doc

    mavlink_msg_command_long_send(_chan, _sysid, _compid, MAV_CMD_DO_DIGICAM_CONTROL, 0,
                                    NV_SET_IR_COLOR,
                                    color,  
                                    0,              
                                    0,                    
                                    0, 
                                    0, 
									0);  // param6 ~ param7 unused
}

void AP_Mount_NextVision::toggle_heat(){
	static bool heat = 0;
	heat = !heat;
	mavlink_msg_command_long_send(_chan, _sysid, _compid, MAV_CMD_DO_DIGICAM_CONTROL, 0,
                                    NV_SET_IR_POLARITY,
                                    heat,  
                                    0,              
                                    0,                    
                                    0, 
                                    0, 
									0);
}

//  start_tracking
//	
//	Start tracking mode using the pixel coordinates passed in.
//
void AP_Mount_NextVision::start_tracking( int x, int y){
	disable_single_yaw();
    _tracking = true;
	clear_roi_target();

	if(_gcs == AP_Mount::GCS_Type_Skynav){
		mavlink_msg_command_long_send(_chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									NV_SET_SYSTEM_MODE,
									NV_SET_TRACKING_MODE,
									(int)(x),
									(int)(y),
									NV_TRACK_ON_XY, 
									NV_CHAN_0, 
									0);  // param7 unused
	}
	else {
		mavlink_msg_command_long_send(_chan,
			_sysid,
			_compid,
			MAV_CMD_DO_DIGICAM_CONTROL,
			0,        // confirmation of zero means this is the first time this message has been sent
			NV_SET_SYSTEM_MODE,
			NV_SET_TRACKING_MODE,
			(int)(x) * (0.666666666),
			(int)(y) * (0.666666666),
			NV_TRACK_ON_XY, 
			NV_CHAN_0, 
			0);  // param7 unused
	}

	/* Keep track of the mode in our local variables */
	_camera_mode = NV_TRACK_ON_XY;
}

//  Turn off tracking mode. In theory, the camera should automatically return to the 
//  previous mode!
//
void AP_Mount_NextVision::stop_tracking(){
    _tracking = false;
	clear_roi_target();

    mavlink_msg_command_long_send(_chan,
                                  _sysid,
                                  _compid,
                                  MAV_CMD_DO_DIGICAM_CONTROL,
								  0,        // confirmation of zero means this is the first time this message has been sent
                                  NV_SET_SYSTEM_MODE,
								  NV_SET_TRACKING_MODE,
                                  640,		// User Center of screen. 
								  360,      //
								  NV_TRACK_DISABLED, 
								  NV_CHAN_0, 
								  0);  // param7 unused
}

//  do_nuc 
//  If we are in IR mode, do a non-uniformity calibration
//
void AP_Mount_NextVision::do_nuc(){
	mavlink_msg_command_long_send(_chan,
									_sysid,
									_compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									NV_DO_IR_NUC,
									0, 0, 0, 0, 0, 0);
}

void AP_Mount_NextVision::set_report_frequency(int report, int freq) {
	if (!HAVE_PAYLOAD_SPACE(_chan, COMMAND_LONG)) {
        return;
    }
		mavlink_msg_command_long_send(_chan,
								_sysid,
								_compid,
								MAV_CMD_DO_DIGICAM_CONTROL,
								0,        // confirmation of zero means this is the first time this message has been sent
								12,
								(float)report, (float)freq, 
								0, 0, 0, 0);  // param5 ~ param 7 unused
}

void AP_Mount_NextVision::set_angles(float pan_input, float tilt_input) {
	mavlink_msg_command_long_send(  _chan, _sysid, _compid,
									MAV_CMD_DO_DIGICAM_CONTROL,
									0,        // confirmation of zero means this is the first time this message has been sent
									NV_SET_SYSTEM_MODE,
									4, tilt_input, pan_input, 0, 0, 0);
}