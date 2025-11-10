#include "AP_Mount_Viewpro_AA.h"
#include <AP_Logger/AP_Logger.h>
#include <AP_SerialManager/AP_SerialManager.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL& hal;

//Constructor
AP_Mount_Viewpro_AA::AP_Mount_Viewpro_AA(AP_Mount &frontend, AP_Mount_Params &params, uint8_t instance) :
    AP_Mount_Backend(frontend, params, instance)
{

}

//Initializer
void AP_Mount_Viewpro_AA::init() {
    const AP_SerialManager& serial_manager = AP::serialmanager();
    
    _port = serial_manager.find_serial(AP_SerialManager::SerialProtocol_ViewPro, 0);
    if (_port) {
        _initialized = true;
        yaw_follow_disable();
    }

}

void AP_Mount_Viewpro_AA::update() {
    if(!_initialized) {
        return;
    }
    write_gimbal_log();
    read_incoming();
    MountTarget angle_target_rad {};
    if (get_mode()==MAV_MOUNT_MODE_GPS_POINT){
        if (get_angle_target_to_roi(angle_target_rad)) {
            ViewProAPI packet;
            packet.command_id = 0x0B; //Manual absolute angle mode (Home position as 0)

            const int16_t pan_vp = (int16_t) wrap_180(degrees(angle_target_rad.yaw) - degrees(AP::ahrs().get_yaw())) * 65536.0f / 360.0f;
            const int16_t tilt_vp = (int16_t) (degrees(angle_target_rad.pitch) * -1) * 65536.0f / 360.0f;
            packet.parameter_1_big = pan_vp >> 8;
            packet.parameter_1_little = pan_vp;
            packet.parameter_2_big = tilt_vp >> 8;
            packet.parameter_2_little = tilt_vp;
            checksum_and_send((uint8_t*)&packet, sizeof(packet));
        }
    }
}

bool AP_Mount_Viewpro_AA::get_attitude_quaternion(Quaternion& att_quat) {
    if (!_initialized) {
        return false;
    }

    // construct quaternion
    att_quat.from_euler(radians(0), radians(_camera_pan_angle), radians(_camera_tilt_angle));
    return true;
}
void AP_Mount_Viewpro_AA::update_10hz() {
    if(!_initialized) {
        return;
    }
    static int counter = 9;
    counter = (counter + 1) % 10;
    if(counter == 0 && !_moving && zoom_state == zoom_stop) update_vehicle_position();
    else {send_heartbeat();}
}

float AP_Mount_Viewpro_AA::get_zoom_level(){return _current_zoom;}

void AP_Mount_Viewpro_AA::send_heartbeat() {
    if(!_initialized) {
        return;
    }
    ViewPro_Heartbeat hb; //Send an A1 packet to provoke sending an angle update 
    checksum_and_send((uint8_t*)&hb, sizeof(hb));
}

void AP_Mount_Viewpro_AA::update_vehicle_position() {
    if(!_initialized) {
        return;
    }
    AHRS_Packet packet;
    Location current_loc;
    Vector3f speed;
    if (!AP::ahrs().get_location(current_loc) && !AP::ahrs().get_velocity_NED(speed)) {
        return;
    }

    packet.lat4 = current_loc.lat >> 24;
    packet.lat3 = current_loc.lat >> 16;
    packet.lat2 = current_loc.lat >> 8;
    packet.lat1 = current_loc.lat;

    packet.lng4 = current_loc.lng >> 24;
    packet.lng3 = current_loc.lng >> 16;
    packet.lng2 = current_loc.lng >> 8;
    packet.lng1 = current_loc.lng;

    packet.alt4 = current_loc.alt >> 24;
    packet.alt3 = current_loc.alt >> 16;
    packet.alt2 = current_loc.alt >> 8;
    packet.alt1 = current_loc.alt;

    /*
        ViewPro: 1 bit is (360/65536) degrees
        Radians to degrees: 180 / pi
    */
    packet.tilt_angle2 = ((int16_t)(AP::ahrs().get_pitch() * (rad_to_viewpro_bit))) >> 8;
    packet.tilt_angle1 = ((int16_t)AP::ahrs().get_pitch() * (rad_to_viewpro_bit));

    packet.roll_angle2 = ((int16_t)(AP::ahrs().get_roll() * (rad_to_viewpro_bit))) >> 8;
    packet.roll_angle1 = ((int16_t)AP::ahrs().get_roll() * (rad_to_viewpro_bit));

    packet.yaw_angle2 = ((int16_t)(AP::ahrs().get_yaw() * (rad_to_viewpro_bit))) >> 8;
    packet.yaw_angle1 = ((int16_t)AP::ahrs().get_yaw() * (rad_to_viewpro_bit));

    packet.gps_yaw2 = ((int16_t)(AP::ahrs().get_yaw() * (rad_to_viewpro_bit))) >> 8;
    packet.gps_yaw1 = ((int16_t)AP::ahrs().get_yaw() * (rad_to_viewpro_bit));

    //NOTE: AHRS data is in cm/sec
    packet.ground_lat_speed2 = ((int16_t)(speed.x * 100)) >> 8;
    packet.ground_lat_speed1 = (int16_t)(speed.x * 100);
    packet.ground_long_speed2 = ((int16_t)speed.y * 100) >> 8;
    packet.ground_long_speed1 = (int16_t)speed.y * 100;
    packet.ground_alt_speed2 = ((int16_t)speed.z * 100) >> 8;
    packet.ground_alt_speed1 = (int16_t)speed.z * 100;

    checksum_and_send((uint8_t*)&packet, sizeof(packet));
}

void AP_Mount_Viewpro_AA::payload_take_picture() {
    if(!_initialized) {
        return;
    }
    optical_control packet;
    uint16_t command = 0x13 << 6;
    packet.byte_1 = command >> 8;
    packet.byte_2 = command;
    checksum_and_send((uint8_t*)&packet, sizeof(packet));
}

void AP_Mount_Viewpro_AA::start_record() {
    if(!_initialized) {
        return;
    }
    optical_control packet;
    uint16_t command = 0x14 << 6;
    packet.byte_1 = command >> 8;
    packet.byte_2 = command;
    checksum_and_send((uint8_t*)&packet, sizeof(packet));
}

void AP_Mount_Viewpro_AA::stop_record() {
    if(!_initialized) {
        return;
    }
    optical_control packet;
    uint16_t command = 0x15 << 6;
    packet.byte_1 = command >> 8;
    packet.byte_2 = command;
    checksum_and_send((uint8_t*)&packet, sizeof(packet));
}

void AP_Mount_Viewpro_AA::eo_full_screen(){
    if(!_initialized) {
        return;
    }
    _pip_state = 3;
    //For bug where Z10TIR would revert to greyscale IR when changing Lens mode (due to using old commands for pallete change with this camerea)
    if(strcmp(camera_models[_cam], "Z10TIR") == 0 || strcmp(camera_models[_cam], "Z40TIR") == 0){
        uint8_t data[48] = {0};
        uint8_t* packet = (uint8_t*)&data;
        packet[0] = 0x7E;
        packet[1] = 0x7E;
        packet[2] = 0x44;
        packet[5] = 0x78;
        packet[6] = current_color;
        packet[14] = _pip_state;
        checksum_and_send_legacy(packet, 48);
    }
    else{
        optical_control packet;
        packet.byte_2 = 0x01;
        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
}

void AP_Mount_Viewpro_AA::ir_full_screen(){
    if(!_initialized) {
        return;
    }
    _pip_state = 1;
    //For bug where Z10TIR would revert to greyscale IR when changing Lens mode (due to using old commands for pallete change with this camerea)
    if(strcmp(camera_models[_cam], "Z10TIR") == 0 || strcmp(camera_models[_cam], "Z40TIR") == 0){
        uint8_t data[48] = {0};
        uint8_t* packet = (uint8_t*)&data;
        packet[0] = 0x7E;
        packet[1] = 0x7E;
        packet[2] = 0x44;
        packet[5] = 0x78;
        packet[6] = current_color;
        packet[14] = _pip_state;
        checksum_and_send_legacy(packet, 48);
    }
    else{
        optical_control packet;
        packet.byte_2 = 0x02;
        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
}

void AP_Mount_Viewpro_AA::eo_ir(){
    if(!_initialized) {
        return;
    }
    _pip_state = 0;
    //For bug where Z10TIR would revert to greyscale IR when changing Lens mode (due to using old commands for pallete change with this camerea)
    if(strcmp(camera_models[_cam], "Z10TIR") == 0 || strcmp(camera_models[_cam], "Z40TIR") == 0){
        uint8_t data[48] = {0};
        uint8_t* packet = (uint8_t*)&data;
        packet[0] = 0x7E;
        packet[1] = 0x7E;
        packet[2] = 0x44;
        packet[5] = 0x78;
        packet[6] = current_color;
        packet[14] = _pip_state;
        checksum_and_send_legacy(packet, 48);
    }
    else{
        optical_control packet;
        packet.byte_2 = 0x03;
        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
}

void AP_Mount_Viewpro_AA::ir_eo(){
    if(!_initialized) {
        return;
    }
    _pip_state = 2;
    //For bug where Z10TIR would revert to greyscale IR when changing Lens mode (due to using old commands for pallete change with this camerea)
    if(strcmp(camera_models[_cam], "Z10TIR") == 0 || strcmp(camera_models[_cam], "Z40TIR") == 0){
        uint8_t data[48] = {0};
        uint8_t* packet = (uint8_t*)&data;
        packet[0] = 0x7E;
        packet[1] = 0x7E;
        packet[2] = 0x44;
        packet[5] = 0x78;
        packet[6] = current_color;
        packet[14] = _pip_state;
        checksum_and_send_legacy(packet, 48);
    }
    else{
        optical_control packet;
        packet.byte_2 = 0x04;
        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
}

void AP_Mount_Viewpro_AA::digital_zoom_1(){
    if(!_initialized) {
        return;
    }
    if(strcmp(camera_models[_cam], "Z10TIR") == 0 || strcmp(camera_models[_cam], "Z40TIR") == 0){
        uint8_t data[48] = {0};
        uint8_t* packet = (uint8_t*)&data;
        packet[0] = 0x7E;
        packet[1] = 0x7E;
        packet[2] = 0x44;
        packet[5] = 0x7D;
        packet[6] = 0x81; //1x zoom
        checksum_and_send_legacy(packet, 48);
    }
    else{
        optical_control_ext packet;
        packet.command_id = 0x56;
        packet.parameter_1_little = 10;
        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
}

void AP_Mount_Viewpro_AA::digital_zoom_2(){
    if(!_initialized) {
        return;
    }
    if(strcmp(camera_models[_cam], "Z10TIR") == 0 || strcmp(camera_models[_cam], "Z40TIR") == 0){
        uint8_t data[48] = {0};
        uint8_t* packet = (uint8_t*)&data;
        packet[0] = 0x7E;
        packet[1] = 0x7E;
        packet[2] = 0x44;
        packet[5] = 0x7D;
        packet[6] = 0x82; //2x zoom
        checksum_and_send_legacy(packet, 48);
    }
    else{
        optical_control_ext packet;
        packet.command_id = 0x56;
        packet.parameter_1_little = 20;
        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
}

void AP_Mount_Viewpro_AA::digital_zoom_4(){
    if(!_initialized) {
        return;
    }
    if(strcmp(camera_models[_cam], "Z10TIR") == 0 || strcmp(camera_models[_cam], "Z40TIR") == 0){
        uint8_t data[48] = {0};
        uint8_t* packet = (uint8_t*)&data;
        packet[0] = 0x7E;
        packet[1] = 0x7E;
        packet[2] = 0x44;
        packet[5] = 0x7D;
        packet[6] = 0x84; //4x zoom
        checksum_and_send_legacy(packet, 48);
    }
    else{
        optical_control_ext packet;
        packet.command_id = 0x56;
        packet.parameter_1_little = 40;
        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
}

void AP_Mount_Viewpro_AA::digital_zoom_8(){
    if(!_initialized) {
        return;
    }
    optical_control_ext packet;
    packet.command_id = 0x56;
    packet.parameter_1_little = 80;
    checksum_and_send((uint8_t*)&packet, sizeof(packet));
}

void AP_Mount_Viewpro_AA::digital_zoom_16(){
    if(!_initialized) {
        return;
    }
    optical_control_ext packet;
    packet.command_id = 0x56;
    packet.parameter_1_little = 160;
    checksum_and_send((uint8_t*)&packet, sizeof(packet));
}

void AP_Mount_Viewpro_AA::start_tracking(int x, int y){
    if(!_initialized) {
        return;
    }

    clear_roi_target();

    if(strcmp(camera_models[_cam], "Q10F") == 0 || strcmp(camera_models[_cam], "Z40K") == 0){
        return;
        /* FAUX TRACK --- Too inconsistent to use with Fly the Camera Mode... Will leave for potential future implementation
        //Need to issue a quick zoom cmd to update zoom due to bug after centering not updating internal zoom value
        ViewProAPI bug;
        uint16_t c1 = zoom_out << 6;
        c1 = c1 | (max_zoom_speed << 3);
        bug.byte_1 = c1 >> 8;
        bug.byte_2 = c1;
        checksum_and_send((uint8_t*)&bug, sizeof(bug));
        c1 = zoom_stop << 6;
        c1 = c1 | (max_zoom_speed << 3);
        bug.byte_1 = c1 >> 8;
        bug.byte_2 = c1;
        checksum_and_send((uint8_t*)&bug, sizeof(bug));

        float target_pan = _camera_pan_angle + ((x - 960) * ((fov_angle_h / _current_zoom)/1920));
        float target_tilt = _camera_tilt_angle + ((y - 540) * ((fov_angle_v / _current_zoom)/1080));
        int16_t param1 = target_pan * 182.0444444;
        int16_t param2 = target_tilt * 182.0444444;
        ViewProAPI packet;
        packet.command_id = 0x0B;
        packet.parameter_1_big = param1 >> 8;
        packet.parameter_1_little = param1;
        packet.parameter_2_big = param2 >> 8;
        packet.parameter_2_little = param2;
        checksum_and_send((uint8_t*)&packet, sizeof(packet));
        */
    }
    else{
        if(!_tracking){
            _tracking = true;
        }

        tracking_control_ext packet;
        packet.command = 0x0A;
        int16_t param1 = x - 960;
        int16_t param2 = y - 540;   
        packet.parameter_1_big = param1 >> 8; 
        packet.parameter_1_little = param1;
        packet.parameter_2_big = param2 >> 8; 
        packet.parameter_2_little = param2;
        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
}

void AP_Mount_Viewpro_AA::stop_tracking(){
    if(!_initialized) {
        return;
    }
    _tracking = false;
    clear_roi_target();

    tracking_control packet;
    packet.command = 0x01;
    checksum_and_send((uint8_t*)&packet, sizeof(packet));
}

void AP_Mount_Viewpro_AA::next_color_pallette(){
    if(!_initialized) {
        return;
    }
    if(strcmp(camera_models[_cam], "Z10TIR") == 0 || strcmp(camera_models[_cam], "Z40TIR") == 0 || strcmp(camera_models[_cam], "Z10TIR Mini") == 0){
        current_color = ((current_color + 1) % 5);
        uint8_t data[48] = {0};
        uint8_t* packet = (uint8_t*)&data;
        packet[0] = 0x7E;
        packet[1] = 0x7E;
        packet[2] = 0x44;
        packet[5] = 0x78;
        packet[6] = current_color;
        packet[14] = _pip_state;
        checksum_and_send_legacy(packet, 48);
    }
    else {
        optical_control packet;
        static uint8_t pallette = 0;
        pallette = (pallette + 1) % 6;
        uint16_t c1 = (pallette + 33) << 6;
        packet.byte_1 = c1 >> 8;
        packet.byte_2 = c1;
        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
}

void AP_Mount_Viewpro_AA::yaw_follow_enable(){
    if(!_initialized) {
        return;
    }
    if(!_following) {
        ////////////////////////////////////////
        // Bring pan and tilt to zero degrees //
        ////////////////////////////////////////
        ViewProAPI packet1;
        packet1.command_id = 0x04; //Home Position
        checksum_and_send((uint8_t*)&packet1, sizeof(packet1));
        _following = true;
        ViewProAPI packet;
        packet.command_id = 0x03; //Follow Yaw Enable
        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
}

void AP_Mount_Viewpro_AA::yaw_follow_disable(){
    if(!_initialized) {
        return;
    }
    if(_following) {
        _following = false;
        ViewProAPI packet;
        packet.command_id = 0x0A; //Follow Yaw Disable
        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
}

void AP_Mount_Viewpro_AA::set_gimbal_rates(float des_pan_speed, float tilt_speed) {
    if(!_initialized) {
        return;
    }
    static int16_t prev_pan_position = 0;
    static int16_t prev_tilt_position = 0;

    float pan_speed;
    if(_following) pan_speed = 0;
    else pan_speed = des_pan_speed;

    if(_tracking){stop_tracking();}

	if ( (get_mode() == MAV_MOUNT_MODE_GPS_POINT) && (!is_zero(pan_speed) || !is_zero(tilt_speed))){
		clear_roi_target(); // Break ROI targetting
	}

    if(_just_centered && !is_zero(pan_speed)){ //Disable yaw following when manually controlling || How is this affected with new yaw following mode?
        if(strcmp(camera_models[_cam], "Q10F") == 0){
            //Need to issue a quick zoom cmd to update zoom due to bug after centering not updating internal zoom value
            ViewProAPI bug;
            uint16_t c1 = zoom_out << 6;
            c1 = c1 | (max_zoom_speed << 3);
            bug.byte_1 = c1 >> 8;
            bug.byte_2 = c1;
            checksum_and_send((uint8_t*)&bug, sizeof(bug));
            c1 = zoom_stop << 6;
            c1 = c1 | (max_zoom_speed << 3);
            bug.byte_1 = c1 >> 8;
            bug.byte_2 = c1;
            checksum_and_send((uint8_t*)&bug, sizeof(bug));
        }
        _just_centered = false;
    } 

    if(is_zero(pan_speed) && is_zero(tilt_speed)){
        _moving = false;
    }
    else{
        _moving = true; //For edge case in some cameras where the zoom command must be included in the gimbal movement command
    }
    if(strcmp(camera_models[_cam], "Q10F") == 0){
        ViewProAPI packet;
        packet.command_id = 0x01; //Manual speed mode
        float speed = MAX((max_gimbal_speed / _current_zoom), 7.5);
        int16_t pan_position = (pan_speed * speed) * 100; //1 bit is 0.01 degrees per second 
        int16_t tilt_position = (tilt_speed * speed) * 100;
        //Change in speed
        if(pan_position == prev_pan_position && tilt_position == prev_tilt_position && zoom_state == prev_zoom_state) {return;}
        prev_pan_position = pan_position;
        prev_tilt_position = tilt_position;
        prev_zoom_state = zoom_state;
        packet.parameter_1_big = pan_position >> 8;
        packet.parameter_1_little = pan_position;
        packet.parameter_2_big = tilt_position >> 8;
        packet.parameter_2_little = tilt_position;

        //Handle zooming
        uint16_t c1 = zoom_state << 6;
        c1 = c1 | (max_zoom_speed << 3);
        packet.byte_1 = c1 >> 8;
        packet.byte_2 = c1;
        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
    else if(strcmp(camera_models[_cam], "Z40TIR") == 0){
        OldViewProAPI packet;
        static float prev_z40_pan = 0.0;
        static float prev_z40_tilt = 0.0;

        float pan_position = (pan_speed * (max_gimbal_speed / _current_zoom))/0.1220740379;
        float tilt_position = (tilt_speed * (max_gimbal_speed / _current_zoom))/-0.1220740379;
        if(pan_position < 2.0 && pan_position > 0.0){pan_position = 2.0;}
        if(pan_position > -2.0 && pan_position < 0.0){pan_position = -2.0;}
        if(tilt_position < 2.0 && tilt_position > 0.0){tilt_position = 2.0;}
        if(tilt_position > -2.0 && tilt_position < 0.0){tilt_position = -2.0;}

        if(is_equal(pan_position,prev_z40_pan) && is_equal(tilt_position, prev_z40_tilt) && zoom_state == prev_zoom_state) {return;}
        prev_z40_pan = pan_position;
        prev_z40_tilt = tilt_position;
        prev_zoom_state = zoom_state;

        packet.yaw_speed_0 = (int16_t)pan_position;
        packet.yaw_speed_1 = (int16_t)pan_position >> 8;
        packet.pitch_speed_0 = (int16_t)tilt_position;
        packet.pitch_speed_1 = (int16_t)tilt_position >> 8;

        // write the commands
        uint16_t sum = 0;
        uint8_t* buf_gimbal_control = (uint8_t*)&packet;
        for (uint8_t i = 4;  i < 19 ; i++) {
            sum	+= buf_gimbal_control[i];
        }
        packet.crc = (uint8_t)(sum % 256);
        if ((size_t)_port->txspace() < sizeof(packet)) {
            return;
        }
        for (uint8_t i = 0;  i != sizeof(packet) ; i++) {
            _port->write(buf_gimbal_control[i]);
        }

        //Handle zooming
        OldViewProZoomAPI packet2;
        if(zoom_state == zoom_in){
            packet2.byte7 = 0x30;
            packet2.byte10 = 0x50;
        }
        else if(zoom_state == zoom_out){
            packet2.byte7 = 0x20;
            packet2.byte10 = 0x40;
        }
        if ((size_t)_port->txspace() <= sizeof(packet2)) {
			return;
		}
		uint8_t* buf_zoom = (uint8_t*)&packet2;

		for (uint8_t i = 0;  i != sizeof(packet2) ; i++) {
			_port->write(buf_zoom[i]);
		}
    }
    else{
        servo_control packet;
        packet.command_id = 0x01; //Manual speed mode

        float speed = MAX((max_gimbal_speed / _current_zoom), 7.5);
        int16_t pan_position = (pan_speed * speed) * 100;
        int16_t tilt_position = (tilt_speed * speed) * 100;

        if(pan_position == prev_pan_position && tilt_position == prev_tilt_position) {return;}
        prev_pan_position = pan_position;
        prev_tilt_position = tilt_position;

        packet.parameter_1_big = pan_position >> 8;
        packet.parameter_1_little = pan_position;
        packet.parameter_2_big = tilt_position >> 8;
        packet.parameter_2_little = tilt_position;

        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
}

void AP_Mount_Viewpro_AA::set_zoom_rate(float zoom_speed) {
    if(!_initialized) {
        return;
    }
    if(strcmp(camera_models[_cam], "Q10F") == 0 || strcmp(camera_models[_cam], "Z40TIR") == 0){
        if(zoom_speed < 0){zoom_state = zoom_in;}
        else if(zoom_speed > 0){zoom_state = zoom_out;}
        else{zoom_state = zoom_stop;}

        if(_moving){return;} //zoom will be handled by control_gimbal in this case
        else{
            if(zoom_state == prev_zoom_state){return;}
            prev_zoom_state = zoom_state;
            ViewProAPI packet;
            uint16_t c1 = zoom_state << 6;
            c1 = c1 | (max_zoom_speed << 3);
            packet.byte_1 = c1 >> 8;
            packet.byte_2 = c1;
            checksum_and_send((uint8_t*)&packet, sizeof(packet));
        }
    }
    else{
        optical_control packet;
        uint16_t c1; //Not a random name, this is how ViewPro labels this type of message
        static uint16_t prev_c1 = 0;

        if(zoom_speed < 0){
            c1 = zoom_in << 6; 
            zoom_state = zoom_in;
        }
        else if(zoom_speed > 0){
            c1 = zoom_out << 6; 
            zoom_state = zoom_out;
        }
        else{
            c1 = zoom_stop << 6; 
            zoom_state = zoom_stop;
        }

        if(c1 == prev_c1){return;}
        else{prev_c1 = c1;}

        c1 = c1 | (max_zoom_speed << 3);
        packet.byte_1 = c1 >> 8;
        packet.byte_2 = c1;

        checksum_and_send((uint8_t*)&packet, sizeof(packet));
    }
}

bool AP_Mount_Viewpro_AA::center_camera(bool follow_camera){
    if(!_initialized) {
        return true; //True so if for some reason this gets called but not initialized, it won't get stuck waiting for the camera to center and return true
    }

    clear_roi_target();

    static int state = 1;
    static bool flag = false;
    static uint32_t prev_time = AP_HAL::millis();

    ///////////////////////////////////////////
    // Break tracking (even if not tracking) //
    ///////////////////////////////////////////
    if(strcmp(camera_models[_cam], "Q10T") == 0 || 
       strcmp(camera_models[_cam], "Z10TIR") == 0 ||
       strcmp(camera_models[_cam], "Z10TIR Mini") == 0 || 
       strcmp(camera_models[_cam], "Z40TIR") == 0 || 
       strcmp(camera_models[_cam], "H30T") == 0)
    {
        stop_tracking();
    }


    switch(state) {
        case 1:
            if(!flag){
                flag = true;
                prev_time = AP_HAL::millis();
                //////////////
                // Zoom out //
                //////////////
                ViewProAPI_ext packet2;
                packet2.C2_command = 0x53; //53 is synonymous with 1x zoom per ViewPro documentation
                checksum_and_send((uint8_t*)&packet2, sizeof(packet2)); 
            }
            else if(AP_HAL::millis() - prev_time >= 100){
                state = 2;
                flag = false;
            }
            break;

        case 2:
            if(!flag){
                flag = true;
                prev_time = AP_HAL::millis();
                ////////////////////////////////////////
                // Bring pan and tilt to zero degrees //
                ////////////////////////////////////////
                ViewProAPI packet1;
                packet1.command_id = 0x04; //Home Position
                checksum_and_send((uint8_t*)&packet1, sizeof(packet1));
            }
            else if(AP_HAL::millis() - prev_time >= 100){
                flag = false;
                state = 1;
                _just_centered = true;
                return true;
            }
            break;
    }

    return false;
}

bool AP_Mount_Viewpro_AA::supports_ftc() const {
    if(strcmp(camera_models[_cam], "Q10F") == 0 || 
       strcmp(camera_models[_cam], "Z10TIR") == 0 || 
       strcmp(camera_models[_cam], "Z10TIR Mini") == 0 || 
       strcmp(camera_models[_cam], "Z40K") == 0){
        return true;
    }
    return false;
}

void AP_Mount_Viewpro_AA::look_down(){
    if(!_initialized) {
        return;
    }

    clear_roi_target();

    if(strcmp(camera_models[_cam], "Q10T") == 0 || 
       strcmp(camera_models[_cam], "Z10TIR") == 0 || 
       strcmp(camera_models[_cam], "Z40TIR") == 0 || 
       strcmp(camera_models[_cam], "H30T") == 0)
    {
        stop_tracking();
    }

    servo_control packet;
    packet.command_id = 0x0B; //Absolute Angle Mode
    packet.parameter_2_big = 0x40;
    packet.parameter_2_little = 0x00;
    checksum_and_send((uint8_t*)&packet, sizeof(packet));
}

void AP_Mount_Viewpro_AA::stow(){
    if(!_initialized) {
        return;
    }

    clear_roi_target();

    if(strcmp(camera_models[_cam], "Q10T") == 0 || 
    strcmp(camera_models[_cam], "Z10TIR") == 0 || 
    strcmp(camera_models[_cam], "Z40TIR") == 0 || 
    strcmp(camera_models[_cam], "H30T") == 0)
    {
        stop_tracking();
    }

    servo_control packet;
    packet.command_id = 0x0B; //Absolute Angle Mode
    packet.parameter_2_big = 0xC0;
    packet.parameter_2_little = 0x00;
    checksum_and_send((uint8_t*)&packet, sizeof(packet));
}

void AP_Mount_Viewpro_AA::toggle_detection() {
    if(!_initialized) {
        return;
    }
    ViewProAPI packet;
    packet.command = 0x05;
    checksum_and_send((uint8_t*)&packet, sizeof(packet));
}

void AP_Mount_Viewpro_AA::disable_eo_dzoom() {
    if(!_initialized) {
        return;
    }
    optical_control_ext packet;
    packet.command_id = 0x07;
    checksum_and_send((uint8_t*)&packet, sizeof(packet));
}

void AP_Mount_Viewpro_AA::enable_eo_dzoom() {
    if(!_initialized) {
        return;
    }
    optical_control_ext packet;
    packet.command_id = 0x06;
    checksum_and_send((uint8_t*)&packet, sizeof(packet));
}

void AP_Mount_Viewpro_AA::checksum_and_send(uint8_t* packet, uint32_t size) {
    uint8_t len = packet[3];
    uint8_t checksum = len;
    for(uint8_t i =0;i < len-2;i++){
        checksum = checksum ^ packet[4+i];
    }
    if ((size_t)_port->txspace() < size) {
        return;
    }
    for (uint8_t i = 0;  i != size ; i++) {
        _port->write(packet[i]);
    }
    _port->write(checksum);
}

void AP_Mount_Viewpro_AA::checksum_and_send_legacy(uint8_t* packet, uint32_t size) {
	// compute checksum
	uint16_t checksum = 0;
	for (uint8_t i = 0;  i < size-1 ; i++) {
		checksum += packet[i];
	}
	packet[size-1] = (uint8_t)(checksum % 256);

	if ((size_t)_port->txspace() < size-1) {
		return;
	}

	for (uint8_t i = 0;  i != size ; i++) {
		_port->write(packet[i]);
	}
}

void AP_Mount_Viewpro_AA::read_incoming() {
    uint8_t data;
    int16_t numc;
    numc = _port->available();
    if (numc < 0 ){
        return;
    }
    for (int16_t i = 0; i < numc; i++) {        // Process bytes received
        data = _port->read();
        if(numc == 54 && i > 6){_buffer[i-7] = data;}
        if(numc == 47){_buffer[i] = data;}
        if(numc == 11){_sdcard_buffer[i] = data;}
    }
    if(numc == 47 || numc == 54){parse_body();}
    if(numc == 11){parse_sd_card_reply();}
}

void AP_Mount_Viewpro_AA::parse_body() {
    _camera_pan_angle = (int16_t)((_buffer.B1_byte_3 << 8) | _buffer.B1_byte_4) * (0.0054931640625); //1bit=360/65536°, signed integer
    _camera_tilt_angle = (int16_t)((_buffer.B1_byte_5 << 8) | _buffer.B1_byte_6) * (0.0054931640625); //1bit=360/65536°, signed integer
    uint16_t temp_zoom = (uint16_t)((_buffer.D1_byte_11 << 8) | _buffer.D1_byte_12); // 1 bit = 0.1x zoom
    _current_zoom = MAX((temp_zoom / 10.0), 1.0);

    fov_angle_h = (uint16_t)((_buffer.D1_byte_9 << 8) | _buffer.D1_byte_10) / 100.0;
    fov_angle_v = (uint16_t)((_buffer.D1_byte_7 << 8) | _buffer.D1_byte_8) / 100.0;

    uint32_t _current_time = AP_HAL::millis();
    _delta_camera_pan_angle = _camera_pan_angle - _prev_camera_pan_angle;
	_camera_pan_rate = (_delta_camera_pan_angle / ((_current_time - _feedback_timeout) * 1000));
	_prev_camera_pan_angle = _camera_pan_angle;
	_feedback_timeout = _current_time;
}

void AP_Mount_Viewpro_AA::parse_sd_card_reply() {
    if(_sdcard_buffer.control_command == 1){
        _sd_card_status.status = _sdcard_buffer.byte2;
        if((_sdcard_buffer.byte3 & 0x01) == 1){_sd_card_status.error = true;}
        if((_sdcard_buffer.byte3 & 0x02) == 1){_sd_card_status.recording = true;}
        else{_sd_card_status.recording = true;}
    }
    else if(_sdcard_buffer.control_command == 2) {
        
    }
    else if(_sdcard_buffer.control_command == 3) {
        
    }
    else if(_sdcard_buffer.control_command == 4) {
        
    }
    else if(_sdcard_buffer.control_command == 5) {
        
    }
}

void AP_Mount_Viewpro_AA::write_gimbal_log() {
	AP_Logger *logger = AP_Logger::get_singleton();
	logger->write_vp_log(_camera_tilt_angle, 0, _camera_pan_angle, 0, 0);
}

bool AP_Mount_Viewpro_AA::healthy() const {
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
    return true;
}