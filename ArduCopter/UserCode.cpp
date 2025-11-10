#include "Copter.h"
#include <AP_DroneCAN/AP_DroneCAN.h>
#include <AP_VisualOdom/AP_VisualOdom.h>

#define SERVO_TEST_DELAY_MS 5000

#ifdef USERHOOK_INIT
void Copter::userhook_init()
{
    //Status flags
    AP_Notify::flags.low_servo_voltage = false;
    AP_Notify::flags.ekf_nav_good = false;

    //Killswitch Variables
    killswitch_timer = 0;
	killswitch_pressed = false;

    //Herelink Gimbal Control Variables
    ch9_button_pressed = false;
    ch10_button_pressed = false;
    ch11_button_pressed = false;

    long_press_flag_ch9 = false;
    long_press_flag_ch10 = false;
    long_press_flag_ch11 = false;

    short_press_flag_ch9 = false;
    short_press_flag_ch10 = false;
    short_press_flag_ch11 = false;

    ch9_button_hold = false;
    ch10_button_hold = false;
    ch11_button_hold = false;

	initial_motor_offsets = compass.get_motor_compensation();

	copter.ap.cam_function_button_pressed = false;

    // startup spirit state
    spirit_state = disarm;

    camera_mount.set_mode_to_default();

    //RPM compensation support
    start_rpm_comp_time = 0;
    rpm_update_counter = 0;
    hover_rpm_filter.set_cutoff_frequency(50.0f, 0.25f);
	hover_rpm_filter.reset(g2.ascent_parameters.get_rpm_hover());
	motors->set_hover_RPM(g2.ascent_parameters.get_rpm_hover());
    motors->set_aft_rotor_RPM(0.0f);

    //Self Config init
    num_battery = g2.ascent_parameters.get_battery_number();
    payload_weight = g2.ascent_parameters.get_payload_weight();
	tube_launch = g2.ascent_parameters.get_tube_launch();
	yaw_p = g2.ascent_parameters.get_yaw_p();
	yaw_i = g2.ascent_parameters.get_yaw_i();
	yaw_p_ll = g2.ascent_parameters.get_yaw_p_ll();
	yaw_i_ll = g2.ascent_parameters.get_yaw_i_ll();

   	if(g2.ascent_parameters.get_auto_config_enabled() != 0){
		auto_config();
    }

	if (logger._params.log_disarmed){
		logger._params.log_disarmed.set((AP_Logger::LogDisarmed)0);
	}

	if(g2.ascent_parameters.get_follow_camera_on_reboot()!=0 && camera_mount.supports_ftc()){
		g2.ascent_parameters.set_follow_camera(true);
		gcs().send_parameter_value("AA_FTC_EN", AP_PARAM_INT8, g2.ascent_parameters.get_follow_camera());
	} else {
		g2.ascent_parameters.set_follow_camera(false);
		gcs().send_parameter_value("AA_FTC_EN", AP_PARAM_INT8, g2.ascent_parameters.get_follow_camera());
	}

	g2.ascent_parameters.set_ekf_source(0);
	gcs().send_parameter_value("AA_VIS_SRC", AP_PARAM_INT8, 0);
}
#endif

#ifdef USERHOOK_FASTLOOP
void Copter::userhook_FastLoop()
{
    // put your 100Hz code here
}
#endif

#ifdef USERHOOK_50HZLOOP
void Copter::userhook_50Hz()
{
	if(motors->get_auto_motor_offsets() != (bool)g2.ascent_parameters.get_auto_mot_offs()) {
		motors->set_auto_motor_offsets((bool)g2.ascent_parameters.get_auto_mot_offs());
	}

	const Vector3f &mag_field = compass.get_field(0); //Is there a get_instance(uavcan compass) type of function?
	const Vector3f &curr_offsets = compass.get_motor_offsets(0);
	const Vector3f field_decomp = mag_field - curr_offsets;
	float current;
	if (battery.current_amps(current)){};

	if(!mot_offs_calculated && !motors->armed() && spirit_state != spoolup && spirit_state != calculating && g2.ascent_parameters.get_auto_mot_offs()) {
		mot_x_rest_avg = mot_x_rest.apply(field_decomp.x);
		mot_y_rest_avg = mot_y_rest.apply(field_decomp.y);
		mot_z_rest_avg = mot_z_rest.apply(field_decomp.z);
		curr_rest_avg = curr_rest.apply(current);
	}

	//detect button input in Herelink at 50Hz
	if(g2.ascent_parameters.get_gcs_type() == AP_Mount::GCS_Type_Herelink){
		Detect_Buttons();
	}

	if (motors->armed()){
		if (!prev_armed && g2.ascent_parameters.get_follow_camera()){
			centering_camera = true;
		}
		prev_armed = true;
	} else if (prev_armed){
		// Stop the recording
		camera_mount.stop_record();

		if (g2.ascent_parameters.get_stow_camera()){ // Expects camera to be stowed for landing.
			centering_camera = true;
		}
			
		// Send battery health message to GCS
		float batt_resistance_estimate = battery.get_resistance();
		float cell_resistance = g2.ascent_parameters.get_battery_number()*g2.ascent_parameters.get_single_pack_parallel()*(batt_resistance_estimate/g2.ascent_parameters.get_single_pack_series());

		mavlink_command_long_t cmd;
		cmd.command = MAV_CMD_ASCENT_PAYLOAD;
		cmd.target_component = 255;
		cmd.param1 = 1;
		cmd.param2 = cell_resistance>g2.ascent_parameters.get_bad_cell_res_mohm()? 1.0f: 0.0f;
		gcs().send_to_active_channels(MAVLINK_MSG_ID_COMMAND_LONG, (const char*)&cmd);

		prev_armed = false;
	}

    //Run center-camera state machine
	if(centering_camera && camera_mount.center_camera(g2.ascent_parameters.get_follow_camera()) && RC_Channels::rc_channel(CH_9)->get_radio_in() < 1800){centering_camera = false;}

	//Call topple sense
	if(motors->armed()){
		topple_sense();
	}

	//RPM compensation
	update_rpm_hover();
	float rpm;
	if (rpm_sensor.get_rpm(1, rpm)){
		motors->set_aft_rotor_RPM(rpm);
	}

	//// State Machine ////
	//Keep track of vehicle state, call state specific functions
	if(!motors->armed()){
		spirit_state = disarm;

	//the following considers the vehicle to be armed...
	//always move through spoolup
	}else if(spirit_state == disarm){
		payload_power(true);
		hal.gpio->write(53, true);
		spirit_state = spoolup;

	}else if(spirit_state == on_ground and (copter.flightmode->is_taking_off() or !ap.land_complete)){
		spirit_state = takeoff;

	}else if(spirit_state == takeoff and (!copter.flightmode->is_taking_off() or copter.flightmode->has_manual_throttle()) ){
		spirit_state = hover;

	}else if((spirit_state == hover or spirit_state == landing) and ap.land_complete){
		spirit_state = on_ground;
	}

	switch(spirit_state){

	case disarm:
		//Prompt for flashing yellow LEDs if no RC input
		if(RC_Channels::rc_channel(CH_3)->get_radio_in() == 0){
			AP_Notify::flags.no_RC_in = true;
		}else{
			AP_Notify::flags.no_RC_in = false;
		}
		_servo_test1_complete = false; // Reset servo test
		_servo_test2_complete = false; // Reset servo test
		_servo_test3_complete = false; // Reset servo test
		motors->servo_test_set_mode(0); // Restart test
		servo_test_start_ms = 0;

		break;
		
	case spoolup:
	{
		//Looks for issues with startup
		servo_voltage_watcher();

		_fwd_thr = motors->get_fwd_rotor_thr();
		_aft_thr = motors->get_aft_rotor_thr();

		rpm_sensor.get_rpm(0,_fwd_rpm);
		rpm_sensor.get_rpm(1,_aft_rpm);

		//Check for hung blades
		if(_fwd_rpm <= 0 and _fwd_thr >= 0.10 and current > g2.ascent_parameters.get_blade_hung_up_current() ){
			gcs().send_text(MAV_SEVERITY_CRITICAL, "DISARMING: FORWARD ROTORS HUNG");
		 	copter.arming.disarm(AP_Arming::Method::MOTORTEST);
			break;
		}

		if(_aft_rpm <= 0 and _aft_thr >= 0.10 and current > g2.ascent_parameters.get_blade_hung_up_current()){
			gcs().send_text(MAV_SEVERITY_CRITICAL, "DISARMING: AFT ROTORS HUNG");
			copter.arming.disarm(AP_Arming::Method::MOTORTEST);
			break;
		}

#if HAL_ENABLE_DRONECAN_DRIVERS
		// TODO: this assumes the servos are connected on a specific driver (0) and configured as servo 1 and 2.
		AP_DroneCAN *dronecan = AP_DroneCAN::get_dronecan(0); 	
		if (dronecan!=nullptr && dronecan->get_servo_bitmask() == 3 && !g2.ascent_parameters.get_tube_launch() &&
		    motors->get_spool_state()!=AP_Motors::SpoolState::SHUT_DOWN){
			if(servo_test_start_ms==0){
				servo_test_start_ms = AP_HAL::millis16();
				gcs().send_text(MAV_SEVERITY_INFO, "SERVO TEST: START");
			}
			if (!_servo_test1_complete){
				if (_fwd_rpm>1000){ // Make sure fwd rotor is spooled up
					motors->servo_test_set_mode(1);
				}
				//check that feedback matches
				if ((dronecan->srv_1_pos<-0.6207f)&&(dronecan->srv_2_pos<-0.6207f)){
					_servo_test1_complete = true;
				}
				if ( (AP_HAL::millis16()-servo_test_start_ms)>SERVO_TEST_DELAY_MS ){
					gcs().send_text(MAV_SEVERITY_CRITICAL, "DISARMING: SERVO MIN THROW FAILED");
					copter.arming.disarm(AP_Arming::Method::MOTORTEST);
				}
				break;
			} else if (!_servo_test2_complete){
				if (_fwd_rpm>1000){
					motors->servo_test_set_mode(2);
				}
				//check that feedback matches
				if ((dronecan->srv_1_pos>0.6207f)&&(dronecan->srv_2_pos>0.6207f)){
					_servo_test2_complete = true;
				}
				if ( (AP_HAL::millis16()-servo_test_start_ms)>SERVO_TEST_DELAY_MS ){
					gcs().send_text(MAV_SEVERITY_CRITICAL, "DISARMING: SERVO MAX THROW FAILED");
					copter.arming.disarm(AP_Arming::Method::MOTORTEST);
				}
				break;
			} else if (!_servo_test3_complete){
				if (_fwd_rpm>1000){
					motors->servo_test_set_mode(3);
				}				
				//check that feedback matches
				if ( (abs(dronecan->srv_1_pos)<0.05f) &&
				     (abs(dronecan->srv_2_pos)<0.05f) ){
					motors->servo_test_set_mode(0); // Finalize test
					_servo_test3_complete = true;
					gcs().send_text(MAV_SEVERITY_INFO, "SERVO TEST: PASS");
				}
				if ( (AP_HAL::millis16()-servo_test_start_ms)>SERVO_TEST_DELAY_MS ){
					gcs().send_text(MAV_SEVERITY_CRITICAL, "DISARMING: SERVO CENTERING FAILED");
					copter.arming.disarm(AP_Arming::Method::MOTORTEST);
				}
				break;
			}
		}
#endif // #if HAL_ENABLE_DRONECAN_DRIVERS

		if(_fwd_rpm > 1000.0f and _aft_rpm > 1000.0f){
			if(g2.ascent_parameters.get_auto_mot_offs()) {
				spirit_state = calculating;
				calc_delay = AP_HAL::millis();
			}
			else {
				motors->spoolup_complete(true);
				spirit_state = on_ground; //advance to on_ground
			}

		}else if(_fwd_rpm > 1000.0f){
			motors->enable_aft_rotor(true);
		}
		//Don't let control system wind up during spoolup
        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_z_controller(0.0f);
		break;
	}

	case calculating:
	{
		//Calculate motor offsets here
		mot_offs_calculated = true;
		if(AP_HAL::millis() - calc_delay >= 1250) {
			float current_delta = current - curr_rest_avg;
			if(!is_zero(current_delta)){
				offs_x_avg = offs_x_avg_filt.apply((mot_x_rest_avg - field_decomp.x) / current_delta);
				offs_y_avg = offs_y_avg_filt.apply((mot_y_rest_avg - field_decomp.y) / current_delta);
				offs_z_avg = offs_z_avg_filt.apply((mot_z_rest_avg - field_decomp.z) / current_delta);
			}
			else {
				copter.arming.disarm(AP_Arming::Method::MOTORTEST);
				gcs().send_text(MAV_SEVERITY_CRITICAL, "DISARMING: CURRENT SENSOR FAILURE");
			}
		}
		if(offs_x_avg_filt.isFull()) {
			gcs().send_text(MAV_SEVERITY_INFO, "MOTOR OFFS X = %f", offs_x_avg);
			gcs().send_text(MAV_SEVERITY_INFO, " MOTOR OFFS Y = %f", offs_y_avg);
			gcs().send_text(MAV_SEVERITY_INFO, "MOTOR OFFS Z = %f", offs_z_avg);

			Vector3f offset_fail;
			Vector3f offset_clip;
			if(num_battery != 0) {
				offset_fail = (initial_motor_offsets*(2.0/(float)num_battery)) * 3.0;
				offset_clip = (initial_motor_offsets*(2.0/(float)num_battery)) * 2.0;
			}
			else {
				copter.arming.disarm(AP_Arming::Method::MOTORTEST);
				gcs().send_text(MAV_SEVERITY_CRITICAL, "DISARMING: BATTERIES SET TO ZERO");
			}
			Vector3f offset_calc = Vector3f(offs_x_avg,offs_y_avg,offs_z_avg);

			//0-50% from standard array = take
			//50-100% from standard array = clip
			//100%+ prevent arming and suggest calibration/battery change/environment relocate/ check configuration
			for (uint8_t i = 0; i < 3; i++) {
				if(initial_motor_offsets[i] > 0) {
					if(offset_calc[i] > offset_fail[i]) {
						copter.arming.disarm(AP_Arming::Method::MOTORTEST);
						gcs().send_text(MAV_SEVERITY_CRITICAL, "DISARMING: UNHEALTHY COMPASS OR BATTERY BEHAVIOR");
						gcs().send_text(MAV_SEVERITY_CRITICAL, "Try checking Spirit configuration, recalibrating compass, or swapping batteries");
						break;
					}
					else if(offset_calc[i] > offset_clip[i]) { //greater than clip but less than fail
						offset_calc[i] = offset_clip[i];
					}
					else if (offset_calc[i] < 0) { //Different sign than param. Probably noise around zero
						offset_calc[i] = 0;
					}
				}
				else {
					if(offset_calc[i] < offset_fail[i]) {
						copter.arming.disarm(AP_Arming::Method::MOTORTEST);
						gcs().send_text(MAV_SEVERITY_CRITICAL, "DISARMING: UNHEALTHY COMPASS OR BATTERY BEHAVIOR");
						gcs().send_text(MAV_SEVERITY_CRITICAL, "Try checking Spirit configuration, recalibrating compass, or swapping batteries");
						break;
					}
					else if(offset_calc[i] <= offset_clip[i]) { 
						offset_calc[i] = offset_clip[i];
					}
					else if (offset_calc[i] > 0) {
						offset_calc[i] = 0;
					}
				}
			}

			compass.set_spirit_mot_correction_factor(1.0f);
			compass.set_motor_compensation(0, offset_calc);
			motors->spoolup_complete(true);
			spirit_state = on_ground; //advance to on_ground
		}
		
		//Don't let control system wind up during spoolup
        attitude_control->reset_rate_controller_I_terms();
        pos_control->relax_z_controller(0.0f);
		break;
	}

	case on_ground:

		break;

	case takeoff:

		break;


	case hover:
		//////// Determine if we are landing //////
		if(copter.flightmode->get_alt_above_ground_cm() <= (int32_t)g2.land_alt_low){   // need to be close to the ground

			if(copter.flightmode->is_landing()){
				spirit_state = landing;
				attitude_control->enable_angle_boost(false);
				break;
			// for manual also need to be decending
			}else if(!copter.flightmode->is_autopilot() and copter.flightmode->get_pilot_desired_climb_rate((float)channel_throttle->get_control_in()) < 0.0f){
				spirit_state = landing;
				attitude_control->enable_angle_boost(false);
				break;
			}
		}
		break;

	case landing:
		if (g2.ascent_parameters.get_stow_camera() && copter.flightmode->get_alt_above_ground_cm()<g2.ascent_parameters.get_stow_camera_height_cm()){
			camera_mount.stow();
		}
		//////cancel landing if no longer auto landing or if pilot is not commanding a decent
        if(!copter.flightmode->is_autopilot() and copter.flightmode->get_pilot_desired_climb_rate(channel_throttle->get_control_in()) >= 0.0f){
            spirit_state = hover;
            attitude_control->enable_angle_boost(true);
            break;
        }

        if(copter.flightmode->is_autopilot() and !copter.flightmode->is_landing()){
            spirit_state = hover;
            attitude_control->enable_angle_boost(true);
            break;
        }

		break;

    }
}
#endif

#ifdef USERHOOK_MEDIUMLOOP
void Copter::userhook_MediumLoop()
{
	if(spirit_state == disarm or spirit_state == hover or spirit_state == landing){
		// Prevent user from moving the gimbal during spoolup and takeoff
		handle_ftc_mode();
	}
    Killswitch();
	camera_mount.update_10hz();
    //Controls when RPM comp is activated
    if(spirit_state == disarm or spirit_state == spoolup or spirit_state == on_ground or (!g2.ascent_parameters.get_tube_launch() && spirit_state == takeoff) ){
		// TODO: rpm_comp during takeoff with very light vehicle might be a corner case of this logic
        motors->enable_rpm_comp(false);
        start_rpm_comp_time = AP_HAL::millis16();
    }else if( g2.ascent_parameters.get_tube_launch() || (AP_HAL::millis16() - start_rpm_comp_time > 3000)){
        motors->enable_rpm_comp(true);
    }

	if (copter.flightmode->mode_number() == Mode::Number::GUIDED && 
		copter.mode_guided.submode() == ModeGuided::SubMode::WP && // Needs GUID_OPTIONS = 64 (bit 6 active)
		wp_nav->reached_wp_destination()) {
		copter.flightmode->set_mode(Mode::Number::LOITER, ModeReason::MISSION_END);
	}

	/*
	Only check health status of FTC mode if...
		1) The mode is enabled
		2) The camera supports FTC mode
			a) The mode should not be enabled with a camera that does not supported, as AscentQ will grey out the button accordingly. If the wrong camera is selected and you try to enable, you'll likely be sent the "unhealthy" message
		3) The flight mode supports FTC mode
	*/
	char fail_msg[MAVLINK_MSG_STATUSTEXT_FIELD_TEXT_LEN+1] = {};
	if(g2.ascent_parameters.get_follow_camera() && arming.is_armed()) { 
		if(!camera_mount.supports_ftc())
		{
			gcs().send_text(MAV_SEVERITY_INFO,"DISABLING FTC MODE: UNSUPPORTED CAMERA");
			g2.ascent_parameters.set_follow_camera(false);
			gcs().send_parameter_value("AA_FTC_EN", AP_PARAM_INT8, g2.ascent_parameters.get_follow_camera());
			if(camera_mount.get_mount_type() == AP_Mount::Type::Viewpro_AA) { //Handle ViewPro differently
				camera_mount.yaw_follow_disable();
			}	
		}
		else if(strcmp(copter.flightmode->name(), "AUTO") == 0 ||
	   			strcmp(copter.flightmode->name(), "GUIDED") == 0 ||
	   			strcmp(copter.flightmode->name(), "CIRCLE") == 0 ||
				strcmp(copter.flightmode->name(), "BRAKE") == 0 ||
	   			strcmp(copter.flightmode->name(), "RTL") == 0 ||
	   			strcmp(copter.flightmode->name(), "LAND") == 0)
		{
			gcs().send_text(MAV_SEVERITY_INFO,"DISABLING FTC MODE: UNSUPPORTED FLIGHT MODE");	
			g2.ascent_parameters.set_follow_camera(false);
			gcs().send_parameter_value("AA_FTC_EN", AP_PARAM_INT8, g2.ascent_parameters.get_follow_camera());
			if(camera_mount.get_mount_type() == AP_Mount::Type::Viewpro_AA) { //Handle ViewPro differently
				camera_mount.yaw_follow_disable();
			}		
	   	}
	   	else if(camera_mount.get_mount_type() != AP_Mount::Type::Viewpro_AA && !camera_mount.pre_arm_checks(fail_msg, sizeof(fail_msg)))
		{
			gcs().send_text(MAV_SEVERITY_INFO,"DISABLING FTC MODE: FEEDBACK UNHEALTHY");
			g2.ascent_parameters.set_follow_camera(false);
			gcs().send_parameter_value("AA_FTC_EN", AP_PARAM_INT8, g2.ascent_parameters.get_follow_camera());		
		}
	}
}
#endif

#ifdef USERHOOK_SLOWLOOP
void Copter::userhook_SlowLoop()
{
    if(!motors->armed()){

        nav_filter_status filter_status = inertial_nav.get_filter_status();

        if(filter_status.flags.const_pos_mode){
            AP_Notify::flags.ekf_nav_good = false;
        }else{
            AP_Notify::flags.ekf_nav_good = true;
        }
    }

    if(g2.ascent_parameters.get_gcs_type() == AP_Mount::GCS_Type_Herelink){
        Decode_Buttons();
    }
}
#endif

#ifdef USERHOOK_SUPERSLOWLOOP
void Copter::userhook_SuperSlowLoop()
{
	//Look for a change in vehicle configuration
	if( (g2.ascent_parameters.get_battery_number() != num_battery) or
		(camera_type != g2.ascent_parameters.get_camera_type()) or
		(fabsf(g2.ascent_parameters.get_payload_weight() - payload_weight) > 0.01f) or
		(g2.ascent_parameters.get_tube_launch() != tube_launch) or
		!is_equal((float)g2.ascent_parameters.get_yaw_p() , yaw_p) or
		!is_equal((float)g2.ascent_parameters.get_yaw_i() , yaw_i) or
		!is_equal((float)g2.ascent_parameters.get_yaw_p_ll() , yaw_p_ll) or
		!is_equal((float)g2.ascent_parameters.get_yaw_i_ll() , yaw_i_ll) or
		(g2.ascent_parameters.get_auto_mot_offs() != auto_mot)
	  ){
		camera_type = g2.ascent_parameters.get_camera_type();
		camera_mount.set_cam_type(camera_type);
		num_battery = g2.ascent_parameters.get_battery_number();
		payload_weight = g2.ascent_parameters.get_payload_weight();
		tube_launch = g2.ascent_parameters.get_tube_launch();
		yaw_p = g2.ascent_parameters.get_yaw_p();
		yaw_i = g2.ascent_parameters.get_yaw_i();
		yaw_p_ll = g2.ascent_parameters.get_yaw_p_ll();
		yaw_i_ll = g2.ascent_parameters.get_yaw_i_ll();
		auto_mot = g2.ascent_parameters.get_auto_mot_offs();
		if (g2.ascent_parameters.get_auto_config_enabled() != 0){
			auto_config();
		}
	}

	camera_mount.set_gcs_type(g2.ascent_parameters.get_gcs_type());
	camera_mount.set_top_mount(g2.ascent_parameters.get_top_mount());

	if(g2.ascent_parameters.get_vehicle_type() == 2 && g2.ascent_parameters.get_battery_number() > 1){
		battery.display_voltage_only(0, true);
		battery.display_voltage_only(1, true);
	}

	if(!motors->armed() and copter.battery.voltage() < 35.0 and (copter.battery.voltage() > 5.0 or !hal.gpio->usb_connected())){
		gcs().send_text(MAV_SEVERITY_CRITICAL,"Battery Critical");
		payload_power(false);
	}else{
		payload_power(true);
	}

	// Call terrain cleaning util
	Location loc;
	if (ahrs.get_location(loc) && !arming.is_armed()){ // Only clear logs if we have a location and are disarmed
		terrain.clear_unused_tiles(loc);
	}

	// EKF source configuration
	if ( visual_odom.get_singleton() != nullptr &&
		 visual_odom.get_type() == AP_VisualOdom::VisualOdom_Type::VOXL &&
		 prev_ekf_source != g2.ascent_parameters.get_ekf_source() && // Source changed
		 AP_HAL::millis()-last_ekf_source_switch_ms > 1000U ) 
	{   // Last change happened at least 1 second ago
		if (g2.ascent_parameters.get_ekf_source() == 0){
			if (gps.get_singleton() != nullptr && gps.is_healthy()){
				gcs().send_text(MAV_SEVERITY_CRITICAL,"Switching to GPS");
				AP::ahrs().set_posvelyaw_source_set(0);
				prev_ekf_source = g2.ascent_parameters.get_ekf_source();
				last_ekf_source_switch_ms = AP_HAL::millis();

			} else {
				gcs().send_text(MAV_SEVERITY_CRITICAL,"GPS unhealthy: switch rejected");
				gcs().send_parameter_value("AA_VIS_SRC", AP_PARAM_INT8, prev_ekf_source);
				g2.ascent_parameters.set_ekf_source(1);
			}

		} 
		else {
			if (visual_odom.get_singleton() != nullptr && visual_odom.healthy()){
				gcs().send_text(MAV_SEVERITY_CRITICAL,"Switching to Visual Odometry");
				AP::ahrs().set_posvelyaw_source_set(1);
				prev_ekf_source = g2.ascent_parameters.get_ekf_source();
				last_ekf_source_switch_ms = AP_HAL::millis();

				g.rtl_speed_cms.set(MIN(g.rtl_speed_cms.get(),1000));
				wp_nav->set_speed_cms(MIN(wp_nav->get_default_speed_xy(),1000));
				loiter_nav->set_speed_cms(wp_nav->get_default_speed_xy());

				gcs().send_parameter_value("RTL_SPEED", AP_PARAM_FLOAT, wp_nav->get_default_speed_xy());

			} else {
				gcs().send_text(MAV_SEVERITY_CRITICAL,"VisOdom unhealthy: switch rejected");
				gcs().send_parameter_value("AA_VIS_SRC", AP_PARAM_INT8, prev_ekf_source);
				g2.ascent_parameters.set_ekf_source(0);
			}
		}
	}

#if AP_OPENDRONEID_ENABLED
	uint8_t pfst_fail = 0;
	copter.opendroneid.get_pfst_fail_status(pfst_fail);

	// Encode as float to avoid problems on the line
	float remote_id_health_flag_enc;
	memcpy(&remote_id_health_flag_enc, &pfst_fail, sizeof(pfst_fail));

	mavlink_msg_command_long_send(MAVLINK_COMM_1, // Assumes Telemetry is on COM1
								255,
								0,
								MAV_CMD_ASCENT_RID_HEALTH,
								0,        // confirmation of zero means this is the first time this message has been sent
								remote_id_health_flag_enc,
								0, 0, 0, 0, 
								0, 0);  // param2 ~ param7 unused
#endif
}
	

#endif

#ifdef USERHOOK_AUXSWITCH
void Copter::userhook_auxSwitch1(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #1 handler here (CHx_OPT = 47)
}

void Copter::userhook_auxSwitch2(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #2 handler here (CHx_OPT = 48)
}

void Copter::userhook_auxSwitch3(const RC_Channel::AuxSwitchPos ch_flag)
{
    // put your aux switch #3 handler here (CHx_OPT = 49)
}
#endif

void Copter::topple_sense(){
	if(!g2.ascent_parameters.get_en_topple_sense()){
		return;
	}

	// Always run the crash case if any of the bits is true
	if(fabsf(attitude_control->get_att_error_angle_deg()) > 100.0f and copter.flightmode->get_alt_above_ground_cm() < 400){
		copter.arming.disarm(AP_Arming::Method::CRASH);
		gcs().send_text(MAV_SEVERITY_CRITICAL,"TOPPLE FLYING: ATT ERROR");
		return;
	}

	if (g2.ascent_parameters.get_en_topple_sense() & (1<<1)) { // 2nd bit controls topple sense on the ground 
		//Disarm in spoolup if inclined over 15 deg
		if(spirit_state == spoolup){
			if(labs(ahrs.pitch_sensor) > 1500 or labs(ahrs.roll_sensor) > 1500){
				copter.arming.disarm(AP_Arming::Method::CRASH);
				gcs().send_text(MAV_SEVERITY_CRITICAL,"TOPPLE SPOOLUP: PITCH EXCEED 15 DEG");
				return;
			}

		//Disarm in takeoff if att error over 30 deg and below 1.5m
		}else if(spirit_state == takeoff){
			if(fabsf(attitude_control->get_att_error_angle_deg()) > 30.0f and copter.flightmode->get_alt_above_ground_cm() < 150){
				copter.arming.disarm(AP_Arming::Method::CRASH);
				gcs().send_text(MAV_SEVERITY_CRITICAL,"TOPPLE SPOOLUP: ATT ERROR");
				return;
			}
		}
	}

	if (g2.ascent_parameters.get_en_topple_sense() & (1<<0)) { // 1st bit controls topple sense on landing
		//Landing topple conditions
		if(spirit_state == landing){
			if(fabsf(attitude_control->get_att_error_angle_deg()) > 45.0f){
				copter.arming.disarm(AP_Arming::Method::CRASH);
				gcs().send_text(MAV_SEVERITY_CRITICAL,"TOPPLE LANDING: ATT ERROR");
				return;
			}
			// TODO: add case that triggers with range finder.
		}
	}
}

void Copter::servo_voltage_watcher(){
	const float servo_voltage = hal.analogin->servorail_voltage();
	if(servo_voltage < AP_BoardConfig::get_minimum_servo_voltage() ){
		copter.arming.disarm(AP_Arming::Method::MOTORTEST);
		AP_Notify::flags.low_servo_voltage = true;
		gcs().send_text(MAV_SEVERITY_CRITICAL,"LOW SERVO VOLTAGE");
	}
}

void Copter::Killswitch(){

	if(!motors->armed()){
		return;
	}

	//If its NOT a herelink and NOT a "none" type (used for Taranis type radios), return
	if (g2.ascent_parameters.get_gcs_type() != AP_Mount::GCS_Type_Herelink && g2.ascent_parameters.get_gcs_type() != AP_Mount::GCS_Type_None){
		return;
	}

	if(RC_Channels::rc_channel(CH_8)->get_radio_in() >= 1700){
		if(!killswitch_pressed) {
			killswitch_pressed = true;
			killswitch_timer = millis();
		}
	}
	else if(killswitch_pressed) {
		killswitch_pressed = false;
	}

	/*
		KILLSWITCH CRITERIA
		1: KS Button is actively being held
		2: KS Button has been held for at least as long as the KS delay parameter
		3: Throttle is zero, land is complete, throttle pwm is 0, or attitude angular error exceeds 45 degrees
	*/
	if(killswitch_pressed && (millis() - killswitch_timer >= (uint32_t)g2.ascent_parameters.get_killswitch_delay_ms()) && (copter.ap.throttle_zero || copter.ap.land_complete || channel_throttle->get_control_in() == 0 || fabsf(attitude_control->get_att_error_angle_deg()) > 45.0f))
	{
		copter.arming.disarm(AP_Arming::Method::TERMINATION);
	}
}

void Copter::handle_ftc_mode(){
	//Do not allow Top Mount to enter FTC. UI will not appear, this would be a result of a parameter change
	if(g2.ascent_parameters.get_follow_camera() && g2.ascent_parameters.get_top_mount()) {
		g2.ascent_parameters.set_follow_camera(false);
	}

	//If camera is a ViewPro, do not do follow the camera, but rather the camera should follow vehicle heading
	if(camera_mount.get_mount_type() == AP_Mount::Type::Viewpro_AA && g2.ascent_parameters.get_follow_camera()) {
		camera_mount.yaw_follow_enable();
	}
	else if(camera_mount.get_mount_type() == AP_Mount::Type::Viewpro_AA && !g2.ascent_parameters.get_follow_camera()){
		camera_mount.yaw_follow_disable();
	}

	//If the GCS is a Herelink or a Laptop, then disregard throttle input if cam button is pressed
	if(g2.ascent_parameters.get_gcs_type() == AP_Mount::GCS_Type_Herelink || g2.ascent_parameters.get_gcs_type() == AP_Mount::GCS_Type_PC){
		static bool gimbal_moving = false;
		//                        In FTC mode,                     moving yaw stick,                        no cam button
		if(g2.ascent_parameters.get_follow_camera() && !is_zero(channel_yaw->norm_input_dz()) && !copter.ap.cam_function_button_pressed) {
			gimbal_moving = true;
			camera_mount.set_gimbal_rates(channel_yaw->norm_input_dz(), 0.0); //Should do nothing when VP is selected. Commanding pan with yaw follow enabled does nothing
		}
		//Cam button pressed (FTC or not)
		else if(copter.ap.cam_function_button_pressed) {
			gimbal_moving = true;
			camera_mount.set_gimbal_rates(channel_yaw->norm_input_dz(), channel_throttle->norm_input_dz());
		}
		else if(gimbal_moving){
			gimbal_moving = false;
			camera_mount.set_gimbal_rates(0.0, 0.0);
		}
	}
	else{
		if(g2.ascent_parameters.get_follow_camera() && !is_zero(channel_yaw->norm_input_dz())) { //Even when in FTC mode, allow camera input to yaw
			camera_mount.set_gimbal_rates(channel_yaw->norm_input_dz(), mav_gimbal_tilt); //Invert pan
		}
		else{
			if(gcs_initialized) camera_mount.set_gimbal_rates(mav_gimbal_pan, mav_gimbal_tilt);
		}
	}
}

void Copter::Detect_Buttons(){ //Herelink only
	if(RC_Channels::rc_channel(CH_7)->get_radio_in() > 1800) copter.ap.cam_function_button_pressed = true;
	else copter.ap.cam_function_button_pressed = false;
	
	//Control zoom
	if(!is_zero(RC_Channels::rc_channel(CH_6)->norm_input_dz())){
		manual_zoom_control = true;
		camera_mount.set_zoom_rate(RC_Channels::rc_channel(CH_6)->norm_input_dz());
	}
	else if(manual_zoom_control) {
		manual_zoom_control = false;
		camera_mount.set_zoom_rate(0.0);
	}

	//CH9, A Button Pressed
	if(RC_Channels::rc_channel(CH_9)->get_radio_in() > 1800){
		if(!ch9_button_hold){
			if(!ch9_button_pressed){
				ch9_button_pressed = true;
				ch9_timer = millis();
			}else{
				if( (millis() - ch9_timer) > 750 ){
					long_press_flag_ch9 = true;  //these are reset in the 10Hz loop
					ch9_button_hold = true;
				}
			}
		}
	}
	else{
		if(ch9_button_pressed){
			if(!ch9_button_hold){  //if hold was active don't do a short_press
				short_press_flag_ch9 = true;//these are reset in the 10Hz loop
			}
		ch9_button_hold = false;
		ch9_button_pressed = false; //reset button press flag
		}
	}

	//CH10, B Button
	if(RC_Channels::rc_channel(CH_10)->get_radio_in() > 1800){
		if(!ch10_button_hold){
			if(!ch10_button_pressed){
				ch10_button_pressed = true;
				ch10_timer = millis();
			}else{
				if( (millis() - ch10_timer) > 750 ){
					long_press_flag_ch10 = true;  //these are reset in the 10Hz loop
					ch10_button_hold = true;
				}
			}
		}
	}
	else{
		if(ch10_button_pressed){
			if(!ch10_button_hold){  //if hold was active don't do a short_press
				short_press_flag_ch10 = true;//these are reset in the 10Hz loop
			}
		ch10_button_hold = false;
		ch10_button_pressed = false; //reset button press flag
		}
	}

	//CH11, C Button
	if(RC_Channels::rc_channel(CH_11)->get_radio_in() > 1800){
		if(!ch11_button_hold){
			if(!ch11_button_pressed){
				ch11_button_pressed = true;
				ch11_timer = millis();
			}else{
				if( (millis() - ch11_timer) > 750 ){
					long_press_flag_ch11 = true;  //these are reset in the 10Hz loop
					ch11_button_hold = true;
				}
			}
		}
	}
	else{
		if(ch11_button_pressed){
			if(!ch11_button_hold){  //if hold was active don't do a short_press
				short_press_flag_ch11 = true;//these are reset in the 10Hz loop
			}
		ch11_button_hold = false;
		ch11_button_pressed = false; //reset button press flag
		}
	}
}

void Copter::Decode_Buttons(){
/*
	Channel 9:  Button A
	Channel 10: Button B
	Channel 11: Button C
	Channel 8:  Button D
	Channel 7:  Cam button
	*/

	if(short_press_flag_ch9) {
		centering_camera = true;
		short_press_flag_ch9 = false;
	}

	if(long_press_flag_ch9) {
		if(notify.get_rgb_led_on_off()) {
			notify.set_rgb_led_on_off(false);
			gcs().send_text(MAV_SEVERITY_INFO, "LEDS OFF");
		}
		else {
			notify.set_rgb_led_on_off(true);
			gcs().send_text(MAV_SEVERITY_INFO, "LEDS ON");
		}
		long_press_flag_ch9 = false;
	}

	if(short_press_flag_ch10) {
		static bool eo = true; //This will get messed up with the on screen buttons... To be addressed
		if(eo){
			camera_mount.ir_full_screen();
			eo = false;
		}
		else {
			camera_mount.eo_full_screen();
			eo = true;
		}
		short_press_flag_ch10 = false;
	}

	if(long_press_flag_ch10) {
		camera_mount.do_nuc();
		long_press_flag_ch10 = false;
	}

	if(short_press_flag_ch11){
		// Switch to GPS-aided manual mode if in ALT_hold or any auto mode
		if(!copter.flightmode->requires_GPS() or copter.flightmode->is_autopilot()){
			if(!copter.set_mode((Mode::Number)copter.flight_modes[2].get(), ModeReason::GCS_COMMAND)){
				//If mode change fails, use ALT-Hold
				copter.set_mode((Mode::Number)copter.flight_modes[1].get(), ModeReason::GCS_COMMAND);
				AP_Notify::events.user_mode_change_failed = 1;
			}else{
				//Flash LED if mode change works
				AP_Notify::events.user_mode_change = 1;
			}
		}else{
			copter.set_mode((Mode::Number)copter.flight_modes[1].get(), ModeReason::GCS_COMMAND);
			AP_Notify::events.user_mode_change = 1;
		}
		short_press_flag_ch11 = false;
	}

	if(long_press_flag_ch11){
		//Switch to Guided
		if(!copter.set_mode(Mode::Number::GUIDED, ModeReason::GCS_COMMAND)){
			//If mode change fails, use ALT-Hold
			copter.set_mode((Mode::Number)copter.flight_modes[1].get(), ModeReason::GCS_COMMAND);
			AP_Notify::events.user_mode_change_failed = 1;
		}else{
			//Flash LED if mode change works
			AP_Notify::events.user_mode_change = 1;
		}
		long_press_flag_ch11 = false;
	}
}

void Copter::update_rpm_hover(){
	if (!motors->armed()) {
		return;
	}
	//don't update during these states
	if(spirit_state == on_ground or spirit_state == takeoff or spirit_state == spoolup or spirit_state == landing){
		return;
	}
	//Following condition must be met
	//TO DO: timeout after climb or decent to not include low (or high) RPM while settling

	Vector3f accel_ef = ahrs.get_accel_ef();
	accel_ef.z += GRAVITY_MSS; // should equal zero

	if(fabsf(accel_ef.z) > 0.5f){
		rpm_update_counter = 0;
		return;
	}
	if(!is_zero(pos_control->get_vel_desired_cms().z)){
		rpm_update_counter = 0;
		return;
	}
	if(fabsf(inertial_nav.get_velocity_z_up_cms()) > 50){
		rpm_update_counter = 0;
		return;
	}
	if(labs(ahrs.pitch_sensor) > 500 or labs(ahrs.roll_sensor) > 500){
		rpm_update_counter = 0;
		return;
	}
	if(position_ok()){
		if(inertial_nav.get_speed_xy_cms() > 300.0){
			rpm_update_counter = 0;
			return;
		}
	}

	if(rpm_update_counter < 100){
		rpm_update_counter++;
		return;
	}

	float hover_rpm_measurement;

	if(rpm_sensor.get_rpm(1,hover_rpm_measurement)){
		hover_rpm_filter.apply(hover_rpm_measurement);
		if(hover_rpm_filter.get() > 2400.0 and hover_rpm_filter.get() < 4100.0){   //no vehicles hover at this low an RPM
			motors->set_hover_RPM(hover_rpm_filter.get());
		}

	}else if(rpm_sensor.get_rpm(0,hover_rpm_measurement)){
		hover_rpm_filter.apply(hover_rpm_measurement);
		if(hover_rpm_filter.get() > 2400.0 and hover_rpm_filter.get() < 4100.0){    //no vehicles hover at this low an RPM
			motors->set_hover_RPM(hover_rpm_filter.get());
		}
	}
}

void Copter::auto_config(){

	float ang_rate;
	float _rpm_hover;

	//all weights in grams
	float vehicle_core_wt_gr = 1705;
	float battery_wt_gr = 1401;
	float gps_lid_wt_gt = 145;

	vehicle_weight = vehicle_core_wt_gr + gps_lid_wt_gt;

	if(g2.ascent_parameters.get_battery_number() == 1){
		vehicle_weight += battery_wt_gr;

	}else if(g2.ascent_parameters.get_battery_number() == 2){
		vehicle_weight += (2* battery_wt_gr);

	}else{
		return;
	}

	if(g2.ascent_parameters.get_camera_type() == 0){

		//No camera attached.  Mass is average of BLG and viewport
		vehicle_weight += 470;

	}else if(g2.ascent_parameters.get_camera_type() == 1){

		//Q10F attached
		vehicle_weight += (470 + 370);

	}else if(g2.ascent_parameters.get_camera_type() == 2){

		//Q10T attached
		vehicle_weight += (470 + 492);

	}else if(g2.ascent_parameters.get_camera_type() == 3 || g2.ascent_parameters.get_camera_type() == 7){

		//Z10TIR-35 attached
		vehicle_weight += (470 + 606);

	}else if(g2.ascent_parameters.get_camera_type() == 4){

		//Z40K attached
		vehicle_weight += (470 + 507);

	}else if(g2.ascent_parameters.get_camera_type() == 5){

		//Z40TIR attached
		vehicle_weight += (470 + 911);

	}else if(g2.ascent_parameters.get_camera_type() == 6){

		//H30T attached
		vehicle_weight += (470 + 980);

	}else if(g2.ascent_parameters.get_camera_type() == 8){
		//NightHawk (With avg of trip 2 and 5)
		vehicle_weight += 848;

	}else if(g2.ascent_parameters.get_camera_type() == 9){
		//DragonEye (With avg of trip 2 and 5)
		vehicle_weight += 634;
	}
	else if(g2.ascent_parameters.get_camera_type() == 10){
		//RAPTOR (With avg of trip 2 and 5)
		vehicle_weight += 1084;
	}
	else if(g2.ascent_parameters.get_camera_type() == 11){
		//A6600 PAYLOAD....Vehicle has not and should not be flown with two batteries
		vehicle_weight += 1728;
	}
	else if(g2.ascent_parameters.get_camera_type() == 12){
		//ILX-LR1 Payload... Can be flown with 2 batteries
		vehicle_weight += 1278;
	}
	else if(g2.ascent_parameters.get_camera_type() == 13){
		//Gremsy Vio Payload
		vehicle_weight += 1242;
	}
	//convert to pounds before adding payload weight in pounds
	vehicle_weight = vehicle_weight/452;

	vehicle_weight += g2.ascent_parameters.get_payload_weight();

	vehicle_weight = constrain_float(vehicle_weight, 7.0f, 14.50f);

	g2.ascent_parameters.set_total_vehicle_wt(vehicle_weight);

	attitude_control->get_rate_pitch_pid().imax(0.6);
	attitude_control->get_rate_roll_pid().imax(0.6);


	if(g.radio_tuning != TUNING_STABILIZE_ROLL_PITCH_KP){

		float ang_kp;
		ang_kp = g2.ascent_parameters.get_ang_kp_intercept() +
		         g2.ascent_parameters.get_ang_kp_slope()*vehicle_weight;
		ang_kp = constrain_float(ang_kp, 
		                         g2.ascent_parameters.get_ang_kp_constraint_lo(),
								 g2.ascent_parameters.get_ang_kp_constraint_hi());

		attitude_control->set_angle_kp(ang_kp);
	}

	if(g.radio_tuning != TUNING_RATE_ROLL_PITCH_KP){

		float rate_kp;
		rate_kp = g2.ascent_parameters.get_rate_kp_intercept() +
		          g2.ascent_parameters.get_rate_kp_slope()*vehicle_weight;
		rate_kp = constrain_float(rate_kp, 
		                         g2.ascent_parameters.get_rate_kp_constraint_lo(),
								 g2.ascent_parameters.get_rate_kp_constraint_hi());

		if(g.radio_tuning != TUNING_RATE_PITCH_KP){
			attitude_control->get_rate_pitch_pid().kP(rate_kp);
		}

		if(g.radio_tuning != TUNING_RATE_ROLL_KP){
			attitude_control->get_rate_roll_pid().kP(rate_kp);
		}
	}

	if(g.radio_tuning != TUNING_RATE_ROLL_PITCH_KI){

		float rate_ki;
		rate_ki = g2.ascent_parameters.get_rate_ki_intercept() +
		          g2.ascent_parameters.get_rate_ki_slope()*vehicle_weight;
		rate_ki = constrain_float(rate_ki, 
		                          g2.ascent_parameters.get_rate_ki_constraint_lo(),
								  g2.ascent_parameters.get_rate_ki_constraint_hi());

		if(g.radio_tuning != TUNING_RATE_PITCH_KI){
			attitude_control->get_rate_pitch_pid().kI(rate_ki);
		}

		if(g.radio_tuning != TUNING_RATE_ROLL_KI){
			attitude_control->get_rate_roll_pid().kI(rate_ki);
		}
	}

	if(g.radio_tuning != TUNING_RATE_ROLL_PITCH_KD){

		float rate_kd;
		rate_kd = g2.ascent_parameters.get_rate_kd_intercept() +
		          g2.ascent_parameters.get_rate_kd_slope()*vehicle_weight;
		rate_kd = constrain_float(rate_kd, 
		                          g2.ascent_parameters.get_rate_kd_constraint_lo(),
								  g2.ascent_parameters.get_rate_kd_constraint_hi());

		if(g.radio_tuning != TUNING_RATE_PITCH_KD){
			attitude_control->get_rate_pitch_pid().kD(rate_kd);
		}

		if(g.radio_tuning != TUNING_RATE_ROLL_KD){
			attitude_control->get_rate_roll_pid().kD(rate_kd);
		}
	}

	if(g2.ascent_parameters.get_tube_launch()) {
		//set to legless values
		if(g.radio_tuning != TUNING_YAW_RATE_KP) {
			attitude_control->get_rate_yaw_pid().kP((float)g2.ascent_parameters.get_yaw_p_ll());
		}
		attitude_control->get_rate_yaw_pid().kI((float)g2.ascent_parameters.get_yaw_i_ll()); //You cannot radio tune the I gain it seems so no need to check
	}
	else {
		//set to ascent yaw params
		if(g.radio_tuning != TUNING_YAW_RATE_KP) {
			attitude_control->get_rate_yaw_pid().kP((float)g2.ascent_parameters.get_yaw_p());
		}
		attitude_control->get_rate_yaw_pid().kI((float)g2.ascent_parameters.get_yaw_i()); //You cannot radio tune the I gain it seems so no need to check
	}

	ang_rate = (-9000*vehicle_weight)+180000;
	ang_rate = constrain_float(ang_rate, 58500, 108000);

	attitude_control->set_accel_roll_max_cdss(ang_rate);
	attitude_control->set_accel_pitch_max_cdss(ang_rate);

	_rpm_hover = g2.ascent_parameters.get_rpmh_slope()*vehicle_weight + 
	             g2.ascent_parameters.get_rpmh_intercept();
	_rpm_hover = constrain_float(_rpm_hover, 
							  g2.ascent_parameters.get_rpmh_constraint_lo(),
							  g2.ascent_parameters.get_rpmh_constraint_hi());
	g2.ascent_parameters.set_rpm_hover(_rpm_hover);

	hover_rpm_filter.reset(g2.ascent_parameters.get_rpm_hover());
	motors->set_hover_RPM(g2.ascent_parameters.get_rpm_hover());

	int32_t total_pack_capacity_mah = g2.ascent_parameters.get_battery_number() * g2.ascent_parameters.get_single_pack_batt_cap_mah();
	battery.set_pack_capacity_mah(total_pack_capacity_mah);
	if(!g2.ascent_parameters.get_auto_mot_offs()){
		compass.set_motor_compensation(0, initial_motor_offsets); //In case you ran an auto calc, landed, then want to fly without autocalc on same boot
		if (g2.ascent_parameters.get_battery_number() ==  1){
			compass.set_spirit_mot_correction_factor(2.0f);
		} else if (g2.ascent_parameters.get_battery_number() ==  2){
			compass.set_spirit_mot_correction_factor(1.0f);
		}
	}
}

void Copter::handle_agc_message(const mavlink_command_int_t &packet){
	int cmd = packet.param1;
	switch(cmd){
		case 0:
			camera_mount.payload_take_picture();
			break;
		case 1:
			camera_mount.start_record();
			break;
		case 2:
			camera_mount.stop_record();
			break;
		case 3:
			camera_mount.eo_full_screen();
			break;
		case 4:
			camera_mount.ir_full_screen();
			break;
		case 5:
			camera_mount.eo_ir();
			break;
		case 6:
			camera_mount.ir_eo();
			break;
		case 7:
			camera_mount.next_color_pallette();
			break;
		case 8:
			camera_mount.digital_zoom_1();
			break;
		case 9:
			camera_mount.digital_zoom_2();
			break;
		case 10:
			camera_mount.digital_zoom_4();
			break;
		case 11:
			camera_mount.yaw_follow_enable();
			break;
		case 12:
			camera_mount.yaw_follow_disable();
			break;
		case 13:
			if(is_equal(packet.param2, 1.0f)){
				copter.ap.cam_function_button_pressed = true;
			}
			else{
				copter.ap.cam_function_button_pressed = false;
			}
			break;
		case 14:
			//Zero zoom
			break;
		case 15:
			camera_mount.start_tracking(packet.param2, packet.param3);
			break;
		case 16:
			camera_mount.stop_tracking();
			break;

		case 17:
			centering_camera = true;
			break;

		case 18:
			camera_mount.set_angles(packet.param2, packet.param3);
			break;

		case 19: {
			if(!gcs_initialized) gcs_initialized = true;
			mav_gimbal_pan = packet.param3 / 32768;
			mav_gimbal_tilt = packet.param2 / -32768;
			break;
		}
		
		case 20:
			camera_mount.look_down();
			break;

		case 21:
			camera_mount.do_nuc();
			break;

		case 22:
			camera_mount.toggle_heat();
			break;
		case 23:
			camera_mount.toggle_detection();
			break;
		case 24:
			camera_mount.enable_eo_dzoom();
			break;
		case 25:
			camera_mount.disable_eo_dzoom();
			break;
		case 26:
			camera_mount.set_zoom_rate(packet.param2);
			break;
		case 27:
			if(is_equal(packet.param2, 1.0f)){
				g2.ascent_parameters.set_follow_camera(true);
				camera_mount.yaw_follow_disable();
				gcs().send_text(MAV_SEVERITY_INFO,"ENABLE CAMERA FOLLOW");
			}
			else{
				g2.ascent_parameters.set_follow_camera(false);
				gcs().send_text(MAV_SEVERITY_INFO,"DISABLE CAMERA FOLLOW");
			}
			break;		
		case 28:
			camera_mount.trigger_menu_control((AP_Mount::GREMSY_MENU_CONTROL)packet.param2);
			break;
		case 29:
			camera_mount.trigger_display();
			break;
		case 30:
			camera_mount.trigger_power();
			break;
		case 31:
			camera_mount.trigger_af();
			break;
		case 32:
			camera_mount.trigger_c1();
			break;
		case 33:
			camera_mount.trigger_c2();
			break;
		case 34:
			camera_mount.trigger_c3();
			break;
		case 35:
			camera_mount.trigger_mf_up();
			break;
		case 36:
			camera_mount.trigger_mf_down();
			break;
		case 37:
			camera_mount.trigger_wb_up();
			break;
		case 38:
			camera_mount.trigger_wb_down();
			break;
		case 39:
			camera_mount.trigger_speed_up();
			break;
		case 40:
			camera_mount.trigger_speed_down();
			break;
		case 41:
			camera_mount.trigger_aperture_up();
			break;
		case 42:
			camera_mount.trigger_aperture_down();
			break;
		case 43:
			camera_mount.trigger_iso_up();
			break;
		case 44:
			camera_mount.trigger_iso_down();
			break;
		case 45:
			camera_mount.trigger_expcorr_up();
			break;
		case 46:
			camera_mount.trigger_expcorr_down();
			break;
		case 47:
			camera_mount.trigger_zoom_up();
			break;
		case 48:
			camera_mount.trigger_zoom_down();
			break;
		case 49:
			camera_mount.trigger_focus_up();
			break;
		case 50:
			camera_mount.trigger_focus_down();
			break;
		case 51:
			camera_mount.start_geotagging();
			break;
		case 52:
			camera_mount.stop_geotagging();
			break;
		case 53:
			mavlink_msg_command_long_send(MAVLINK_COMM_2,
											1,
											190,
											MAV_CMD_DO_DIGICAM_CONTROL,
											0,
											packet.param2, //MavExtCmd_SetGroundCrossingAlt
											packet.param3,
											0,
											0,
											0,
											0,
											0);
			break;
		case 54:
			camera_mount.fusion();
			break;
		case 55:
			camera_mount.set_detection(packet.param2);
			break;
		case 56:
			gcs().send_text(MAV_SEVERITY_INFO,"VisOdom MAG align request received");
            if (copter.visual_odom.get_singleton() != nullptr) {
                copter.visual_odom.request_align_yaw_to_ahrs();
				gcs().send_text(MAV_SEVERITY_INFO,"VisOdom MAG align request submitted");
            }
			break;
		case 57:
			camera_mount.set_osd_mode(packet.param2);
			break;
		case 58:
			camera_mount.set_record_source(packet.param2);
			break;
		case 59:
			camera_mount.set_zoom_mode(packet.param2);
			break;
		case 60:
			camera_mount.ir_zoom(packet.param2);
			break;
		case 61:
			camera_mount.request_payload_info();
			break;
		case 99:
			//Set to whatever function for debugging purposes!
			//For example -> camera_mount.stop_tracking();
			break;

		case 100:
			//Set to whatever function for debugging purposes!
			//For example -> camera_mount.yaw_follow_disable();
			break;
	}
}

void Copter::handle_microhard(const mavlink_command_int_t &packet){
	logger.write_mh(packet.param1, packet.param2, packet.param3, packet.param4, packet.x, packet.y);
}

void Copter::payload_power(bool on) {
	switch(g2.ascent_parameters.get_vehicle_type()) {
		case 1: //Spirit
			if(on) {
				hal.gpio->write(52, 0);
			}
			else {
				hal.gpio->write(52, 1);
			}
			break;
		case 2: //Spartan
			if(g2.ascent_parameters.get_camera_type() != 0) { //Assumption is a camera is attached and CiB logic is required
				if(on) {
					hal.gpio->write(52, 0);
				}
				else {
					hal.gpio->write(52, 1);
				}
			}
			else{ //Assumption is no camera is attached and NX breakout board logic is required
				if(on) {
					hal.gpio->write(52, 1);
				}
				else {
					hal.gpio->write(52, 0);
				}
			}
			break;
	}
}
