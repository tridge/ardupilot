#include "Copter.h"


/*
 * Init and run calls for althold, flight mode
 */

// althold_init - initialise althold controller
bool ModeAltHold::init(bool ignore_checks)
{

    // initialise the vertical position controller
    if (!pos_control->is_active_z()) {
        pos_control->init_z_controller();
    }

    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);
    pos_control->set_correction_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    return true;
}

// althold_run - runs the althold controller
// should be called at 100hz or more
void ModeAltHold::run()
{
    // set vertical speed and acceleration limits
    pos_control->set_max_speed_accel_z(-get_pilot_speed_dn(), g.pilot_speed_up, g.pilot_accel_z);

    // apply SIMPLE mode transform to pilot inputs
    update_simple_mode();

    // get pilot desired lean angles
    float target_roll, target_pitch;
    float target_climb_rate, target_yaw_rate;
    get_pilot_desired_lean_angles(target_roll, target_pitch, copter.aparm.angle_max, attitude_control->get_althold_lean_angle_max_cd());
    
    // get pilot's desired yaw rate
    // In follow the camera mode and not using a ViewPro camera
    if(g2.ascent_parameters.get_follow_camera() && copter.camera_mount.get_mount_type() != AP_Mount::Type::Viewpro_AA){
        copter.camera_mount.align_roll_pitch_command_to_camera(target_roll, target_pitch);
        target_yaw_rate = (copter.camera_mount.get_camera_pan() / 180) * 4500 * FTC_YAW_GAIN;
        if(copter.ap.cam_function_button_pressed){target_climb_rate = 0.0f;}
        else{
            // get pilot desired climb rate (for alt-hold mode and take-off)
            target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
        }
    }

    else if(g2.ascent_parameters.get_follow_camera() && copter.camera_mount.get_mount_type() == AP_Mount::Type::Viewpro_AA) {
        //scale to zoom
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz())/copter.camera_mount.get_zoom_level();
        //If cam button pressed, climb rate should remain 0 and tilt channel is used in camera driver
        if(!copter.ap.cam_function_button_pressed){
            // get pilot desired climb rate
            target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
            target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
        }
        else{
            target_climb_rate = 0.0f;
        }
    }

    // get pilot's desired yaw rate

    //Not in follow the camera mode, but cam button is pressed, so the left stick (left and right movement) is used for camera pan
    else if(copter.ap.cam_function_button_pressed){
        target_yaw_rate = 0.0f;
        target_climb_rate = 0.0f;
    }

    //Regular behavior. Cam Button not pressed, not in FTC mode
    else{
		//get pilot's desired yaw rate
        target_yaw_rate = get_pilot_desired_yaw_rate(channel_yaw->norm_input_dz());

        // get pilot desired climb rate
        target_climb_rate = get_pilot_desired_climb_rate(channel_throttle->get_control_in());
        target_climb_rate = constrain_float(target_climb_rate, -get_pilot_speed_dn(), g.pilot_speed_up);
    }

    // Alt Hold State Machine Determination
    AltHoldModeState althold_state = get_alt_hold_state(target_climb_rate);

    // Alt Hold State Machine
    switch (althold_state) {

    case AltHold_MotorStopped:
        attitude_control->reset_rate_controller_I_terms();
        attitude_control->reset_yaw_target_and_rate(false);
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Landed_Ground_Idle:
        attitude_control->reset_yaw_target_and_rate();
        FALLTHROUGH;

    case AltHold_Landed_Pre_Takeoff:
        attitude_control->reset_rate_controller_I_terms_smoothly();
        pos_control->relax_z_controller(0.0f);   // forces throttle output to decay to zero
        break;

    case AltHold_Takeoff:
        // initiate take-off
        if (!takeoff.running()) {
            takeoff.start(constrain_float(g.pilot_takeoff_alt,0.0f,1000.0f));
        }

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // set position controller targets adjusted for pilot input
        takeoff.do_pilot_takeoff(target_climb_rate);
        break;

    case AltHold_Flying:
        motors->set_desired_spool_state(AP_Motors::DesiredSpoolState::THROTTLE_UNLIMITED);

#if AC_AVOID_ENABLED == ENABLED
        // apply avoidance
        copter.avoid.adjust_roll_pitch(target_roll, target_pitch, copter.aparm.angle_max);
#endif

        // get avoidance adjusted climb rate
        target_climb_rate = get_avoidance_adjusted_climbrate(target_climb_rate);

        // update the vertical offset based on the surface measurement
        copter.surface_tracking.update_surface_offset();

        // Send the commanded climb rate to the position controller
        pos_control->set_pos_target_z_from_climb_rate_cm(target_climb_rate);
        break;
    }

    // call attitude controller
    attitude_control->input_euler_angle_roll_pitch_euler_rate_yaw(target_roll, target_pitch, target_yaw_rate);

    // run the vertical position controller and set output throttle
    pos_control->update_z_controller();
}