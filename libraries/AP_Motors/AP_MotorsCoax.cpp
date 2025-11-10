/*
   This program is free software: you can redistribute it and/or modify
   it under the terms of the GNU General Public License as published by
   the Free Software Foundation, either version 3 of the License, or
   (at your option) any later version.

   This program is distributed in the hope that it will be useful,
   but WITHOUT ANY WARRANTY; without even the implied warranty of
   MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
   GNU General Public License for more details.

   You should have received a copy of the GNU General Public License
   along with this program.  If not, see <http://www.gnu.org/licenses/>.
 */

#include <AP_HAL/AP_HAL.h>
#include <AP_Math/AP_Math.h>
#include "AP_MotorsCoax.h"
#include <GCS_MAVLink/GCS.h>
#include <SRV_Channel/SRV_Channel.h>

extern const AP_HAL::HAL& hal;

// init
void AP_MotorsCoax::init(motor_frame_class frame_class, motor_frame_type frame_type)
{
    // make sure 4 output channels are mapped
    for (uint8_t i = 0; i < NUM_ACTUATORS_COAX; i++) {
        add_motor_num(CH_1 + i);
    }

    // set the motor_enabled flag so that the main ESC can be calibrated like other frame types
    motor_enabled[AP_MOTORS_MOT_3] = true;
    motor_enabled[AP_MOTORS_MOT_4] = true;

    // setup actuator scaling
    for (uint8_t i = 0; i < NUM_SERVO_COAX; i++) {
        SRV_Channels::set_angle(SRV_Channels::get_motor_function(i), AP_MOTORS_COAX_SERVO_INPUT_RANGE);
    }

    _mav_type = MAV_TYPE_COAXIAL;

    // record successful initialisation if what we setup was the desired frame_class
    set_initialised_ok(frame_class == MOTOR_FRAME_COAX);
    _remain_stabilized = false;
	_stabilized_counter = 0;

	_spoolup_complete = false;
	_enable_rpm_compensation = false;
}

// set frame class (i.e. quad, hexa, heli) and type (i.e. x, plus)
void AP_MotorsCoax::set_frame_class_and_type(motor_frame_class frame_class, motor_frame_type frame_type)
{
    set_initialised_ok(frame_class == MOTOR_FRAME_COAX);
}

// set update rate to motors - a value in hertz
void AP_MotorsCoax::set_update_rate(uint16_t speed_hz)
{
    // record requested speed
    _speed_hz = speed_hz;

    uint32_t mask =
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_4 ;
    rc_set_freq(mask, _speed_hz);
}

void AP_MotorsCoax::output_to_motors()
{
    const float loop_rate_hz = 1.0f/constrain_float(_dt, 0.0001f, 10.0f); 
    switch (_spool_state) {
        case SpoolState::SHUT_DOWN:
            if(_remain_stabilized){
                for (uint8_t i = 0; i < NUM_SERVO_COAX; i++) {
                    rc_write_angle(AP_MOTORS_MOT_1 + i, _actuator_out[i] * AP_MOTORS_COAX_SERVO_INPUT_RANGE);
                }
                _stabilized_counter++;

                if(_stabilized_counter > 400){
                	_remain_stabilized = false;
                	_stabilized_counter = 0;
                }

        	}else{
                rc_write_angle(AP_MOTORS_MOT_1, (cosf(_roll_pitch_lag_rad)*_pitch_radio_passthrough + sinf(_roll_pitch_lag_rad)*_roll_radio_passthrough) * AP_MOTORS_COAX_SERVO_INPUT_RANGE);
                rc_write_angle(AP_MOTORS_MOT_2, (-sinf(_roll_pitch_lag_rad)*_pitch_radio_passthrough + cosf(_roll_pitch_lag_rad)*_roll_radio_passthrough) * AP_MOTORS_COAX_SERVO_INPUT_RANGE);
        	}

        	_actuator[2] = 0;
			_actuator[3] = 0;
            // sends minimum values out to the motors
            rc_write(AP_MOTORS_MOT_3, output_to_pwm(0));
            rc_write(AP_MOTORS_MOT_4, output_to_pwm(0));

            _fwd_thrust_out = 0;
			_aft_thrust_out = 0;
            _enable_aft_rotor = false;
            _spoolup_complete = false;
            _aft_rotor_start = 0;
            break;
        case SpoolState::GROUND_IDLE:
            // Protect against edge cases. If aft rotor has been enabled, do not run servo tests
            if (!_enable_aft_rotor && _servo_test_mode == 1){
                rc_write(AP_MOTORS_MOT_1,1147);
                rc_write(AP_MOTORS_MOT_2,1147);
            } else if (!_enable_aft_rotor && _servo_test_mode == 2) {
                rc_write(AP_MOTORS_MOT_1,1853);
                rc_write(AP_MOTORS_MOT_2,1853);
            } else if (!_enable_aft_rotor && _servo_test_mode == 3) {
                rc_write(AP_MOTORS_MOT_1,1500);
                rc_write(AP_MOTORS_MOT_2,1500);
            } else {
                for (uint8_t i = 0; i < NUM_SERVO_COAX; i++) {
                    rc_write_angle(AP_MOTORS_MOT_1 + i, _actuator_out[i] * AP_MOTORS_COAX_SERVO_INPUT_RANGE);
                }
            }

            // always start fwd rotor
            set_actuator_with_slew(_actuator[3], actuator_spin_up_to_ground_idle());
			rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[3]));
			//handle aft rotor spoolup
            if(_spoolup_complete){
                if(_auto_motor_offsets){set_actuator_with_slew(_actuator[2], AP_MOTORS_MODIFIED_ARM_SPIN);}
                else{set_actuator_with_slew(_actuator[2], _spin_arm);}
                _aft_thrust_out = _actuator[2];
                rc_write(AP_MOTORS_MOT_3,  output_to_pwm(_actuator[2]));
            }else if(_enable_aft_rotor){
                if(_spin_arm <= 0.05){
                    _aft_rotor_start += (1.0f/(_spool_up_time * loop_rate_hz)) * thr_lin.get_spin_min();
                    _aft_rotor_start = constrain_float(_aft_rotor_start, 0.0f, thr_lin.get_spin_min());
                }else{
                    if(_auto_motor_offsets){
                        _aft_rotor_start += (1.0f/(_spool_up_time * loop_rate_hz)) * AP_MOTORS_MODIFIED_ARM_SPIN;
                        _aft_rotor_start = constrain_float(_aft_rotor_start, 0.0f, AP_MOTORS_MODIFIED_ARM_SPIN);
                    }
                    else{
                        _aft_rotor_start += (1.0f/(_spool_up_time * loop_rate_hz)) * _spin_arm;
                        _aft_rotor_start = constrain_float(_aft_rotor_start, 0.0f, _spin_arm);
                    }
                }
                _actuator[2] = _aft_rotor_start;
                rc_write(AP_MOTORS_MOT_3,  (int16_t)((float)get_pwm_output_min() + _aft_rotor_start * (float)(get_pwm_output_max()-get_pwm_output_min())) );
            }else{
                _actuator[2] = 0;
                rc_write(AP_MOTORS_MOT_3, output_to_pwm(0));
                _aft_rotor_start = 0;
                _spoolup_complete = false;
                _enable_aft_rotor = false;
            }
            _aft_thrust_out = _actuator[2];
		    _fwd_thrust_out = _actuator[3];
            break;
        case SpoolState::SPOOLING_UP:
        case SpoolState::THROTTLE_UNLIMITED:
        case SpoolState::SPOOLING_DOWN:
            // Protect against edge cases. If aft rotor has been enabled, do not run servo tests
            if (!_enable_aft_rotor && _servo_test_mode == 1){
                rc_write(AP_MOTORS_MOT_1,1147);
                rc_write(AP_MOTORS_MOT_2,1147);
            } else if (!_enable_aft_rotor && _servo_test_mode == 2) {
                rc_write(AP_MOTORS_MOT_1,1853);
                rc_write(AP_MOTORS_MOT_2,1853);
            } else if (!_enable_aft_rotor && _servo_test_mode == 3) {
                rc_write(AP_MOTORS_MOT_1,1500);
                rc_write(AP_MOTORS_MOT_2,1500);
            } else {
                for (uint8_t i = 0; i < NUM_SERVO_COAX; i++) {
                    rc_write_angle(AP_MOTORS_MOT_1 + i, _actuator_out[i] * AP_MOTORS_COAX_SERVO_INPUT_RANGE);
                }
            }

            //handle aft rotor spoolup
            if(_spoolup_complete){

                _remain_stabilized = true;

                set_actuator_with_slew(_actuator[2], thr_lin.thrust_to_actuator(_thrust_yt_cw));
                set_actuator_with_slew(_actuator[3], thr_lin.thrust_to_actuator(_thrust_yt_ccw));

                rc_write(AP_MOTORS_MOT_3, output_to_pwm(_actuator[2]));
                rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[3]));

            }else if(_enable_aft_rotor){
                
                if(_spin_arm <= 0.05){
                    _aft_rotor_start += (1.0f/(_spool_up_time * loop_rate_hz)) * thr_lin.get_spin_min();
                    _aft_rotor_start = constrain_float(_aft_rotor_start, 0.0f, thr_lin.get_spin_min());
                }else{
                    if(_auto_motor_offsets){
                        _aft_rotor_start += (1.0f/(_spool_up_time * loop_rate_hz)) * AP_MOTORS_MODIFIED_ARM_SPIN;
                        _aft_rotor_start = constrain_float(_aft_rotor_start, 0.0f, AP_MOTORS_MODIFIED_ARM_SPIN);
                    }
                    else{
                        _aft_rotor_start += (1.0f/(_spool_up_time * loop_rate_hz)) * _spin_arm;
                        _aft_rotor_start = constrain_float(_aft_rotor_start, 0.0f, _spin_arm);
                    }
                }

                _actuator[2] = _aft_rotor_start;
                if(_auto_motor_offsets){ _actuator[3] = AP_MOTORS_MODIFIED_ARM_SPIN; }
                else{ _actuator[3] = _spin_arm; }

                rc_write(AP_MOTORS_MOT_3,  output_to_pwm(_actuator[2]));
                rc_write(AP_MOTORS_MOT_4, output_to_pwm(_actuator[3]));

            }else{

                _actuator[2] = 0;

                rc_write(AP_MOTORS_MOT_3, output_to_pwm(0));
                    _aft_rotor_start = 0;
                    _spoolup_complete = false;
                    _enable_aft_rotor = false;
            }
            _aft_thrust_out = _actuator[2];
            _fwd_thrust_out = _actuator[3];
            break;
    }
}

// get_motor_mask - returns a bitmask of which outputs are being used for motors or servos (1 means being used)
//  this can be used to ensure other pwm outputs (i.e. for servos) do not conflict
uint32_t AP_MotorsCoax::get_motor_mask()
{
    uint32_t motor_mask =
        1U << AP_MOTORS_MOT_1 |
        1U << AP_MOTORS_MOT_2 |
        1U << AP_MOTORS_MOT_3 |
        1U << AP_MOTORS_MOT_4;
    uint16_t mask = motor_mask_to_srv_channel_mask(motor_mask);

    // add parent's mask
    mask |= AP_MotorsMulticopter::get_motor_mask();

    return mask;
}

// sends commands to the motors
void AP_MotorsCoax::output_armed_stabilizing()
{
    float   roll_thrust;                // roll thrust input value, +/- 1.0
    float   pitch_thrust;               // pitch thrust input value, +/- 1.0
    float   yaw_thrust;                 // yaw thrust input value, +/- 1.0
    float   throttle_thrust;            // throttle thrust input value, 0.0 - 1.0

    // apply voltage and air pressure compensation
    //  const float compensation_gain = get_compensation_gain();
    roll_thrust = (_roll_in + _roll_in_ff);// * compensation_gain;
    pitch_thrust = (_pitch_in + _pitch_in_ff);// * compensation_gain;
    yaw_thrust = (_yaw_in + _yaw_in_ff);// * compensation_gain;
    throttle_thrust = get_throttle();

    //-------------RPM Scaling----------------//

	////sanity check RPM comp parameters, if set wrong -> disable RPM Comp ///////
    if(_rpm_scale_factor > 0.6f or _rpm_scale_factor < 0.0f){
        _enable_rpm_compensation = 0;
    }

    if(_rpm_deadband > 750 or _rpm_deadband < 0 ){
        _enable_rpm_compensation = 0;
    }

    if(_enable_rpm_comp and _enable_rpm_compensation){
        if(_aft_rotor_RPM > 1500.0f and _aft_rotor_RPM < 5500.0f){

            float rpm_delta = _aft_rotor_RPM - _rpm_hover;

            rpm_delta = constrain_float(rpm_delta, -750.0f, 750.0f);

            float half_deadband = _rpm_deadband / 2.0f;

            if(rpm_delta < -half_deadband ){   /// rpm is below average
                _servo_rpm_comp = ((rpm_delta + half_deadband)/(half_deadband - 750.0f))*_rpm_scale_factor;

                pitch_thrust += pitch_thrust*_servo_rpm_comp;
                roll_thrust += roll_thrust*_servo_rpm_comp;

            }else if(rpm_delta > half_deadband){       //// rpm is above average
                _servo_rpm_comp = ((rpm_delta - (half_deadband))/(half_deadband - 750.0f))*_rpm_scale_factor;

                pitch_thrust += pitch_thrust*_servo_rpm_comp;
                roll_thrust +=  roll_thrust*_servo_rpm_comp;

            }else{
                _servo_rpm_comp = 0;
            }
        }else{
            _servo_rpm_comp = -1;
        }
    }else{
        _servo_rpm_comp = -2;
    }
    //-------------RPM Scaling----------------//

    // sanity check throttle is above zero and below current limited throttle
    if (throttle_thrust <= 0.0f) {
        throttle_thrust = 0.0f;
        limit.throttle_lower = true;
    }
    if (throttle_thrust >= _throttle_thrust_max) {
        throttle_thrust = _throttle_thrust_max;
        limit.throttle_upper = true;
    }

    //normalize servo input
    float total_out = norm(pitch_thrust, roll_thrust);

    //if servos are saturated scale input and give throttle boost
   	if (total_out > 1.0f) {
   		float ratio = 1.0f / total_out;
   		pitch_thrust *= ratio;
   		roll_thrust *= ratio;
        limit.roll = true;
        limit.pitch = true;
    }

    _actuator_out[0] =  cosf(_roll_pitch_lag_rad)*pitch_thrust + sinf(_roll_pitch_lag_rad)*roll_thrust;
	_actuator_out[1] = -sinf(_roll_pitch_lag_rad)*pitch_thrust + cosf(_roll_pitch_lag_rad)*roll_thrust;

    if (fabsf(_actuator_out[0]) > 1.0f) {
		//limit.roll_pitch = true;  Handled above
		_actuator_out[0] = constrain_float(_actuator_out[0], -1.0f, 1.0f);
 	}

    if (fabsf(_actuator_out[1]) > 1.0f) {
	    //  limit.roll_pitch = true;   Handled above
	    _actuator_out[1] = constrain_float(_actuator_out[1], -1.0f, 1.0f);
 	}

    //compute headroom for yaw
    float yaw_headroom_available_max =  _throttle_thrust_max - (throttle_thrust + (0.5f *fabsf(yaw_thrust)));   /// goes negative if you don't have enough headroom
    float yaw_headroom_available_min =  throttle_thrust - (0.5f *fabsf(yaw_thrust));			//// goes negative if you don't have enough headroom

    //handle running out of headroom
    if( yaw_headroom_available_max  >  0.0f   and   yaw_headroom_available_min  >  0.0f){
        _thrust_yt_ccw = throttle_thrust + (0.5f * yaw_thrust);
        _thrust_yt_cw = throttle_thrust - (0.5f * yaw_thrust);
    }else if(yaw_headroom_available_max  <=  0.0f){
        limit.yaw = true;
        limit.throttle_upper = true;
        _thrust_yt_ccw = (throttle_thrust + (0.5f * yaw_thrust)) + yaw_headroom_available_max;
        _thrust_yt_cw = (throttle_thrust - (0.5f * yaw_thrust)) + yaw_headroom_available_max;
    }else{
        limit.yaw = true;
        limit.throttle_lower = true;
        _thrust_yt_ccw = (throttle_thrust + (0.5f * yaw_thrust)) - yaw_headroom_available_min;
        _thrust_yt_cw = (throttle_thrust - (0.5f * yaw_thrust)) - yaw_headroom_available_min;
    }
}

// output_test_seq - spin a motor at the pwm value specified
//  motor_seq is the motor's sequence number from 1 to the number of motors on the frame
//  pwm value is an actual pwm value that will be output, normally in the range of 1000 ~ 2000
void AP_MotorsCoax::_output_test_seq(uint8_t motor_seq, int16_t pwm)
{
    // output to motors and servos
    switch (motor_seq) {
        case 1:
            // flap servo 1
            rc_write(AP_MOTORS_MOT_1, pwm);
            break;
        case 2:
            // flap servo 2
            rc_write(AP_MOTORS_MOT_2, pwm);
            break;
        case 3:
            // flap servo 3
            rc_write(AP_MOTORS_MOT_3, pwm);
            break;
        case 4:
            // flap servo 4
            rc_write(AP_MOTORS_MOT_4, pwm);
            break;
        default:
            // do nothing
            break;
    }
}
