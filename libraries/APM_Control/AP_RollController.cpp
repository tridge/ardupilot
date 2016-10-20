// -*- tab-width: 4; Mode: C++; c-basic-offset: 4; indent-tabs-mode: nil -*-
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

//	Code by Jon Challinger
//  Modified by Paul Riseborough
//

#include <AP_HAL/AP_HAL.h>
#include "AP_RollController.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_RollController::var_info[] = {
	// @Param: TCONST
	// @DisplayName: Roll Time Constant
	// @Description: This controls the time constant in seconds from demanded to achieved bank angle. A value of 0.5 is a good default and will work with nearly all models. Advanced users may want to reduce this time to obtain a faster response but there is no point setting a time less than the aircraft can achieve.
	// @Range: 0.4 1.0
	// @Units: seconds
	// @Increment: 0.1
	// @User: Advanced
	AP_GROUPINFO("TCONST",      0, AP_RollController, gains.tau,       0.5f),

	// @Param: P
	// @DisplayName: Proportional Gain
	// @Description: This is the gain from bank angle error to aileron.
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("P",        1, AP_RollController, gains.P,        0.6f),

	// @Param: D
	// @DisplayName: Damping Gain
	// @Description: This is the gain from roll rate to aileron. This adjusts the damping of the roll control loop. It has the same effect as RLL2SRV_D in the old PID controller but without the spikes in servo demands. This gain helps to reduce rolling in turbulence. It should be increased in 0.01 increments as too high a value can lead to a high frequency roll oscillation that could overstress the airframe.
	// @Range: 0 0.1
	// @Increment: 0.01
	// @User: User
	AP_GROUPINFO("D",        2, AP_RollController, gains.D,        0.02f),

	// @Param: I
	// @DisplayName: Integrator Gain
	// @Description: This is the gain from the integral of bank angle to aileron. It has the same effect as RLL2SRV_I in the old PID controller. Increasing this gain causes the controller to trim out steady offsets due to an out of trim aircraft.
	// @Range: 0 1.0
	// @Increment: 0.05
	// @User: User
	AP_GROUPINFO("I",        3, AP_RollController, gains.I,        0.1f),

	// @Param: RMAX
	// @DisplayName: Maximum Roll Rate
	// @Description: This sets the maximum roll rate that the controller will demand (degrees/sec). Setting it to zero disables the limit. If this value is set too low, then the roll can't keep up with the navigation demands and the plane will start weaving. If it is set too high (or disabled by setting to zero) then ailerons will get large inputs at the start of turns. A limit of 60 degrees/sec is a good default.
	// @Range: 0 180
	// @Units: degrees/second
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("RMAX",   4, AP_RollController, gains.rmax,       0),

	// @Param: IMAX
	// @DisplayName: Integrator limit
	// @Description: This limits the number of degrees of aileron in centi-degrees over which the integrator will operate. At the default setting of 3000 centi-degrees, the integrator will be limited to +- 30 degrees of servo travel. The maximum servo deflection is +- 45 centi-degrees, so the default value represents a 2/3rd of the total control throw which is adequate unless the aircraft is severely out of trim.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
	AP_GROUPINFO("IMAX",      5, AP_RollController, gains.imax,        3000),

	// @Param: FF
	// @DisplayName: Feed forward Gain
	// @Description: This is the gain from demanded rate to aileron output. 
	// @Range: 0.1 4.0
	// @Increment: 0.1
	// @User: User
	AP_GROUPINFO("FF",        6, AP_RollController, gains.FF,          0.0f),

    // adaptive control parameters
    AP_GROUPINFO_FLAGS("AD_CH", 7, AP_RollController, adap.enable_chan, 0, AP_PARAM_FLAG_ENABLE),
    AP_GROUPINFO("ALPHA", 8, AP_RollController, adap.alpha, 20),
    AP_GROUPINFO("GAMMA", 9, AP_RollController, adap.gamma, 0.03),
    AP_GROUPINFO("W0",    10, AP_RollController, adap.W0, 100),
    AP_GROUPINFO("K1UP",  11, AP_RollController, adap.K1_upper_limit, 1),
    AP_GROUPINFO("K1LOW", 12, AP_RollController, adap.K1_lower_limit, -1),
    AP_GROUPINFO("DBAND", 13, AP_RollController, adap.deadband, 0.25),
    AP_GROUPINFO("K2",    14, AP_RollController, adap.K2, 0.42),
    
	AP_GROUPEND
};


/*
  internal rate controller, called by attitude and rate controller
  public functions
*/
int32_t AP_RollController::_get_rate_out(float desired_rate, float scaler, bool disable_integrator)
{
	uint32_t tnow = AP_HAL::millis();
	uint32_t dt = tnow - _last_t;
	if (_last_t == 0 || dt > 1000) {
		dt = 0;
	}
	_last_t = tnow;
	
	// Calculate equivalent gains so that values for K_P and K_I can be taken across from the old PID law
    // No conversion is required for K_D
	float ki_rate = gains.I * gains.tau;
    float eas2tas = _ahrs.get_EAS2TAS();
	float kp_ff = MAX((gains.P - gains.I * gains.tau) * gains.tau  - gains.D , 0) / eas2tas;
    float k_ff = gains.FF / eas2tas;
	float delta_time    = (float)dt * 0.001f;
	
	// Limit the demanded roll rate
	if (gains.rmax && desired_rate < -gains.rmax) {
        desired_rate = - gains.rmax;
    } else if (gains.rmax && desired_rate > gains.rmax) {
        desired_rate = gains.rmax;
    }
	
    // Get body rate vector (radians/sec)
	float omega_x = _ahrs.get_gyro().x;
	
	// Calculate the roll rate error (deg/sec) and apply gain scaler
    float achieved_rate = ToDeg(omega_x);
	float rate_error = (desired_rate - achieved_rate) * scaler;
	
	// Get an airspeed estimate - default to zero if none available
	float aspeed;
	if (!_ahrs.airspeed_estimate(&aspeed)) {
        aspeed = 0.0f;
    }

	// Multiply roll rate error by _ki_rate, apply scaler and integrate
	// Scaler is applied before integrator so that integrator state relates directly to aileron deflection
	// This means aileron trim offset doesn't change as the value of scaler changes with airspeed
	// Don't integrate if in stabilise mode as the integrator will wind up against the pilots inputs
	if (!disable_integrator && ki_rate > 0) {
		//only integrate if gain and time step are positive and airspeed above min value.
		if (dt > 0 && aspeed > float(aparm.airspeed_min)) {
		    float integrator_delta = rate_error * ki_rate * delta_time * scaler;
			// prevent the integrator from increasing if surface defln demand is above the upper limit
			if (_last_out < -45) {
                integrator_delta = MAX(integrator_delta , 0);
            } else if (_last_out > 45) {
                // prevent the integrator from decreasing if surface defln demand  is below the lower limit
                 integrator_delta = MIN(integrator_delta, 0);
            }
			_pid_info.I += integrator_delta;
		}
	} else {
		_pid_info.I = 0;
	}
	
    // Scale the integration limit
    float intLimScaled = gains.imax * 0.01f;

    // Constrain the integrator state
    _pid_info.I = constrain_float(_pid_info.I, -intLimScaled, intLimScaled);
	
	// Calculate the demanded control surface deflection
	// Note the scaler is applied again. We want a 1/speed scaler applied to the feed-forward
	// path, but want a 1/speed^2 scaler applied to the rate error path. 
	// This is because acceleration scales with speed^2, but rate scales with speed.
    _pid_info.D = rate_error * gains.D * scaler;
    _pid_info.P = desired_rate * kp_ff * scaler;
    _pid_info.FF = desired_rate * k_ff * scaler;
    _pid_info.desired = desired_rate;

	_last_out = _pid_info.FF + _pid_info.P + _pid_info.D;

    if (autotune.running && aspeed > aparm.airspeed_min) {
        // let autotune have a go at the values 
        // Note that we don't pass the integrator component so we get
        // a better idea of how much the base PD controller
        // contributed
        autotune.update(desired_rate, achieved_rate, _last_out);
    }

	_last_out += _pid_info.I;
	
	// Convert to centi-degrees and constrain
	return constrain_float(_last_out * 100, -4500, 4500);
}


/*
 Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) desired roll rate in degrees/sec
 2) control gain scaler = scaling_speed / aspeed
*/
int32_t AP_RollController::get_rate_out(float desired_rate, float scaler)
{
    return _get_rate_out(desired_rate, scaler, false);
}

/*
 Function returns an equivalent elevator deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) demanded bank angle in centi-degrees
 2) control gain scaler = scaling_speed / aspeed
 3) boolean which is true when stabilise mode is active
 4) minimum FBW airspeed (metres/sec)
*/
int32_t AP_RollController::get_servo_out(int32_t angle_err, float scaler, bool disable_integrator)
{
    if (adap.enable_chan > 0 && hal.rcin->read(adap.enable_chan-1) >= 1700) {
        // the user has enabled adaptive control test code
        return adaptive_control(radians(angle_err*0.01f)) * 4500;
    }

    if (gains.tau < 0.1f) {
        gains.tau.set(0.1f);
    }
	
	// Calculate the desired roll rate (deg/sec) from the angle error
	float desired_rate = angle_err * 0.01f / gains.tau;

    return _get_rate_out(desired_rate, scaler, disable_integrator);
}

void AP_RollController::reset_I()
{
	_pid_info.I = 0;
}


/*
  adaptive control test code. Maths thanks to Ryan Beall
 */
float AP_RollController::adaptive_control(float phi_error)
{
    float dt;
    float phi = _ahrs.roll; 		 //pitch from ahrs/ekf
    float V_air = 0;

    // get airspeed estimate
    _ahrs.airspeed_estimate(&V_air);

    uint64_t now = AP_HAL::micros64();
    if (adap.last_run_us == 0 || now - adap.last_run_us > 200000UL) {
        // reset after not running for 0.2s
        adap.phi_cm = phi;
        adap.last_run_us = now;
        adap.delta_aileron = 0;
        return 0;
    }    

    dt = (now - adap.last_run_us) * 1.0e-6f;
    adap.last_run_us = now;

    float phi_command = phi_error + phi;

    // Companion Model
    adap.phi_cm += dt*(-adap.alpha*(phi_command - adap.phi_cm) + adap.alpha*(phi - adap.phi_cm) - (V_air*adap.K1_hat - V_air*adap.delta_aileron*adap.K2));          float model_error = adap.phi_cm - phi; 
    
    if (fabsf(model_error) > radians(adap.deadband)) {          
           // Parameter Update
           adap.K1_hat += dt*(-adap.gamma * (phi - adap.phi_cm) * V_air);
           // Protection for robustness of K1
           adap.K1_hat = constrain_float(adap.K1_hat, adap.K1_lower_limit, adap.K1_upper_limit);
           //  lowpass filter adaptive estimate
           float alpha_filt = (dt * adap.W0 / (1 + dt * adap.W0)); //local variable for quick discrete calculation of lowpass time constant
           alpha_filt = constrain_float(alpha_filt, 0.0, 1.0);          
           adap.K1_hat_lowpass = (1 - alpha_filt)*adap.K1_hat_lowpass + alpha_filt*(adap.K1_hat);
        }

     // Control output
     float control = V_air*adap.K2;
   
     if (fabsf(control) > 0.0001) {
            adap.delta_aileron = (adap.K1_hat_lowpass/adap.K2) + ((adap.alpha*(phi_command - phi) + adap.alpha*(phi_command - adap.phi_cm))/control); 
     }

    DataFlash_Class::instance()->Log_Write("ADAR", "TimeUS,Dt,K1H,K1HL,DE,TCM,TErr,Roll,VAir", "Qffffffff",
                                           now,
                                           dt,
                                           adap.K1_hat, adap.K1_hat_lowpass,
                                           adap.delta_aileron,
                                           degrees(adap.phi_cm),
                                           degrees(phi_error),
                                           degrees(phi),
                                           V_air);

    _pid_info.P = adap.K1_hat;
    _pid_info.I = 0.98 * _pid_info.I + 0.02 * adap.K1_hat_lowpass;
    _pid_info.FF = adap.K1_hat_lowpass;
    _pid_info.D = adap.phi_cm;
    _pid_info.desired = adap.delta_aileron;
    
    return constrain_float(adap.delta_aileron, -1, 1);
}
