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
    // @Param: 2SRV_TCONST
	// @DisplayName: Roll Time Constant
	// @Description: Time constant in seconds from demanded to achieved roll angle. Most models respond well to 0.5. May be reduced for faster responses, but setting lower than a model can achieve will not help.
	// @Range: 0.4 1.0
	// @Units: s
	// @Increment: 0.1
	// @User: Advanced
    AP_GROUPINFO("2SRV_TCONST",      0, AP_RollController, gains.tau,       0.5f),

    // @Param: 2SRV_P
	// @DisplayName: Proportional Gain
	// @Description: Proportional gain from roll angle demands to ailerons. Higher values allow more servo response but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0.1 4.0
	// @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("2SRV_P",        1, AP_RollController, gains.P,        1.0f),

    // @Param: 2SRV_D
	// @DisplayName: Damping Gain
	// @Description: Damping gain from roll acceleration to ailerons. Higher values reduce rolling in turbulence, but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 0.2
	// @Increment: 0.01
    // @User: Standard
    AP_GROUPINFO("2SRV_D",        2, AP_RollController, gains.D,        0.08f),

    // @Param: 2SRV_I
	// @DisplayName: Integrator Gain
	// @Description: Integrator gain from long-term roll angle offsets to ailerons. Higher values "trim" out offsets faster but can cause oscillations. Automatically set and adjusted by AUTOTUNE mode.
	// @Range: 0 1.0
	// @Increment: 0.05
    // @User: Standard
    AP_GROUPINFO("2SRV_I",        3, AP_RollController, gains.I,        0.3f),

    // @Param: 2SRV_RMAX
	// @DisplayName: Maximum Roll Rate
	// @Description: Maximum roll rate that the roll controller demands (degrees/sec) in ACRO mode.
	// @Range: 0 180
	// @Units: deg/s
	// @Increment: 1
	// @User: Advanced
    AP_GROUPINFO("2SRV_RMAX",   4, AP_RollController, gains.rmax,       0),

    // @Param: 2SRV_IMAX
	// @DisplayName: Integrator limit
	// @Description: Limit of roll integrator gain in centi-degrees of servo travel. Servos are assumed to have +/- 4500 centi-degrees of travel, so a value of 3000 allows trim of up to 2/3 of servo travel range.
	// @Range: 0 4500
	// @Increment: 1
	// @User: Advanced
    AP_GROUPINFO("2SRV_IMAX",      5, AP_RollController, gains.imax,        3000),

    // @Param: 2SRV_FF
	// @DisplayName: Feed forward Gain
	// @Description: Gain from demanded rate to aileron output. 
	// @Range: 0.1 4.0
	// @Increment: 0.1
    // @User: Standard
    AP_GROUPINFO("2SRV_FF",        6, AP_RollController, gains.FF,          0.0f),

    // @Param: 2SRV_SRMAX
    // @DisplayName: Servo slew rate limit
    // @Description: Sets an upper limit on the servo slew rate produced by the D-gain (roll rate feedback). If the amplitude of the control action produced by the roll rate feedback exceeds this value, then the D-gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive D-gain. The parameter should be set to no more than 25% of the servo's specified slew rate to allow for inertia and aerodynamic load effects. Note: The D-gain will not be reduced to less than 10% of the nominal value. A valule of zero will disable this feature.
    // @Units: deg/s
    // @Range: 0 500
    // @Increment: 10.0
    // @User: Advanced
    AP_GROUPINFO("2SRV_SRMAX", 7, AP_RollController, _slew_rate_max, 150.0f),

    // @Param: 2SRV_SRTAU
    // @DisplayName: Servo slew rate decay time constant
    // @Description: This sets the time constant used to recover the D-gain after it has been reduced due to excessive servo slew rate.
    // @Units: s
    // @Range: 0.5 5.0
    // @Increment: 0.1
    // @User: Advanced
    AP_GROUPINFO("2SRV_SRTAU", 8, AP_RollController, _slew_rate_tau, 1.0f),

    // @Param: _RATE_P
    // @DisplayName: Roll axis rate controller P gain
    // @Description: Roll axis rate controller P gain.  Converts the difference between desired roll rate and actual roll rate into a motor speed output
    // @Range: 0.08 0.35
    // @Increment: 0.005
    // @User: Standard

    // @Param: _RATE_I
    // @DisplayName: Roll axis rate controller I gain
    // @Description: Roll axis rate controller I gain.  Corrects long-term difference in desired roll rate vs actual roll rate
    // @Range: 0.01 0.6
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_IMAX
    // @DisplayName: Roll axis rate controller I gain maximum
    // @Description: Roll axis rate controller I gain maximum.  Constrains the maximum motor output that the I gain will output
    // @Range: 0 1
    // @Increment: 0.01
    // @User: Standard

    // @Param: _RATE_D
    // @DisplayName: Roll axis rate controller D gain
    // @Description: Roll axis rate controller D gain.  Compensates for short-term change in desired roll rate vs actual roll rate
    // @Range: 0.001 0.03
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FF
    // @DisplayName: Roll axis rate controller feed forward
    // @Description: Roll axis rate controller feed forward
    // @Range: 0 3.0
    // @Increment: 0.001
    // @User: Standard

    // @Param: _RATE_FLTT
    // @DisplayName: Roll axis rate controller target frequency in Hz
    // @Description: Roll axis rate controller target frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTE
    // @DisplayName: Roll axis rate controller error frequency in Hz
    // @Description: Roll axis rate controller error frequency in Hz
    // @Range: 2 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_FLTD
    // @DisplayName: Roll axis rate controller derivative frequency in Hz
    // @Description: Roll axis rate controller derivative frequency in Hz
    // @Range: 0 50
    // @Increment: 1
    // @Units: Hz
    // @User: Standard

    // @Param: _RATE_SMAX
    // @DisplayName: Roll slew rate limit
    // @Description: Sets an upper limit on the slew rate produced by the combined P and D gains. If the amplitude of the control action produced by the rate feedback exceeds this value, then the D+P gain is reduced to respect the limit. This limits the amplitude of high frequency oscillations caused by an excessive gain. The limit should be set to no more than 25% of the actuators maximum slew rate to allow for load effects. Note: The gain will not be reduced to less than 10% of the nominal value. A value of zero will disable this feature.
    // @Range: 0 200
    // @Increment: 0.5
    // @User: Advanced

    // @Param: _RATE_STAU
    // @DisplayName: Roll slew rate decay time constant
    // @Description: This sets the time constant used to recover the P+D gain after it has been reduced due to excessive slew rate.
    // @Units: s
    // @Range: 0.5 5.0
    // @Increment: 0.1
    // @User: Advanced

    AP_SUBGROUPINFO(rate_pid, "_RATE_", 9, AP_RollController, AC_PID),
    
    AP_GROUPEND
};


/*
  internal rate controller, called by attitude and rate controller
  public functions
*/
int32_t AP_RollController::_get_rate_out_old(float desired_rate, float scaler, bool disable_integrator)
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
    // Get body rate vector (radians/sec)
	float omega_x = _ahrs.get_gyro().x;
	
	// Calculate the roll rate error (deg/sec) and apply gain scaler
    float achieved_rate = ToDeg(omega_x);
    _pid_info.error = desired_rate - achieved_rate;
    float rate_error = _pid_info.error * scaler;
    _pid_info.target = desired_rate;
    _pid_info.actual = achieved_rate;
	
	// Get an airspeed estimate - default to zero if none available
	float aspeed;
	if (!_ahrs.airspeed_estimate(aspeed)) {
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
    const float last_pid_info_D = _pid_info.D;

    _pid_info.D = rate_error * gains.D * scaler;
    _pid_info.P = desired_rate * kp_ff * scaler;
    _pid_info.FF = desired_rate * k_ff * scaler;

    if (dt > 0 && _slew_rate_max > 0) {
        // Calculate the slew rate amplitude produced by the unmodified D term

        // calculate a low pass filtered slew rate
        float Dterm_slew_rate = _slew_rate_filter.apply((fabsf(_pid_info.D - last_pid_info_D)/ delta_time), delta_time);

        // rectify and apply a decaying envelope filter
        float alpha = 1.0f - constrain_float(delta_time/_slew_rate_tau, 0.0f , 1.0f);
        _slew_rate_amplitude = fmaxf(fabsf(Dterm_slew_rate), alpha * _slew_rate_amplitude);
        _slew_rate_amplitude = fminf(_slew_rate_amplitude, 10.0f*_slew_rate_max);

        // Calculate and apply the D gain adjustment
        _pid_info.Dmod = _slew_rate_max / fmaxf(_slew_rate_amplitude, _slew_rate_max);
        _pid_info.D *= _pid_info.Dmod;
    }

    _last_out = _pid_info.D + _pid_info.FF + _pid_info.P;

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
  AC_PID based rate controller
*/
int32_t AP_RollController::_get_rate_out_ac_pid(float desired_rate, float scaler, bool disable_integrator)
{
    convert_pid();

    const float dt = AP::scheduler().get_loop_period_s();
    const float eas2tas = _ahrs.get_EAS2TAS();
    bool limit_I = fabsf(last_ac_out) >= 45;
    float rate_x = _ahrs.get_gyro().x;
    float aspeed;
    float old_I = rate_pid.get_i();

    rate_pid.set_dt(dt);

    if (!_ahrs.airspeed_estimate(aspeed)) {
        aspeed = 0;
    }
    bool underspeed = aspeed <= float(aparm.airspeed_min);
    if (underspeed) {
        limit_I = true;
    }

    // the P and I elements are scaled by sq(scaler). To use an
    // unmodified AC_PID object we scale the inputs and calculate FF separately
    //
    // note that we run AC_PID in radians so that the normal scaling
    // range for IMAX in AC_PID applies (usually an IMAX value less than 1.0)
    rate_pid.update_all(radians(desired_rate) * scaler * scaler, rate_x * scaler * scaler, limit_I);

    if (underspeed) {
        // when underspeed we lock the integrator
        rate_pid.set_integrator(old_I);
    }
    
    // FF should be scaled by scaler/eas2tas, but since we have scaled
    // the AC_PID target above by scaler*scaler we need to instead
    // divide by scaler*eas2tas to get the right scaling
    const float ff = degrees(rate_pid.get_ff() / (scaler * eas2tas));

    if (disable_integrator) {
        rate_pid.reset_I();
    }

    // convert AC_PID info object to same scale as old controller
    _pid_info_ac_pid = rate_pid.get_pid_info();
    auto &pinfo = _pid_info_ac_pid;

    const float deg_scale = degrees(1);
    pinfo.FF = ff;
    pinfo.P *= deg_scale;
    pinfo.I *= deg_scale;
    pinfo.D *= deg_scale;

    // fix the logged target and actual values to not have the scalers applied
    pinfo.target = desired_rate;
    pinfo.actual = degrees(rate_x);

    // sum components
    float out = pinfo.FF + pinfo.P + pinfo.I + pinfo.D;

    // remember the last output to trigger the I limit
    last_ac_out = out;

    // output is scaled to notional centidegrees of deflection
    return constrain_int32(out * 100, -4500, 4500);
}


/*
  rate controller selector
*/
int32_t AP_RollController::_get_rate_out(float desired_rate, float scaler, bool disable_integrator)
{
    int32_t ret_ac_pid = _get_rate_out_ac_pid(desired_rate, scaler, disable_integrator);
    int32_t ret_old = _get_rate_out_old(desired_rate, scaler, disable_integrator);
    const auto &pinfo_ac = _pid_info_ac_pid;
    const auto &pinfo_old = _pid_info;
    AP::logger().Write("PIXR", "TimeUS,AC,Old,ACSum,OldSum", "Qiiff",
                       AP_HAL::micros64(),
                       ret_ac_pid,
                       ret_old,
                       pinfo_ac.FF + pinfo_ac.P + pinfo_ac.I + pinfo_ac.D,
                       pinfo_old.FF + pinfo_old.P + pinfo_old.I + pinfo_old.D);
    return use_ac_pid ? ret_ac_pid : ret_old;
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
 Function returns an equivalent aileron deflection in centi-degrees in the range from -4500 to 4500
 A positive demand is up
 Inputs are: 
 1) demanded bank angle in centi-degrees
 2) control gain scaler = scaling_speed / aspeed
 3) boolean which is true when stabilise mode is active
 4) minimum FBW airspeed (metres/sec)
*/
int32_t AP_RollController::get_servo_out(int32_t angle_err, float scaler, bool disable_integrator)
{
    if (gains.tau < 0.1f) {
        gains.tau.set(0.1f);
    }
	
	// Calculate the desired roll rate (deg/sec) from the angle error
	float desired_rate = angle_err * 0.01f / gains.tau;

    // Limit the demanded roll rate
    if (gains.rmax && desired_rate < -gains.rmax) {
        desired_rate = - gains.rmax;
    } else if (gains.rmax && desired_rate > gains.rmax) {
        desired_rate = gains.rmax;
    }

    return _get_rate_out(desired_rate, scaler, disable_integrator);
}

void AP_RollController::reset_I()
{
	_pid_info.I = 0;
}

/*
  convert from old to new PIDs
  this is a temporary conversion function during development
 */
void AP_RollController::convert_pid()
{
    if (done_init && is_positive(rate_pid.ff())) {
        return;
    }
    done_init = true;
    AP_Float &ff = rate_pid.ff();
    if (is_positive(ff) && ff.configured_in_storage()) {
        return;
    }
    const float kp_ff = MAX((gains.P - gains.I * gains.tau) * gains.tau  - gains.D, 0);
    rate_pid.ff().set_and_save(gains.FF + kp_ff);
    rate_pid.kI().set_and_save_ifchanged(gains.I * gains.tau);
    rate_pid.kP().set_and_save_ifchanged(gains.D);
    rate_pid.kD().set_and_save_ifchanged(0);
    rate_pid.kIMAX().set_and_save_ifchanged(gains.imax/4500.0);
}
