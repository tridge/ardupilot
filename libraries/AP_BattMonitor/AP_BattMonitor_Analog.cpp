#include "AP_BattMonitor_config.h"

#if AP_BATTERY_ANALOG_ENABLED

#include <AP_HAL/AP_HAL.h>
#include <AP_Common/AP_Common.h>
#include <AP_Math/AP_Math.h>
#include <AP_Logger/AP_Logger.h>

#include "AP_BattMonitor_Analog.h"

extern const AP_HAL::HAL& hal;

const AP_Param::GroupInfo AP_BattMonitor_Analog::var_info[] = {

    // @Param: VOLT_PIN
    // @DisplayName: Battery Voltage sensing pin
    // @Description: Sets the analog input pin that should be used for voltage monitoring.
    // @Values: -1:Disabled, 2:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 5:Navigator, 13:Pixhawk2_PM2/CubeOrange_PM2, 14:CubeOrange, 16:Durandal, 100:PX4-v1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("VOLT_PIN", 1, AP_BattMonitor_Analog, _volt_pin, AP_BATT_VOLT_PIN),

    // @Param: CURR_PIN
    // @DisplayName: Battery Current sensing pin
    // @Description: Sets the analog input pin that should be used for current monitoring.
    // @Values: -1:Disabled, 3:Pixhawk/Pixracer/Navio2/Pixhawk2_PM1, 4:CubeOrange_PM2/Navigator, 14:Pixhawk2_PM2, 15:CubeOrange, 17:Durandal, 101:PX4-v1
    // @User: Standard
    // @RebootRequired: True
    AP_GROUPINFO("CURR_PIN", 2, AP_BattMonitor_Analog, _curr_pin, AP_BATT_CURR_PIN),

    // @Param: VOLT_MULT
    // @DisplayName: Voltage Multiplier
    // @Description: Used to convert the voltage of the voltage sensing pin (@PREFIX@VOLT_PIN) to the actual battery's voltage (pin_voltage * VOLT_MULT). For the 3DR Power brick with a Pixhawk, this should be set to 10.1. For the Pixhawk with the 3DR 4in1 ESC this should be 12.02. For the PX using the PX4IO power supply this should be set to 1.
    // @User: Advanced
    AP_GROUPINFO("VOLT_MULT", 3, AP_BattMonitor_Analog, _volt_multiplier, AP_BATT_VOLTDIVIDER_DEFAULT),

    // @Param: AMP_PERVLT
    // @DisplayName: Amps per volt
    // @Description: Number of amps that a 1V reading on the current sensor corresponds to. With a Pixhawk using the 3DR Power brick this should be set to 17. For the Pixhawk with the 3DR 4in1 ESC this should be 17. For Synthetic Current sensor monitors, this is the maximum, full throttle current draw.
    // @Units: A/V
    // @User: Standard
    AP_GROUPINFO("AMP_PERVLT", 4, AP_BattMonitor_Analog, _curr_amp_per_volt, AP_BATT_CURR_AMP_PERVOLT_DEFAULT),

    // @Param: AMP_OFFSET
    // @DisplayName: AMP offset
    // @Description: Voltage offset at zero current on current sensor for Analog Sensors. For Synthetic Current sensor, this offset is the zero throttle system current and is added to the calculated throttle base current.
    // @Units: V
    // @User: Standard
    AP_GROUPINFO("AMP_OFFSET", 5, AP_BattMonitor_Analog, _curr_amp_offset, AP_BATT_CURR_AMP_OFFSET_DEFAULT),

    // @Param: VLT_OFFSET
    // @DisplayName: Voltage offset
    // @Description: Voltage offset on voltage pin. This allows for an offset due to a diode. This voltage is subtracted before the scaling is applied.
    // @Units: V
    // @User: Advanced
    AP_GROUPINFO("VLT_OFFSET", 6, AP_BattMonitor_Analog, _volt_offset, 0),
    
    // Param indexes must be less than 10 to avoid conflict with other battery monitor param tables loaded by pointer

    AP_GROUPEND
};

/// Constructor
AP_BattMonitor_Analog::AP_BattMonitor_Analog(AP_BattMonitor &mon,
                                             AP_BattMonitor::BattMonitor_State &mon_state,
                                             AP_BattMonitor_Params &params) :
    AP_BattMonitor_Backend(mon, mon_state, params)
{
    AP_Param::setup_object_defaults(this, var_info);

    // no other good way of setting these defaults
#if AP_BATT_MONITOR_MAX_INSTANCES > 1
    if (mon_state.instance == 1) {
#ifdef HAL_BATT2_VOLT_PIN
        _volt_pin.set_default(HAL_BATT2_VOLT_PIN);
#endif
#ifdef HAL_BATT2_CURR_PIN
        _curr_pin.set_default(HAL_BATT2_CURR_PIN);
#endif
#ifdef HAL_BATT2_VOLT_SCALE
        _volt_multiplier.set_default(HAL_BATT2_VOLT_SCALE);
#endif
#ifdef HAL_BATT2_CURR_SCALE
        _curr_amp_per_volt.set_default(HAL_BATT2_CURR_SCALE);
#endif
    }
#endif
    _state.var_info = var_info;
    
    _volt_pin_analog_source = hal.analogin->channel(_volt_pin);
    _curr_pin_analog_source = hal.analogin->channel(_curr_pin);

}

// read - read the voltage and current
void
AP_BattMonitor_Analog::read()
{
    // this copes with changing the pin at runtime
    _state.healthy = _volt_pin_analog_source->set_pin(_volt_pin);

    // get voltage
    _state.voltage = (_volt_pin_analog_source->voltage_average() - _volt_offset) * _volt_multiplier;

    // read current
    if (has_current()) {
        // calculate time since last current read
        const uint32_t tnow = AP_HAL::micros();
        const uint32_t dt_us = tnow - _state.last_time_micros;

        // this copes with changing the pin at runtime
        _state.healthy &= _curr_pin_analog_source->set_pin(_curr_pin);

        // read current
        _state.current_amps = (_curr_pin_analog_source->voltage_average() - _curr_amp_offset) * _curr_amp_per_volt;

        update_consumed(_state, dt_us);

        // record time
        _state.last_time_micros = tnow;
    }
}

// read - read the voltage and current WITH NUMBER OF BATTERIES
void
AP_BattMonitor_Analog::read(bool armed)
{
    // this copes with changing the pin at runtime
    _state.healthy = _volt_pin_analog_source->set_pin(_volt_pin);

    // get voltage
    _state.voltage = (_volt_pin_analog_source->voltage_average() - _volt_offset) * _volt_multiplier;

    // read current
    if (has_current()) {
        // calculate time since last current read
        uint32_t tnow = AP_HAL::micros();
        float dt = tnow - _state.last_time_micros;

        // this copes with changing the pin at runtime
        _state.healthy &= _curr_pin_analog_source->set_pin(_curr_pin);

        // read current
        _state.current_amps = (_curr_pin_analog_source->voltage_average() - _curr_amp_offset) * _curr_amp_per_volt;

        // update total current drawn since startup (2 second buffer after start)
        if (_state.last_time_micros != 0 && dt < 2000000.0f) {
            // .0002778 is 1/3600 (conversion to hours)
            float mah = _state.current_amps * dt * 0.0000002778f;
            _state.consumed_mah += mah;
            _state.consumed_wh  += 0.001f * mah * _state.voltage;
        }

        // record time
        _state.last_time_micros = tnow;
        if(!_initialized && !isnan(_state.voltage) && !is_zero(_state.voltage) && armed){
            _initialized = true;
            reset();
        }
        calculate_percent_remaining(dt, armed);
    }
}

/// return true if battery provides current info
bool AP_BattMonitor_Analog::has_current() const
{
    return ((AP_BattMonitor::Type)_params._type.get() == AP_BattMonitor::Type::ANALOG_VOLTAGE_AND_CURRENT);
}

bool AP_BattMonitor_Analog::capacity_remaining_pct(uint8_t &percentage) const
{
    if(_display_voltage_only) {
        percentage = 0;
        return false;
    }
    else {
        percentage = (uint8_t)ceilf(_percent_remaining);
        return true;
    }
}

void AP_BattMonitor_Analog::calculate_percent_remaining(const uint32_t dt, const bool armed)
{
    if(_initialized){
        static int32_t prev_pack_capacity = _params._pack_capacity;
        if(_params._pack_capacity != prev_pack_capacity){reset();}

        float total_capacity_wh = AP_BATT_NOMINAL_VOLTAGE*(float)_params._pack_capacity/1000.0f;
        float dt_s = (float)dt/1000000.0;
        float soc_v = get_percentage_from_voltage(_state.voltage);
        float soc_wh = MIN(MAX(_starting_percentage - ((_state.consumed_wh / total_capacity_wh) * 100.0), 0.0), 100.0);

        float alpha = MIN(MAX(dt_s/(dt_s+10.0), 0.0), 1.0);
        float dVlpf = alpha*(_state.voltage - _prev_v)/dt_s + (1.0-alpha)*_prev_dVlpf;

        float w = 0;
        if(dVlpf < 0) { //Voltage must be decreasing for us to consider incorporating voltage reading into percentage
            w = 1e-5*(100-soc_v);
        }

        AP::logger().Write("BATW", "TimeUS,Weight,Alpha,dVlpf,soc_v,soc_wh,critv", "Qffffff",
                            AP_HAL::micros64(),
                            (double)w,
                            (double)alpha,
                            (double)dVlpf,
                            (double)soc_v,
                            (double)soc_wh,
                            (double)_params._critical_voltage);

        float d_wh = _state.consumed_wh - _prev_wh;

        _percent_remaining = (1-w)*_prev_perc + w*soc_v; //If disarmed, just give lpf Voltage estimate
        if(armed){
            _percent_remaining = MAX(_percent_remaining-100*d_wh/total_capacity_wh, soc_v); //Percentage should never go below SOC_V
            _percent_remaining = MIN(_percent_remaining, soc_wh); //Percentage should never go above SOC_WH
        }

        if(_percent_remaining > _prev_perc) {_percent_remaining = _prev_perc;}

        //May have to capture these states at start of function, but in theory they are set in the read() function in which this current function exists
        _prev_v = _state.voltage;
        _prev_wh = _state.consumed_wh;
        _prev_perc = _percent_remaining;
        _prev_dVlpf = dVlpf;
    }
    else{
        _percent_remaining = get_percentage_from_voltage(_state.voltage);
    }
}

float AP_BattMonitor_Analog::get_percentage_from_voltage(const float voltage) const
{
    float norm_V = MAX((voltage - MIN(_params._critical_voltage,AP_BATT_MONITOR_PRC_EST_MIN_VOLT)), 0.0) / (AP_BATT_MONITOR_PRC_EST_MAX_VOLT - MIN(_params._critical_voltage,AP_BATT_MONITOR_PRC_EST_MIN_VOLT));
    return 50-50*(cosf(M_PI*powf(norm_V,AP_BATT_MONITOR_PRC_EST_BLND_EST)));
}

void AP_BattMonitor_Analog::reset(){
    _starting_percentage = get_percentage_from_voltage(_state.voltage);
    _prev_v = _state.voltage;
    _prev_wh = _state.consumed_wh;
    _prev_perc = _percent_remaining;
    _prev_dVlpf = 0.0;
}

void AP_BattMonitor_Analog::display_voltage_only(bool display)
{
    if (display) {
        _display_voltage_only = true;
    }
}

#endif  // AP_BATTERY_ANALOG_ENABLED
