#pragma once

#include "AP_BattMonitor_Backend.h"

#if AP_BATTERY_ANALOG_ENABLED

#include "AP_BattMonitor.h"

// default pins and dividers
#if defined(HAL_BATT_VOLT_PIN)
 // pins defined in board config (hwdef.dat on ChibiOS)
 # define AP_BATT_VOLT_PIN                  HAL_BATT_VOLT_PIN
 # define AP_BATT_CURR_PIN                  HAL_BATT_CURR_PIN
 # define AP_BATT_VOLTDIVIDER_DEFAULT       HAL_BATT_VOLT_SCALE
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  HAL_BATT_CURR_SCALE
#elif CONFIG_HAL_BOARD == HAL_BOARD_CHIBIOS
 # define AP_BATT_VOLT_PIN                  4
 # define AP_BATT_CURR_PIN                  3
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
#elif CONFIG_HAL_BOARD == HAL_BOARD_SITL
 # define AP_BATT_VOLT_PIN                  13
 # define AP_BATT_CURR_PIN                  12
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX &&  (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_ERLEBOARD || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_PXF)
 # define AP_BATT_VOLT_PIN                  5
 # define AP_BATT_CURR_PIN                  6
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BBBMINI
 # define AP_BATT_VOLT_PIN                  0
 # define AP_BATT_CURR_PIN                  1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_BLUE
 # define AP_BATT_VOLT_PIN                  0
 # define AP_BATT_CURR_PIN                  1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_POCKET
 # define AP_BATT_VOLT_PIN                  1
 # define AP_BATT_CURR_PIN                  0
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO || CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIO2)
 # define AP_BATT_VOLT_PIN                  2
 # define AP_BATT_CURR_PIN                  3
 # define AP_BATT_VOLTDIVIDER_DEFAULT       11.3f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_NAVIGATOR)
 # define AP_BATT_VOLT_PIN                  5
 # define AP_BATT_CURR_PIN                  4
 # define AP_BATT_VOLTDIVIDER_DEFAULT       11.0f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  37.8788f
 # define AP_BATT_CURR_AMP_OFFSET_DEFAULT   0.330f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_EDGE)
 # define AP_BATT_VOLT_PIN                  3
 # define AP_BATT_CURR_PIN                  2
 # define AP_BATT_VOLTDIVIDER_DEFAULT       18.62
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  62.98f

 # define AP_BATT2_VOLT_PIN                  5
 # define AP_BATT2_CURR_PIN                  4
 # define AP_BATT2_VOLTDIVIDER_DEFAULT       18.62
 # define AP_BATT2_CURR_AMP_PERVOLT_DEFAULT  62.98f

#elif CONFIG_HAL_BOARD == HAL_BOARD_LINUX && (CONFIG_HAL_BOARD_SUBTYPE == HAL_BOARD_SUBTYPE_LINUX_OBAL_V1)
 # define AP_BATT_VOLT_PIN                  0
 # define AP_BATT_CURR_PIN                  1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f

#else
 # define AP_BATT_VOLT_PIN                  -1
 # define AP_BATT_CURR_PIN                  -1
 # define AP_BATT_VOLTDIVIDER_DEFAULT       10.1f
 # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT  17.0f
#endif

// This is 0 for the majority of the power modules.
#ifndef AP_BATT_CURR_AMP_OFFSET_DEFAULT
 #define AP_BATT_CURR_AMP_OFFSET_DEFAULT 0.0f
#endif

#define AP_BATT_MONITOR_PRC_EST_MAX_VOLT    50.4f
#define AP_BATT_MONITOR_PRC_EST_MIN_VOLT    35.0f // GCS will show 0% at this voltage
#define AP_BATT_MONITOR_PRC_EST_BLND_EST    1.2f

#define AP_BATT_NOMINAL_VOLTAGE             44.4f

// Other values normally set directly by mission planner
// # define AP_BATT_VOLTDIVIDER_DEFAULT 15.70   // Volt divider for AttoPilot 50V/90A sensor
// # define AP_BATT_VOLTDIVIDER_DEFAULT 4.127   // Volt divider for AttoPilot 13.6V/45A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 27.32  // Amp/Volt for AttoPilot 50V/90A sensor
// # define AP_BATT_CURR_AMP_PERVOLT_DEFAULT 13.66  // Amp/Volt for AttoPilot 13.6V/45A sensor

class AP_BattMonitor_Analog : public AP_BattMonitor_Backend
{
public:

    /// Constructor
    AP_BattMonitor_Analog(AP_BattMonitor &mon, AP_BattMonitor::BattMonitor_State &mon_state, AP_BattMonitor_Params &params);

    /// Read the battery voltage and current.  Should be called at 10hz
    void read() override;
    void read(bool armed) override;

    /// returns true if battery monitor provides consumed energy info
    virtual bool has_consumed_energy() const override { return has_current(); }

    /// returns true if battery monitor provides current info
    virtual bool has_current() const override;

    virtual void init(void) override {}

    bool capacity_remaining_pct(uint8_t &percentage) const override;

    static const struct AP_Param::GroupInfo var_info[];

    virtual void display_voltage_only(bool display) override;

protected:

    AP_HAL::AnalogSource *_volt_pin_analog_source;
    AP_HAL::AnalogSource *_curr_pin_analog_source;

    // Parameters
    AP_Float _volt_multiplier;          /// voltage on volt pin multiplied by this to calculate battery voltage
    AP_Float _curr_amp_per_volt;        /// voltage on current pin multiplied by this to calculate current in amps
    AP_Float _curr_amp_offset;          /// offset voltage that is subtracted from current pin before conversion to amps
    AP_Float _volt_offset;              /// offset voltage that is subtracted from voltage pin before conversion
    AP_Int8  _volt_pin;                 /// board pin used to measure battery voltage
    AP_Int8  _curr_pin;                 /// board pin used to measure battery current

private:
    float get_percentage_from_voltage(const float voltage) const;
    void calculate_percent_remaining(const uint32_t dt, const bool armed);

    void reset();

    bool _initialized = false;
    float _starting_percentage = 100.0;
    float _percent_remaining = 100.0;

    float _prev_v = _state.voltage;
    float _prev_wh = _state.consumed_wh;
    float _prev_perc = _percent_remaining;
    float _prev_dVlpf = 0.0;
    bool _display_voltage_only = false;
};

#endif  // AP_BATTERY_ANALOG_ENABLED
