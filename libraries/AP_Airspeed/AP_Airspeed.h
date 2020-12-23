#pragma once

#include <AP_Common/AP_Common.h>
#include <AP_HAL/AP_HAL.h>
#include <AP_Param/AP_Param.h>
#include <GCS_MAVLink/GCS_MAVLink.h>
#include <AP_Math/AP_Math.h>
#include <AP_MSP/msp.h>

class AP_Airspeed_Backend;

#ifndef AIRSPEED_MAX_SENSORS
#define AIRSPEED_MAX_SENSORS 2
#endif

#ifndef AP_AIRSPEED_AUTOCAL_ENABLE
#define AP_AIRSPEED_AUTOCAL_ENABLE !defined(HAL_BUILD_AP_PERIPH)
#endif

#ifndef HAL_MSP_AIRSPEED_ENABLED
#define HAL_MSP_AIRSPEED_ENABLED HAL_MSP_SENSORS_ENABLED
#endif
class Airspeed_Calibration {
public:
    friend class AP_Airspeed;
    // constructor
    Airspeed_Calibration();

    // initialise the calibration
    void init(float initial_ratio);

    // take current airspeed in m/s and ground speed vector and return
    // new scaling factor
    float update(float airspeed, const Vector3f &vg, int16_t max_airspeed_allowed_during_cal);

private:
    // state of kalman filter for airspeed ratio estimation
    Matrix3f P; // covarience matrix
    const float Q0; // process noise matrix top left and middle element
    const float Q1; // process noise matrix bottom right element
    Vector3f state; // state vector
    const float DT; // time delta
};

class AP_Airspeed
{
public:
    friend class AP_Airspeed_Backend;
    
    // constructor
    AP_Airspeed();

    void init(void);

#if AP_AIRSPEED_AUTOCAL_ENABLE
    // inflight ratio calibration
    void set_calibration_enabled(bool enable) {calibration_enabled = enable;}
#endif //AP_AIRSPEED_AUTOCAL_ENABLE

    // read the analog source and update airspeed
    void update(bool log);

    // calibrate the airspeed. This must be called on startup if the
    // altitude/climb_rate/acceleration interfaces are ever used
    void calibrate(bool in_startup);

    // return the current airspeed in m/s
    float get_airspeed(uint8_t i) const {
        return state[i].airspeed;
    }
    float get_airspeed(void) const { return get_airspeed(primary); }

    // return the unfiltered airspeed in m/s
    float get_raw_airspeed(uint8_t i) const {
        return state[i].raw_airspeed;
    }
    float get_raw_airspeed(void) const { return get_raw_airspeed(primary); }

    // return the current old-style airspeed ratio (dimensionless)
    float get_airspeed_ratio(uint8_t i) const {
        return param[i].pcorrect*correction_to_ratio;
    }
    float get_airspeed_ratio(void) const { return get_airspeed_ratio(primary); }

    // get temperature if available
    bool get_temperature(uint8_t i, float &temperature);
    bool get_temperature(float &temperature) { return get_temperature(primary, temperature); }

    // return true if airspeed is enabled, and airspeed use is set
    bool use(uint8_t i) const;
    bool use(void) const { return use(primary); }

    // force disabling of all airspeed sensors
    void force_disable_use(bool value) {
        _force_disable_use = value;
    }

    // return true if airspeed is enabled
    bool enabled(uint8_t i) const {
        if (i < AIRSPEED_MAX_SENSORS) {
            return param[i].type.get() != TYPE_NONE;
        }
        return false;
    }
    bool enabled(void) const { return enabled(primary); }

    // return the differential pressure in Pascal for the last airspeed reading
    float get_differential_pressure(uint8_t i) const {
        return state[i].last_pressure;
    }
    float get_differential_pressure(void) const { return get_differential_pressure(primary); }

    // update airspeed ratio calibration
    void update_calibration(const Vector3f &vground, int16_t max_airspeed_allowed_during_cal);

    // return health status of sensor
    bool healthy(uint8_t i) const {
        bool ok = state[i].healthy && enabled(i);
#ifndef HAL_BUILD_AP_PERIPH
        ok &= (fabsf(param[i].offset) > 0 || state[i].use_zero_offset);
#endif
        return ok;
    }
    bool healthy(void) const { return healthy(primary); }

    // return true if all enabled sensors are healthy
    bool all_healthy(void) const;
    
    // return time in ms of last update
    uint32_t last_update_ms(uint8_t i) const { return state[i].last_update_ms; }
    uint32_t last_update_ms(void) const { return last_update_ms(primary); }

    static const struct AP_Param::GroupInfo var_info[];

    enum pitot_tube_order { PITOT_TUBE_ORDER_POSITIVE = 0,
                            PITOT_TUBE_ORDER_NEGATIVE = 1,
                            PITOT_TUBE_ORDER_AUTO     = 2 };

    enum OptionsMask {
        ON_FAILURE_AHRS_WIND_MAX_DO_DISABLE                   = (1<<0),   // If set then use airspeed failure check
        ON_FAILURE_AHRS_WIND_MAX_RECOVERY_DO_REENABLE         = (1<<1),   // If set then automatically enable the airspeed sensor use when healthy again.
    };

    enum airspeed_type {
        TYPE_NONE=0,
        TYPE_I2C_MS4525=1,
        TYPE_ANALOG=2,
        TYPE_I2C_MS5525=3,
        TYPE_I2C_MS5525_ADDRESS_1=4,
        TYPE_I2C_MS5525_ADDRESS_2=5,
        TYPE_I2C_SDP3X=6,
        TYPE_I2C_DLVR_5IN=7,
        TYPE_UAVCAN=8,
        TYPE_I2C_DLVR_10IN=9,
        TYPE_I2C_DLVR_20IN=10,
        TYPE_I2C_DLVR_30IN=11,
        TYPE_I2C_DLVR_60IN=12,
        TYPE_NMEA_WATER=13,
        TYPE_MSP=14,
        TYPE_I2C_ASP5033=15,
    };

    // get current primary sensor
    uint8_t get_primary(void) const { return primary; }

    // get number of sensors
    uint8_t get_num_sensors(void) const { return num_sensors; }
    
    static AP_Airspeed *get_singleton() { return _singleton; }

    // return the current corrected pressure, public for AP_Periph
    float get_corrected_pressure(uint8_t i) const {
        return state[i].corrected_pressure;
    }
    float get_corrected_pressure(void) const {
        return get_corrected_pressure(primary);
    }

#if HAL_MSP_AIRSPEED_ENABLED
    void handle_msp(const MSP::msp_airspeed_data_message_t &pkt);
#endif
    
private:
    static AP_Airspeed *_singleton;

    AP_Int8 primary_sensor;
    AP_Int32 _options;    // bitmask options for airspeed
    AP_Float _wind_max;
    AP_Float _wind_warn;

    struct {
        AP_Float offset;
        AP_Float pcorrect;
        AP_Float psi_range;
        AP_Int8  use;
        AP_Int8  type;
        AP_Int8  pin;
        AP_Int8  bus;
        AP_Int8  autocal;
        AP_Int8  tube_order;
        AP_Int8  skip_cal;
    } param[AIRSPEED_MAX_SENSORS];

    struct airspeed_state {
        float   raw_airspeed;
        float   airspeed;
        float	last_pressure;
        float   filtered_pressure;
        float	corrected_pressure;
        uint32_t last_update_ms;
        bool use_zero_offset;
        bool	healthy;

        // state of runtime calibration
        struct {
            uint32_t start_ms;
            float    sum;
            uint16_t count;
            uint16_t read_count;
        } cal;

#if AP_AIRSPEED_AUTOCAL_ENABLE
        Airspeed_Calibration calibration;
        float last_saved_pcorrect;
        uint8_t counter;
#endif // AP_AIRSPEED_AUTOCAL_ENABLE

        struct {
            uint32_t last_check_ms;
            float health_probability;
            int8_t param_use_backup;
            uint32_t last_warn_ms;
        } failures;
    } state[AIRSPEED_MAX_SENSORS];

    bool calibration_enabled;

    // can be set to true to disable the use of the airspeed sensor
    bool _force_disable_use;

    // current primary sensor
    uint8_t primary;
    uint8_t num_sensors;

    void read(uint8_t i);
    // return the differential pressure in Pascal for the last airspeed reading for the requested instance
    // returns 0 if the sensor is not enabled
    float get_pressure(uint8_t i);

    // get the failure health probability
    float get_health_failure_probability(uint8_t i) const {
        return state[i].failures.health_probability;
    }
    float get_health_failure_probability(void) const {
        return get_health_failure_probability(primary);
    }

    void update_calibration(uint8_t i, float raw_pressure);
    void update_calibration(uint8_t i, const Vector3f &vground, int16_t max_airspeed_allowed_during_cal);
    void send_airspeed_calibration(const Vector3f &vg);
    // return the current calibration offset
    float get_offset(uint8_t i) const {
        return param[i].offset;
    }
    float get_offset(void) const { return get_offset(primary); }

    void check_sensor_failures();
    void check_sensor_ahrs_wind_max_failures(uint8_t i);

    AP_Airspeed_Backend *sensor[AIRSPEED_MAX_SENSORS];

    void Log_Airspeed();

    // convert pitot differential pressure to EAS
    float calc_EAS(float diff_pressure, float static_pressure);

    // convert pitot differential pressure to EAS, applying corrections
    float calc_corrected_EAS(uint8_t sensor, float diff_pressure);

    /*
      this converts between an old-style ratio and a pressure correction factor. It comes from a nominal 20m/s
      with this formula
      EAS=20
      (pow(EAS**2 / ((1 / SSL_AIR_DENSITY ) * 7 * SSL_AIR_PRESSURE)+1,7/2)-1) * SSL_AIR_PRESSURE / (EAS**2)

      It is quite insensitive to the conversion EAS, with a difference of 1% between 20m/s and 100m/s

      This fixes our old airspeed EAS calculation, which was: EAS = sqrt(differential_pressure * ratio)
     */
    const float ratio_to_correction = 0.53411955733862;
    const float correction_to_ratio = 1.8722399999407286;
};

namespace AP {
    AP_Airspeed *airspeed();
};
