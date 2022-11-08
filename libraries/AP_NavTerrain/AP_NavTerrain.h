/*
  position estimate correction using rangefinder terrain matching
 */
#pragma once

#include <AP_Math/AP_Math.h>
#include <AP_Common/Location.h>
#include <AP_NavEKF/AP_Nav_Common.h>
#include <AP_HAL/utility/RingBuffer.h>
#include <AP_Mount/AP_Mount.h>
#include <AP_Vehicle/AP_Vehicle_Type.h>

class AP_DAL;
class AP_DAL_RangeFinder_Backend;
class NavEKF3;
class AP_Mount;
struct test_sample;

#define MAX_CORES 3

#define SITL_OR_REPLAY (CONFIG_HAL_BOARD == HAL_BOARD_SITL || APM_BUILD_TYPE(APM_BUILD_Replay))

class AP_NavTerrain
{
public:
    // Constructor
    AP_NavTerrain(NavEKF3 &ekf3);
    friend class GaussianSumFilter;

    // update, gathering data samples
    void update();
    void init(void);

    static const struct AP_Param::GroupInfo var_info[];

private:
    NavEKF3 &ekf3;
    AP_DAL &dal;
    AP_DAL_RangeFinder_Backend *rng;

    class Sample {
    public:
        uint32_t sample_ms;
        float range;
        Quaternion quat;
        Location ek3_loc;
        Location ek2_loc;
        Location gps_loc;
    };

    AP_Int8 enable;
    AP_Int8 debug;
    AP_Int16 options;
    AP_Float min_range;
    AP_Float adsb_rate;
    AP_Int8 gps_relay;
    AP_Int16 n_estimators;
    AP_Int8 _n_lanes;
    AP_Int16 rng_sample_time_msec;
    AP_Float vel_offset_stddev;
    AP_Float vel_offset_alpha;
    AP_Float aspd_offset_stddev;
    AP_Float pos_error_mul;
    AP_Float hgt_drift_rate;
    AP_Float rng_gate_size;
    AP_Int8 roll_step_deg;
    AP_Int8 mount_index;
    AP_Int16 roll_slew_time_msec;
    AP_Int8 rng_sample_distance_m;
    AP_Int8 n_samples_required;
    AP_Float velocity_offset_limit;
    AP_Float hvel_drift_rate;
    AP_Float herr_threshold;
    AP_Float sd_max;
    AP_Float var_scale;

    uint8_t n_lanes;

    uint32_t last_adsb_out_ms;
    bool last_gps_relay;

    Location rng_sample_start_loc;

    struct Gather {
        uint8_t ekf_lane;
        EK3_correction correction;
        uint32_t last_log_ms;
        bool last_using_gps;
        bool restart_terrain_estimation;
        // keep up to 3 samples
        ObjectBuffer_TS<Sample> samples{3};
        Sample sample_pending;
        uint32_t last_sample_ms;
        uint32_t last_gsf_major_update_ms;
        uint32_t last_process_ms;

        // sample time of last gsf processing
        uint32_t last_gsf_sample_ms;
    } gather[MAX_CORES];

    uint32_t last_range_ms;
    uint8_t last_range_lane;

    // average terrain lookups per update
    float lookups_per_update;

    uint32_t last_median_log_ms;

    void gather_data(Gather &g);
    void set_median(void);
    void median_posvel(Location &pos, Vector3f &vel);
    void log_positions(Gather &g);
    void update_thread(void);
    void update_process(Gather &g);
    void process_sample(Gather &g, Sample &sample);

    void set_ahrs_pos_correction(Gather &g);

    enum class Option {
        ROLL_STABILIZED = 0,
    };

    bool option_set(Option opt) const {
        return (options & 1U<<uint32_t(opt)) != 0;
    }


    class OffsetEstimator {
    public:
        OffsetEstimator(const AP_NavTerrain &_frontend) : frontend(_frontend) {}

        bool update(float dt, uint16_t instance, float range_measured, Quaternion &quat_in,
                    Location &loc_in,
                    const Location &median_loc);
        void reset(const uint32_t time_ms, float const aspd_offset, const Vector2f &vel_offset, const Vector3f &pos_offset, const Vector3f &pos_cov);
        bool get_estimates(uint32_t &n_samples, Vector3f &offsets, Vector3f &variances);
        bool get_composite_gaussian_density(float &gaussian_density);
        void set_not_initialised(void) { ofs_ekf.reset_ms = 0; };
        bool is_initialised(void) { return ofs_ekf.reset_ms != 0; };

    private:
        const AP_NavTerrain &frontend;
        struct ekf_struct {
            Matrix3f P; // state covariance matrix
            Vector3f X; // state vector of NED offsets (m)
            uint32_t reset_ms; // last time the filter was reset
            Vector2f vel_offset; // NE velocity offset applied during the prediction step (m/s)
            float aspd_offset; // true airspeed offset applied during the prediction step (m/s)
            float gaussian_density_sum; // sum of gaussian probabilities
            uint16_t residual_count; // number of range residuals and gaussian points summed
        } ofs_ekf;

        float gaussian_density(const float &innovation, const float &innovation_variance);

        bool projected_rangefinder(const Location &loc, const Matrix3f &Tnb, float h_amsl, float &range, const Location &median_loc);
    };

    class GaussianSumFilter {
    public:
        GaussianSumFilter(AP_NavTerrain &_frontend, uint16_t _n_estimators, uint8_t _ekf_lane) :
            frontend(_frontend),
            ekf_lane(_ekf_lane),
            n_estimators(_n_estimators) {}

        bool init(void);
        void update(uint32_t time_ms, float dt, float range_measured, Quaternion &quat_in,
                    Location &loc_in, const Location &median_loc);
        uint32_t get_offsets_and_variances(Vector2f &vel_ofs, Vector3f &pos_ofs, Vector3f &pos_ofs_variance);
        void initialise(uint32_t time_ms, Vector3f &offsets, Vector3f &variances);

    private:
        const AP_NavTerrain &frontend;
        const uint8_t ekf_lane;
        const uint16_t n_estimators;
        float time_step;
        float *cdf_sigma_points; // array of sigma points with uniform probability density
        Vector2f vel_offset_avg; // NE velocity offset averaged across the last major sample period
        uint32_t reset_time_ms;
        uint8_t rng_sample_count; // number of range finder samples processed since the last GSF update
        uint32_t major_update_time_ms; // last time the GSF state and variance estimates were updated
        Vector2f get_velocity_offset_sample(const uint16_t instance, const float radius);
        float *weights;
        Vector3f states; // NED position offset (m)
        Vector3f state_variances; // NED position offset variance (m**2)
        OffsetEstimator *offset_estimators;
        void resample_states(uint32_t time_ms);
        void init_CDF_sigma_points(void);
        float randn_fast(void);

        Vector3f *all_variances;
        Vector3f *all_states;
        bool *done_update;
        bool *estimate_is_valid;
    };

    GaussianSumFilter *gsf[MAX_CORES];
};
