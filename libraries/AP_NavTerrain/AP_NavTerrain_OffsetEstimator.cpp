/*
  Methods based on statistical estimation theory used to estimate the position drift from a sequence
  of range finder measurements and pose estimates.
*/
#include "AP_NavTerrain.h"
#include <AP_DAL/AP_DAL.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_Logger/AP_Logger.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

#define SITL_OR_REPLAY (CONFIG_HAL_BOARD == HAL_BOARD_SITL || APM_BUILD_TYPE(APM_BUILD_Replay))

bool AP_NavTerrain::OffsetEstimator::get_estimates(uint32_t &n_samples, Vector3f &offsets, Vector3f &variances)
{
    if (ofs_ekf.residual_count == 0 || ofs_ekf.reset_ms == 0) {
        return false;
    }

    n_samples = ofs_ekf.residual_count;
    offsets = ofs_ekf.X;
    variances.x = ofs_ekf.P.a.x;
    variances.y = ofs_ekf.P.b.y;
    variances.z = ofs_ekf.P.c.z;
    return true;
}

void AP_NavTerrain::OffsetEstimator::reset(const uint32_t time_ms, const float aspd_offset, const Vector2f &vel_offset, const Vector3f &pos_offset, const Vector3f &pos_var)
{
    ofs_ekf.P.zero();
    ofs_ekf.P.a.x = pos_var.x;
    ofs_ekf.P.b.y = pos_var.y;
    ofs_ekf.P.c.z = pos_var.z;
    ofs_ekf.X = pos_offset;
    ofs_ekf.vel_offset = vel_offset;
    ofs_ekf.aspd_offset = aspd_offset;
    ofs_ekf.reset_ms = time_ms;
    ofs_ekf.gaussian_density_sum = 0.0f;
    ofs_ekf.residual_count = 0;
}

bool AP_NavTerrain::OffsetEstimator::projected_rangefinder(const Location &loc, const Matrix3f &Tbn, float hgt_amsl,
                                                           float &range_predicted, const Location &median_loc)
{
    auto *terrain = AP::terrain();
    if (terrain == nullptr) {
        return false;
    }

    return terrain->projected_rangefinder(loc, Tbn, hgt_amsl, range_predicted);
}

/*
  update offset estimator. Return false if we fail to lookup terrain data, so a retry may be worthwhile.
 */
bool AP_NavTerrain::OffsetEstimator::update(float dt, uint16_t instance, float range_measured, Quaternion &quat_in,
                                            Location &loc_in, const Location &median_loc)
{
    if (ofs_ekf.reset_ms == 0) {
        return true;
    }

    // state prediction - accumulate effect of velocity offset on position offset
    Matrix3f Tbn;
    quat_in.rotation_matrix(Tbn);
    Vector2f aspd_vel_offset = Vector2f(Tbn.a.x, Tbn.b.x);
    aspd_vel_offset.normalize();
    aspd_vel_offset *= ofs_ekf.aspd_offset;
    const Vector2f vel_offset_combined = ofs_ekf.vel_offset + aspd_vel_offset;
    ofs_ekf.X.x += dt * vel_offset_combined.x;
    ofs_ekf.X.y += dt * vel_offset_combined.y;

    // covariance prediction - grow height offset uncertainty at a rate of 3 m/min
    ofs_ekf.P.c.z += sq(dt * frontend.hgt_drift_rate);

    // fuse range finder measurement

    // evaluate observation Jacobian for range finder measurement wrt each state
    Vector3f H;

    if (Tbn.c.z < 0.5f) {
        // excessively tilted past 60 degrees from vertical so roll and pitch errors
        // and ground obstructions will cause excessive error
        return true;
    }

    // get predicted range for expected state vector
    Location loc = loc_in;
    loc.offset(ofs_ekf.X.x, ofs_ekf.X.y);
    const float hgt_amsl = 0.01f * (float)loc.alt - ofs_ekf.X.z;
    float range_predicted;
    if (!projected_rangefinder(loc, Tbn, hgt_amsl, range_predicted, median_loc)) {
        return false;
    }

    // range derivative wrt North position offset
    const float px_sigma = MAX(sqrtf(ofs_ekf.P[0][0]), 1.0f);
    loc = loc_in;
    loc.offset(ofs_ekf.X.x+px_sigma, ofs_ekf.X.y);
    float range_px_plus;
    if (!projected_rangefinder(loc, Tbn, hgt_amsl, range_px_plus, median_loc)) {
        return false;
    }
    loc = loc_in;
    loc.offset(ofs_ekf.X.x-px_sigma, ofs_ekf.X.y);
    float range_px_minus;
    if (!projected_rangefinder(loc, Tbn, hgt_amsl, range_px_minus, median_loc)) {
        return false;
    }
    H.x = (range_px_plus - range_px_minus)/(2.0f * px_sigma);

    // range derivative wrt East position offset
    const float py_sigma = MAX(sqrtf(ofs_ekf.P[1][1]), 1.0f);
    loc = loc_in;
    loc.offset(ofs_ekf.X.x, ofs_ekf.X.y+py_sigma);
    float range_py_plus;
    if (!projected_rangefinder(loc, Tbn, hgt_amsl, range_py_plus, median_loc)) {
        return false;
    }
    loc = loc_in;
    loc.offset(ofs_ekf.X.x, ofs_ekf.X.y-py_sigma);
    float range_py_minus;
    if (!projected_rangefinder(loc, Tbn, hgt_amsl, range_py_minus, median_loc)) {
        return false;
    }
    H.y = (range_py_plus - range_py_minus)/(2.0f * py_sigma);

    // range derivative wrt vertical position offset is approximated
    // assuming terrain gradient is small wrt range finder elevation angle
    H.z = -1.0f / Tbn.c.z;

    // define observation variance including effect of trees and other objects that can cause the range to read incorrectly
    const float R_OBS = sq(5.0f); // TODO set this adaptively based on range finder noise

    // calculate innovation variance S using S = H * P * H.T + R
    Vector3f PHT = ofs_ekf.P * H;
    float S = R_OBS + H*PHT;
    if (S < R_OBS) {
        // calculation is badly conditioned
        S = R_OBS;
    }

    // calculate innovation and check for consistency
    const float innovation = range_predicted - range_measured;
    const float test_ratio = sq(innovation) / S;
    if (test_ratio < sq(frontend.rng_gate_size)) {
        // only compute the gaussian density for samples that are used
        ofs_ekf.gaussian_density_sum += gaussian_density(innovation, S);
        ofs_ekf.residual_count++;

        // calculate Kalman gain K using K = (P * H.T) / S
        const float S_inv = 1.0f / S;
        Vector3f K = PHT * S_inv;

        // update covariance matrix using P_new = P - K * S * K.T
        for (uint16_t row=0; row<3; row++) {
            for (uint16_t col=0; col<3; col++) {
                ofs_ekf.P[row][col] = ofs_ekf.P[row][col] - K[row] * S * K[col];
            }
        }

        // stop diagonals becoming too small
        ofs_ekf.P[0][0] = MAX(ofs_ekf.P[0][0],0.0f);
        ofs_ekf.P[1][1] = MAX(ofs_ekf.P[1][1],0.0f);
        ofs_ekf.P[2][2] = MAX(ofs_ekf.P[2][2],0.0f);

        // Make symmetrical
        for (uint16_t row = 0; row < 3; row++) {
            // copy off diagonals
            for (uint16_t column = 0 ; column < row; column++) {
                const float temp = 0.5f * (ofs_ekf.P[row][column] + ofs_ekf.P[column][row]);
                ofs_ekf.P[row][column] = ofs_ekf.P[column][row] = temp;
            }
        }
        // update state vector
        for (uint16_t col=0; col<3; col++) {
            ofs_ekf.X[col] -= innovation * K[col];
        }

    }
#if SITL_OR_REPLAY
    auto &dal = AP::dal();
    if (instance < 9) {
        AP::logger().WriteStreaming("NTK1",
                                    "TimeUS,C,PN,PE,PD,RI,RIV,SPN,SPE,SPD",
                                    "s#--------",    // units
                                    "F---------",    // mults
                                    "QBffffffff",    // fmt
                                    dal.micros64(),
                                    DAL_CORE(instance),
                                    ofs_ekf.X[0],
                                    ofs_ekf.X[1],
                                    ofs_ekf.X[2],
                                    innovation,
                                    S,
                                    sqrtf(ofs_ekf.P[0][0]),
                                    sqrtf(ofs_ekf.P[1][1]),
                                    sqrtf(ofs_ekf.P[2][2])
                                    );
    }
#endif
    return true;
}

float AP_NavTerrain::OffsetEstimator::gaussian_density(const float &innovation, const float &innovation_variance)
{
    // convert from a normalised variance to a probability assuming a Gaussian distribution
    return expf(-0.5f * sq(innovation) / innovation_variance) / sqrtf(innovation_variance * M_2PI);
}

bool AP_NavTerrain::OffsetEstimator::get_composite_gaussian_density(float &gaussian_density)
{
    if (ofs_ekf.residual_count == 0 || ofs_ekf.reset_ms == 0) {
        return false;
    }
    gaussian_density = ofs_ekf.gaussian_density_sum / (float)ofs_ekf.residual_count;
    return true;
}

// implement a repeating 3x3 grid of velocity offsets with a defined radius
Vector2f AP_NavTerrain::GaussianSumFilter::get_velocity_offset_sample(const uint16_t instance, const float radius)
{
    Vector2f ret;
    const uint16_t index = instance%9;
    if (index == 0) {
        ret.zero();
        return ret;
    }
    const float angle = M_PI_4 * (float)(index);
    ret.x = radius * cosf(angle);
    ret.y = radius * sinf(angle);
    return ret;
}

void AP_NavTerrain::GaussianSumFilter::resample_states(uint32_t time_ms)
{
    // resample the position and velocity offsets
    Vector3f pos_offset;
    Vector2f vel_offset;
    float aspd_offset;
    for (uint16_t i=0; i<n_estimators; i++) {
        pos_offset.x = states.x + sqrtf(state_variances.x) * randn_fast() * frontend.pos_error_mul;
        pos_offset.y = states.y + sqrtf(state_variances.y) * randn_fast() * frontend.pos_error_mul;
        pos_offset.z = states.z;
        vel_offset.x = vel_offset_avg.x + frontend.vel_offset_stddev * randn_fast();
        vel_offset.y = vel_offset_avg.y + frontend.vel_offset_stddev * randn_fast();
        aspd_offset = frontend.aspd_offset_stddev * randn_fast();
        offset_estimators[i].reset(time_ms, aspd_offset, vel_offset, pos_offset, state_variances);
    }
}

void AP_NavTerrain::GaussianSumFilter::update(uint32_t time_ms, float dt, float range_measured, Quaternion &quat_in,
                                              Location &loc_in, const Location &median_loc)
{
    auto &dal = AP::dal();
    // run multiple estimators with different initial position and velocity offsets
    if (reset_time_ms == 0) {
        Vector3f pos_offset {};
        Vector3f pos_offset_variances = Vector3f(sq(5.0f) + sq(15.0f), sq(5.0f) + sq(15.0f), sq(10.0f));
        initialise(time_ms, pos_offset, pos_offset_variances);
        time_step = 0.0f;
        rng_sample_count = 0;
    } else {
        time_step = dt;
        rng_sample_count++;
    }

    /*
      run update on each estimator. We give each estimator multiple
      tries to allow for the terrain library to load blocks from the
      microSD
     */
    memset(done_update, 0, sizeof(bool)*n_estimators);
    uint32_t total_done = 0;
    for (uint8_t t=0; t<10; t++) {
        bool all_done = true;
        for (uint16_t instance=0; instance<n_estimators; instance++) {
            if (!done_update[instance]) {
                done_update[instance] = offset_estimators[instance].update(time_step, instance, range_measured, quat_in, loc_in, median_loc);
                if (done_update[instance]) {
                    total_done++;
                }
                if (!done_update[instance]) {
                    all_done = false;
                }
            }
        }
        if (all_done) {
            break;
        }
#if APM_BUILD_TYPE(APM_BUILD_Replay)
        // when under replay we don't have a thread re-filling the
        // terrain cache in the background. This simulates 5 updates
        // of the thread instead of a delay of 20ms
        auto *terrain = AP::terrain();
        if (terrain != nullptr) {
            for (uint8_t i=0; i<5; i++) {
                terrain->update();
            }
        }
#else
        hal.scheduler->delay(20);
#endif
    }

    /*
      if any still haven't updated then invalidate the particle
     */
    for (uint16_t instance=0; instance<n_estimators; instance++) {
        if (!done_update[instance]) {
            offset_estimators[instance].set_not_initialised();
        }
    }

    // Use a sum of gaussians to estimate offset from available trajectories every n_samples
    // and reset filters
    if (rng_sample_count >= (uint8_t)frontend.n_samples_required) {
        rng_sample_count=0;
        const float major_time_step = 0.001f * (float)(time_ms - reset_time_ms);
        reset_time_ms = time_ms;

        // calculate weights
        float weight_sum = 0.0f;
        for (uint16_t instance=0; instance<n_estimators; instance++) {
            if (offset_estimators[instance].get_composite_gaussian_density(weights[instance])) {
                weight_sum += weights[instance];
            } else {
                weights[instance] = 0.0f;
            }
        }
        if (weight_sum < FLT_EPSILON) {
            // we have not had any range samples that the individual EKF's could use so exit
            // and allow the point cloud to continue propagating
            return;
        }
        const float weight_sum_inv = 1.0f / weight_sum;
        for (uint16_t i=0; i<n_estimators; i++) {
            weights[i] *= weight_sum_inv;
        }

        // calculate states using a weighted average of all valid estimators
        const Vector3f states_prev = states;
        states.zero();
        float max_weight = FLT_MIN;
        float min_weight = FLT_MAX;
        for (uint16_t instance=0; instance<n_estimators; instance++) {
            uint32_t n_samples;
            Vector3f pos_offsets;
            Vector3f variances;
            if (offset_estimators[instance].get_estimates(n_samples, pos_offsets, variances)) {
                states += pos_offsets * weights[instance];
                all_variances[instance] = variances;
                all_states[instance] = pos_offsets;
                estimate_is_valid[instance] = true;
                if (weights[instance] > max_weight) {
                    max_weight = weights[instance];
                }
                if (weights[instance] < min_weight) {
                    min_weight = weights[instance];
                }
                if (instance < 9) {
                    AP::logger().WriteStreaming("NTK2",
                                                "TimeUS,C,PN,PE,PD,SPN,SPE,SPD,W",
                                                "s#-------",    // units
                                                "F--------",    // mults
                                                "QBfffffff",    // fmt
                                                dal.micros64(),
                                                DAL_CORE(instance),
                                                pos_offsets.x,
                                                pos_offsets.y,
                                                pos_offsets.z,
                                                sqrtf(variances.x),
                                                sqrtf(variances.y),
                                                sqrtf(variances.z),
                                                weights[instance]
                                                );
                }

            } else {
                estimate_is_valid[instance] = false;
            }

        }

        AP::logger().WriteStreaming("NTK4",
                                    "TimeUS,C,Wmin,Wmax,VN,VE",
                                    "s#----",    // units
                                    "F-----",    // mults
                                    "QBffff",    // fmt
                                    dal.micros64(),
                                    DAL_CORE(ekf_lane),
                                    min_weight,
                                    max_weight,
                                    vel_offset_avg.x,
                                    vel_offset_avg.y
                                    );

        // protect against insane values for major_time_step that would make velocity update excessively noisy
        // or cause /0 error
        if (major_time_step > (0.5f * 0.001f) * (float)frontend.rng_sample_time_msec * (float)frontend.n_samples_required) {
            Vector2f vel_offset_new = (states.xy() - states_prev.xy()) *  (1.0f / major_time_step);
            const Vector2f vel_offset_delta = vel_offset_new - vel_offset_avg;
            const float vel_offset_delta_length = vel_offset_delta.length();
            const float limit = frontend.hvel_drift_rate * major_time_step;
            if (vel_offset_delta_length > limit) {
                vel_offset_new = vel_offset_new * (limit / vel_offset_delta_length);
            }
            const float alpha = constrain_float(frontend.vel_offset_alpha, 0.0f, 1.0f);
            vel_offset_avg = vel_offset_avg * (1.0f - alpha) +  vel_offset_new * alpha;
            const float offset_length = vel_offset_avg.length();
            if (offset_length > frontend.velocity_offset_limit) {
                vel_offset_avg *= frontend.velocity_offset_limit / offset_length;
            }
        }

        // calculate variances using a weighted average of the deviation from the state estimate for all valid estimators
        state_variances.zero();
        for (uint16_t instance=0; instance<n_estimators; instance++) {
            if (estimate_is_valid[instance]) {
                state_variances.x += (all_variances[instance].x + sq(all_states[instance].x - states.x)) * weights[instance];
                state_variances.y += (all_variances[instance].y + sq(all_states[instance].y - states.y)) * weights[instance];
                state_variances.z += (all_variances[instance].z + sq(all_states[instance].z - states.z)) * weights[instance];
            }
        }

        const float max_variance = sq(frontend.sd_max);
        const float variance_scale = frontend.var_scale;
        state_variances.x = MIN(state_variances.x*variance_scale, max_variance);
        state_variances.y = MIN(state_variances.y*variance_scale, max_variance);
        state_variances.z = MIN(state_variances.z*variance_scale, max_variance);
        
        resample_states(time_ms);

        major_update_time_ms = time_ms;
    }
}

uint32_t AP_NavTerrain::GaussianSumFilter::get_offsets_and_variances(Vector2f &vel_ofs, Vector3f &pos_ofs, Vector3f &pos_ofs_variance)
{
    vel_ofs = vel_offset_avg;
    pos_ofs = states;
    pos_ofs_variance = state_variances;
    return major_update_time_ms;
}

// initialise the table of CDF sigma points contained in the cdf_sigma_points array
// such that the spacing between points is inversely proportional to the probability density
void AP_NavTerrain::GaussianSumFilter::init_CDF_sigma_points(void)
{
    const double p_min = 1.0 / (double)n_estimators;
    const double p_max = 1.0 - p_min;
    const double p_delta = (p_max - p_min) / (double)(n_estimators - 1);
    for (uint16_t i=0; i<n_estimators; i++) {
        const float p = (float)(p_min + p_delta * (double)i);
        cdf_sigma_points[i] = NormalCDFInverse(p);
    }
}

void AP_NavTerrain::GaussianSumFilter::initialise(uint32_t time_ms, Vector3f &offsets, Vector3f &variances)
{
    reset_time_ms = time_ms;
    major_update_time_ms = 0;
    state_variances = variances;
    states = offsets;
    for (uint16_t i=0; i<n_estimators; i++) {
        offset_estimators[i].reset(time_ms, 0.0f, Vector2f(0.0f, 0.0f), Vector3f(0.0f, 0.0f, 0.0f), state_variances);
    }
    vel_offset_avg.zero();
    init_CDF_sigma_points();
    resample_states(time_ms);
    rng_sample_count = 0;
}

float AP_NavTerrain::GaussianSumFilter::randn_fast(void)
{
    return cdf_sigma_points[get_random16() % n_estimators];
}

/*
  allocated arrays
 */
bool AP_NavTerrain::GaussianSumFilter::init(void)
{
    offset_estimators = (OffsetEstimator *)malloc(sizeof(OffsetEstimator)*n_estimators);
    if (offset_estimators == nullptr) {
        goto fail;
    }
    // run constructors on each estimator
    for (uint16_t i=0; i<n_estimators; i++) {
        new (&offset_estimators[i]) OffsetEstimator(frontend);
    }
    cdf_sigma_points = new float[n_estimators];
    if (cdf_sigma_points == nullptr) {
        goto fail;
    }
    all_variances = new Vector3f[n_estimators];
    if (all_variances == nullptr) {
        goto fail;
    }
    all_states = new Vector3f[n_estimators];
    if (all_states == nullptr) {
        goto fail;
    }
    estimate_is_valid = new bool[n_estimators];
    if (estimate_is_valid == nullptr) {
        goto fail;
    }
    weights = new float[n_estimators];
    if (weights == nullptr) {
        goto fail;
    }
    done_update = new bool[n_estimators];
    if (done_update == nullptr) {
        goto fail;
    }

    return true;

fail:
    delete[] offset_estimators;
    delete[] cdf_sigma_points;
    delete[] all_variances;
    delete[] all_states;
    delete[] estimate_is_valid;
    delete[] weights;
    delete[] done_update;
    return false;
}
