/*
  position estimate correction using rangefinder terrain matching
 */
#include "AP_NavTerrain.h"
#include <AP_DAL/AP_DAL.h>
#include <AP_AHRS/AP_AHRS.h>
#include <AP_Terrain/AP_Terrain.h>
#include <AP_NavEKF3/AP_NavEKF3.h>
#include <AP_Param/AP_Param.h>
#include <AP_Logger/AP_Logger.h>
#include <AP_Common/sorting.h>
#include <AP_Relay/AP_Relay.h>
#include <RC_Channel/RC_Channel.h>
#include <GCS_MAVLink/GCS.h>

extern const AP_HAL::HAL &hal;

#define SITL_OR_REPLAY (CONFIG_HAL_BOARD == HAL_BOARD_SITL || APM_BUILD_TYPE(APM_BUILD_Replay))

const AP_Param::GroupInfo AP_NavTerrain::var_info[] = {

    // @Param: ENABLE
    // @DisplayName: Enable terrain dead reckoning
    // @Description: This enables dead reckoning using terrain data and a rangefinder
    // @Values: 0:Disabled, 1:Enabled
    // @User: Advanced
    AP_GROUPINFO_FLAGS("ENABLE", 1, AP_NavTerrain, enable, 1, AP_PARAM_FLAG_ENABLE),

    // @Param: DEBUG
    // @DisplayName: Debug
    AP_GROUPINFO("DEBUG", 4, AP_NavTerrain, debug, 1),

    // @Param: MINRNG
    // @DisplayName: minimum rangefinder to use
    AP_GROUPINFO("MINRNG", 9, AP_NavTerrain, min_range, 30),

    // @Param: ADSB_RT
    // @DisplayName: Rate to send ADSB_VEHICLE messages
    // @Description: Sets rate in Hz for ADSB_VEHICLE messages with GPS location
    AP_GROUPINFO("ADSB_RT", 15, AP_NavTerrain, adsb_rate, 2),

    // @Param: GPS_RLY
    // @DisplayName: Relay to use for GPS enable/disable
    // @Description: Sets a relay number to use for in-flight GPS enable/disable. When this relay is enabled the GPS will be disabled (show as no lock, whiile still reporting position)
    AP_GROUPINFO("GPS_RLY", 16, AP_NavTerrain, gps_relay, 0),

    // @Param: NEST
    // @DisplayName: Number of EKF estimators
    // @Description: Number of EKF estimators
    AP_GROUPINFO("NEST", 20, AP_NavTerrain, n_estimators, 650),

    // @Param: NLANES
    // @DisplayName: Number of EKF lanes to run
    // @Description: Number of EKF lanes to run
    AP_GROUPINFO("NLANES", 21, AP_NavTerrain, _n_lanes, 3),

    // @Param: RNG_DT
    // @DisplayName: Minimum range finder sample window time duration (msec)
    // @Description: This is the minimum time duration of the sample window used to gather range finder data that is then processed to provide a downsampled range measurement for the terrain navigation algorithm. If EK3_TNAV_OPTIONS bit 0 is set then the actual sampling period available to the range finder will be EK3_TNAV_RNG_DT - EK3_TNAV_SLEW_MS. The minimum distance travelled per sample window is set by EK3_TNAV_RNG_DD.
    AP_GROUPINFO("RNG_DT", 22, AP_NavTerrain, rng_sample_time_msec, 1000),

    // 23, was GSF_DT

    // 24, was V_DRIFT

    // @Param: V_SIGMA
    // @DisplayName: Velocity offset 1-sigma uncertainty (m/s)
    // @Description: This parameter sets the assumed 1-sigma uncertainty in velocity in the North and East direction.
    AP_GROUPINFO("V_SIGMA", 25, AP_NavTerrain, vel_offset_stddev, 1.0f),

    // @Param: V_ALPHA
    // @DisplayName: Velocity offset learning coefficient
    // @Description: This parameter specifies the faction of velocity error estimated from the change in position correction used to correct the velocity.
    AP_GROUPINFO("V_ALPHA", 26, AP_NavTerrain, vel_offset_alpha, 0.5f),

    // @Param: A_SIGMA
    // @DisplayName: Airspeed offset 1-sigma uncertainty (m/s)
    // @Description: This parameter sets the assumed 1-sigma uncertainty in speed along the vehicle heading vector.
    AP_GROUPINFO("A_SIGMA", 27, AP_NavTerrain, aspd_offset_stddev, 0.5f),

    // @Param: ERR_MUL
    // @DisplayName: Position offset error multiplier
    // @Description: This parameter is used by the particle filter to scale up the amount of position uncertainty assumed when resampling the random draw trajectory offsets. Larger values make the terrain navigation less accurate but more robust to unexpected dead reckoning errors.
    AP_GROUPINFO("ERR_MUL", 28, AP_NavTerrain, pos_error_mul, 1.0f),

    // @Param: H_DRIFT
    // @DisplayName: Height drift rate (m/s)
    // @Description: This parameter sets the 1-sigma rate of drift assumed for the vehicles height datum relative to the terrain. Thsi should be set to a value that allows for drift in barometric pressure altitude due to sensor thermal stability and weather changes.
    AP_GROUPINFO("H_DRIFT", 29, AP_NavTerrain, hgt_drift_rate, 0.05f),

    // @Param: R_GATE
    // @DisplayName: Range gate size (Sigma)
    // @Description: This parameter sets the number of standard deviations applied to range finder innovation consistency check.
    AP_GROUPINFO("R_GATE", 30, AP_NavTerrain, rng_gate_size, 3.0f),

    // @Param: OPTIONS
    // @DisplayName: Behaviour Changes
    // @Description: When bit 0 is set the payload mount function controlled by the MNT parameters will be used to roll stabilise the range finder. When not doing terrain aiding, the range finder will be roll stabilised using the servo mount function to point down. When doing terrain aiding, the range finder will be roll stabilised to point down, left and right of track by the angle specified by EK3_TNAV_RLLSTEP. Refer to EK3_TNAV_MNT_ID for further details.
    // @Bitmask: 0:RangeFinderRollServo
    AP_GROUPINFO("OPTIONS", 31, AP_NavTerrain, options, 0),

    // @Param: RLLSTEP
    // @DisplayName: Range finder roll offset step size (deg)
    // @Description: This parameter sets the size of the alternating range finder roll offset to either side of track used when doing terrain aiding with bit position 0 of EK3_TNAV_OPTIONS set.
    AP_GROUPINFO("RLLSTEP", 32, AP_NavTerrain, roll_step_deg, 30),

    // @Param: MNT_ID
    // @DisplayName: Range finder payload mount index
    // @Description: This sets the index of payload mount used for range finder roll stabilisation. The selected mount index must have the corresponding MNT parameters set where x = EK3_TNAV_MNT_ID+1, MNTx_TYPE=1, MNTx_DEFLT_MODE=1, MNTx_ROLL_MIN=-4500, MNTx_ROLL_MAX=4500, MNTx_NEUTRAL_X=0, MNTx_STAB_ROLL=1, MNTx_STAB_TILT=0, MNTx_STAB_PAN=0. If only one mount instance is available, then this parameter must be set to 0. The range finder roll servo function must be set to 8 and the pwm TRIM, MIN and MAX values set such that the range finder is level, rolled left 45 deg and rolled right 45 deg respectively. When testing this function in SITL, use EK3_TNAV_MNT_ID=0, SERVO6_FUNCTION=8, SERVO6_MAX=2000, SERVO6_MIN=1000, SERVO6_TRIM=1500, SERVO6_REVERSED=0.
    // @Values: 0:First Mount,1:Second Mount
    AP_GROUPINFO("MNT_ID", 33, AP_NavTerrain, mount_index, 0),

    // @Param: SLEW_MS
    // @DisplayName: Range finder roll offset step settling time (msec)
    // @Description: Set to the time the range finder mount roll servo requires to achieve a step change in roll angle. Range finder measurements will be flagged as invalid and not used for EK3_TNAV_SLEW_MS msec after each roll step event.
    AP_GROUPINFO("SLEW_MS", 34, AP_NavTerrain, roll_slew_time_msec, 100),

    // @Param: RNG_DD
    // @DisplayName: Minimum range finder sample window distance delta (m)
    // @Description: This is the minimum distance travelled across the sample window used to gather range finder data that is then processed to provide a downsampled range measurement for the terrain navigation algorithm. The minimum time duration for the sample window is set by EK3_TNAV_RNG_DT.
    AP_GROUPINFO("RNG_DD", 35, AP_NavTerrain, rng_sample_distance_m, 5),

    // @Param: N_MEAS
    // @DisplayName: Number of downsampled range finder measurements per correction
    // @Description: This parameter sets the number of downsampled range finder measurements used for each position correction update.
    AP_GROUPINFO("N_MEAS", 36, AP_NavTerrain, n_samples_required, 30),

    // @Param: V_LIMIT
    // @DisplayName: Velocity offset limit (m/s)
    // @Description: This parameter sets a circular limit applied to the horizontal velocity offset correction.
    AP_GROUPINFO("V_LIMIT", 37, AP_NavTerrain, velocity_offset_limit, 2.5f),

    // @Param: V_DRIFT
    // @DisplayName: Horizontal velocity drift rate (m/s/s)
    // @Description: This parameter sets the maximum rate of change of the horizontal velocity offset correction. This should be set to a value that allows for wind variation when not using optical flow.
    AP_GROUPINFO("V_DRIFT", 38, AP_NavTerrain, hvel_drift_rate, 0.08f),

    // @Param: HERR_TH
    // @DisplayName: Horizontal position error threshold (m)
    // @Description: When the reported terrain aided navigation 1-sigma uncertainty exceeds this value, the amount of correction applied to the dead reckoning solution is reduced.
    AP_GROUPINFO("HERR_TH", 39, AP_NavTerrain, herr_threshold, 50.0f),

    // 40 was RADMAX

    // @Param: SD_MAX
    // @DisplayName: Maximum standard deviation in particle distribution
    // @Description: The standard distribution used for distribution of particules is limited to less than this value
    AP_GROUPINFO("SD_MAX", 41, AP_NavTerrain, sd_max, 200.0),

    // @Param: VSCALE
    // @DisplayName: Variance scale factor
    // @Description: A scale factor applied to the learned standard deviation before re-sampling
    AP_GROUPINFO("VSCALE", 42, AP_NavTerrain, var_scale, 0.65),
    
    AP_GROUPEND
};

/*
  optionally discard 2 outlier values in error function
 */
#define NAV_DISCARD_OUTLIERS 0

struct test_sample {
    Vector2f ofs_NE;
    Vector2f vel_NE;
    uint32_t base_time_ms;
};

AP_NavTerrain::AP_NavTerrain(NavEKF3 &_ekf3) :
    ekf3(_ekf3),
    dal(AP::dal())
{
    AP_Param::setup_object_defaults(this, var_info);
}

void AP_NavTerrain::init(void)
{
    if (enable.get() <= 0) {
        return;
    }
    const auto r = dal.rangefinder();
    if (r == nullptr) {
        return;
    }
    rng = r->get_backend(0);
    if (rng == nullptr) {
        return;
    }
    if (rng->orientation() != ROTATION_PITCH_270) {
        rng = nullptr;
        return;
    }

    n_lanes = MIN(_n_lanes, ekf3.activeCores());

    for (uint8_t i=0; i<n_lanes; i++) {
        gather[i].ekf_lane = i;
        gather[i].sample_pending.range = -1;
        gather[i].restart_terrain_estimation = true;
        gsf[i] = new GaussianSumFilter(*this, n_estimators, i);
        if (gsf[i] == nullptr || !gsf[i]->init()) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Failed to allocate GSF[%u]", unsigned(i));
            rng = nullptr;
            delete gsf[i];
            gsf[i] = nullptr;
            return;
        }
    }

    
#if !APM_BUILD_TYPE(APM_BUILD_Replay)
    if (!hal.scheduler->thread_create(FUNCTOR_BIND_MEMBER(&AP_NavTerrain::update_thread, void), "NavTerrain", 8192,
                                      AP_HAL::Scheduler::PRIORITY_IO, -1)) {
        GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Failed to start NavTerrain thread");
        rng = nullptr;
    }
#endif

    GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "Init NavTerrain");

#if SITL_OR_REPLAY
    unlink("err.dat");
    unlink("adelta.dat");
#endif
}

/*
  periodic update
 */
void AP_NavTerrain::update(void)
{
    if (enable.get() <= 0) {
        for (uint8_t i=0; i<n_lanes; i++) {
            gather[i].samples.clear();
        }
        return;
    }
    if (rng != nullptr) {
#if !APM_BUILD_TYPE(APM_BUILD_Replay)
        if (option_set(Option::ROLL_STABILIZED)) {
            // we are using a roll stabilised range finder
            bool all_lanes_using_gps = true;
            for (uint8_t i=0; i<n_lanes; i++) {
                all_lanes_using_gps &= ekf3.using_gps(i);
            }
        }
#endif
        for (uint8_t i=0; i<n_lanes; i++) {
            const uint8_t lane = (i+last_range_lane+1) % n_lanes;
            if (gsf[lane] != nullptr) {

                gather_data(gather[lane]);
#if APM_BUILD_TYPE(APM_BUILD_Replay)
                update_process(gather[lane]);
#endif
            }
        }
        set_median();
    }
}

void AP_NavTerrain::gather_data(Gather &g)
{
    // zero correction if GPS is active
    if (ekf3.using_gps(g.ekf_lane)) {
        Vector2f offset;
        Location loc;
        if (ekf3.using_gps(g.ekf_lane) && ekf3.getDeadReckonLLH(g.ekf_lane, loc, offset)) {
            EK3_correction c {};
            c.ofs_NE.xy() = -offset;
            AP::ahrs().set_pos_correction(g.ekf_lane, g.correction, false);
            g.correction = c;
        }
        if (!g.last_using_gps) {
            GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TNAV[%u] using GPS", unsigned(g.ekf_lane));
            g.last_using_gps = true;
        }
        g.samples.clear();
    } else if (g.last_using_gps) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "TNAV[%u] non-GPS mode", unsigned(g.ekf_lane));
        g.last_using_gps = false;
        g.restart_terrain_estimation = true;
    }

    const uint32_t sample_ms = rng->sample_ms();
    if (sample_ms == last_range_ms) {
        // note that this is not per-lane. This gives us different data per gsf
        return;
    }
    const auto rng_status = rng->status();
    if (rng_status != AP_DAL_RangeFinder::Status::Good) {
        if (debug > 3) {
            GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "rng status %d", int(rng_status));
        }
        return;
    }
    const float dist = rng->distance_cm() * 0.01;
    last_range_ms = sample_ms;
    last_range_lane = g.ekf_lane;
    const uint32_t now_ms = dal.millis();

    // we keep the highest distance over a 1s period
    if (dist > g.sample_pending.range) {
        // Use the velocity integral dead reckoning position to ensure a smooth trajectory
        // that doesn't have position jumps.
        Vector2f offset;
        if (!ekf3.getDeadReckonLLH(g.ekf_lane, g.sample_pending.ek3_loc, offset)) {
            if (debug > 2) {
                GCS_SEND_TEXT(MAV_SEVERITY_DEBUG, "ek3 dr fail %d", int(g.ekf_lane));
            }
            return;
        }
        AP::ahrs().get_location_EKF2(g.sample_pending.ek2_loc);
        g.sample_pending.gps_loc = dal.gps().location();
        ekf3.getQuaternion(g.ekf_lane, g.sample_pending.quat);

        g.sample_pending.range = dist;
        g.sample_pending.sample_ms = sample_ms;
    }
    bool distance_is_sufficient = true;
    bool have_location = false;
    Location loc;
    if (AP::ahrs().get_location_EKF3_corrected(loc)) {
        have_location = true;
        const float distance_travelled = rng_sample_start_loc.get_distance(loc);
        distance_is_sufficient = distance_travelled >= (float)MAX(rng_sample_distance_m, 0);
    }
    if (now_ms - g.last_sample_ms >= (uint32_t)rng_sample_time_msec &&
        distance_is_sufficient &&
        g.sample_pending.range > min_range) {
        if (have_location) {
            rng_sample_start_loc = loc;
        }
        // push a sample to ring buffer
        g.samples.push_force(g.sample_pending);
        g.last_sample_ms = now_ms;
        g.sample_pending.range = -1;
    }
    log_positions(g);

#if SITL_OR_REPLAY
    Matrix3f Tnb;
    ekf3.getRotationBodyToNED(-1,Tnb);
    const auto &gps_loc = dal.gps().location();
    float alt_amsl;
    auto *terrain = AP::terrain();
    if (terrain &&
        terrain->projected_height_amsl(gps_loc, Tnb, dist, alt_amsl)) {
        AP::logger().WriteStreaming("NTGA",
                                    "TimeUS,C,Lat,Lng,Alt,TAlt",  // labels
                                    "s#DUmm",    // units
                                    "F---00",    // mults
                                    "QBLLff",    // fmt
                                    dal.micros64(),
                                    DAL_CORE(g.ekf_lane),
                                    gps_loc.lat,
                                    gps_loc.lng,
                                    gps_loc.alt*0.01,
                                    alt_amsl);
    }
#endif

    if (gps_relay > 0) {
        AP_Relay *relay = AP::relay();
        if (relay != nullptr) {
            const bool enabled = relay->enabled(gps_relay-1) && relay->get(gps_relay-1);
            if (enabled != last_gps_relay) {
                rc().run_aux_function(RC_Channel::AUX_FUNC::GPS_DISABLE,
                                      enabled?RC_Channel::AuxSwitchPos::HIGH:RC_Channel::AuxSwitchPos::LOW,
                                      RC_Channel::AuxFuncTriggerSource::BUTTON);
                last_gps_relay = enabled;
            }
        }
    }
}

void AP_NavTerrain::log_positions(Gather &g)
{
    uint32_t now = dal.millis();
    // log at max 10Hz
    if (now - g.last_log_ms < 100) {
        return;
    }
    g.last_log_ms = now;
    Location cloc, ek3_loc, ek3_dr_loc;
    Vector2f offset;

    ekf3.getLLH(g.ekf_lane, ek3_loc);
    ekf3.getDeadReckonLLH(g.ekf_lane, ek3_dr_loc, offset);

    cloc = ek3_dr_loc;
    Vector2f ofs = g.correction.ofs_NE.xy();
    const float dt = int32_t(now - g.correction.vel_start_ms) * 0.001;
    ofs += g.correction.vel_NE * dt;
    cloc.offset(ofs.x, ofs.y);

    Vector3f cvel;
    ekf3.getVelNED(g.ekf_lane, cvel);
    cvel.xy() += g.correction.vel_NE;

    if (g.last_using_gps) {
        cloc = ek3_loc;
    }

    AP::logger().WriteStreaming("NTUC",
                                "TimeUS,C,Lat,Lng,Alt",  // labels
                                "s#DUm",    // units
                                "F---0",    // mults
                                "QBLLf",    // fmt
                                dal.micros64(),
                                DAL_CORE(g.ekf_lane),
                                ek3_loc.lat,
                                ek3_loc.lng,
                                ek3_loc.alt*0.01);

    AP::logger().WriteStreaming("NTDR",
                                "TimeUS,C,Lat,Lng,Alt",  // labels
                                "s#DUm",    // units
                                "F---0",    // mults
                                "QBLLf",    // fmt
                                dal.micros64(),
                                DAL_CORE(g.ekf_lane),
                                ek3_dr_loc.lat,
                                ek3_dr_loc.lng,
                                ek3_dr_loc.alt*0.01);

    AP::logger().WriteStreaming("NTCO",
                                "TimeUS,C,Lat,Lng,Alt,TCx,TCy",  // labels
                                "s#DUmmm",    // units
                                "F---000",    // mults
                                "QBLLfff",    // fmt
                                dal.micros64(),
                                DAL_CORE(g.ekf_lane),
                                cloc.lat,
                                cloc.lng,
                                cloc.alt*0.01,
                                g.correction.ofs_NE.x,
                                g.correction.ofs_NE.y);

    AP::logger().WriteStreaming("NTCV",
                                "TimeUS,C,VN,VE",  // labels
                                "s#nn",    // units
                                "F---",    // mults
                                "QBff",    // fmt
                                dal.micros64(),
                                DAL_CORE(g.ekf_lane),
                                cvel.x,
                                cvel.y);

    if (is_positive(adsb_rate) && now - last_adsb_out_ms >= 1000.0/adsb_rate) {
        const auto &gps = dal.gps();
        const auto &gps_loc = gps.location();
        Location mloc;
        AP::ahrs().get_location_EKF3_corrected(mloc);

        mavlink_adsb_vehicle_t packet {};
        packet.ICAO_address = gcs().sysid_this_mav();
        packet.lat = gps_loc.lat;
        packet.lon = gps_loc.lng;
        packet.altitude = gps_loc.alt*10;
        const auto &vel = gps.velocity();
        packet.heading = uint16_t(wrap_360(degrees(vel.xy().angle())) * 100);
        packet.hor_velocity = uint16_t(vel.length()*100);
        packet.ver_velocity = int16_t(vel.z * -100);
        packet.flags = ADSB_FLAGS_VALID_COORDS | ADSB_FLAGS_VALID_ALTITUDE | ADSB_FLAGS_VALID_HEADING | ADSB_FLAGS_VALID_VELOCITY | ADSB_FLAGS_SIMULATED;
        packet.squawk = gcs().sysid_this_mav();
        packet.altitude_type = ADSB_ALTITUDE_TYPE_GEOMETRIC;
        strncpy(packet.callsign, "GPS", 4);
        packet.emitter_type = ADSB_EMITTER_TYPE_NO_INFO;
        packet.tslc = uint8_t((now - last_adsb_out_ms)*0.001);

        gcs().send_to_active_channels(MAVLINK_MSG_ID_ADSB_VEHICLE, (const char *)&packet);

        gcs().send_named_float("GPS_ERR", mloc.get_distance(gps_loc));

        last_adsb_out_ms = now;
    }
}

/*
  process one sample with GSF
 */
void AP_NavTerrain::process_sample(Gather &g, Sample &sample)
{
    if (g.restart_terrain_estimation) {
        g.restart_terrain_estimation = false;
        /*
          we recently had GPS available, re-init GSF
          set horizontal offset using difference between ekf3 dead reckon and 'truth' location provided
          the offset returned by ekf3.getDeadReckonLLH is 'dead reckoned' - 'truth' position
        */
        Vector2f ofs;
        Location loc;
        ekf3.getDeadReckonLLH(g.ekf_lane, loc, ofs);
        Vector3f pos_offsets = Vector3f(-ofs.x, -ofs.y, 0.0f);
        // when initialising initial state uncertainty assume we enter dead reckoning with a uncertainty of 5m from the last good GPS plus a 15m error growth
        // vertical offset uncertainty is the combination of vehicle and other errors so add a margin - TODO scale with height
        Vector3f pos_offset_variances = Vector3f(sq(5.0f) + sq(15.0f), sq(5.0f) + sq(15.0f), sq(10.0f));
        gsf[g.ekf_lane]->initialise(sample.sample_ms, pos_offsets, pos_offset_variances);
        g.last_gsf_sample_ms = 0;
        AP::ahrs().set_terrain_nav_status(true);
        return;
    }
    
    auto *terrain = AP::terrain();
    const uint32_t terrain_loopups = terrain->get_total_lookups();
    const uint32_t failed_loopups = terrain->get_failed_lookups();
    const uint32_t now_ms = dal.millis();

    const float dt = g.last_gsf_sample_ms==0 ? 0 : 0.001 * (sample.sample_ms - g.last_gsf_sample_ms);
    g.last_gsf_sample_ms = sample.sample_ms;

    Location median_pos;
    Vector3f median_vel;
    median_posvel(median_pos, median_vel);

    // calling this runs all instances of the lower level EKF estimators followed by the GSF
    gsf[g.ekf_lane]->update(sample.sample_ms, dt, sample.range, sample.quat, sample.ek3_loc, median_pos);

    // keep average lookups per update for timing
    const uint32_t sample_lookups = terrain->get_total_lookups() - terrain_loopups;
    lookups_per_update = 0.9 * lookups_per_update + 0.1 * sample_lookups;

    const uint32_t dt_ms = dal.millis() - now_ms;
    if (debug > 1) {
        GCS_SEND_TEXT(MAV_SEVERITY_INFO, "process[%u] %u/%u dt=%.2f", unsigned(g.ekf_lane), unsigned(dt_ms), unsigned(sample_lookups), dt);
    }

    AP::logger().WriteStreaming("NTST",
                                "TimeUS,C,Dt,DtMS,Lk,LkF",
                                "s#----",    // units
                                "F-----",    // mults
                                "QBfIIf",    // fmt
                                dal.micros64(),
                                DAL_CORE(g.ekf_lane),
                                dt,
                                dt_ms,
                                sample_lookups,
                                lookups_per_update);

    AP::logger().WriteStreaming("NTLK",
                                "TimeUS,C,LkTot,LkFail",
                                "s#--",    // units
                                "F---",    // mults
                                "QBII",    // fmt
                                dal.micros64(),
                                DAL_CORE(g.ekf_lane),
                                terrain_loopups,
                                failed_loopups);
    

#if SITL_OR_REPLAY
    // log the 'ground truth' lidar trace path
    // the range finder sample is the same for each ekf lane so only need to calculate this once
    if (g.ekf_lane == 0) {
        Vector3f v{0,0,sample.range};
        Matrix3f Tns; // matrix that rotates a vector from sensor to navigation frame
        sample.quat.rotation_matrix(Tns);
        v = Tns * v;

        if (v.z > 0) {
            // calculate ground position where rangefinder hits the ground
            Location loc2 = dal.gps().location();
            loc2.offset(v.x,v.y);
            loc2.alt -= v.z*100;

            // get the predicted height from the terrain database for the 'truth' location
            float terrain_height;
            if (!terrain->height_amsl(loc2, terrain_height)) {
                terrain_height = 0.01f * (float)loc2.alt;
            }

            AP::logger().WriteStreaming("NTRG",
                                        "TimeUS,Lat,Lng,Alt,TAlt,RDist",  // labels
                                        "sDUmmm",    // units
                                        "F---00",    // mults
                                        "QLLfff",    // fmt
                                        dal.micros64(),
                                        loc2.lat,
                                        loc2.lng,
                                        loc2.alt*0.01,
                                        terrain_height,
                                        sample.range);
        }
    }
#endif

    // we need to keep checking for an updated NED position offset estimate
    // where offset = truth - dead reckoned
    Vector3f pos_ofs, pos_ofs_variance;
    Vector2f vel_ofs;
    const uint32_t time_stamp_ms = gsf[g.ekf_lane]->get_offsets_and_variances(vel_ofs, pos_ofs, pos_ofs_variance);
    if (time_stamp_ms > g.last_gsf_major_update_ms) {
        g.last_gsf_major_update_ms = time_stamp_ms;

        // Update the AHRS position offset state factoring in the uncertainty of the terrain navigation algorithms offset estimate
        const float state_variance = sq(vel_offset_stddev * (float)n_samples_required * 0.001f * (float)rng_sample_time_msec);
        const Vector2f innovation_variance = Vector2f(state_variance + MAX(pos_ofs_variance.x - sq(herr_threshold), 0.0f), state_variance + MAX(pos_ofs_variance.x - sq(herr_threshold), 0.0f));
        g.correction.ofs_NE.x -= (g.correction.ofs_NE.x - pos_ofs.x) * (state_variance / innovation_variance.x);
        g.correction.ofs_NE.y -= (g.correction.ofs_NE.y - pos_ofs.y) * (state_variance / innovation_variance.y);

        // update the AHRS velocity offset state
        g.correction.vel_NE = vel_ofs;

        g.correction.vel_start_ms = time_stamp_ms;

        set_ahrs_pos_correction(g);

        if (debug > 0) {
            GCS_SEND_TEXT(MAV_SEVERITY_NOTICE, "GSF[%u] SD(%.1f,%.1f) VOFS(%.1f,%.1f)",
                          unsigned(g.ekf_lane),
                          sqrt(pos_ofs_variance.x), sqrt(pos_ofs_variance.y), vel_ofs.x, vel_ofs.y);
        }

        AP::logger().WriteStreaming("NTK3",
                                    "TimeUS,C,PN,PE,PD,SPN,SPE,SPD,VN,VE",
                                    "s#--------",    // units
                                    "F---------",    // mults
                                    "QBffffffff",    // fmt
                                    dal.micros64(),
                                    DAL_CORE(g.ekf_lane),
                                    pos_ofs.x,
                                    pos_ofs.y,
                                    pos_ofs.z,
                                    sqrtf(pos_ofs_variance.x),
                                    sqrtf(pos_ofs_variance.y),
                                    sqrtf(pos_ofs_variance.z),
                                    vel_ofs.x,
                                    vel_ofs.y
            );
    }
}

/*
  this is called by the low priority thread to process data gathered by the gather_data() function
 */
void AP_NavTerrain::update_process(Gather &g)
{
    if (enable.get() <= 0 || gsf[g.ekf_lane] == nullptr) {
        AP::ahrs().set_terrain_nav_status(false);
        return;
    }

    // always give 250ms for other threads to run
    uint32_t process_delay = 250;

#if SITL_OR_REPLAY
    // add replay processing delay to approximately match H7 processing speed
    const float ms_per_lookup = 650.0 / 110079.0;
    process_delay += uint32_t(ms_per_lookup * lookups_per_update);
#endif

    Sample sample;
    auto *terrain = AP::terrain();
    if (AP::dal().millis() - g.last_process_ms >= process_delay &&
        terrain != nullptr &&
        g.samples.pop(sample)) {
        process_sample(g, sample);
        g.last_process_ms = AP::dal().millis();
    }
}

void AP_NavTerrain::update_thread(void)
{
    while (true) {
        hal.scheduler->delay(1);
        for (uint8_t i=0; i<n_lanes; i++) {
            update_process(gather[i]);
        }
    }
}

/*
  set AHRS pos correction for the active EKF lanes
 */
void AP_NavTerrain::set_ahrs_pos_correction(Gather &g)
{
    uint32_t now_ms = dal.millis();

    // get first lane corrected position and corrected velocity
    Location ek3_dr_loc;
    Vector2f offset;
    ekf3.getDeadReckonLLH(g.ekf_lane, ek3_dr_loc, offset);
    Vector2f ofs = g.correction.ofs_NE.xy();
    const float dt = int32_t(now_ms - g.correction.vel_start_ms) * 0.001;
    ofs += g.correction.vel_NE * dt;

    Location cloc = ek3_dr_loc;
    cloc.offset(ofs.x, ofs.y);

    Vector3f vel_NED;
    ekf3.getVelNED(g.ekf_lane, vel_NED);
    vel_NED.xy() += g.correction.vel_NE;

    for (uint8_t i=0; i<ekf3.activeCores(); i++) {
        if (i != g.ekf_lane && i < n_lanes) {
            // only fill in our lane or unhandled lanes
            continue;
        }
        Location ek3_loc;
        Vector3f ek3_vel;
        ekf3.getDeadReckonLLH(i, ek3_loc, offset);
        ekf3.getVelNED(i, ek3_vel);

        EK3_correction c {};
        c.ofs_NE.xy() = ek3_loc.get_distance_NE(cloc);
        c.vel_NE = vel_NED.xy() - ek3_vel.xy();
        c.vel_start_ms = now_ms;

        AP::ahrs().set_pos_correction(i, c, true);
    }
}

/*
  calculate median position and velocity
 */
void AP_NavTerrain::median_posvel(Location &pos, Vector3f &vel)
{
    Vector3f ofs[MAX_CORES];
    Vector3f velc[MAX_CORES];
    Location origin;
    uint32_t now_ms = dal.millis();
    if (!ekf3.getOriginLLH(origin)) {
        return;
    }
    for (uint8_t i=0; i<n_lanes; i++) {
        auto &g = gather[i];
        Location dr_loc;
        Vector2f offset;
        ekf3.getDeadReckonLLH(i, dr_loc, offset);
        Vector2f tofs = g.correction.ofs_NE.xy();
        const float dt = int32_t(now_ms - g.correction.vel_start_ms) * 0.001;
        tofs += g.correction.vel_NE * dt;
        Location cloc = dr_loc;
        cloc.offset(tofs.x, tofs.y);
        ofs[i] = origin.get_distance_NED(cloc);
        Vector3f velned;
        ekf3.getVelNED(i, velned);
        velc[i] = velned;
        velc[i].xy() += g.correction.vel_NE;
    }

    switch (n_lanes) {
    case 1: {
        pos = origin;
        pos.offset(ofs[0].x, ofs[0].y);
        pos.alt -= ofs[0].z*100;
        vel = velc[0];
        break;
    }
    case 2: {
        pos = origin;
        pos.offset((ofs[0].x+ofs[1].x)*0.5,
                   (ofs[1].y+ofs[1].y)*0.5);
        pos.alt -= (ofs[0].z+ofs[1].z)*0.5*100;
        vel = (velc[0]+velc[1])*0.5;
        break;
    }
    case 3: {
        pos = origin;
        pos.offset(median3f(ofs[0].x,ofs[1].x,ofs[2].x),
                   median3f(ofs[0].y,ofs[1].y,ofs[2].y));
        pos.alt -= median3f(ofs[0].z,ofs[1].z,ofs[2].z)*100;
        vel.x = median3f(velc[0].x, velc[1].x, velc[2].x);
        vel.y = median3f(velc[0].y, velc[1].y, velc[2].y);
        vel.z = median3f(velc[0].z, velc[1].z, velc[2].z);
        break;
    }
    }
}

void AP_NavTerrain::set_median(void)
{
    Location pos;
    Vector3f vel;
    median_posvel(pos, vel);

    const uint32_t now_ms = dal.millis();
    if (now_ms - last_median_log_ms >= 40 && pos.initialised()) {
        last_median_log_ms = now_ms;
        AP::logger().WriteStreaming("NTMD",
                                    "TimeUS,C,Lat,Lng,Alt",  // labels
                                    "s#DUm",    // units
                                    "F---0",    // mults
                                    "QBLLf",    // fmt
                                    dal.micros64(),
                                    DAL_CORE(0),
                                    pos.lat,
                                    pos.lng,
                                    pos.alt*0.01);

        AP::logger().WriteStreaming("NTMV",
                                    "TimeUS,C,VN,VE,VD",  // labels
                                    "s#nnn",    // units
                                    "F----",    // mults
                                    "QBfff",    // fmt
                                    dal.micros64(),
                                    DAL_CORE(0),
                                    vel.x,
                                    vel.y,
                                    vel.z);
    }
}
