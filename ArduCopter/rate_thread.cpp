#include "Copter.h"

#if AP_INERTIALSENSOR_RATE_LOOP_WINDOW_ENABLED && FRAME_CONFIG != HELI_FRAME
#include <AP_HAL/CondMutex.h>

#pragma GCC optimize("O2")

/*************************************************************
 *  Attitude Rate controller thread
 ****************************************************************/

#define DIV_ROUND_INT(x, d) ((x + d/2) / d)

uint8_t Copter::calc_gyro_decimation(uint16_t gyro_decimation, uint16_t rate_hz)
{
    return uint8_t(DIV_ROUND_INT(ins.get_raw_gyro_rate_hz() / gyro_decimation, rate_hz));
}

//#define RATE_LOOP_TIMING_DEBUG
/*
  thread for rate control
*/
void Copter::rate_controller_thread()
{
    uint8_t target_rate_decimation = constrain_int16(g2.att_decimation.get(), 1, DIV_ROUND_INT(ins.get_raw_gyro_rate_hz(), 400));
    uint8_t rate_decimation = target_rate_decimation;
    uint32_t rate_loop_count = 0;
    uint32_t prev_loop_count = 0;

    HAL_CondMutex cmutex;
    ins.set_rate_loop_mutex(&cmutex);
    hal.rcout->set_dshot_rate(SRV_Channels::get_dshot_rate(), ins.get_raw_gyro_rate_hz() / rate_decimation);

    uint32_t last_run_us = AP_HAL::micros();
    float max_dt = 0.0;
    float min_dt = 1.0;
    uint32_t now_ms = AP_HAL::millis();
    uint32_t last_rate_check_ms = 0;
    uint32_t last_rate_increase_ms = 0;
#if HAL_LOGGING_ENABLED
    uint32_t last_rtdt_log_ms = now_ms;
#endif
    uint32_t last_notch_sample_ms = now_ms;
    bool was_using_rate_thread = false;
    bool notify_fixed_rate_active = true;
    bool was_armed = false;
    uint32_t running_slow = 0;
#ifdef RATE_LOOP_TIMING_DEBUG
    uint32_t gyro_sample_time_us = 0;
    uint32_t rate_controller_time_us = 0;
    uint32_t motor_output_us = 0;
    uint32_t log_output_us = 0;
    uint32_t ctrl_output_us = 0;
    uint32_t timing_count = 0;
    uint32_t last_timing_msg_us = 0;
#endif

    // run the filters at half the gyro rate
    uint8_t filter_rate_decimate = 2;
    uint8_t log_fast_rate_decimate = calc_gyro_decimation(rate_decimation, 1000);   // 1Khz
#if HAL_LOGGING_ENABLED
    uint8_t log_med_rate_decimate = calc_gyro_decimation(rate_decimation, 10);      // 10Hz
    uint8_t log_loop_count = 0;
#endif
    uint8_t main_loop_rate_decimate = calc_gyro_decimation(rate_decimation, AP::scheduler().get_filtered_loop_rate_hz());
    uint8_t main_loop_count = 0;
    uint8_t filter_loop_count = 0;

    while (true) {

#ifdef RATE_LOOP_TIMING_DEBUG
        uint32_t rate_now_us = AP_HAL::micros();
#endif

        // allow changing option at runtime
        if (get_fast_rate_type() == FastRateType::FAST_RATE_DISABLED || ap.motor_test) {
            using_rate_thread = false;
            if (was_using_rate_thread) {
                rate_controller_set_rates(calc_gyro_decimation(1, AP::scheduler().get_filtered_loop_rate_hz()), false);
                ins.set_rate_decimation(0);
                hal.rcout->force_trigger_groups(false);
                was_using_rate_thread = false;
            }
            hal.scheduler->delay_microseconds(500);
            last_run_us = AP_HAL::micros();
            continue;
        }

        // set up rate thread requirements
        using_rate_thread = true;
        hal.rcout->force_trigger_groups(true);
        ins.set_rate_decimation(rate_decimation);

        // wait for an IMU sample
        Vector3f gyro;
        ins.get_next_gyro_sample(gyro);

#ifdef RATE_LOOP_TIMING_DEBUG
        gyro_sample_time_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

        // we must use multiples of the actual sensor rate
        const float sensor_dt = 1.0f * rate_decimation / ins.get_raw_gyro_rate_hz();
        const uint32_t now_us = AP_HAL::micros();
        const uint32_t dt_us = now_us - last_run_us;
        const float dt = dt_us * 1.0e-6;
        last_run_us = now_us;

        max_dt = MAX(dt, max_dt);
        min_dt = MIN(dt, min_dt);

#if HAL_LOGGING_ENABLED
// @LoggerMessage: RTDT
// @Description: Attitude controller time deltas
// @Field: TimeUS: Time since system startup
// @Field: dt: current time delta
// @Field: dtAvg: current time delta average
// @Field: dtMax: Max time delta since last log output
// @Field: dtMin: Min time delta since last log output

        if (now_ms - last_rtdt_log_ms >= 10) {    // 100 Hz
            AP::logger().WriteStreaming("RTDT", "TimeUS,dt,dtAvg,dtMax,dtMin", "Qffff",
                                                AP_HAL::micros64(),
                                                dt, sensor_dt, max_dt, min_dt);
            max_dt = sensor_dt;
            min_dt = sensor_dt;
            last_rtdt_log_ms = now_ms;
        }
#endif

        motors->set_dt(sensor_dt);
        // check if we are falling behind
        if (ins.get_num_gyro_samples() > 2) {
            running_slow++;
        } else if (running_slow > 0) {
            running_slow--;
        }
        if (AP::scheduler().get_extra_loop_us() == 0) {
            rate_loop_count++;
        }

        // run the rate controller on all available samples
        // it is important not to drop samples otherwise the filtering will be fubar
        // there is no need to output to the motors more than once for every batch of samples
        attitude_control->rate_controller_run_dt(sensor_dt, gyro + ahrs.get_gyro_drift());

#ifdef RATE_LOOP_TIMING_DEBUG
        rate_controller_time_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

        // immediately output the new motor values
        if (main_loop_count++ >= main_loop_rate_decimate) {
            main_loop_count = 0;
        }
        motors_output(main_loop_count == 0);

        // process filter updates
        if (filter_loop_count++ >= filter_rate_decimate) {
            filter_loop_count = 0;
            rate_controller_filter_update();
        }

#ifdef RATE_LOOP_TIMING_DEBUG
        motor_output_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

#if HAL_LOGGING_ENABLED
        // fast logging output
        if (should_log(MASK_LOG_ATTITUDE_FAST)) {
            if (log_fast_rate_decimate > 0 && log_loop_count++ >= log_fast_rate_decimate) {
                log_loop_count = 0;
                rate_controller_log_update();
            }
        } else if (should_log(MASK_LOG_ATTITUDE_MED)) {
            if (log_med_rate_decimate > 0 && log_loop_count++ >= log_med_rate_decimate) {
                log_loop_count = 0;
                rate_controller_log_update();
            }
        }
#else
        (void)log_fast_rate_decimate;
#endif

#ifdef RATE_LOOP_TIMING_DEBUG
        log_output_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();
#endif

        now_ms = AP_HAL::millis();

        // make sure we have the latest target rate
        target_rate_decimation = constrain_int16(g2.att_decimation.get(), 1, DIV_ROUND_INT(ins.get_raw_gyro_rate_hz(), 400));
        if (now_ms - last_notch_sample_ms >= 1000 || !was_using_rate_thread) {
            // update the PID notch sample rate at 1Hz if we are
            // enabled at runtime
            last_notch_sample_ms = now_ms;
            attitude_control->set_notch_sample_rate(1.0 / sensor_dt);
        }

        // interlock for printing fixed rate active
        if (was_armed != motors->armed()) {
            notify_fixed_rate_active = !was_armed;
            was_armed = motors->armed();
        }

        // Once armed, switch to the fast rate if configured to do so
        if ((rate_decimation != target_rate_decimation || notify_fixed_rate_active)
            && ((get_fast_rate_type() == FastRateType::FAST_RATE_FIXED_ARMED && motors->armed())
                || get_fast_rate_type() == FastRateType::FAST_RATE_FIXED)) {
            rate_decimation = target_rate_decimation;
            log_fast_rate_decimate = rate_controller_set_rates(rate_decimation, false);
            notify_fixed_rate_active = false;
        }

        // check that the CPU is not pegged, if it is drop the attitude rate
        if (now_ms - last_rate_check_ms >= 100
            && (get_fast_rate_type() == FastRateType::FAST_RATE_DYNAMIC
                || (get_fast_rate_type() == FastRateType::FAST_RATE_FIXED_ARMED && !motors->armed())
                || target_rate_decimation > rate_decimation)) {
            last_rate_check_ms = now_ms;
            const uint32_t att_rate = ins.get_raw_gyro_rate_hz()/rate_decimation;
            if (running_slow > 5 || AP::scheduler().get_extra_loop_us() > 0
#if HAL_LOGGING_ENABLED
                || AP::logger().in_log_download()
#endif
                || target_rate_decimation > rate_decimation) {
                const uint8_t new_rate_decimation = MAX(rate_decimation + 1, target_rate_decimation);
                const uint32_t new_attitude_rate = ins.get_raw_gyro_rate_hz() / new_rate_decimation;
                if (new_attitude_rate > AP::scheduler().get_filtered_loop_rate_hz()) {
                    rate_decimation = new_rate_decimation;
                    log_fast_rate_decimate = rate_controller_set_rates(rate_decimation, true);
                    prev_loop_count = rate_loop_count;
                    rate_loop_count = 0;
                    running_slow = 0;
                }
            } else if (rate_decimation > target_rate_decimation && rate_loop_count > att_rate/10 // ensure 100ms worth of good readings
                && (prev_loop_count > att_rate/10   // ensure there was 100ms worth of good readings at the higher rate
                    || prev_loop_count == 0         // last rate was actually a lower rate so keep going quickly
                    || now_ms - last_rate_increase_ms >= 10000)) { // every 10s retry
                rate_decimation = rate_decimation - 1;

                log_fast_rate_decimate = rate_controller_set_rates(rate_decimation, false);
                prev_loop_count = 0;
                rate_loop_count = 0;
                last_rate_increase_ms = now_ms;
            }
        }

#ifdef RATE_LOOP_TIMING_DEBUG
        timing_count++;
        ctrl_output_us += AP_HAL::micros() - rate_now_us;
        rate_now_us = AP_HAL::micros();

        if (rate_now_us - last_timing_msg_us > 1e6) {
            hal.console->printf("Rate loop timing: gyro=%uus, rate=%uus, motors=%uus, log=%uus, ctrl=%uus\n",
                                unsigned(gyro_sample_time_us/timing_count), unsigned(rate_controller_time_us/timing_count),
                                unsigned(motor_output_us/timing_count), unsigned(log_output_us/timing_count), unsigned(ctrl_output_us/timing_count));
            last_timing_msg_us = rate_now_us;
            timing_count = 0;
            gyro_sample_time_us = rate_controller_time_us = motor_output_us = log_output_us = ctrl_output_us = 0;
        }
#endif

        was_using_rate_thread = true;
    }
}

/*
  update rate controller filters. on an H7 this is about 30us
*/
void Copter::rate_controller_filter_update()
{
    // update the frontend center frequencies of notch filters
    for (auto &notch : ins.harmonic_notches) {
        update_dynamic_notch(notch);
    }

    // this copies backend data to the frontend and updates the notches
    ins.update_backend_filters();
}

/*
  update rate controller rates and return the logging rate
*/
uint8_t Copter::rate_controller_set_rates(uint8_t rate_decimation, bool warn_cpu_high)
{
    const uint32_t attitude_rate = ins.get_raw_gyro_rate_hz() / rate_decimation;
    attitude_control->set_notch_sample_rate(attitude_rate);
    hal.rcout->set_dshot_rate(SRV_Channels::get_dshot_rate(), attitude_rate);
    motors->set_dt(1.0f / attitude_rate);
    gcs().send_text(warn_cpu_high ? MAV_SEVERITY_WARNING : MAV_SEVERITY_INFO,
                    "Attitude CPU %s, rate set to %uHz",
                    warn_cpu_high ? "high" : "normal", (unsigned) attitude_rate);
#if HAL_LOGGING_ENABLED
    if (attitude_rate > 1000) {
        return calc_gyro_decimation(rate_decimation, 1000);
    } else {
        return calc_gyro_decimation(rate_decimation, AP::scheduler().get_filtered_loop_rate_hz());
    }
#endif
    return 0;
}

/*
  log only those items that are updated at the rate loop rate
 */
void Copter::rate_controller_log_update()
{
#if HAL_LOGGING_ENABLED
    if (!copter.flightmode->logs_attitude()) {
        Log_Write_Rate();
        Log_Write_PIDS(); // only logs if PIDS bitmask is set
    }
#if AP_INERTIALSENSOR_HARMONICNOTCH_ENABLED
    if (should_log(MASK_LOG_FTN_FAST)) {
        AP::ins().write_notch_log_messages();
    }
#endif
#endif
}

#endif // AP_INERTIALSENSOR_RATE_LOOP_WINDOW_ENABLED
