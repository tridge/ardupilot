#include "AP_Stats.h"

#include <AP_Math/AP_Math.h>
#include <AP_RTC/AP_RTC.h>

const extern AP_HAL::HAL& hal;

// table of user settable parameters
const AP_Param::GroupInfo AP_Stats::var_info[] = {

    // @Param: _BOOTCNT
    // @DisplayName: Boot Count
    // @Description: Number of times board has been booted
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_BOOTCNT",    0, AP_Stats, params.bootcount, 0),

    // @Param: _FLTTIME
    // @DisplayName: Total FlightTime
    // @Description: Total FlightTime (seconds)
    // @Units: s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_FLTTIME",    1, AP_Stats, params.flttime, 0),

    // @Param: _RUNTIME
    // @DisplayName: Total RunTime
    // @Description: Total time autopilot has run
    // @Units: s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_RUNTIME",    2, AP_Stats, params.runtime, 0),

    // @Param: _RESET
    // @DisplayName: Statistics Reset Time
    // @Description: Seconds since January 1st 2016 (Unix epoch+1451606400) since statistics reset (set to 0 to reset statistics)
    // @Units: s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_RESET",    3, AP_Stats, params.reset, 1),

    // @Param: _RMFLTTIME1
    // @DisplayName: Remaining flight time before the first maintenance task is due (commonly an inspection)
    // @Description: This counter denotes the time remaining to next maintenance. If negative, maintenance is due. Used by GCS to show information to the user.
    // @Units: s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_RMFLTTIME1",    4, AP_Stats, params.rem_flttime1, 0),

    // @Param: _RMFLTTIME2
    // @DisplayName: Remaining flight time before the second maintenance task is due (commonly a service)
    // @Description: This counter denotes the time remaining to next maintenance. If negative, maintenance is due. Used by GCS to show information to the user.
    // @Units: s
    // @ReadOnly: True
    // @User: Standard
    AP_GROUPINFO("_RMFLTTIME2",    5, AP_Stats, params.rem_flttime2, 0),

    AP_GROUPEND
};

AP_Stats *AP_Stats::_singleton;

// constructor
AP_Stats::AP_Stats(void)
{
    AP_Param::setup_object_defaults(this, var_info);
    _singleton = this;
}

void AP_Stats::copy_variables_from_parameters()
{
    flttime = params.flttime;
    runtime = params.runtime;
    reset = params.reset;
    flttime_boot = flttime;
    rem_flttime1 = params.rem_flttime1;
    rem_flttime2 = params.rem_flttime2;
}

void AP_Stats::init()
{
    params.bootcount.set_and_save(params.bootcount+1);

    // initialise our variables from parameters:
    copy_variables_from_parameters();
}


void AP_Stats::flush()
{
    params.flttime.set_and_save_ifchanged(flttime);
    params.runtime.set_and_save_ifchanged(runtime);
    params.rem_flttime1.set_and_save_ifchanged(rem_flttime1);
    params.rem_flttime2.set_and_save_ifchanged(rem_flttime2);
}

void AP_Stats::update_flighttime()
{
    if (_flying_ms) {
        WITH_SEMAPHORE(sem);
        const uint32_t now = AP_HAL::millis();
        const uint32_t delta = (now - _flying_ms)/1000;
        flttime += delta;
        _flying_ms += delta*1000;
        rem_flttime1 -= delta;
        rem_flttime2 -= delta;
    }
}

void AP_Stats::update_runtime()
{
    const uint32_t now = AP_HAL::millis();
    const uint32_t delta = (now - _last_runtime_ms)/1000;
    runtime += delta;
    _last_runtime_ms += delta*1000;
}

void AP_Stats::update()
{
    WITH_SEMAPHORE(sem);
    const uint32_t now_ms = AP_HAL::millis();
    if (now_ms -  last_flush_ms > flush_interval_ms) {
        // If user changes the service times, update the internal variable
        if (params.rem_flttime1 != rem_flttime1){
            rem_flttime1 = params.rem_flttime1;
        }
        if (params.rem_flttime2 != rem_flttime2){
            rem_flttime2 = params.rem_flttime2;
        }
        update_flighttime();
        update_runtime();
        flush();
        last_flush_ms = now_ms;
    }
    const uint32_t params_reset = params.reset;
    if (params_reset != reset || params_reset == 0) {
        params.bootcount.set_and_save_ifchanged(params_reset == 0 ? 1 : 0);
        params.flttime.set_and_save_ifchanged(0);
        params.runtime.set_and_save_ifchanged(0);
        uint32_t system_clock = 0; // in seconds
#if AP_RTC_ENABLED
        uint64_t rtc_clock_us;
        if (AP::rtc().get_utc_usec(rtc_clock_us)) {
            system_clock = rtc_clock_us / 1000000;
            // can't store Unix seconds in a 32-bit float.  Change the
            // time base to Jan 1st 2016:
            system_clock -= 1451606400;
        }
#endif
        params.reset.set_and_save_ifchanged(system_clock);
        copy_variables_from_parameters();
    }

}

void AP_Stats::set_flying(const bool is_flying)
{
    if (is_flying) {
        if (!_flying_ms) {
            _flying_ms = AP_HAL::millis();
        }
    } else {
        update_flighttime();
        _flying_ms = 0;
    }
}

/*
  get time in flight since boot
 */
uint32_t AP_Stats::get_flight_time_s(void)
{
    update_flighttime();
    return flttime - flttime_boot;
}

AP_Stats *AP::stats(void)
{
    return AP_Stats::get_singleton();
}
