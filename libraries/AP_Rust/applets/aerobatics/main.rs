// SPDX-License-Identifier: GPL-3.0-or-later
//! Experimental single-aircraft numeric-schedule runner.
#![no_std]
#![no_main]
extern crate alloc;
mod controller;
#[cfg(ap_rust_aerobatics_faults)]
#[path = "../../tests/modules/aerobatics_faults.rs"]
mod faults;
mod logging;
mod maneuvers;
mod math;
mod parameters;
mod path;
mod primitives;
mod schedule;
mod support;

use alloc::string::String;
use ap_rust_sdk::{
    enums::{RcChannelAuxFunc, SrvChannelFunction},
    Context, Error, Result,
};
use controller::{Configuration, Controller, Input, Step};
use parameters::Parameters;
use path::{Evaluation, Path};
use support::{message, retry, Text};
ap_rust_sdk::module!(run);

struct Loaded {
    id: i32,
    name: String,
    path: Path,
}

fn observation(c: &Context<'_>) -> Result<Input> {
    let start = c.time_us();
    let input = Input {
        now: (c.time_us() / 1000) as u32 as f32 * 0.001,
        pos: retry(c, || c.ahrs_get_relative_position_ned_origin())?.ok_or(Error::Unavailable)?,
        gyro: retry(c, || c.ahrs_get_gyro())?,
        velocity: retry(c, || c.ahrs_get_velocity_ned())?.ok_or(Error::Unavailable)?,
        airspeed: retry(c, || c.ahrs_airspeed_eas())?.ok_or(Error::Unavailable)?,
        eas2tas: retry(c, || c.ahrs_get_eas2tas())?,
        attitude: retry(c, || c.ahrs_get_quaternion())?.ok_or(Error::Unavailable)?,
        throttle: retry(c, || {
            c.srv_channels_get_output_scaled(SrvChannelFunction(70))
        })?,
    };
    if c.time_us().saturating_sub(start) > 10000 {
        return Err(Error::Busy);
    }
    input.validate()?;
    Ok(input)
}

fn load(c: &Context<'_>, id: i32, p: Configuration, input: Input, width: f32) -> Result<Loaded> {
    if !(1..=255).contains(&id) {
        return Err(Error::Invalid);
    }
    let mut found = None;
    for directory in ["APM/scripts/", "scripts/", ""] {
        let filename = Text::<32>::new(format_args!("{directory}trick{id}.txt"))?;
        if let Some(schedule) = schedule::Schedule::load(c, filename.as_str())? {
            found = Some(schedule);
            break;
        }
    }
    let schedule = found.ok_or(Error::Unavailable)?;
    let path = schedule.build(
        c,
        width,
        Evaluation {
            speed: controller::target_speed(c, input, p)?,
            entry_rate: p.entry_rate,
            announce: true,
        },
    )?;
    Ok(Loaded {
        id,
        name: schedule.name,
        path,
    })
}

fn release(c: &Context<'_>) -> Result {
    c.vehicle_nav_scripting_enable(255)?;
    Ok(())
}

fn run(c: &Context<'_>) -> Result {
    let params = match Parameters::new(c) {
        Ok(params) => params,
        Err(error) => {
            message(c, 3, format_args!("Aerobatics parameters: {:?}", error))?;
            return Err(error);
        }
    };
    message(c, 6, format_args!("Loaded plane_aerobatics Rust"))?;
    if params.trick_count > 0 {
        message(c, 6, format_args!("Enabled {} tricks", params.trick_count))?;
    }
    let mut loaded: Option<Loaded> = None;
    let mut active: Option<Controller> = None;
    let mut channel = None;
    let mut last_selection = None;
    let mut last_action = if params.trick_count > 0 {
        retry(c, || {
            c.rc_get_aux_cached(RcChannelAuxFunc(params.t(3)? as i32))
        })
        .unwrap_or(None)
    } else {
        None
    };
    let mut owned = false;
    // Lua's path_var.count is global, not reset when a new trick starts.
    let mut count = 0;
    let mut last_named_float = 0.0;
    let mut last_error = 0;
    let mut last_command = 0;
    #[cfg(ap_rust_aerobatics_faults)]
    let mut faults = faults::Faults::default();
    loop {
        let previous_selection = last_selection;
        let previous_action = last_action;
        let mut starting = false;
        let mut stepped = false;
        let mut delivering = false;
        let result = (|| -> Result {
            c.sleep_ms(25)?;
            let tick_started = c.time_us();
            if active.is_some() && tick_started.saturating_sub(last_command) > 1000000 {
                message(c, 3, format_args!("Aerobatics: control timeout"))?;
                return Err(Error::Invalid);
            }
            if owned && active.is_none() {
                release(c)?;
                owned = false;
            }
            if params.trick_count == 0 {
                return Ok(());
            }
            let action = retry(c, || {
                c.rc_get_aux_cached(RcChannelAuxFunc(params.t(3)? as i32))
            })?;
            if last_action.is_none() && action.is_some() {
                // No RC sample at boot/reload: adopt the first valid switch
                // position, never interpret reception itself as execute.
                last_action = action;
            }
            if action == Some(0) && active.is_some() {
                count = active.take().unwrap().count;
                last_selection = None;
                release(c)?;
                owned = false;
                message(c, 1, format_args!("Trick aborted"))?;
                return Ok(());
            }
            let mode = retry(c, || c.vehicle_get_mode())?;
            // Mission and dual-aircraft support are not silently approximated.
            if mode == 10 || params.a(34)? as u32 & 4 != 0 {
                if active.take().is_some() {
                    release(c)?;
                    owned = false;
                }
                return Ok(());
            }
            if channel.is_none() {
                channel = retry(c, || {
                    c.rc_find_channel_for_option(RcChannelAuxFunc(params.t(2)? as i32))
                })?;
            }
            let selection = match channel.as_ref() {
                Some(ch) => {
                    let norm = retry(c, || ch.norm_input_ignore_trim(c))?;
                    let n = c.libm_floorf(
                        params.t(4)? * math::constrain(0.5 * (norm + 1.0), 0.0, 0.999) + 1.0,
                    )? as usize;
                    if n > 0 && n <= params.trick_count {
                        n
                    } else {
                        0
                    }
                }
                None => 0,
            };
            if active.is_none()
                && selection > 0
                && ((action == Some(1) && last_selection != Some(selection))
                    || action != last_action)
            {
                starting = true;
                last_selection = Some(selection);
                last_action = action;
                let id = params.t(5 * selection)? as i32;
                let p = retry(c, || params.configuration())?;
                p.validate()?;
                if loaded.as_ref().map(|l| l.id) != Some(id) {
                    // Keep only one composed schedule within the module heap.
                    loaded = None;
                    match load(c, id, p, observation(c)?, params.a(22)?) {
                        Ok(l) => loaded = Some(l),
                        Err(Error::Stopped) => return Err(Error::Stopped),
                        Err(Error::Busy) => return Err(Error::Busy),
                        Err(error) => {
                            message(c, 3, format_args!("Trick {id} load: {:?}", error))?;
                            return Ok(());
                        }
                    }
                }
                let task = loaded.as_mut().unwrap();
                if action == Some(1) {
                    message(
                        c,
                        6,
                        format_args!("Trick {selection} selected ({})", task.name),
                    )?;
                } else if action == Some(2) {
                    let input = observation(c)?;
                    let yaw = math::wrap_180(
                        c,
                        math::deg(c.libm_atan2f(input.velocity.y, input.velocity.x)?),
                    )?;
                    // Construct/validate before enabling control of the vehicle.
                    let controller = Controller::start(c, &mut task.path, input, p, yaw, count)?;
                    if !retry(c, || c.vehicle_nav_scripting_enable(mode))? {
                        message(c, 1, format_args!("Tricks not available in mode"))?;
                        return Ok(());
                    }
                    owned = true;
                    last_command = c.time_us();
                    message(
                        c,
                        6,
                        format_args!("Trick {selection} started ({})", task.name),
                    )?;
                    last_selection = None;
                    active = Some(controller);
                    return Ok(());
                }
            }
            if let Some(controller) = active.as_mut() {
                let task = loaded.as_mut().ok_or(Error::Invalid)?;
                #[cfg(ap_rust_aerobatics_faults)]
                faults.check(c, params.t(9)? as u8, false, false)?;
                let p = retry(c, || params.configuration())?;
                let input = observation(c)?;
                if c.time_us().saturating_sub(last_command) > 1000000 {
                    return Err(Error::Invalid);
                }
                let start = c.time_us();
                stepped = true;
                let result = controller.step(c, &mut task.path, input, p);
                count = controller.count;
                let elapsed = c.time_us().saturating_sub(start);
                let (output, finish) = match result {
                    Ok(Step::Waiting) => return Ok(()),
                    Ok(Step::Finished) => (None, true),
                    Ok(Step::Command(o)) => (Some(o), false),
                    Ok(Step::AltitudeAbort(o)) => {
                        message(c, 0, format_args!("Too low altitude, aborting"))?;
                        (Some(o), true)
                    }
                    Err(Error::Stopped) => return Err(Error::Stopped),
                    Err(_) => {
                        message(c, 0, format_args!("Invalid path - aborting"))?;
                        (None, true)
                    }
                };
                if let Some(o) = output {
                    // An altitude abort is authoritative even when its final
                    // command is contended: do not defer release to next tick.
                    delivering = !finish;
                    #[cfg(ap_rust_aerobatics_faults)]
                    faults.check(c, params.t(9)? as u8, true, finish)?;
                    retry(c, || {
                        c.vehicle_set_target_throttle_rate_rpy(
                            o.throttle, o.rates.x, o.rates.y, o.rates.z,
                        )
                    })?;
                    last_command = c.time_us();
                    retry(c, || c.vehicle_set_rudder_offset(o.rudder, true))?;
                    delivering = false;
                    // Diagnostic I/O follows command delivery and never
                    // prevents delivery of an already computed command.
                    match logging::record(c, input, &o) {
                        Err(Error::Stopped) => return Err(Error::Stopped),
                        _ => (),
                    }
                    if o.path_rate_fallback {
                        message(c, 0, format_args!("path_rate_ef_rads: invalid"))?;
                    }
                    if input.now - last_named_float > 0.5 {
                        last_named_float = input.now;
                        let _ = c.gcs_send_named_float("PERR", o.position_error);
                    }
                    // Experimental timing record: controller only, excluding log I/O.
                    let _ = c.log_write(
                        "ARUN",
                        "Us,Total,Perr,T,Flags",
                        None,
                        &[
                            ap_rust_sdk::LogValue::U32(elapsed.min(u32::MAX as u64) as u32),
                            ap_rust_sdk::LogValue::U32(
                                c.time_us()
                                    .saturating_sub(tick_started)
                                    .min(u32::MAX as u64) as u32,
                            ),
                            ap_rust_sdk::LogValue::Float(o.position_error),
                            ap_rust_sdk::LogValue::Float(o.path_t),
                            ap_rust_sdk::LogValue::U8(
                                u8::from(o.path_rate_fallback)
                                    | (u8::from(o.lookahead_fallback) << 1),
                            ),
                        ],
                    );
                }
                if finish {
                    release(c)?;
                    owned = false;
                    message(c, 6, format_args!("Finishing {}!", task.name))?;
                    active = None;
                }
            }
            Ok(())
        })();
        if let Err(error) = result {
            if starting && matches!(error, Error::Busy | Error::Unavailable) && !owned {
                last_selection = previous_selection;
                last_action = previous_action;
            }
            if error == Error::Busy && (!stepped || delivering) {
                // A pre-step read or command delivery was contended. Never
                // repeat a partially completed step; the next tick uses fresh
                // observations. The watchdog above bounds delivery starvation.
                continue;
            }
            if let Some(controller) = active.take() {
                count = controller.count;
            }
            if owned && release(c).is_ok() {
                owned = false;
            }
            if error == Error::Stopped {
                // The host rejects bindings after stop. Plane's existing
                // one-second NavScript timeout is also Lua restart's backstop.
                return Err(error);
            }
            let now = c.time_us();
            if now.saturating_sub(last_error) > 1000000 {
                last_error = now;
                message(c, 3, format_args!("Aerobatics error {:?}", error))?;
            }
        }
    }
}
