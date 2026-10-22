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
mod mission;
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
    args: [f32; 4],
    name: String,
    path: Path,
}

fn observation(c: &Context<'_>) -> Result<Input> {
    let start = c.time_us();
    let input = Input {
        now: (c.time_us() / 1000) as u32 as f32 * 0.001,
        pos: retry(c, || c.ahrs().get_relative_position_ned_origin())?.ok_or(Error::Unavailable)?,
        gyro: retry(c, || c.ahrs().get_gyro())?,
        velocity: retry(c, || c.ahrs().get_velocity_ned())?.ok_or(Error::Unavailable)?,
        airspeed: retry(c, || c.ahrs().airspeed_eas())?.ok_or(Error::Unavailable)?,
        eas2tas: retry(c, || c.ahrs().get_eas2_tas())?,
        attitude: retry(c, || c.ahrs().get_quaternion())?.ok_or(Error::Unavailable)?,
        throttle: retry(c, || {
            c.srv_channels().get_output_scaled(SrvChannelFunction(70))
        })?,
    };
    if c.time_us().saturating_sub(start) > 10000 {
        return Err(Error::Busy);
    }
    input.validate()?;
    Ok(input)
}

fn load(
    c: &Context<'_>,
    id: i32,
    args: [f32; 4],
    p: Configuration,
    input: Input,
    width: f32,
) -> Result<Loaded> {
    if !(1..=255).contains(&id) {
        return Err(Error::Invalid);
    }
    let eval = Evaluation {
        speed: controller::target_speed(c, input, p)?,
        entry_rate: p.entry_rate,
        announce: true,
    };
    if let Some((command, name)) = maneuvers::Command::builtin(id) {
        let mut text = String::new();
        text.try_reserve_exact(name.len())
            .map_err(|_| Error::NoMemory)?;
        text.push_str(name);
        return Ok(Loaded {
            id,
            args,
            name: text,
            path: maneuvers::build(c, command, args, width, eval)?,
        });
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
    // As in Lua's PathTask, file schedules already contain their arguments.
    let path = schedule.build(c, width, eval)?;
    Ok(Loaded {
        id,
        args,
        name: schedule.name,
        path,
    })
}

#[derive(Clone, Copy)]
enum Owner {
    Rc(u8),
    Mission(u16),
}

fn release(c: &Context<'_>, owner: Owner) -> Result {
    match owner {
        Owner::Mission(id) => c.vehicle().nav_script_time_done(id)?,
        // A mode transition has already revoked RC control. Do not disable a
        // newly started AUTO item while cleaning up the old RC controller.
        Owner::Rc(mode) if c.vehicle().get_mode()? == mode => {
            c.vehicle().nav_scripting_enable(255)?;
        }
        Owner::Rc(_) => (),
    }
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
            c.rc().get_aux_cached(RcChannelAuxFunc(params.t(3)? as i32))
        })
        .unwrap_or(None)
    } else {
        None
    };
    let mut owner: Option<Owner> = None;
    let mut last_mission_id = None;
    let mut last_preload = None;
    // Lua's path_var.count is global, not reset when a new trick starts.
    let mut count = 0;
    let mut last_named_float = 0.0;
    let mut last_error = 0;
    let mut last_command = 0;
    let mut language_selected = false;
    #[cfg(ap_rust_aerobatics_faults)]
    let mut faults = faults::Faults::default();
    loop {
        let previous_selection = last_selection;
        let previous_action = last_action;
        let mut starting = false;
        let mut starting_mission = false;
        let mut preloading = false;
        let mut attempted_trick = None;
        let mut stepped = false;
        let mut delivering = false;
        let result = (|| -> Result {
            c.sleep_ms(25)?;
            if params.a(35)? != 0.0 {
                language_selected = false;
                return Ok(());
            }
            if !language_selected {
                message(c, 6, format_args!("Aerobatics RUST selected"))?;
                language_selected = true;
            }
            let tick_started = c.time_us();
            if active.is_some() && tick_started.saturating_sub(last_command) > 1000000 {
                message(c, 3, format_args!("Aerobatics: control timeout"))?;
                return Err(Error::Invalid);
            }
            if let Some(o) = owner {
                if active.is_none() {
                    release(c, o)?;
                    owner = None;
                }
            }
            let mode = retry(c, || c.vehicle().get_mode())?;
            let mission_command = if mode == mission::AUTO {
                retry(c, || c.vehicle().nav_script_time())?
            } else {
                None
            };
            // Never deliver an old task's rates into a different mission item
            // or after a timeout/mode change has revoked its control.
            let stale = match owner {
                Some(Owner::Mission(id)) => mission_command.map(|m| m.0) != Some(id),
                Some(Owner::Rc(old_mode)) => old_mode != mode,
                None => false,
            };
            if stale {
                if let Some(controller) = active.take() {
                    count = controller.count;
                }
                if let Some(o) = owner {
                    release(c, o)?;
                    owner = None;
                }
            }
            // Dual-aircraft time synchronization is not implemented.
            if params.a(34)? as u32 & 4 != 0 {
                if let Some(o) = owner {
                    release(c, o)?;
                    owner = None;
                }
                active = None;
                return Ok(());
            }
            if mode == mission::AUTO {
                if let Some((id, command, arg1, arg2, arg3, arg4)) = mission_command {
                    if last_mission_id != Some(id) {
                        let args = [arg1, arg2, arg3 as f32, arg4 as f32];
                        let p = retry(c, || params.configuration())?;
                        p.validate()?;
                        let input = observation(c)?;
                        // As in Lua, attempt each item once. Only contention
                        // retries below undo this marker, not missing files.
                        last_mission_id = Some(id);
                        starting_mission = true;
                        attempted_trick = Some(command as i32);
                        if loaded.as_ref().map(|l| (l.id, l.args)) != Some((command as i32, args)) {
                            loaded = None;
                            loaded = Some(load(c, command as i32, args, p, input, params.a(22)?)?);
                        }
                        let course = math::deg(c.libm().atan2f(input.velocity.y, input.velocity.x)?);
                        let yaw = mission::heading(c, params.a(33)?, course)?;
                        let task = loaded.as_mut().unwrap();
                        let controller =
                            Controller::start(c, &mut task.path, observation(c)?, p, yaw, count)?;
                        // Loading/heading lookup may outlast a mission timeout.
                        if retry(c, || c.vehicle().get_mode())? != mission::AUTO
                            || retry(c, || c.vehicle().nav_script_time())?.map(|m| m.0) != Some(id)
                        {
                            message(c, 4, format_args!("Trick {command} skipped: item expired"))?;
                            return Ok(());
                        }
                        owner = Some(Owner::Mission(id));
                        last_command = c.time_us();
                        active = Some(controller);
                        message(c, 6, format_args!("Starting {}!", task.name))?;
                    }
                } else {
                    // Lua preloads the immediately following script item. This
                    // keeps SD reads/path construction outside the 1s watchdog.
                    let index = retry(c, || c.mission().get_current_nav_index())?;
                    if last_preload != Some(index) {
                        last_preload = Some(index);
                        preloading = true;
                        if let Some(next) = index.checked_add(1) {
                            if let Some(item) = retry(c, || c.mission().get_item(next))? {
                                if item.command == mission::NAV_SCRIPT_TIME {
                                    attempted_trick = Some(item.param1 as i32);
                                    let p = retry(c, || params.configuration())?;
                                    let args =
                                        [item.param3, item.param4, item.x as f32, item.y as f32];
                                    loaded = None;
                                    loaded = Some(load(
                                        c,
                                        item.param1 as i32,
                                        args,
                                        p,
                                        observation(c)?,
                                        params.a(22)?,
                                    )?);
                                }
                            }
                        }
                    }
                    return Ok(());
                }
            } else {
                if params.trick_count == 0 {
                    return Ok(());
                }
                let action = retry(c, || {
                    c.rc().get_aux_cached(RcChannelAuxFunc(params.t(3)? as i32))
                })?;
                if last_action.is_none() && action.is_some() {
                    // No RC sample at boot/reload: adopt the first valid switch
                    // position, never interpret reception itself as execute.
                    last_action = action;
                }
                if action == Some(0) && active.is_some() {
                    count = active.take().unwrap().count;
                    last_selection = None;
                    if let Some(o) = owner {
                        release(c, o)?;
                        owner = None;
                    }
                    message(c, 1, format_args!("Trick aborted"))?;
                    return Ok(());
                }
                if channel.is_none() {
                    channel = retry(c, || {
                        c.rc().find_channel_for_option(RcChannelAuxFunc(params.t(2)? as i32))
                    })?;
                }
                let selection = match channel.as_ref() {
                    Some(ch) => {
                        let norm = retry(c, || ch.norm_input_ignore_trim(c))?;
                        let n = c.libm().floorf(
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
                    let args = [
                        params.t(5 * selection + 1)?,
                        params.t(5 * selection + 2)?,
                        params.t(5 * selection + 3)?,
                        params.t(5 * selection + 4)?,
                    ];
                    let p = retry(c, || params.configuration())?;
                    p.validate()?;
                    if loaded.as_ref().map(|l| (l.id, l.args)) != Some((id, args)) {
                        // Keep only one composed schedule within the module heap.
                        loaded = None;
                        match load(c, id, args, p, observation(c)?, params.a(22)?) {
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
                            math::deg(c.libm().atan2f(input.velocity.y, input.velocity.x)?),
                        )?;
                        // Construct/validate before enabling control of the vehicle.
                        let controller =
                            Controller::start(c, &mut task.path, input, p, yaw, count)?;
                        if !retry(c, || c.vehicle().nav_scripting_enable(mode))? {
                            message(c, 1, format_args!("Tricks not available in mode"))?;
                            return Ok(());
                        }
                        owner = Some(Owner::Rc(mode));
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
                    // Recheck after the controller and its observation reads.
                    // An expired item must not receive another task's command.
                    let valid = match owner {
                        Some(Owner::Mission(id)) => {
                            retry(c, || c.vehicle().get_mode())? == mission::AUTO
                                && retry(c, || c.vehicle().nav_script_time())?.map(|m| m.0)
                                    == Some(id)
                        }
                        Some(Owner::Rc(mode)) => retry(c, || c.vehicle().get_mode())? == mode,
                        None => false,
                    };
                    if !valid {
                        active = None;
                        owner = None;
                        return Ok(());
                    }
                    retry(c, || {
                        c.vehicle().set_target_throttle_rate_rpy(
                            o.throttle, o.rates.x, o.rates.y, o.rates.z,
                        )
                    })?;
                    last_command = c.time_us();
                    retry(c, || c.vehicle().set_rudder_offset(o.rudder, true))?;
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
                        let _ = c.gcs().send_named_float("PERR", o.position_error);
                    }
                    // Experimental timing record: controller only, excluding log I/O.
                    let _ = c.logger().write(
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
                    if let Some(o) = owner {
                        release(c, o)?;
                        owner = None;
                    }
                    message(c, 6, format_args!("Finishing {}!", task.name))?;
                    active = None;
                }
            }
            Ok(())
        })();
        if let Err(error) = result {
            if error == Error::Busy {
                if starting_mission && owner.is_none() {
                    last_mission_id = None;
                }
                if preloading {
                    last_preload = None;
                }
            }
            if starting && matches!(error, Error::Busy | Error::Unavailable) && owner.is_none() {
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
            if let Some(o) = owner {
                if release(c, o).is_ok() {
                    owner = None;
                }
            }
            if error == Error::Stopped {
                // The host rejects bindings after stop. Plane's existing
                // one-second NavScript timeout is also Lua restart's backstop.
                return Err(error);
            }
            let now = c.time_us();
            if let Some(id) = attempted_trick {
                message(
                    c,
                    3,
                    format_args!("Aerobatics error: trick {id}: {:?}", error),
                )?;
            } else if now.saturating_sub(last_error) > 1000000 {
                last_error = now;
                message(c, 3, format_args!("Aerobatics error {:?}", error))?;
            }
        }
    }
}
