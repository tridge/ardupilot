// SPDX-License-Identifier: GPL-3.0-or-later
//! SIMPLE_GEOFENCE_SETUP handling, as AC_DroneShowManager::configure_fences().

use crate::{coords::ShowFrame, mavlink::GeofenceSetup, vehicle};
use alloc::vec::Vec;
use ap_rust_sdk::{enums::MavSeverity, Context, Result};

const FENCE_TYPE_ALT_MAX: u8 = 1;
const FENCE_TYPE_CIRCLE: u8 = 2;
const FENCE_TYPE_POLYGON: u8 = 4;

// AC_Fence::Action values the fork accepts
const ACCEPTED_ACTIONS: [u8; 6] = [0, 1, 2, 3, 4, 5];

fn error(c: &Context<'_>, text: &str) -> Result<bool> {
    c.gcs().send_text(MavSeverity::ERROR, text)?;
    Ok(false)
}

fn set_param(c: &Context<'_>, name: &str, value: f32) -> Result<bool> {
    if !c.param().set_and_save(name, value)? {
        return Ok(false);
    }
    Ok(true)
}

/// Returns true if the fences were configured.
pub fn configure(c: &Context<'_>, frame: Option<&ShowFrame>, g: &GeofenceSetup<'_>) -> Result<bool> {
    let mut mask = 0u8;
    if g.max_altitude_dm > 0 {
        if !set_param(c, "FENCE_ALT_MAX", g.max_altitude_dm as f32 / 10.0)? {
            return error(c, "FENCE_ALT_MAX parameter cannot be set");
        }
        mask |= FENCE_TYPE_ALT_MAX;
    }
    if g.radius_dm > 0 {
        if !set_param(c, "FENCE_RADIUS", g.radius_dm as f32 / 10.0)? {
            return error(c, "FENCE_RADIUS parameter cannot be set");
        }
        mask |= FENCE_TYPE_CIRCLE;
    }
    let action = g.flags & 0x0f;
    if !ACCEPTED_ACTIONS.contains(&action) {
        return error(c, "Unsupported fence action");
    }
    if !set_param(c, "FENCE_ACTION", action as f32)? {
        return error(c, "FENCE_ACTION parameter cannot be set");
    }
    if g.num_points > 0 {
        mask |= FENCE_TYPE_POLYGON;
    }
    // the fork writes the type mask into FENCE_ENABLE
    if !set_param(c, "FENCE_ENABLE", mask as f32)? {
        return error(c, "FENCE_ENABLE parameter cannot be set");
    }
    // polygon last so the caller's ack follows it immediately: SITL reports
    // "Fence Indexed OK" from the write and GCSes expect it after the ack
    if g.num_points > 0 {
        let Some(frame) = frame else {
            return error(c, "Show coordinate system is not valid");
        };
        let mut points = Vec::with_capacity(g.num_points);
        for i in 0..g.num_points {
            let (x, y) = g.point(i);
            points.push(frame.to_global(c, [x as f32 * 100.0, y as f32 * 100.0, 0.0])?);
        }
        if !vehicle::write_inclusion_polygon(c, &points)? {
            return error(c, "Failed to write polygon fence");
        }
    }
    Ok(true)
}
