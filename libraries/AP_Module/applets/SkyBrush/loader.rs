// SPDX-License-Identifier: GPL-3.0-or-later
//! Show file loading and trajectory properties, as AC_DroneShowManager_Loader.

use crate::show::{
    controller::{Controller, SCENE_FLAG_DYNAMIC_DURATION},
    stats::{Stats, StatsCalculator},
    Vec3,
};
use alloc::vec::Vec;
use ap_rust_sdk::{Context, Result};

#[cfg(target_arch = "x86_64")]
pub const SHOW_DIR: &str = "collmot";
#[cfg(not(target_arch = "x86_64"))]
pub const SHOW_DIR: &str = "/COLLMOT";
#[cfg(target_arch = "x86_64")]
pub const SHOW_FILE: &str = "collmot/show.skyb";
#[cfg(not(target_arch = "x86_64"))]
pub const SHOW_FILE: &str = "/COLLMOT/show.skyb";

const CIRCULAR_THRESHOLD_MM: f32 = 500.0;
const SCENE_TAG_MAIN_SHOW: u8 = 1;

pub struct Show {
    pub controller: Controller,
    pub stats: Stats,
    pub takeoff_time_sec: f32,
    pub landing_time_sec: f32,
    pub takeoff_position_mm: Vec3,
    pub circular: bool,
    pub modified_for_landing: bool,
    /// GCS light control setup block (pixel grid), if any
    pub light_control: Option<Vec<u8>>,
}

impl Show {
    pub fn plausible(&self) -> bool {
        self.takeoff_time_sec >= 0.0 && self.landing_time_sec > self.takeoff_time_sec
    }
}

/// Inputs from vehicle parameters used for trajectory statistics.
pub struct LoadConfig {
    pub takeoff_alt_m: f32,
    pub takeoff_speed_ms: f32,
    pub takeoff_accel_mss: f32,
    pub spool_time_sec: f32,
    pub pyro_tcomp_msec: i32,
}

fn read_file(c: &Context<'_>, path: &str) -> Result<Option<Vec<u8>>> {
    let mut data = Vec::new();
    let mut buf = [0u8; 256];
    loop {
        match c.file().read_at(path, data.len() as u32, &mut buf)? {
            None => return Ok(if data.is_empty() { None } else { Some(data) }),
            Some(0) => return Ok(Some(data)),
            Some(n) => data.extend_from_slice(&buf[..n]),
        }
    }
}

/// Load and prepare the show; Ok(None) if there is no show file.
pub fn load(c: &Context<'_>, cfg: &LoadConfig) -> Result<Option<Show>> {
    let Some(data) = read_file(c, SHOW_FILE)? else {
        return Ok(None);
    };
    let Ok(mut controller) = Controller::from_show_bytes(&data) else {
        return Ok(None);
    };
    let light_control = crate::show::binary::ShowFile::parse(&data)
        .ok()
        .and_then(|f| f.find_first(crate::show::binary::BLOCK_LIGHT_CONTROL).ok().map(|b| b.to_vec()));
    drop(data);
    let Some(scene) = controller.screenplay_mut().scenes.get_mut(0) else {
        return Ok(None);
    };
    let Some(traj) = scene.trajectory.as_ref() else {
        return Ok(None);
    };
    let calc = StatsCalculator::for_show(
        cfg.takeoff_alt_m * 1000.0,
        cfg.takeoff_speed_ms * 1000.0,
        cfg.takeoff_accel_mss * 1000.0,
    );
    let (stats, ok) = match calc.run(traj) {
        Ok(s) => (s, true),
        Err(_) => (Stats::default(), false),
    };
    let mut takeoff = stats.takeoff_time_sec - cfg.spool_time_sec;
    if takeoff < 0.0 {
        takeoff = 0.0;
    }
    let (takeoff, landing) = if ok && stats.landing_time_sec >= takeoff {
        (takeoff, stats.landing_time_sec)
    } else {
        (-1.0, -1.0)
    };
    scene.duration_msec = if landing > 0.0 { (landing * 1000.0) as u32 } else { 0 };
    scene.flags |= SCENE_FLAG_DYNAMIC_DURATION;
    scene.tag = SCENE_TAG_MAIN_SHOW;
    if cfg.pyro_tcomp_msec != 0 {
        if let Some(ev) = scene.events.as_mut() {
            ev.adjust_timestamps_by_type(crate::show::events::EVENT_TYPE_PYRO, -cfg.pyro_tcomp_msec);
        }
    }
    controller.reset_players();
    Ok(Some(Show {
        circular: ok && stats.start_to_end_distance_xy <= CIRCULAR_THRESHOLD_MM,
        takeoff_position_mm: if ok { stats.initial_pos } else { Vec3::ZERO },
        controller,
        stats,
        takeoff_time_sec: takeoff,
        landing_time_sec: landing,
        modified_for_landing: false,
        light_control,
    }))
}

/// Land at the actual takeoff point at the end of a circular show (SHOW_OPTIONS bit1).
pub fn correct_landing(show: &mut Show, end: Vec3, land_speed_mm_s: f32) -> bool {
    if !show.circular || show.modified_for_landing {
        return false;
    }
    let Some(scene) = show.controller.screenplay_mut().scenes.get_mut(0) else {
        return false;
    };
    let Some(traj) = scene.trajectory.as_mut() else {
        return false;
    };
    if traj
        .replace_end_to_land_at(&mut show.stats, end, land_speed_mm_s, land_speed_mm_s)
        .is_err()
    {
        return false;
    }
    if scene.has_flag(SCENE_FLAG_DYNAMIC_DURATION) {
        let d = scene.time_axis.reverse_map(show.stats.duration_sec);
        scene.duration_msec = if d <= 0 { 1 } else { d as u32 };
    }
    show.controller.notify_trajectory_changed();
    show.modified_for_landing = true;
    true
}
