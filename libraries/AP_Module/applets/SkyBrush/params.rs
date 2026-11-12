// SPDX-License-Identifier: GPL-3.0-or-later
//! SHOW_ parameter table, matching the Skybrush fork's names, indices and defaults.

use ap_rust_sdk::{Context, Error, Parameter, Result};

// dynamic parameter table key; storage identity for SHOW_
pub const TABLE_KEY: u8 = 112;
pub const PREFIX: &str = "SHOW_";

#[derive(Clone, Copy, PartialEq, Eq)]
#[repr(usize)]
pub enum P {
    StartTime,
    StartMsec,
    OriginLat,
    OriginLng,
    OriginAmsl,
    Orientation,
    StartAuth,
    Led0Type,
    Led0Chan,
    Led0Count,
    Led0Gamma,
    Led0Wtemp,
    Led0Minbri,
    ModeBoot,
    PreLights,
    CtrlMode,
    Group,
    CtrlRate,
    VelFfGain,
    TakeoffAlt,
    TakeoffErr,
    SyncMode,
    HfenceEn,
    HfenceDist,
    HfenceTo,
    MaxXyErr,
    MaxZErr,
    PostAction,
    BfenceEn,
    BfenceDxy,
    BfenceDz,
    BfenceTo,
    BfenceAct,
    PyroMinalt,
    PyroType,
    PyroTcomp,
    PyroItime,
    Options,
    TelProfile,
    MaxEscErr,
}

pub const COUNT: usize = 40;

// (index, name, default) in P order
const TABLE: [(u8, &str, f32); COUNT] = [
    (1, "START_TIME", -1.0),
    (40, "START_MSEC", 0.0),
    (2, "ORIGIN_LAT", 0.0),
    (3, "ORIGIN_LNG", 0.0),
    (12, "ORIGIN_AMSL", -10000000.0),
    (4, "ORIENTATION", -1.0),
    (5, "START_AUTH", 0.0),
    (6, "LED0_TYPE", 0.0),
    (8, "LED0_CHAN", 0.0),
    (7, "LED0_COUNT", 16.0),
    (19, "LED0_GAMMA", 1.0),
    (23, "LED0_WTEMP", 0.0),
    (32, "LED0_MINBRI", 0.0),
    (9, "MODE_BOOT", 2.0),
    (10, "PRE_LIGHTS", 2.0),
    (11, "CTRL_MODE", 1.0),
    (13, "GROUP", 0.0),
    (14, "CTRL_RATE", 10.0),
    (16, "VEL_FF_GAIN", 1.0),
    (17, "TAKEOFF_ALT", 2.5),
    (15, "TAKEOFF_ERR", 3.0),
    (18, "SYNC_MODE", 1.0),
    (20, "HFENCE_EN", 0.0),
    (21, "HFENCE_DIST", 25.0),
    (22, "HFENCE_TO", 5.0),
    (24, "MAX_XY_ERR", 3.0),
    (25, "MAX_Z_ERR", 3.0),
    (26, "POST_ACTION", 1.0),
    (27, "BFENCE_EN", 1.0),
    (28, "BFENCE_DXY", 10.0),
    (29, "BFENCE_DZ", 10.0),
    (30, "BFENCE_TO", 5.0),
    (31, "BFENCE_ACT", 1.0),
    (33, "PYRO_MINALT", 20.0),
    (34, "PYRO_TYPE", 0.0),
    (35, "PYRO_TCOMP", 0.0),
    (36, "PYRO_ITIME", 3000.0),
    (37, "OPTIONS", 0.0),
    (38, "TEL_PROFILE", 0.0),
    (39, "MAX_ESC_ERR", -1.0),
];

// highest index in use; the table needs index+1 slots
const TABLE_SIZE: u8 = 41;

// parameters stored as exact int32 (the fork's AP_Int32 values)
const INT32: [P; 6] = [P::StartTime, P::OriginLat, P::OriginLng, P::OriginAmsl, P::PyroTcomp, P::PyroItime];

pub struct Params<'a, 'h> {
    c: &'a Context<'h>,
    handles: [Option<Parameter<'a, 'h>>; COUNT],
    cache: [f32; COUNT],
    cache_i32: [i32; COUNT],
}

fn is_int32(i: usize) -> bool {
    INT32.iter().any(|p| *p as usize == i)
}

fn full_name(name: &str, buf: &mut [u8; 16]) -> usize {
    let n = PREFIX.len() + name.len();
    buf[..PREFIX.len()].copy_from_slice(PREFIX.as_bytes());
    buf[PREFIX.len()..n].copy_from_slice(name.as_bytes());
    n
}

impl<'a, 'h> Params<'a, 'h> {
    /// Register the table and look up handles; values come from storage or defaults files.
    pub fn register(c: &'a Context<'h>) -> Result<Self> {
        if !c.param().add_table(TABLE_KEY, PREFIX, TABLE_SIZE)? {
            return Err(Error::Invalid);
        }
        let mut handles: [Option<Parameter<'a, 'h>>; COUNT] = core::array::from_fn(|_| None);
        let mut cache = [0.0; COUNT];
        let mut cache_i32 = [0; COUNT];
        for (i, (idx, name, default)) in TABLE.iter().enumerate() {
            let mut buf = [0u8; 16];
            let n = full_name(name, &mut buf);
            let full = core::str::from_utf8(&buf[..n]).map_err(|_| Error::Invalid)?;
            if is_int32(i) {
                if !c.param().add_param_int32(TABLE_KEY, *idx, name, *default as i32)? {
                    return Err(Error::Invalid);
                }
                cache_i32[i] = c.param().get_int32(full)?.unwrap_or(*default as i32);
                cache[i] = cache_i32[i] as f32;
                continue;
            }
            if !c.param().add_param(TABLE_KEY, *idx, name, *default)? {
                return Err(Error::Invalid);
            }
            let p = c.parameter(full)?.ok_or(Error::Invalid)?;
            cache[i] = p.get()?.unwrap_or(*default);
            cache_i32[i] = cache[i] as i32;
            handles[i] = Some(p);
        }
        Ok(Self { c, handles, cache, cache_i32 })
    }

    /// Refresh cached values; returns a bitmask of changed parameters.
    pub fn refresh(&mut self) -> Result<u64> {
        let mut changed = 0u64;
        for i in 0..COUNT {
            if is_int32(i) {
                let mut buf = [0u8; 16];
                let n = full_name(TABLE[i].1, &mut buf);
                let full = core::str::from_utf8(&buf[..n]).map_err(|_| Error::Invalid)?;
                if let Some(v) = self.c.param().get_int32(full)? {
                    if v != self.cache_i32[i] {
                        self.cache_i32[i] = v;
                        self.cache[i] = v as f32;
                        changed |= 1 << i;
                    }
                }
            } else if let Some(p) = &self.handles[i] {
                if let Some(v) = p.get()? {
                    if v != self.cache[i] {
                        self.cache[i] = v;
                        self.cache_i32[i] = v as i32;
                        changed |= 1 << i;
                    }
                }
            }
        }
        Ok(changed)
    }

    pub fn get(&self, p: P) -> f32 {
        self.cache[p as usize]
    }

    pub fn get_i32(&self, p: P) -> i32 {
        self.cache_i32[p as usize]
    }

    fn write(&mut self, p: P, v: f32, exact: Option<i32>, save: bool) -> Result {
        let i = p as usize;
        if is_int32(i) {
            let v = exact.unwrap_or(v as i32);
            let mut buf = [0u8; 16];
            let n = full_name(TABLE[i].1, &mut buf);
            let full = core::str::from_utf8(&buf[..n]).map_err(|_| Error::Invalid)?;
            if save {
                self.c.param().set_and_save_int32(full, v)?;
            } else {
                self.c.param().set_int32(full, v)?;
            }
            self.cache_i32[i] = v;
            self.cache[i] = v as f32;
            return Ok(());
        }
        if let Some(h) = &self.handles[i] {
            if save {
                h.set_and_save(v)?;
            } else {
                h.set(v)?;
            }
        }
        self.cache[i] = v;
        self.cache_i32[i] = v as i32;
        Ok(())
    }

    pub fn set(&mut self, p: P, v: f32) -> Result {
        self.write(p, v, None, false)
    }

    pub fn set_i32(&mut self, p: P, v: i32) -> Result {
        self.write(p, v as f32, Some(v), false)
    }

    pub fn set_and_save_i32(&mut self, p: P, v: i32) -> Result {
        self.write(p, v as f32, Some(v), true)
    }

    /// The fork treats start time and authorization as volatile.
    pub fn reset_volatile(&mut self) -> Result {
        self.set_i32(P::StartTime, -1)?;
        self.set(P::StartMsec, 0.0)?;
        self.set(P::StartAuth, 0.0)
    }
}

pub fn changed(mask: u64, p: P) -> bool {
    mask & (1 << p as usize) != 0
}
