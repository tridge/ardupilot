# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Owned wire value for scripting battery-monitor updates."""

CELL_COUNT = 32
WIRE_SIZE = 90
POLICY = ('v1:complete-owned-copy,no-retained-module-pointers,all-f32-bits,'
          'u8-255-and-u16-65535-mean-unavailable,nan-means-unavailable-or-auto-integrate,'
          'cell-count-max32,zero-pad-none')


def specification(first_id):
    return dict(id=first_id, name='battery.handle_scripting',
                arguments=[dict(type='u8', direction='in'),
                           dict(type=dict(name='BattMonitorScriptState', fields=[
                               ['voltage', 'f32-le'], ['healthy', 'bool-u8'], ['cell_count', 'u8'],
                               ['capacity_remaining_pct', 'u8,255-unavailable'],
                               ['state_of_health_pct', 'u8,255-unavailable'],
                               ['cell_voltages', 'u16-le[32]'], ['cycle_count', 'u16-le,65535-unavailable'],
                               ['current_amps', 'f32-le,nan-unavailable'],
                               ['consumed_mah', 'f32-le,nan-auto-integrate'],
                               ['consumed_wh', 'f32-le,nan-unavailable'],
                               ['temperature', 'f32-le,nan-unavailable'],
                           ]), direction='in')], result='bool-u8', policy=POLICY)


def rust(method, notice):
    return notice + f'''use crate::{{result, Error, Result}};

/// A complete local battery update. `None` maps to the firmware sentinel used
/// by Lua's default-constructed `BattMonitorScript_State`.
#[derive(Clone, Copy, Debug, PartialEq)]
pub struct BattMonitorScriptState {{
    pub voltage: f32,
    pub healthy: bool,
    pub cell_count: u8,
    pub capacity_remaining_pct: Option<u8>,
    pub state_of_health_pct: Option<u8>,
    pub cell_voltages: [u16; {CELL_COUNT}],
    pub cycle_count: Option<u16>,
    pub current_amps: Option<f32>,
    pub consumed_mah: Option<f32>,
    pub consumed_wh: Option<f32>,
    pub temperature: Option<f32>,
}}

impl Default for BattMonitorScriptState {{
    fn default() -> Self {{
        Self {{
            voltage: 0.0,
            healthy: false,
            cell_count: 0,
            capacity_remaining_pct: None,
            state_of_health_pct: None,
            cell_voltages: [0; {CELL_COUNT}],
            cycle_count: None,
            current_amps: None,
            consumed_mah: None,
            consumed_wh: None,
            temperature: None,
        }}
    }}
}}

impl crate::bindings::Battery<'_, '_> {{
    /// Feed one scripting battery backend. `false` means the index is absent or
    /// is not configured as a scripting monitor. The host copies the full state.
    pub fn handle_scripting(&self, index: u8, state: &BattMonitorScriptState) -> Result<bool> {{
        if state.cell_count as usize > {CELL_COUNT}
            || state.capacity_remaining_pct == Some(u8::MAX)
            || state.state_of_health_pct == Some(u8::MAX)
            || state.cycle_count == Some(u16::MAX)
        {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; {WIRE_SIZE + 1}];
        input[0] = index;
        input[1..5].copy_from_slice(&state.voltage.to_bits().to_le_bytes());
        input[5] = u8::from(state.healthy);
        input[6] = state.cell_count;
        input[7] = state.capacity_remaining_pct.unwrap_or(u8::MAX);
        input[8] = state.state_of_health_pct.unwrap_or(u8::MAX);
        for (i, value) in state.cell_voltages.iter().enumerate() {{
            input[9+i*2..11+i*2].copy_from_slice(&value.to_le_bytes());
        }}
        input[73..75].copy_from_slice(&state.cycle_count.unwrap_or(u16::MAX).to_le_bytes());
        for (offset, value) in [(75, state.current_amps), (79, state.consumed_mah),
                                (83, state.consumed_wh), (87, state.temperature)] {{
            input[offset..offset+4].copy_from_slice(&value.unwrap_or(f32::NAN).to_bits().to_le_bytes());
        }}
        let mut output = [0u8; 1];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw, crate::imports::BINDING_{method['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        match output[0] {{
            0 => Ok(false),
            1 => Ok(true),
            _ => Err(Error::Host),
        }}
    }}
}}
'''
