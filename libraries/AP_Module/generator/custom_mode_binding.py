# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Lifecycle-scoped handles for persistent custom vehicle modes."""

OWNER = 'AP_Vehicle'
STATE = 'AP_Vehicle::custom_mode_state'
FULL_NAME_MAX = 31
SHORT_NAME_MAX = 4
POLICY = ('v1:persistent-mode-registration,module-context-owned-state-handle,'
          'cleanup-forces-allow-entry-false,full-name-max31,short-name-max4,'
          'blocking-scheduler-lock,no-retained-module-pointers')


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    methods = [] if owner is None else [method for method in owner['methods']
                                        if method['name'] == 'register_custom_mode']
    if len(methods) != 1:
        raise ValueError('missing AP_Vehicle.register_custom_mode')
    method = methods[0]
    args = [(item['kind'], item['name'], item['skip_check']) for item in method['arguments']]
    if (owner['rename'] != 'vehicle' or owner['depends'] != 'AP_SCRIPTING_BINDING_VEHICLE_ENABLED' or
            not owner['scheduler_semaphore'] or method['return']['kind'] != 'ap_object' or
            method['return']['name'] != STATE or args != [
                ('uint8_t', None, True), ('string', None, False), ('string', None, False)]):
        raise ValueError('unaudited custom mode registration declaration')
    state = next((item for item in ir['ap_objects'] if item['name'] == STATE), None)
    if state is None or state['depends'] != 'AP_SCRIPTING_BINDING_VEHICLE_ENABLED':
        raise ValueError('unaudited custom mode state declaration')
    fields = state['fields']
    if (len(fields) != 1 or fields[0]['name'] != 'allow_entry' or not fields[0]['read'] or
            not fields[0]['write'] or fields[0]['type']['kind'] != 'boolean'):
        raise ValueError('unaudited custom mode allow_entry field')
    return specification(first_id)


def specification(first_id):
    return dict(methods=[
        dict(id=first_id, name='vehicle.register_custom_mode',
             arguments=[dict(type='u8', name='number'),
                        dict(type='nonempty-nul-free-bytes-max31-zero-pad32', name='full_name'),
                        dict(type='nonempty-nul-free-bytes-max4-zero-pad5', name='short_name')],
             result='optional-nonzero-u32-le', policy=POLICY),
        dict(id=first_id + 1, name='CustomMode.allow_entry',
             arguments=[dict(type='nonzero-u32-le', name='handle')], result='bool-u8', policy=POLICY),
        dict(id=first_id + 2, name='CustomMode.set_allow_entry',
             arguments=[dict(type='nonzero-u32-le', name='handle'), dict(type='bool-u8', name='value')],
             result='void', policy=POLICY),
    ], full_name_max=FULL_NAME_MAX, short_name_max=SHORT_NAME_MAX)


def rust(spec, notice):
    register, get, set_value = spec['methods']
    return notice + f'''use core::num::NonZeroU32;
use crate::{{result, Context, Error, Result}};

pub const CUSTOM_MODE_FULL_NAME_MAX: usize = {FULL_NAME_MAX};
pub const CUSTOM_MODE_SHORT_NAME_MAX: usize = {SHORT_NAME_MAX};

pub struct CustomMode {{ token: NonZeroU32 }}

impl crate::bindings::Vehicle<'_, '_> {{
    pub fn register_custom_mode(&self, number: u8, full_name: &str,
                                short_name: &str) -> Result<Option<CustomMode>> {{
        let full = full_name.as_bytes();
        let short = short_name.as_bytes();
        if full.is_empty() || full.len() > CUSTOM_MODE_FULL_NAME_MAX || full.contains(&0) ||
                short.is_empty() || short.len() > CUSTOM_MODE_SHORT_NAME_MAX || short.contains(&0) {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; 40];
        input[0] = number; input[1] = full.len() as u8;
        input[2..2 + full.len()].copy_from_slice(full);
        input[34] = short.len() as u8;
        input[35..35 + short.len()].copy_from_slice(short);
        let mut output = [0u8; 4];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{register['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), output.len() as u32) }})?;
        Ok(NonZeroU32::new(u32::from_le_bytes(output)).map(|token| CustomMode {{ token }}))
    }}
}}

impl CustomMode {{
    pub fn allow_entry(&self, context: &Context<'_>) -> Result<bool> {{
        let input = self.token.get().to_le_bytes();
        let mut output = [0u8; 1];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{get['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), output.len() as u32) }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    pub fn set_allow_entry(&self, context: &Context<'_>, value: bool) -> Result {{
        let mut input = [0u8; 5];
        input[..4].copy_from_slice(&self.token.get().to_le_bytes()); input[4] = value as u8;
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{set_value['id']}.slot(), input.as_ptr(), input.len() as u32,
            core::ptr::null_mut(), 0) }})
    }}
}}
'''
