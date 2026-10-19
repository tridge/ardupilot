# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Single-module initialization adapter for the persistent ONVIF client."""

OWNER = 'AP_ONVIF'
METHOD = 'start'
STRING_MAX = 255
STRING_WIRE_SIZE = STRING_MAX + 1
POLICY = ('v1:single-module-owner-claimed-before-start,cleanup-owner-after-worker-join,'
          'persistent-client-allocation-not-released,reconfigurable-after-restart,'
          'three-nul-free-byte-strings-max255-zero-pad256,blocking-lock,'
          'blocking-firmware-network-call,no-retained-module-pointers')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_owner = dict(rename='onvif', depends='ENABLE_ONVIF == 1', get=None, literal=False,
                          reference=False, semaphore=False, scheduler_semaphore=False,
                          semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected_owner.items()):
        raise ValueError('unaudited AP_ONVIF owner metadata')
    methods = [method for method in owner['methods'] if method['name'] == METHOD]
    expected_result = ('boolean', None, False, False, False, False, None)
    expected_argument = ('string', None, False, False, False, False, None)
    if len(methods) != 1:
        raise ValueError('missing AP_ONVIF.start method')
    method = methods[0]
    actual = (_field_shape(method['return']), [_field_shape(argument) for argument in method['arguments']])
    if (method['rename'] is not None or method['depends'] is not None or method['deprecate'] is not None or
            actual != (expected_result, [expected_argument] * 3)):
        raise ValueError('unaudited AP_ONVIF.start declaration')
    return specification(first_id)


def specification(first_id):
    return dict(
        id=first_id,
        name='onvif.start',
        arguments=[dict(type=f'nul-free-bytes-max{STRING_MAX}-zero-pad{STRING_WIRE_SIZE}', name=name)
                   for name in ('username', 'password', 'hostname')],
        result='bool-u8',
        policy=POLICY,
    )


def rust(method, notice):
    return notice + f'''use crate::{{result, Error, Result}};

pub const ONVIF_STRING_MAX: usize = {STRING_MAX};
const ONVIF_STRING_WIRE_SIZE: usize = ONVIF_STRING_MAX + 1;

impl crate::bindings::Onvif<'_, '_> {{
    /// Start or reconfigure the firmware-wide ONVIF client for this module.
    ///
    /// A start attempt claims the client until this module exits, even when the
    /// camera probe returns false. Firmware allocations and configuration persist;
    /// cleanup releases only the Rust ownership claim after the worker has joined.
    pub fn start(&self, username: &str, password: &str, hostname: &str) -> Result<bool> {{
        let values = [username, password, hostname];
        let mut input = [0u8; 3 * ONVIF_STRING_WIRE_SIZE];
        for (index, value) in values.iter().enumerate() {{
            let bytes = value.as_bytes();
            if bytes.len() > ONVIF_STRING_MAX || bytes.contains(&0) {{
                return Err(Error::Invalid);
            }}
            let offset = index * ONVIF_STRING_WIRE_SIZE;
            input[offset..offset + bytes.len()].copy_from_slice(bytes);
        }}
        let mut output = [0u8; 1];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{method['id']}.slot(),
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
