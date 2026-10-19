# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Module-owned auxiliary arming authorisation bindings."""

OWNER = 'AP_Arming'
FAILURE_MESSAGE_MAX = 41
POLICY = ('v1:module-context-owned-id,owner-validated-updates,cleanup-after-worker-join,'
          'lua-reset-isolation,max3-firmware-wide,failure-message-nul-free-bytes-max41-zero-pad41,'
          'no-retained-module-pointers')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_owner = dict(rename='arming', depends='AP_ARMING_ENABLED', get=None, literal=False, reference=False,
                          semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected_owner.items()):
        raise ValueError('unaudited AP_Arming owner metadata')

    expected = {
        'get_aux_auth_id': (
            ('boolean', None, False, False, False, False, None),
            [('uint8_t', None, True, False, False, False, None)]),
        'set_aux_auth_passed': (
            ('void', None, False, False, False, False, None),
            [('uint8_t', None, False, False, False, True, None)]),
        'set_aux_auth_failed': (
            ('void', None, False, False, False, False, None),
            [('uint8_t', None, False, False, False, True, None),
             ('string', None, False, False, False, False, None)]),
    }
    methods = {method['name']: method for method in owner['methods'] if method['name'] in expected}
    if set(methods) != set(expected):
        raise ValueError('missing auxiliary arming method')
    for name, (result, arguments) in expected.items():
        method = methods[name]
        actual = (_field_shape(method['return']), [_field_shape(argument) for argument in method['arguments']])
        if (method['rename'] is not None or method['depends'] is not None or method['deprecate'] is not None or
                actual != (result, arguments)):
            raise ValueError(f'unaudited AP_Arming.{name} declaration')
    return specification(first_id)


def specification(first_id):
    methods = [
        dict(id=first_id, name='arming.get_aux_auth_id', arguments=[],
             result=dict(presence='bool-u8', auth_id='u8,module-owned-token'), policy=POLICY),
        dict(id=first_id + 1, name='arming.set_aux_auth_passed',
             arguments=[dict(type='u8,module-owned-token', name='auth_id')], result='void', policy=POLICY),
        dict(id=first_id + 2, name='arming.set_aux_auth_failed',
             arguments=[dict(type='u8,module-owned-token', name='auth_id'),
                        dict(type=f'nul-free-bytes-max{FAILURE_MESSAGE_MAX}-zero-pad{FAILURE_MESSAGE_MAX}',
                             name='message')],
             result='void', policy=POLICY),
    ]
    return dict(methods=methods)


def rust(spec, notice):
    get, passed, failed = spec['methods']
    return notice + f'''use crate::{{result, Error, Result}};

pub const AUX_AUTH_FAILURE_MESSAGE_MAX: usize = {FAILURE_MESSAGE_MAX};

impl crate::bindings::Arming<'_, '_> {{
    /// Allocate an auxiliary-authorisation ID owned by this module instance.
    /// Firmware releases all IDs owned by the module after its worker exits.
    pub fn get_aux_auth_id(&self) -> Result<Option<u8>> {{
        let mut output = [0u8; 2];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw, crate::imports::BINDING_{get['id']}.slot(),
                core::ptr::null(), 0, output.as_mut_ptr(), output.len() as u32)
        }})?;
        match output[0] {{
            0 if output[1] == 0 => Ok(None),
            1 => Ok(Some(output[1])),
            _ => Err(Error::Host),
        }}
    }}

    /// Mark a module-owned auxiliary-authorisation ID as passed.
    pub fn set_aux_auth_passed(&self, auth_id: u8) -> Result {{
        let input = [auth_id];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{passed['id']}.slot(),
                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)
        }})
    }}

    /// Mark a module-owned auxiliary-authorisation ID as failed.
    pub fn set_aux_auth_failed(&self, auth_id: u8, message: &str) -> Result {{
        let bytes = message.as_bytes();
        if bytes.len() > AUX_AUTH_FAILURE_MESSAGE_MAX || bytes.contains(&0) {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; 2 + AUX_AUTH_FAILURE_MESSAGE_MAX];
        input[0] = auth_id;
        input[1] = bytes.len() as u8;
        input[2..2 + bytes.len()].copy_from_slice(bytes);
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{failed['id']}.slot(),
                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)
        }})
    }}
}}
'''
