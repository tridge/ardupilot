# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Leak-free old-parameter value lookup replacing Parameter.init_by_info."""

OWNER = 'Parameter'
POLICY = ('v1:stack-owned-conversion-value,types-int8-int16-int32-float,'
          'optional-f32-all-bits,no-retained-param-pointer,no-module-reload-leak')


def _shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def select(ir, first_id):
    owner = next((item for item in ir['userdata'] if item['name'] == OWNER), None)
    method = next((item for item in owner['methods'] if item['name'] == 'init_by_info'), None) if owner else None
    expected_arguments = [
        ('uint16_t', None, False, False, False, True, None),
        ('uint32_t', None, False, False, False, True, None),
        ('enum', 'ap_var_type', False, False, False, False, ('AP_PARAM_INT8', 'AP_PARAM_FLOAT')),
    ]
    if (method is None or _shape(method['return']) !=
            ('boolean', None, False, False, False, False, None) or
            [_shape(arg) for arg in method['arguments']] != expected_arguments or
            any(method[key] is not None for key in ('rename', 'depends', 'deprecate'))):
        raise ValueError('unaudited Parameter.init_by_info')
    return specification(first_id)


def specification(first_id):
    return dict(methods=[dict(id=first_id, name='parameter.old_value',
                              arguments=['u16-le-key', 'u32-le-group-element', 'type-u8-range1-4'],
                              result='optional-f32-le', policy=POLICY)])


def rust(spec, notice):
    method = spec['methods'][0]
    return notice + f'''use crate::{{result, Error, Result}};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum OldParameterType {{ Int8 = 1, Int16 = 2, Int32 = 3, Float = 4 }}

impl crate::bindings::Param<'_, '_> {{
    /// Read a legacy parameter conversion value without retaining or allocating an AP_Param object.
    pub fn old_value(&self, key: u16, group_element: u32,
                     parameter_type: OldParameterType) -> Result<Option<f32>> {{
        let mut input = [0u8; 7];
        input[..2].copy_from_slice(&key.to_le_bytes());
        input[2..6].copy_from_slice(&group_element.to_le_bytes());
        input[6] = parameter_type as u8;
        let mut output = [0u8; 5];
        result(unsafe {{ self.context.api.binding_call.unwrap()(
            self.context.raw, crate::imports::BINDING_{method['id']}.slot(), input.as_ptr(), 7,
            output.as_mut_ptr(), 5)
        }})?;
        match output[0] {{
            0 if output[1..].iter().all(|byte| *byte == 0) => Ok(None),
            1 => Ok(Some(f32::from_bits(u32::from_le_bytes(output[1..5].try_into().unwrap())))),
            _ => Err(Error::Host),
        }}
    }}
}}
'''
