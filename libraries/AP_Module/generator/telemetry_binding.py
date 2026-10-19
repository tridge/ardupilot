# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Owned adapters for script-fed ESC data and measured servo position."""

ESC_OWNER = 'AP_ESC_Telem'
ESC_VALUE = 'AP_ESC_Telem_Backend::TelemetryData'
SERVO_OWNER = 'AP_Servo_Telem'
SERVO_VALUE = 'AP_Servo_Telem::TelemetryData'
ESC_DEPENDS = 'HAL_WITH_ESC_TELEM == 1'
SERVO_DEPENDS = 'AP_SERVO_TELEM_ENABLED'
ESC_WIRE_SIZE = 12
SERVO_WIRE_SIZE = 6
SERVO_UPDATE_WIRE_SIZE = 33
POLICY = ('v1:owned-value-copy,esc-options-map-to-temperature-voltage-current-validity-bits,'
          'esc-nonempty-and-index-bounded,servo-overall-presence-plus-measured-position-validity,'
          'all-f32-bits-preserved,no-retained-firmware-pointers')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _method_shape(method):
    return (_field_shape(method['return']), tuple(_field_shape(arg) for arg in method['arguments']),
            method['rename'], method['depends'], method['deprecate'])


def _owner(ir, group, name, rename, depends):
    owner = next((item for item in ir[group] if item['name'] == name), None)
    expected = dict(rename=rename, depends=depends, get=None, literal=False, reference=False,
                    semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected.items()) or owner['aliases']:
        raise ValueError(f'unaudited {name} metadata')
    return owner


def _method(owner, name, expected):
    methods = [method for method in owner['methods'] if method['name'] == name]
    if len(methods) != 1 or _method_shape(methods[0]) != expected:
        raise ValueError(f'unaudited {owner["name"]}.{name} method')


def _field(owner, name, expected):
    fields = [field for field in owner['fields'] if field['name'] == name]
    if len(fields) != 1:
        raise ValueError(f'unaudited {owner["name"]}.{name} field')
    field = fields[0]
    actual = (_field_shape(field['type']), field['read'], field['write'], field['rename'], field['depends'],
              field['array_len'], field['valid_mask_name'], field['valid_mask_value'])
    if actual != expected:
        raise ValueError(f'unaudited {owner["name"]}.{name} field')


def select(ir, first_id):
    esc = _owner(ir, 'singletons', ESC_OWNER, 'esc_telem', ESC_DEPENDS)
    _method(esc, 'update_telem_data', (
        ('void', None, False, False, False, False, None),
        (('uint8_t', None, False, False, False, False, ('0', 'ESC_TELEM_MAX_ESCS')),
         ('userdata', ESC_VALUE, False, False, False, False, None),
         ('uint16_t', None, False, False, False, True, None)), None, None, None))
    esc_value = _owner(ir, 'userdata', ESC_VALUE, 'ESCTelemetryData', f'({ESC_DEPENDS})')
    write_only = (False, True, None, None, None, None, None)
    _field(esc_value, 'temperature_cdeg',
           (('int16_t', None, False, False, False, True, None),) + write_only)
    for name in ('voltage', 'current'):
        _field(esc_value, name, (('float', None, False, False, False, True, None),) + write_only)

    servo = _owner(ir, 'singletons', SERVO_OWNER, 'servo_telem', SERVO_DEPENDS)
    _method(servo, 'get_telem', (
        ('boolean', None, False, False, False, False, None),
        (('uint8_t', None, False, False, False, True, None),
         ('userdata', SERVO_VALUE, True, False, False, False, None)), None, None, None))
    _method(servo, 'update_telem_data', (
        ('void', None, False, False, False, False, None),
        (('uint8_t', None, False, False, False, True, None),
         ('userdata', SERVO_VALUE, False, False, False, False, None)), None, None, None))
    servo_value = _owner(ir, 'userdata', SERVO_VALUE, 'ServoTelemetryData', SERVO_DEPENDS)
    servo_fields = {
        'command_position': ('float', 'COMMANDED_POSITION'),
        'measured_position': ('float', 'MEASURED_POSITION'),
        'force': ('float', 'FORCE'),
        'speed': ('float', 'SPEED'),
        'voltage': ('float', 'VOLTAGE'),
        'current': ('float', 'CURRENT'),
        'duty_cycle': ('uint8_t', 'DUTY_CYCLE'),
        'motor_temperature_cdeg': ('int16_t', 'MOTOR_TEMP'),
        'pcb_temperature_cdeg': ('int16_t', 'PCB_TEMP'),
        'status_flags': ('uint8_t', 'STATUS'),
    }
    for name, (kind, mask) in servo_fields.items():
        _field(servo_value, name, (
            (kind, None, False, False, False, True, None), True, True, None, None, None,
            'present_types', f'AP_Servo_Telem::TelemetryData::Types::{mask}'))
    _field(servo_value, 'last_update_ms', (
        ('uint32_t', None, False, False, False, True, None), True, False, None, None, None, None, None))
    return specification(first_id)


def specification(first_id):
    esc_value = dict(name='EscTelemetryUpdate', fields=[
        ['temperature_cdeg', 'optional-i16-le,presence-bit0'],
        ['voltage', 'optional-f32-le,presence-bit1'],
        ['current', 'optional-f32-le,presence-bit2'],
    ])
    servo_value = dict(name='ServoTelemetryData', fields=[
        ['measured_position', 'optional-f32-le,present-types-bit1'],
    ])
    return dict(methods=[
        dict(id=first_id, name='esc_telem.update_telem_data',
             arguments=[dict(type='u8', name='esc_index'), dict(type=esc_value, direction='in')],
             result='void', policy=POLICY),
        dict(id=first_id + 1, name='servo_telem.get_telem',
             arguments=[dict(type='u8', name='servo_index')],
             result=dict(presence='bool-u8', value=servo_value), policy=POLICY),
        dict(id=first_id + 2, name='servo_telem.update_telem_data',
             arguments=[dict(type='u8', name='servo_index'),
                        dict(type='ServoTelemetryUpdate,optional-fields-mask-u16', direction='in')],
             result='void', policy=POLICY),
    ])


def rust(spec, notice):
    esc, servo, servo_update = spec['methods']
    return notice + f'''use crate::{{result, Error, Result}};

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EscTelemetryUpdate {{
    pub temperature_cdeg: Option<i16>,
    pub voltage: Option<f32>,
    pub current: Option<f32>,
}}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ServoTelemetryData {{
    pub measured_position: Option<f32>,
}}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct ServoTelemetryUpdate {{
    pub command_position: Option<f32>,
    pub measured_position: Option<f32>,
    pub force: Option<f32>,
    pub speed: Option<f32>,
    pub voltage: Option<f32>,
    pub current: Option<f32>,
    pub duty_cycle: Option<u8>,
    pub motor_temperature_cdeg: Option<i16>,
    pub pcb_temperature_cdeg: Option<i16>,
    pub status_flags: Option<u8>,
}}

impl crate::bindings::EscTelem<'_, '_> {{
    pub fn update_telem_data(&self, esc_index: u8, data: &EscTelemetryUpdate) -> Result {{
        let mut input = [0u8; {ESC_WIRE_SIZE}];
        input[0] = esc_index;
        if let Some(value) = data.temperature_cdeg {{
            input[1] |= 1;
            input[2..4].copy_from_slice(&value.to_le_bytes());
        }}
        if let Some(value) = data.voltage {{
            input[1] |= 2;
            input[4..8].copy_from_slice(&value.to_bits().to_le_bytes());
        }}
        if let Some(value) = data.current {{
            input[1] |= 4;
            input[8..12].copy_from_slice(&value.to_bits().to_le_bytes());
        }}
        if input[1] == 0 {{ return Err(Error::Invalid); }}
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw, crate::imports::BINDING_{esc['id']}.slot(),
                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)
        }})
    }}

}}

impl crate::bindings::ServoTelem<'_, '_> {{
    pub fn get_telem(&self, servo_index: u8) -> Result<Option<ServoTelemetryData>> {{
        let input = [servo_index];
        let mut output = [0u8; {SERVO_WIRE_SIZE}];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{servo['id']}.slot(),
                input.as_ptr(), 1, output.as_mut_ptr(), output.len() as u32)
        }})?;
        match (output[0], output[1]) {{
            (0, 0) => Ok(None),
            (1, present @ 0..=1) => Ok(Some(ServoTelemetryData {{
                measured_position: if present != 0 {{
                    Some(f32::from_bits(u32::from_le_bytes(output[2..6].try_into().unwrap())))
                }} else {{ None }},
            }})),
            _ => Err(Error::Host),
        }}
    }}

    pub fn update_telem_data(&self, servo_index: u8,
                             data: &ServoTelemetryUpdate) -> Result {{
        let mut input = [0u8; {SERVO_UPDATE_WIRE_SIZE}];
        input[0] = servo_index;
        let mut present = 0u16;
        macro_rules! put_float {{ ($field:ident, $bit:expr, $offset:expr) => {{
            if let Some(value) = data.$field {{
                present |= 1 << $bit;
                input[$offset..$offset + 4].copy_from_slice(&value.to_bits().to_le_bytes());
            }}
        }}; }}
        put_float!(command_position, 0, 3);
        put_float!(measured_position, 1, 7);
        put_float!(force, 2, 11);
        put_float!(speed, 3, 15);
        put_float!(voltage, 4, 19);
        put_float!(current, 5, 23);
        if let Some(value) = data.duty_cycle {{ present |= 1 << 6; input[27] = value; }}
        if let Some(value) = data.motor_temperature_cdeg {{
            present |= 1 << 7; input[28..30].copy_from_slice(&value.to_le_bytes());
        }}
        if let Some(value) = data.pcb_temperature_cdeg {{
            present |= 1 << 8; input[30..32].copy_from_slice(&value.to_le_bytes());
        }}
        if let Some(value) = data.status_flags {{ present |= 1 << 9; input[32] = value; }}
        if present == 0 {{ return Err(Error::Invalid); }}
        input[1..3].copy_from_slice(&present.to_le_bytes());
        result(unsafe {{
            self.context.api.binding_call.unwrap()(
                self.context.raw, crate::imports::BINDING_{servo_update['id']}.slot(),
                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)
        }})
    }}
}}
'''
