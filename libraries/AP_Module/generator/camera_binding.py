# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Owned camera values used by the AP_Scripting examples."""

OWNER = 'AP_Camera'
STATE = 'AP_Camera::camera_state_t'
INFORMATION = 'mavlink_camera_information_t'
VENDOR_SIZE = 32
MODEL_SIZE = 32
URI_SIZE = 140
STATE_WIRE_SIZE = 31
INFORMATION_WIRE_SIZE = 231
POLICY = ('v1:owned-camera-values,explicit-fields,no-struct-padding,camera-instance-u8,'
          'blocking-camera-semaphore,no-retained-module-pointers')


def _shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _method(owner, name, result, arguments, depends=None):
    matches = [method for method in owner['methods'] if method['name'] == name]
    if len(matches) != 1:
        raise ValueError(f'missing {OWNER}.{name}')
    method = matches[0]
    actual = (_shape(method['return']), [_shape(argument) for argument in method['arguments']])
    if (method['rename'] is not None or method['depends'] != depends or method['deprecate'] is not None or
            actual != (result, arguments)):
        raise ValueError(f'unaudited {OWNER}.{name} declaration')


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected = dict(rename='camera', depends='AP_CAMERA_ENABLED && (AP_CAMERA_SCRIPTING_ENABLED == 1)',
                    get=None, literal=False, reference=False, semaphore=True,
                    scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited camera owner metadata')
    _method(owner, 'get_state', ('boolean', None, False, False, False, False, None),
            [('uint8_t', None, False, False, False, True, None),
             ('userdata', STATE, True, False, False, False, None)])
    _method(owner, 'set_camera_information', ('void', None, False, False, False, False, None),
            [('uint8_t', None, False, False, False, True, None),
             ('userdata', INFORMATION, False, False, False, False, None)],
            depends='AP_CAMERA_INFO_FROM_SCRIPT_ENABLED')
    return specification(first_id)


def specification(first_id):
    return dict(methods=[
        dict(id=first_id, name='camera.get_state', arguments=[dict(type='u8', name='instance')],
             result='optional-camera-state31', policy=POLICY),
        dict(id=first_id + 1, name='camera.set_information',
             arguments=[dict(type='u8', name='instance'),
                        dict(type='owned-camera-information230', name='information')],
             result='void', policy=POLICY),
    ], vendor_size=VENDOR_SIZE, model_size=MODEL_SIZE, uri_size=URI_SIZE)


def cpp(spec):
    state, information = spec['methods']
    return [
        f'// {state["name"]}',
        'static __attribute__((noinline)) int32_t binding_camera_get_state(',
        '    const uint8_t *input, uint32_t, uint8_t *output, uint32_t)',
        '{',
        '    (void)input;',
        f'    memset(output, 0, {STATE_WIRE_SIZE});',
        '#if AP_CAMERA_ENABLED && (AP_CAMERA_SCRIPTING_ENABLED == 1)',
        '    auto *camera = AP_Camera::get_singleton();',
        '    if (camera == nullptr) { return AP_MODULE_STATUS_UNAVAILABLE; }',
        '    BindingLock lock(camera->get_semaphore());',
        '    AP_Camera::camera_state_t value{};',
        '    if (!camera->get_state(input[0], value)) { return AP_MODULE_STATUS_OK; }',
        '    output[0] = 1;',
        '    BindingWire::write(output + 1, value.take_pic_incr, 2);',
        '    output[3] = value.recording_video ? 1 : 0;',
        '    output[4] = value.zoom_type;',
        '    BindingWire::write_float(output + 5, value.zoom_value);',
        '    output[9] = value.focus_type;',
        '    BindingWire::write_float(output + 10, value.focus_value);',
        '    output[14] = value.tracking_type;',
        '    BindingWire::write_float(output + 15, value.tracking_p1.x);',
        '    BindingWire::write_float(output + 19, value.tracking_p1.y);',
        '    BindingWire::write_float(output + 23, value.tracking_p2.x);',
        '    BindingWire::write_float(output + 27, value.tracking_p2.y);',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
        f'// {information["name"]}',
        'static __attribute__((noinline)) int32_t binding_camera_set_information(',
        '    const uint8_t *input, uint32_t, uint8_t *, uint32_t)',
        '{',
        '    (void)input;',
        '#if AP_CAMERA_ENABLED && (AP_CAMERA_SCRIPTING_ENABLED == 1) && AP_CAMERA_INFO_FROM_SCRIPT_ENABLED',
        '    auto *camera = AP_Camera::get_singleton();',
        '    if (camera == nullptr) { return AP_MODULE_STATUS_UNAVAILABLE; }',
        '    mavlink_camera_information_t value{};',
        '    value.firmware_version = BindingWire::read(input + 1, 4);',
        '    value.focal_length = BindingWire::read_float(input + 5);',
        '    value.sensor_size_h = BindingWire::read_float(input + 9);',
        '    value.sensor_size_v = BindingWire::read_float(input + 13);',
        '    value.flags = BindingWire::read(input + 17, 4);',
        '    value.resolution_h = uint16_t(BindingWire::read(input + 21, 2));',
        '    value.resolution_v = uint16_t(BindingWire::read(input + 23, 2));',
        '    value.cam_definition_version = uint16_t(BindingWire::read(input + 25, 2));',
        f'    memcpy(value.vendor_name, input + 27, {VENDOR_SIZE});',
        f'    memcpy(value.model_name, input + 59, {MODEL_SIZE});',
        f'    memcpy(value.cam_definition_uri, input + 91, {URI_SIZE});',
        '    BindingLock lock(camera->get_semaphore());',
        '    camera->set_camera_information(input[0], value);',
        '    return AP_MODULE_STATUS_OK;',
        '#else',
        '    return AP_MODULE_STATUS_UNAVAILABLE;',
        '#endif',
        '}',
        '',
    ]


def rust(spec, notice):
    state, information = spec['methods']
    return notice + f'''use crate::{{result, Error, Result, Vector2f}};

pub const CAMERA_VENDOR_NAME_SIZE: usize = {VENDOR_SIZE};
pub const CAMERA_MODEL_NAME_SIZE: usize = {MODEL_SIZE};
pub const CAMERA_DEFINITION_URI_SIZE: usize = {URI_SIZE};

#[derive(Clone, Copy, Debug, PartialEq)]
pub struct CameraState {{
    pub take_picture_increment: u16,
    pub recording_video: bool,
    pub zoom_type: u8,
    pub zoom_value: f32,
    pub focus_type: u8,
    pub focus_value: f32,
    pub tracking_type: u8,
    pub tracking_point_1: Vector2f,
    pub tracking_point_2: Vector2f,
}}

#[derive(Clone, Debug, PartialEq)]
pub struct CameraInformation {{
    pub firmware_version: u32,
    pub focal_length: f32,
    pub sensor_size_h: f32,
    pub sensor_size_v: f32,
    pub flags: u32,
    pub resolution_h: u16,
    pub resolution_v: u16,
    pub definition_version: u16,
    vendor_name: [u8; CAMERA_VENDOR_NAME_SIZE],
    model_name: [u8; CAMERA_MODEL_NAME_SIZE],
    definition_uri: [u8; CAMERA_DEFINITION_URI_SIZE],
}}

impl Default for CameraInformation {{
    fn default() -> Self {{ Self {{ firmware_version: 0, focal_length: 0.0, sensor_size_h: 0.0,
        sensor_size_v: 0.0, flags: 0, resolution_h: 0, resolution_v: 0, definition_version: 0,
        vendor_name: [0; CAMERA_VENDOR_NAME_SIZE], model_name: [0; CAMERA_MODEL_NAME_SIZE],
        definition_uri: [0; CAMERA_DEFINITION_URI_SIZE] }} }}
}}

fn set_text(output: &mut [u8], value: &str) -> Result {{
    let bytes = value.as_bytes();
    if bytes.len() >= output.len() || bytes.contains(&0) {{ return Err(Error::Invalid); }}
    output.fill(0); output[..bytes.len()].copy_from_slice(bytes); Ok(())
}}

impl CameraInformation {{
    pub fn set_vendor_name(&mut self, value: &str) -> Result {{ set_text(&mut self.vendor_name, value) }}
    pub fn set_model_name(&mut self, value: &str) -> Result {{ set_text(&mut self.model_name, value) }}
    pub fn set_definition_uri(&mut self, value: &str) -> Result {{ set_text(&mut self.definition_uri, value) }}
}}

impl crate::bindings::Camera<'_, '_> {{
    pub fn get_state(&self, instance: u8) -> Result<Option<CameraState>> {{
        let input = [instance];
        let mut output = [0u8; {STATE_WIRE_SIZE}];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{state['id']}.slot(), input.as_ptr(), 1,
            output.as_mut_ptr(), output.len() as u32) }})?;
        match output[0] {{
            0 if output[1..].iter().all(|byte| *byte == 0) => Ok(None),
            0 => Err(Error::Host),
            1 => Ok(Some(CameraState {{
                take_picture_increment: u16::from_le_bytes(output[1..3].try_into().unwrap()),
                recording_video: match output[3] {{ 0 => false, 1 => true, _ => return Err(Error::Host) }},
                zoom_type: output[4], zoom_value: f32::from_le_bytes(output[5..9].try_into().unwrap()),
                focus_type: output[9], focus_value: f32::from_le_bytes(output[10..14].try_into().unwrap()),
                tracking_type: output[14],
                tracking_point_1: Vector2f::decode(&output[15..23]),
                tracking_point_2: Vector2f::decode(&output[23..31]),
            }})),
            _ => Err(Error::Host),
        }}
    }}

    pub fn set_information(&self, instance: u8, value: &CameraInformation) -> Result {{
        let mut input = [0u8; {INFORMATION_WIRE_SIZE}]; input[0] = instance;
        input[1..5].copy_from_slice(&value.firmware_version.to_le_bytes());
        input[5..9].copy_from_slice(&value.focal_length.to_le_bytes());
        input[9..13].copy_from_slice(&value.sensor_size_h.to_le_bytes());
        input[13..17].copy_from_slice(&value.sensor_size_v.to_le_bytes());
        input[17..21].copy_from_slice(&value.flags.to_le_bytes());
        input[21..23].copy_from_slice(&value.resolution_h.to_le_bytes());
        input[23..25].copy_from_slice(&value.resolution_v.to_le_bytes());
        input[25..27].copy_from_slice(&value.definition_version.to_le_bytes());
        input[27..59].copy_from_slice(&value.vendor_name);
        input[59..91].copy_from_slice(&value.model_name);
        input[91..231].copy_from_slice(&value.definition_uri);
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{information['id']}.slot(), input.as_ptr(), input.len() as u32,
            core::ptr::null_mut(), 0) }})
    }}
}}
'''
