# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Module-owned raw CAN buffers with an owned fixed-size frame value."""

OWNER = 'CAN'
BUFFER = 'ScriptingCANBuffer'
FRAME = 'AP_HAL::CANFrame'
DEPENDS = 'AP_SCRIPTING_CAN_SENSOR_ENABLED'
BUFFER_LENGTH_MAX = 25
BUFFER_COUNT_MAX = 16
FILTER_COUNT_MAX = 8
FRAME_DATA_MAX = 64
FRAME_WIRE_SIZE = 4 + 1 + 1 + FRAME_DATA_MAX
WRITE_TIMEOUT_MAX_US = 1_000_000
POLICY = ('v1:module-context-owned-buffer,max16-firmware-wide,buffer-length1-25,filter-max8,'
          'cleanup-after-worker-join,shared-sensor-with-lua,no-retained-module-pointers,'
          'token-revalidated-by-context,zero-skipped-live-token-collision-scan,'
          'owned-frame-data64,dlc-and-canfd-validated,'
          'write-timeout-max1000000us')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _method_shape(method):
    return (_field_shape(method['return']), tuple(_field_shape(arg) for arg in method['arguments']),
            method['rename'], method['depends'], method['deprecate'])


def _value_metadata(owner):
    keys = ('rename', 'depends', 'get', 'literal', 'reference', 'semaphore',
            'scheduler_semaphore', 'semaphore_pointer', 'creation', 'creation_args', 'creation_manual',
            'has_fields', 'has_enums', 'operations', 'operator_getter', 'operators_manual')
    return tuple(owner[key] for key in keys) + (tuple(owner['operator_names']),)


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_owner = dict(rename=None, depends=DEPENDS, get=None, literal=False, reference=False,
                          semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    expected_aliases = {
        'get_device': ('lua_get_CAN_device', None, True),
        'get_device2': ('lua_get_CAN_device2', None, True),
    }
    if (owner is None or any(owner[key] != value for key, value in expected_owner.items()) or
            owner['methods'] or owner['fields'] or
            {item['alias']: (item['name'], item['depends'], item['manual'])
             for item in owner['aliases']} != expected_aliases or len(owner['aliases']) != len(expected_aliases)):
        raise ValueError('unaudited CAN owner metadata')

    frame = next((item for item in ir['userdata'] if item['name'] == FRAME), None)
    expected_frame_metadata = ('CANFrame', DEPENDS, None, False, False, False, False, False,
                               None, 0, False, True, False, 0, None, False, ())
    expected_fields = {
        'id': (('uint32_t', None, False, False, False, True, None), None),
        'data': (('uint8_t', None, False, False, False, True, None), 'int(ARRAY_SIZE(ud->data))'),
        'dlc': (('uint8_t', None, False, False, False, False,
                 ('0', 'int(ARRAY_SIZE(ud->data))')), None),
    }
    actual_fields = {
        field['name']: ((_field_shape(field['type'])), field['array_len'])
        for field in frame['fields']
    } if frame is not None else {}
    if (frame is None or _value_metadata(frame) != expected_frame_metadata or frame['aliases'] or
            len(frame['fields']) != len(expected_fields) or actual_fields != expected_fields or
            any(not field['read'] or not field['write'] or field['rename'] is not None or
                field['depends'] is not None or field['valid_mask_name'] is not None or
                field['valid_mask_value'] is not None for field in frame['fields'])):
        raise ValueError('unaudited CAN frame fields')
    expected_frame_methods = {
        'id_signed': (('int32_t', None, False, False, False, False, None), (), None, None, None),
        'isExtended': (('boolean', None, False, False, False, False, None), (), None, None, None),
        'isRemoteTransmissionRequest': (
            ('boolean', None, False, False, False, False, None), (), None, None, None),
        'isErrorFrame': (('boolean', None, False, False, False, False, None), (), None, None, None),
    }
    actual_frame_methods = {method['name']: _method_shape(method) for method in frame['methods']}
    if len(frame['methods']) != len(expected_frame_methods) or actual_frame_methods != expected_frame_methods:
        raise ValueError('unaudited CAN frame methods')

    buffer = next((item for item in ir['ap_objects'] if item['name'] == BUFFER), None)
    expected_buffer_metadata = (None, DEPENDS, None, False, False, False, False, False,
                                None, 0, False, False, False, 0, None, False, ())
    if (buffer is None or _value_metadata(buffer) != expected_buffer_metadata or buffer['aliases'] or
            buffer['fields']):
        raise ValueError('unaudited CAN buffer metadata')
    boolean = ('boolean', None, False, False, False, False, None)
    frame_value = ('userdata', FRAME, False, False, False, False, None)
    expected_buffer_methods = {
        'write_frame': (boolean, (frame_value, ('uint32_t', None, False, False, False, True, None)),
                        None, None, None),
        'read_frame': (boolean, (('userdata', FRAME, True, False, False, False, None),),
                       None, None, None),
        'add_filter': (boolean, (('uint32_t', None, False, False, False, True, None),
                                 ('uint32_t', None, False, False, False, True, None)), None, None, None),
    }
    actual_buffer_methods = {method['name']: _method_shape(method) for method in buffer['methods']}
    if len(buffer['methods']) != len(expected_buffer_methods) or actual_buffer_methods != expected_buffer_methods:
        raise ValueError('unaudited CAN buffer methods')
    return specification(first_id)


def specification(first_id):
    handle = dict(type='u32-le,nonzero', direction='module-owned-receiver')
    frame = dict(name='CanFrame', fields=[['id', 'u32-le'], ['dlc', 'u8'], ['can_fd', 'bool-u8'],
                                          ['data', f'bytes{FRAME_DATA_MAX}']])
    methods = [
        dict(id=first_id, name='CAN.get_device',
             arguments=[dict(type='u8', range=[1, BUFFER_LENGTH_MAX], name='buffer_length')],
             result=dict(presence='u32-le,zero-is-none', handle='CanBuffer'), policy=POLICY),
        dict(id=first_id + 1, name='CAN.get_device2',
             arguments=[dict(type='u8', range=[1, BUFFER_LENGTH_MAX], name='buffer_length')],
             result=dict(presence='u32-le,zero-is-none', handle='CanBuffer'), policy=POLICY),
        dict(id=first_id + 2, name='CanBuffer.read_frame', arguments=[handle],
             result=dict(presence='bool-u8', frame=frame), policy=POLICY),
        dict(id=first_id + 3, name='CanBuffer.write_frame',
             arguments=[handle, dict(type=frame, direction='in'),
                        dict(type='u32-le', name='timeout_us', range=[0, WRITE_TIMEOUT_MAX_US])],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 4, name='CanBuffer.add_filter',
             arguments=[handle, dict(type='u32-le', name='mask'), dict(type='u32-le', name='value')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 5, name='CanBuffer.close', arguments=[handle], result='void', policy=POLICY),
    ]
    return dict(methods=methods)


def rust(spec, notice):
    get, get2, read, write, add_filter, close = spec['methods']
    return notice + f'''use crate::{{result, Context, Error, Result}};
use core::num::NonZeroU32;

pub const CAN_FRAME_DATA_MAX: usize = {FRAME_DATA_MAX};
pub const CAN_BUFFER_LENGTH_MAX: u8 = {BUFFER_LENGTH_MAX};
pub const CAN_WRITE_TIMEOUT_MAX_US: u32 = {WRITE_TIMEOUT_MAX_US};
pub const CAN_FLAG_EFF: u32 = 1 << 31;
pub const CAN_FLAG_RTR: u32 = 1 << 30;
pub const CAN_FLAG_ERR: u32 = 1 << 29;
pub const CAN_MASK_STD_ID: u32 = 0x7ff;
pub const CAN_MASK_EXT_ID: u32 = 0x1fff_ffff;

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CanFrame {{
    id: u32,
    dlc: u8,
    can_fd: bool,
    data: [u8; CAN_FRAME_DATA_MAX],
}}

impl Default for CanFrame {{
    fn default() -> Self {{ Self {{ id: 0, dlc: 0, can_fd: false, data: [0; CAN_FRAME_DATA_MAX] }} }}
}}

impl CanFrame {{
    pub fn id(&self) -> u32 {{ self.id }}
    pub fn set_id(&mut self, id: u32) {{ self.id = id; }}
    pub fn dlc(&self) -> u8 {{ self.dlc }}
    pub fn set_dlc(&mut self, dlc: u8) -> Result {{
        if dlc > 15 || (!self.can_fd && dlc > 8) {{ return Err(Error::Invalid); }}
        self.dlc = dlc;
        Ok(())
    }}
    pub fn can_fd(&self) -> bool {{ self.can_fd }}
    pub fn set_can_fd(&mut self, can_fd: bool) -> Result {{
        if !can_fd && self.dlc > 8 {{ return Err(Error::Invalid); }}
        self.can_fd = can_fd;
        Ok(())
    }}
    pub fn data(&self, index: usize) -> Result<u8> {{
        self.data.get(index).copied().ok_or(Error::Invalid)
    }}
    pub fn set_data(&mut self, index: usize, value: u8) -> Result {{
        *self.data.get_mut(index).ok_or(Error::Invalid)? = value;
        Ok(())
    }}
    pub fn payload(&self) -> &[u8] {{ &self.data[..self.data_length()] }}
    pub fn data_length(&self) -> usize {{
        match self.dlc {{ 0..=8 => self.dlc as usize, 9 => 12, 10 => 16, 11 => 20,
                         12 => 24, 13 => 32, 14 => 48, _ => 64 }}
    }}
    pub fn id_signed(&self) -> i32 {{
        (self.id & if self.is_extended() {{ CAN_MASK_EXT_ID }} else {{ CAN_MASK_STD_ID }}) as i32
    }}
    pub fn is_extended(&self) -> bool {{ self.id & CAN_FLAG_EFF != 0 }}
    pub fn is_remote_transmission_request(&self) -> bool {{ self.id & CAN_FLAG_RTR != 0 }}
    pub fn is_error_frame(&self) -> bool {{ self.id & CAN_FLAG_ERR != 0 }}

    fn encode(&self, output: &mut [u8; {FRAME_WIRE_SIZE}]) -> Result {{
        if self.dlc > 15 || (!self.can_fd && self.dlc > 8) {{ return Err(Error::Invalid); }}
        output[..4].copy_from_slice(&self.id.to_le_bytes());
        output[4] = self.dlc;
        output[5] = u8::from(self.can_fd);
        output[6..].copy_from_slice(&self.data);
        Ok(())
    }}

    fn decode(input: &[u8]) -> Result<Self> {{
        if input.len() != {FRAME_WIRE_SIZE} || input[5] > 1 || input[4] > 15 ||
            (input[5] == 0 && input[4] > 8) {{ return Err(Error::Host); }}
        let mut frame = Self::default();
        frame.id = u32::from_le_bytes(input[..4].try_into().unwrap());
        frame.dlc = input[4];
        frame.can_fd = input[5] != 0;
        frame.data.copy_from_slice(&input[6..]);
        Ok(frame)
    }}
}}

#[derive(Debug, Eq, PartialEq)]
pub struct CanBuffer {{ token: NonZeroU32 }}

fn decode_buffer(output: [u8; 4]) -> Option<CanBuffer> {{
    NonZeroU32::new(u32::from_le_bytes(output)).map(|token| CanBuffer {{ token }})
}}

impl crate::bindings::Can<'_, '_> {{
    fn get(&self, id: u32, buffer_length: u8) -> Result<Option<CanBuffer>> {{
        if buffer_length == 0 || buffer_length > CAN_BUFFER_LENGTH_MAX {{ return Err(Error::Invalid); }}
        let input = [buffer_length];
        let mut output = [0u8; 4];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw, id, input.as_ptr(), 1,
                                                        output.as_mut_ptr(), 4) }})?;
        Ok(decode_buffer(output))
    }}

    pub fn get_device(&self, buffer_length: u8) -> Result<Option<CanBuffer>> {{
        self.get(crate::imports::BINDING_{get['id']}.slot(), buffer_length)
    }}

    pub fn get_device2(&self, buffer_length: u8) -> Result<Option<CanBuffer>> {{
        self.get(crate::imports::BINDING_{get2['id']}.slot(), buffer_length)
    }}
}}

impl CanBuffer {{
    pub fn read_frame(&self, context: &Context<'_>) -> Result<Option<CanFrame>> {{
        let input = self.token.get().to_le_bytes();
        let mut output = [0u8; {FRAME_WIRE_SIZE + 1}];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{read['id']}.slot(), input.as_ptr(), 4,
            output.as_mut_ptr(), output.len() as u32) }})?;
        match output[0] {{ 0 => Ok(None), 1 => Ok(Some(CanFrame::decode(&output[1..])?)),
                          _ => Err(Error::Host) }}
    }}

    pub fn write_frame(&self, context: &Context<'_>, frame: &CanFrame, timeout_us: u32) -> Result<bool> {{
        if timeout_us > CAN_WRITE_TIMEOUT_MAX_US {{ return Err(Error::Invalid); }}
        let mut input = [0u8; {4 + FRAME_WIRE_SIZE + 4}];
        input[..4].copy_from_slice(&self.token.get().to_le_bytes());
        let encoded: &mut [u8; {FRAME_WIRE_SIZE}] = (&mut input[4..{4 + FRAME_WIRE_SIZE}]).try_into().unwrap();
        frame.encode(encoded)?;
        input[{4 + FRAME_WIRE_SIZE}..].copy_from_slice(&timeout_us.to_le_bytes());
        let mut output = [0u8; 1];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{write['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), 1) }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    pub fn add_filter(&self, context: &Context<'_>, mask: u32, value: u32) -> Result<bool> {{
        let mut input = [0u8; 12];
        input[..4].copy_from_slice(&self.token.get().to_le_bytes());
        input[4..8].copy_from_slice(&mask.to_le_bytes());
        input[8..].copy_from_slice(&value.to_le_bytes());
        let mut output = [0u8; 1];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{add_filter['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), 1) }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    pub fn close(self, context: &Context<'_>) -> Result {{
        let input = self.token.get().to_le_bytes();
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{close['id']}.slot(), input.as_ptr(), 4,
            core::ptr::null_mut(), 0) }})
    }}
}}
'''
