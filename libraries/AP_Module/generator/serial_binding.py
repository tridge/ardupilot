# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Revalidated, non-owning serial-port selectors and bounded transfers."""

TRANSFER_MAX = 256
OWNER = 'serial'
USERDATA = 'AP_Scripting_SerialAccess'
POLICY = ('v1:non-owning-selector,revalidate-every-call,simulated-generation-checked,'
          'shared-stream-with-lua-and-firmware,no-exclusive-ownership,no-config-rollback-on-exit,'
          'blocking-binding-lock,transfer-max256,fixed-zero-padded-wire,no-retained-module-pointers')


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    if owner is None:
        raise ValueError('missing serial singleton')
    expected_owner = dict(rename=None, depends='AP_SERIALMANAGER_ENABLED', get=None, literal=False,
                          reference=False, semaphore=False, scheduler_semaphore=False,
                          semaphore_pointer=False)
    if any(owner[key] != value for key, value in expected_owner.items()) or owner['methods']:
        raise ValueError('unaudited serial owner metadata')
    expected_aliases = {
        'find_serial': dict(name='lua_serial_find_serial', alias='find_serial', depends=None, manual=True),
        'find_simulated_device': dict(name='lua_serial_find_simulated_device', alias='find_simulated_device',
                                      depends='AP_SCRIPTING_SERIALDEVICE_ENABLED', manual=True),
    }
    if {item['alias']: item for item in owner['aliases']} != expected_aliases:
        raise ValueError('unaudited serial aliases')

    userdata = next((item for item in ir['userdata'] if item['name'] == USERDATA), None)
    expected_userdata = dict(rename=None, depends=None, get=None, creation='null', creation_manual=True,
                             creation_args=-1, has_fields=False, has_enums=False, operations=0,
                             operator_getter=None, operators_manual=False, operator_names=[], literal=False,
                             reference=False, semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if userdata is None or any(userdata[key] != value for key, value in expected_userdata.items()):
        raise ValueError('unaudited serial userdata metadata')

    def field_shape(field):
        return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
                field['skip_check'], tuple(field['range']) if field['range'] else None)

    def shape(result, arguments):
        return (result, None, False, False, False, False, None), arguments

    expected_methods = {
        'available': shape('uint32_t', []),
        'configure_parity': shape('void', [('uint8_t', None, False, False, False, False, ('0', '2'))]),
        'read': shape('int16_t', []),
        'set_flow_control': shape('void', [('enum', 'AP_HAL::UARTDriver::flow_control', False, False, False,
                                            False, ('AP_HAL::UARTDriver::FLOW_CONTROL_DISABLE',
                                                    'AP_HAL::UARTDriver::FLOW_CONTROL_RTS_DE'))]),
        'set_stop_bits': shape('void', [('uint8_t', None, False, False, False, False, ('1', '2'))]),
        'set_unbuffered_writes': shape('void', [('boolean', None, False, False, False, False, None)]),
        'write': shape('uint32_t', [('uint8_t', None, False, False, False, True, None)]),
    }
    actual_methods = {
        method['name']: (field_shape(method['return']), [field_shape(arg) for arg in method['arguments']])
        for method in userdata['methods']
        if method['rename'] is None and method['depends'] is None and method['deprecate'] is None
    }
    if actual_methods != expected_methods:
        raise ValueError('unaudited serial userdata methods')
    expected_userdata_aliases = {
        'begin': 'lua_serial_begin', 'readstring': 'lua_serial_readstring',
        'writestring': 'lua_serial_writestring',
    }
    actual_aliases = {item['alias']: item['name'] for item in userdata['aliases']
                      if item['manual'] and item['depends'] is None}
    if len(actual_aliases) != len(userdata['aliases']) or actual_aliases != expected_userdata_aliases:
        raise ValueError('unaudited serial userdata aliases')
    return specification(first_id)


def specification(first_id):
    handle = dict(type='u32-le,nonzero,revalidated-serial-selector', direction='borrowed-receiver')
    optional_handle = dict(presence='u32-le,zero-is-none', handle='SerialPort')
    methods = [
        dict(id=first_id, name='Serial.find_serial',
             arguments=[dict(type='u8', direction='in', name='instance')],
             result=optional_handle, policy=POLICY),
        dict(id=first_id + 1, name='Serial.find_simulated_device',
             arguments=[dict(type='u7', direction='in', name='protocol'),
                        dict(type='u16-le', direction='in', name='instance')],
             result=optional_handle, policy=POLICY),
        dict(id=first_id + 2, name='SerialPort.begin',
             arguments=[handle, dict(type='optional-nonzero-u32-le', direction='in', name='baud')],
             result='void', policy=POLICY),
        dict(id=first_id + 3, name='SerialPort.configure_parity',
             arguments=[handle, dict(type='u8', direction='in', range=[0, 2])],
             result='void', policy=POLICY),
        dict(id=first_id + 4, name='SerialPort.set_stop_bits',
             arguments=[handle, dict(type='u8', direction='in', range=[1, 2])],
             result='void', policy=POLICY),
        dict(id=first_id + 5, name='SerialPort.set_unbuffered_writes',
             arguments=[handle, dict(type='bool-u8', direction='in')], result='void', policy=POLICY),
        dict(id=first_id + 6, name='SerialPort.set_flow_control',
             arguments=[handle, dict(type='u8', direction='in', range=[0, 3])],
             result='void', policy=POLICY),
        dict(id=first_id + 7, name='SerialPort.available', arguments=[handle],
             result='u32-le', policy=POLICY),
        dict(id=first_id + 8, name='SerialPort.read',
             arguments=[handle, dict(type='u16-le', direction='in', range=[1, TRANSFER_MAX])],
             result=dict(length='u16-le', data=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}'),
             policy=POLICY),
        dict(id=first_id + 9, name='SerialPort.write',
             arguments=[handle, dict(type=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}', direction='in')],
             result='u16-le', policy=POLICY),
    ]
    return dict(methods=methods, transfer_max=TRANSFER_MAX)


def rust(spec, notice):
    (find, simulated, begin, parity, stop_bits, unbuffered, flow_control,
     available, read, write) = spec['methods']
    return notice + f'''use crate::{{result, Context, Error, Result}};
use core::num::NonZeroU32;

pub const SERIAL_TRANSFER_MAX: usize = {TRANSFER_MAX};

/// A non-owning serial selector. Firmware resolves it again on every call, so
/// a removed/reconfigured port returns `Unavailable` instead of dereferencing
/// a stale pointer. Clones share the same byte stream with firmware and Lua.
/// Configuration changes are not rolled back when a module exits.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SerialPort {{ token: NonZeroU32 }}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum SerialParity {{ None = 0, Odd = 1, Even = 2 }}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum SerialStopBits {{ One = 1, Two = 2 }}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
#[repr(u8)]
pub enum SerialFlowControl {{ Disabled = 0, Enabled = 1, Auto = 2, RtsDe = 3 }}

fn decode_port(output: [u8; 4]) -> Option<SerialPort> {{
    NonZeroU32::new(u32::from_le_bytes(output)).map(|token| SerialPort {{ token }})
}}

impl crate::bindings::Serial<'_, '_> {{
    /// Find the nth physical UART configured for SERIAL_PROTOCOL=28.
    pub fn find_serial(&self, instance: u8) -> Result<Option<SerialPort>> {{
        let input = [instance];
        let mut output = [0u8; 4];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{find['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        Ok(decode_port(output))
    }}

    /// Find the nth shared SCR_SDEV port configured for `protocol`.
    /// This pool is available to Rust even when the Lua runtime is disabled.
    pub fn find_simulated_device(&self, protocol: u8,
                                 instance: u16) -> Result<Option<SerialPort>> {{
        if protocol > i8::MAX as u8 {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; 3];
        input[0] = protocol;
        input[1..].copy_from_slice(&instance.to_le_bytes());
        let mut output = [0u8; 4];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{simulated['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        Ok(decode_port(output))
    }}
}}

impl SerialPort {{
    fn setting(&self, context: &Context<'_>, id: u32, value: u8) -> Result {{
        let mut input = [0u8; 5];
        input[..4].copy_from_slice(&self.token.get().to_le_bytes());
        input[4] = value;
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, id, input.as_ptr(), input.len() as u32,
                core::ptr::null_mut(), 0)
        }})
    }}

    /// Start a physical UART. `None` reapplies its configured baud rate.
    /// Beginning a simulated-device view is a validated no-op, matching Lua.
    pub fn begin(&self, context: &Context<'_>, baud: Option<u32>) -> Result {{
        if baud == Some(0) {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; 9];
        input[..4].copy_from_slice(&self.token.get().to_le_bytes());
        if let Some(baud) = baud {{
            input[4] = 1;
            input[5..].copy_from_slice(&baud.to_le_bytes());
        }}
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{begin['id']}.slot(),
                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)
        }})
    }}

    pub fn configure_parity(&self, context: &Context<'_>, parity: SerialParity) -> Result {{
        self.setting(context, crate::imports::BINDING_{parity['id']}.slot(), parity as u8)
    }}

    pub fn set_stop_bits(&self, context: &Context<'_>, stop_bits: SerialStopBits) -> Result {{
        self.setting(context, crate::imports::BINDING_{stop_bits['id']}.slot(), stop_bits as u8)
    }}

    pub fn set_unbuffered_writes(&self, context: &Context<'_>, enabled: bool) -> Result {{
        self.setting(context, crate::imports::BINDING_{unbuffered['id']}.slot(), enabled as u8)
    }}

    pub fn set_flow_control(&self, context: &Context<'_>, mode: SerialFlowControl) -> Result {{
        self.setting(context, crate::imports::BINDING_{flow_control['id']}.slot(), mode as u8)
    }}

    pub fn available(&self, context: &Context<'_>) -> Result<u32> {{
        let input = self.token.get().to_le_bytes();
        let mut output = [0u8; 4];
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{available['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        Ok(u32::from_le_bytes(output))
    }}

    /// Nonblocking read of at most 256 bytes. Only returned bytes are changed.
    pub fn read(&self, context: &Context<'_>, buffer: &mut [u8]) -> Result<usize> {{
        if buffer.is_empty() {{
            return Ok(0);
        }}
        if buffer.len() > SERIAL_TRANSFER_MAX {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; 6];
        input[..4].copy_from_slice(&self.token.get().to_le_bytes());
        input[4..].copy_from_slice(&(buffer.len() as u16).to_le_bytes());
        let mut output = [0u8; SERIAL_TRANSFER_MAX + 2];
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{read['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        let length = u16::from_le_bytes(output[..2].try_into().unwrap()) as usize;
        if length > buffer.len() || output[2 + length..].iter().any(|byte| *byte != 0) {{
            return Err(Error::Host);
        }}
        buffer[..length].copy_from_slice(&output[2..2 + length]);
        Ok(length)
    }}

    pub fn read_byte(&self, context: &Context<'_>) -> Result<Option<u8>> {{
        let mut byte = [0u8];
        Ok((self.read(context, &mut byte)? == 1).then_some(byte[0]))
    }}

    /// Nonblocking write of at most 256 bytes. Partial writes are reported.
    pub fn write(&self, context: &Context<'_>, buffer: &[u8]) -> Result<usize> {{
        if buffer.is_empty() {{
            return Ok(0);
        }}
        if buffer.len() > SERIAL_TRANSFER_MAX {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; SERIAL_TRANSFER_MAX + 6];
        input[..4].copy_from_slice(&self.token.get().to_le_bytes());
        input[4..6].copy_from_slice(&(buffer.len() as u16).to_le_bytes());
        input[6..6 + buffer.len()].copy_from_slice(buffer);
        let mut output = [0u8; 2];
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{write['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        let written = u16::from_le_bytes(output) as usize;
        if written > buffer.len() {{
            return Err(Error::Host);
        }}
        Ok(written)
    }}

    pub fn write_byte(&self, context: &Context<'_>, byte: u8) -> Result<bool> {{
        Ok(self.write(context, &[byte])? == 1)
    }}
}}
'''
