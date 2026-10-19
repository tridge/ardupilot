# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Lifecycle-owned raw MAVLink service for native modules."""

OWNER = 'mavlink'
MESSAGE_SIZE = 298
PAYLOAD_MAX = 255
RECEIVE_WIRE_SIZE = 1 + MESSAGE_SIZE + 1 + 4
SEND_WIRE_SIZE = 1 + 4 + 2 + 1 + 4 + PAYLOAD_MAX
ALIASES = {
    'init': 'lua_mavlink_init',
    'register_rx_msgid': 'lua_mavlink_register_rx_msgid',
    'receive_chan': 'lua_mavlink_receive_chan',
    'send_chan': 'lua_mavlink_send_chan',
    'block_command': 'lua_mavlink_block_command',
}
POLICY = ('v1:module-context-owned-rx,cleanup-after-worker-join,max8-modules,queue-and-id-max25,'
          'full-mavlink-message298,channel-u8,timestamp-ms-u32,queue-full-drops-newest,'
          'payload-max255,target-system-u32,target32-compatible,no-retained-module-pointers')
BLOCK_POLICY = ('v1:module-context-owned-command-blocks,cleanup-after-worker-join,u16-command-id,'
                'duplicate-owner-idempotent,shared-with-lua-owner-isolation,no-retained-module-pointers')


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    if owner is None:
        raise ValueError('missing mavlink singleton')
    expected = dict(rename=None, depends='(HAL_GCS_ENABLED && !defined(HAL_BUILD_AP_PERIPH))',
                    get=None, literal=False, reference=False, semaphore=False,
                    scheduler_semaphore=False, semaphore_pointer=False)
    if any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited mavlink owner metadata')
    for alias, target in ALIASES.items():
        matches = [item for item in owner['aliases'] if item['alias'] == alias]
        if matches != [dict(name=target, alias=alias, depends=None, manual=True)]:
            raise ValueError(f'unaudited mavlink.{alias} declaration')
    return specification(first_id)


def specification(first_id):
    methods = [
        dict(id=first_id, name='mavlink.init',
             arguments=[dict(type='u8', range='0..25', name='queue_depth'),
                        dict(type='u8', range='0..25', name='message_ids')], result='void', policy=POLICY),
        dict(id=first_id + 1, name='mavlink.register_rx_msgid',
             arguments=[dict(type='u24-in-u32-le', name='msgid')], result='bool-u8', policy=POLICY),
        dict(id=first_id + 2, name='mavlink.receive_chan', arguments=[],
             result=dict(presence='bool-u8', message=f'bytes{MESSAGE_SIZE}', channel='u8',
                         timestamp_ms='u32-le'), policy=POLICY),
        dict(id=first_id + 3, name='mavlink.send_chan',
             arguments=[dict(type='u8', name='channel'), dict(type='u24-in-u32-le', name='msgid'),
                        dict(type=f'bytes-max{PAYLOAD_MAX}-zero-pad{PAYLOAD_MAX}', name='payload'),
                        dict(type='optional-u32-le', name='target_system')],
             result='optional-bool-u8', policy=POLICY),
        dict(id=first_id + 4, name='mavlink.block_command',
             arguments=[dict(type='u16-le', name='command')], result='bool-u8', policy=BLOCK_POLICY),
    ]
    return dict(methods=methods)


def rust(spec, notice):
    init, register, receive, send, block = spec['methods']
    return notice + f'''use crate::{{result, Error, Result}};

pub const MAVLINK_MESSAGE_SIZE: usize = {MESSAGE_SIZE};
pub const MAVLINK_PAYLOAD_MAX: usize = {PAYLOAD_MAX};

/// An owned copy of the firmware's complete `mavlink_message_t`, including
/// its checksum, header, payload storage and MAVLink2.1 extended target.
#[derive(Clone, Debug, Eq, PartialEq)]
pub struct MavlinkMessage {{
    bytes: [u8; MAVLINK_MESSAGE_SIZE],
    pub channel: u8,
    pub timestamp_ms: u32,
}}

impl MavlinkMessage {{
    pub fn bytes(&self) -> &[u8; MAVLINK_MESSAGE_SIZE] {{ &self.bytes }}
}}

impl crate::bindings::Mavlink<'_, '_> {{
    /// Allocate this module's bounded receive queue and message-ID table.
    /// Repeated calls preserve the first allocation, matching the Lua API.
    pub fn init(&self, queue_depth: u8, message_ids: u8) -> Result {{
        if queue_depth > 25 || message_ids > 25 {{
            return Err(Error::Invalid);
        }}
        let input = [queue_depth, message_ids];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw, crate::imports::BINDING_{init['id']}.slot(),
                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)
        }})
    }}

    /// Add one 24-bit MAVLink message ID. Duplicate registrations return
    /// `false`; exhausting the configured table returns `Error::NoMemory`.
    pub fn register_rx_msgid(&self, msgid: u32) -> Result<bool> {{
        if msgid >= (1 << 24) {{
            return Err(Error::Invalid);
        }}
        let input = msgid.to_le_bytes();
        let mut output = [0u8; 1];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{register['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    /// Pop the oldest subscribed packet with its receive channel and timestamp.
    pub fn receive_chan(&self) -> Result<Option<MavlinkMessage>> {{
        let mut output = [0u8; {RECEIVE_WIRE_SIZE}];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{receive['id']}.slot(),
                core::ptr::null(), 0, output.as_mut_ptr(), output.len() as u32)
        }})?;
        match output[0] {{
            0 if output[1..].iter().all(|byte| *byte == 0) => Ok(None),
            0 => Err(Error::Host),
            1 => {{
                let mut bytes = [0u8; MAVLINK_MESSAGE_SIZE];
                bytes.copy_from_slice(&output[1..1 + MAVLINK_MESSAGE_SIZE]);
                Ok(Some(MavlinkMessage {{
                    bytes,
                    channel: output[1 + MAVLINK_MESSAGE_SIZE],
                    timestamp_ms: u32::from_le_bytes(
                        output[2 + MAVLINK_MESSAGE_SIZE..{RECEIVE_WIRE_SIZE}].try_into().unwrap()),
                }}))
            }}
            _ => Err(Error::Host),
        }}
    }}

    /// Send a known MAVLink payload. `None` means the channel does not exist;
    /// `Some(false)` means the packet could not currently be sent.
    pub fn send_chan(&self, channel: u8, msgid: u32, payload: &[u8],
                     target_system: Option<u32>) -> Result<Option<bool>> {{
        if msgid >= (1 << 24) || payload.len() > MAVLINK_PAYLOAD_MAX {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; {SEND_WIRE_SIZE}];
        input[0] = channel;
        input[1..5].copy_from_slice(&msgid.to_le_bytes());
        input[5..7].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        if let Some(target) = target_system {{
            input[7] = 1;
            input[8..12].copy_from_slice(&target.to_le_bytes());
        }}
        input[12..12 + payload.len()].copy_from_slice(payload);
        let mut output = [0u8; 2];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw, crate::imports::BINDING_{send['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        match output {{
            [0, 0] => Ok(None),
            [1, 0] => Ok(Some(false)),
            [1, 1] => Ok(Some(true)),
            _ => Err(Error::Host),
        }}
    }}

    /// Block a MAVLink command until this module exits. Repeated calls for the
    /// same module and command are idempotent; other owners remain independent.
    pub fn block_command(&self, command: u16) -> Result<bool> {{
        let input = command.to_le_bytes();
        let mut output = [0u8; 1];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{block['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}
}}
'''
