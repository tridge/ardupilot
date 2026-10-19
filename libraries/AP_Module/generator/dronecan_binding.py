# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Lifecycle-owned DroneCAN broadcast subscriptions for native modules."""

PAYLOAD_MAX = 256
MESSAGE_WIRE_SIZE = 1 + 2 + PAYLOAD_MAX + 1 + 8 + 1
POLICY = ('v1:module-context-owned-subscription,max16-firmware-wide,cleanup-after-worker-join,'
          'subscription-transfer-type-broadcast,queue-depth8,payload-max256,oversize-and-full-queue-drop,'
          'owned-broadcast-handle,priority0,timeout10ms,per-handle-transfer-id,can-fd-option,'
          'no-retained-module-pointers,token-revalidated-by-context,'
          'zero-skipped-live-token-collision-scan')
REQUEST_POLICY = (POLICY + ',request-node1-to127,response-node-and-5bit-transfer-id-filter,'
                  'failed-request-removes-response-subscription')


def select(ir, first_id):
    owner = next((item for item in ir['userdata'] if item['name'] == 'DroneCAN_Handle'), None)
    expected = dict(rename=None, depends='HAL_ENABLE_DRONECAN_DRIVERS', get=None,
                    creation='DroneCAN_Handle::new_handle', creation_manual=True, creation_args=4,
                    has_fields=False, has_enums=False, operator_getter=None, operators_manual=True,
                    operator_names=[], literal=False, reference=False, semaphore=False,
                    scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited DroneCAN_Handle metadata')
    subscribe = [method for method in owner['methods'] if method['name'] == 'subscribe']
    if len(subscribe) != 1:
        raise ValueError('unaudited DroneCAN_Handle.subscribe method')
    method = subscribe[0]
    result = method['return']
    result_shape = (result['kind'], result['name'], result['nullable'], result['reference'],
                    result['pointer'], result['skip_check'], result['range'])
    if (result_shape != ('boolean', None, False, False, False, False, None) or method['arguments'] or
            method['rename'] is not None or method['depends'] is not None or method['deprecate'] is not None):
        raise ValueError('unaudited DroneCAN_Handle.subscribe method')
    aliases = sorted((alias['name'], alias['alias'], alias['depends'], alias['manual'])
                     for alias in owner['aliases'])
    expected_aliases = sorted([
        ('DroneCAN_Handle::__gc', '__gc', None, True),
        ('DroneCAN_Handle::broadcast', 'broadcast', None, True),
        ('DroneCAN_Handle::check_message', 'check_message', None, True),
        ('DroneCAN_Handle::request', 'request', None, True),
    ])
    if aliases != expected_aliases:
        raise ValueError('unaudited DroneCAN_Handle manual operations')
    return specification(first_id)


def specification(first_id):
    subscribe = dict(id=first_id, name='DroneCan.subscribe',
                     arguments=[dict(type='u8', direction='in'), dict(type='u64-le', direction='in'),
                                dict(type='u16-le', direction='in')],
                     result=dict(presence='u32-le,zero-is-none', handle='DroneCanSubscription'), policy=POLICY)
    check = dict(id=first_id + 1, name='DroneCanSubscription.check_message',
                 contract_aliases=['DroneCanHandle.check_message'],
                 arguments=[dict(type='u32-le,nonzero', direction='module-owned-receiver')],
                 result=dict(presence='bool-u8', length='u16-le',
                             payload=f'bytes-max{PAYLOAD_MAX}-zero-pad{PAYLOAD_MAX}', node_id='u8',
                             timestamp_us='u64-le', can_fd='bool-u8'), policy=POLICY)
    close = dict(id=first_id + 2, name='DroneCanSubscription.close',
                 arguments=[dict(type='u32-le,nonzero', direction='module-owned-receiver')],
                 result='void', policy=POLICY)
    open_handle = dict(id=first_id + 3, name='DroneCan.open',
                       arguments=[dict(type='u8', direction='in'), dict(type='u64-le', direction='in'),
                                  dict(type='u16-le', direction='in'), dict(type='bool-u8', direction='in')],
                       result=dict(presence='u32-le,zero-is-none', handle='DroneCanHandle'), policy=POLICY)
    broadcast = dict(id=first_id + 4, name='DroneCanHandle.broadcast',
                     arguments=[dict(type='u32-le,nonzero', direction='module-owned-receiver'),
                                dict(type=f'bytes-max{PAYLOAD_MAX}', direction='in')],
                     result='bool-u8', policy=POLICY)
    handle_close = dict(id=first_id + 5, name='DroneCanHandle.close',
                        arguments=[dict(type='u32-le,nonzero', direction='module-owned-receiver')],
                        result='void', policy=POLICY)
    request = dict(id=first_id + 6, name='DroneCanHandle.request',
                   arguments=[dict(type='u32-le,nonzero', direction='module-owned-receiver'),
                              dict(type='u8', range=[1, 127], direction='target-node'),
                              dict(type=f'bytes-max{PAYLOAD_MAX}', direction='in')],
                   result='bool-u8', policy=REQUEST_POLICY)
    return dict(methods=[subscribe, check, close, open_handle, broadcast, handle_close, request])


def rust(spec, notice):
    subscribe, check, close, open_handle, broadcast, handle_close, request = spec['methods']
    return notice + f'''use crate::{{result, Context, Error, Result}};
use core::num::NonZeroU32;

pub const DRONECAN_PAYLOAD_MAX: usize = {PAYLOAD_MAX};

/// An owned firmware subscription token. The host also closes every remaining
/// token after this module's worker has joined, including after a stack fault.
/// Passing the token through another module context is rejected.
#[derive(Debug, Eq, PartialEq)]
pub struct DroneCanSubscription {{ token: NonZeroU32 }}

/// An owned outgoing DroneCAN handle. Its transfer-ID is kept in firmware and
/// advances with each successfully queued broadcast or request.
#[derive(Debug, Eq, PartialEq)]
pub struct DroneCanHandle {{ token: NonZeroU32 }}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DroneCanMessage {{
    payload: [u8; DRONECAN_PAYLOAD_MAX],
    length: u16,
    pub node_id: u8,
    pub timestamp_us: u64,
    pub can_fd: bool,
}}

impl DroneCanMessage {{
    pub fn payload(&self) -> &[u8] {{ &self.payload[..self.length as usize] }}
}}

fn dronecan_check_message(context: &Context<'_>, token: NonZeroU32) -> Result<Option<DroneCanMessage>> {{
    let input = token.get().to_le_bytes();
    let mut output = [0u8; {MESSAGE_WIRE_SIZE}];
    result(unsafe {{
        context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{check['id']}.slot(),
            input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
    }})?;
    match output[0] {{
        0 => Ok(None),
        1 => {{
            let length = u16::from_le_bytes(output[1..3].try_into().unwrap());
            if length as usize > DRONECAN_PAYLOAD_MAX || output[{MESSAGE_WIRE_SIZE - 1}] > 1 {{
                return Err(Error::Host);
            }}
            let mut payload = [0u8; DRONECAN_PAYLOAD_MAX];
            payload.copy_from_slice(&output[3..{3 + PAYLOAD_MAX}]);
            Ok(Some(DroneCanMessage {{
                payload,
                length,
                node_id: output[{3 + PAYLOAD_MAX}],
                timestamp_us: u64::from_le_bytes(output[{4 + PAYLOAD_MAX}..{12 + PAYLOAD_MAX}].try_into().unwrap()),
                can_fd: output[{12 + PAYLOAD_MAX}] != 0,
            }}))
        }}
        _ => Err(Error::Host),
    }}
}}

impl crate::bindings::Dronecan<'_, '_> {{
    /// Subscribe to one DroneCAN broadcast type on a configured driver.
    /// `None` means that driver index is not available.
    pub fn subscribe(&self, driver: u8, signature: u64,
                     data_type: u16) -> Result<Option<DroneCanSubscription>> {{
        let mut input = [0u8; 11];
        input[0] = driver;
        input[1..9].copy_from_slice(&signature.to_le_bytes());
        input[9..11].copy_from_slice(&data_type.to_le_bytes());
        let mut output = [0u8; 4];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{subscribe['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        Ok(NonZeroU32::new(u32::from_le_bytes(output)).map(|token| DroneCanSubscription {{ token }}))
    }}

    /// Open a bounded outgoing handle without installing a receive subscriber.
    /// `None` means that driver index is not available.
    pub fn open(&self, driver: u8, signature: u64, data_type: u16,
                can_fd: bool) -> Result<Option<DroneCanHandle>> {{
        let mut input = [0u8; 12];
        input[0] = driver;
        input[1..9].copy_from_slice(&signature.to_le_bytes());
        input[9..11].copy_from_slice(&data_type.to_le_bytes());
        input[11] = can_fd as u8;
        let mut output = [0u8; 4];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{open_handle['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        Ok(NonZeroU32::new(u32::from_le_bytes(output)).map(|token| DroneCanHandle {{ token }}))
    }}
}}

impl DroneCanSubscription {{
    /// Pop the oldest pending transfer. Queues hold eight transfers; payloads
    /// over 256 bytes and arrivals while full are dropped by the host.
    pub fn check_message(&self, context: &Context<'_>) -> Result<Option<DroneCanMessage>> {{
        dronecan_check_message(context, self.token)
    }}

    /// Close before module exit. Dropping without calling this is still safe:
    /// firmware lifecycle cleanup closes it after the worker joins.
    pub fn close(self, context: &Context<'_>) -> Result {{
        let input = self.token.get().to_le_bytes();
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{close['id']}.slot(),
                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)
        }})
    }}
}}

impl DroneCanHandle {{
    /// Pop a response installed by the most recent successful request.
    pub fn check_message(&self, context: &Context<'_>) -> Result<Option<DroneCanMessage>> {{
        dronecan_check_message(context, self.token)
    }}

    /// Queue one broadcast with priority 0 and a 10 ms transmit deadline.
    /// Payloads are copied by the host call and are limited to 256 bytes.
    pub fn broadcast(&self, context: &Context<'_>, payload: &[u8]) -> Result<bool> {{
        if payload.len() > DRONECAN_PAYLOAD_MAX {{ return Err(Error::Invalid); }}
        let mut input = [0u8; DRONECAN_PAYLOAD_MAX + 6];
        input[..4].copy_from_slice(&self.token.get().to_le_bytes());
        input[4..6].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        input[6..6 + payload.len()].copy_from_slice(payload);
        let mut output = [0u8; 1];
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{broadcast['id']}.slot(),
                input.as_ptr(), (payload.len() + 6) as u32, output.as_mut_ptr(), 1)
        }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    /// Queue a service request and replace the pending response subscription.
    pub fn request(&self, context: &Context<'_>, target_node: u8, payload: &[u8]) -> Result<bool> {{
        if !(1..=127).contains(&target_node) || payload.len() > DRONECAN_PAYLOAD_MAX {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; DRONECAN_PAYLOAD_MAX + 7];
        input[..4].copy_from_slice(&self.token.get().to_le_bytes());
        input[4] = target_node;
        input[5..7].copy_from_slice(&(payload.len() as u16).to_le_bytes());
        input[7..7 + payload.len()].copy_from_slice(payload);
        let mut output = [0u8; 1];
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{request['id']}.slot(),
                input.as_ptr(), (payload.len() + 7) as u32, output.as_mut_ptr(), 1)
        }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    /// Close before module exit. Lifecycle cleanup also closes leaked handles
    /// after the module worker has joined or faulted.
    pub fn close(self, context: &Context<'_>) -> Result {{
        let input = self.token.get().to_le_bytes();
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{handle_close['id']}.slot(),
                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)
        }})
    }}
}}
'''
