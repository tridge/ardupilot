# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Module-owned bounded TCP/UDP sockets for native modules."""

OWNER = 'SocketAPM'
NETWORK_OWNER = 'AP_Networking'
TRANSFER_MAX = 1024
PATH_SIZE = 256
POLL_TIMEOUT_MAX_MS = 100
POLICY = ('v1:module-context-owned,max8-modules,max50-sockets-per-module,generation-checked-handles,'
          'generation-seeds-persist-across-context-restart,cleanup-after-worker-join,'
          'blocking-registry-lock,descriptor-always-nonblocking,logical-blocking-connect-and-io-poll20ms,'
          'poll-timeout-max100ms,transfer-max1024,fixed-zero-padded-wire,accept-bounded,'
          'sendfile-transfers-fd-and-socket-to-network-worker,no-retained-module-pointers')


def select(ir, first_id):
    owner = next((item for item in ir['ap_objects'] if item['name'] == OWNER), None)
    expected_owner = dict(rename=None, depends='(AP_NETWORKING_ENABLED==1)', get=None, creation=None,
                          creation_manual=False, creation_args=0, has_fields=False, has_enums=False,
                          operations=0, operator_getter=None, operators_manual=False, operator_names=[],
                          literal=False, reference=False, semaphore=False, scheduler_semaphore=False,
                          semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected_owner.items()):
        raise ValueError('unaudited SocketAPM metadata')

    def field_shape(field):
        return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
                field['skip_check'], tuple(field['range']) if field['range'] else None)

    expected_methods = {
        'bind': (field_shape(dict(kind='boolean', name=None, nullable=False, reference=False,
                                  pointer=False, skip_check=False, range=None)),
                 [('string', None, False, False, False, False, None),
                  ('uint16_t', None, False, False, False, True, None)]),
        'connect': (field_shape(dict(kind='boolean', name=None, nullable=False, reference=False,
                                     pointer=False, skip_check=False, range=None)),
                    [('string', None, False, False, False, False, None),
                     ('uint16_t', None, False, False, False, True, None)]),
        'is_connected': (('boolean', None, False, False, False, False, None), []),
        'is_pending': (('boolean', None, False, False, False, False, None), []),
        'listen': (('boolean', None, False, False, False, False, None),
                   [('uint8_t', None, False, False, False, True, None)]),
        'pollin': (('boolean', None, False, False, False, False, None),
                   [('uint32_t', None, False, False, False, True, None)]),
        'pollout': (('boolean', None, False, False, False, False, None),
                    [('uint32_t', None, False, False, False, True, None)]),
        'send': (('int32_t', None, False, False, False, False, None),
                 [('string', None, False, False, False, False, None),
                  ('uint32_t', None, False, False, False, True, None)]),
        'sendto': (('int32_t', None, False, False, False, False, None),
                   [('string', None, False, False, False, False, None),
                    ('uint32_t', None, False, False, False, True, None),
                    ('uint32_t', None, False, False, False, True, None),
                    ('uint16_t', None, False, False, False, True, None)]),
        'reuseaddress': (('boolean', None, False, False, False, False, None), []),
        'set_blocking': (('boolean', None, False, False, False, False, None),
                         [('boolean', None, False, False, False, False, None)]),
    }
    methods = {method['name']: (field_shape(method['return']), [field_shape(arg) for arg in method['arguments']])
               for method in owner['methods'] if method['name'] in expected_methods}
    if methods != expected_methods:
        raise ValueError('unaudited SocketAPM methods')
    aliases = {item['alias']: item['name'] for item in owner['aliases'] if item['manual'] and item['depends'] is None}
    if aliases != {'accept': 'SocketAPM_accept', 'recv': 'SocketAPM_recv',
                   'close': 'SocketAPM_close', 'sendfile': 'SocketAPM_sendfile'}:
        raise ValueError('unaudited SocketAPM manual aliases')

    networking = next((item for item in ir['singletons'] if item['name'] == NETWORK_OWNER), None)
    expected_networking = dict(rename='networking', depends='AP_NETWORKING_ENABLED', get=None,
                               literal=False, reference=False, semaphore=False,
                               scheduler_semaphore=False, semaphore_pointer=False)
    if networking is None or any(networking[key] != value for key, value in expected_networking.items()):
        raise ValueError('unaudited AP_Networking metadata')
    address = next((method for method in networking['methods'] if method['name'] == 'address_to_str'), None)
    if (address is None or field_shape(address['return']) !=
            ('string', None, False, False, False, False, None) or
            [field_shape(arg) for arg in address['arguments']] !=
            [('uint32_t', None, False, False, False, True, None)]):
        raise ValueError('unaudited AP_Networking.address_to_str method')
    return specification(first_id)


def specification(first_id):
    handle = dict(type='u32-le,nonzero,module-owned-generation-handle', direction='in')
    methods = [
        dict(id=first_id, name='network.socket_open',
             arguments=[dict(type='bool-u8', name='datagram')], result='nonzero-u32-le-handle', policy=POLICY),
        dict(id=first_id + 1, name='network.socket_close', arguments=[handle], result='void', policy=POLICY),
        dict(id=first_id + 2, name='network.socket_bind',
             arguments=[handle, dict(type='nonempty-nul-free-bytes-max255-zero-pad256', name='address'),
                        dict(type='nonzero-u16-le', name='port')], result='bool-u8', policy=POLICY),
        dict(id=first_id + 3, name='network.socket_listen',
             arguments=[handle, dict(type='nonzero-u8', name='backlog')], result='bool-u8', policy=POLICY),
        dict(id=first_id + 4, name='network.socket_set_blocking',
             arguments=[handle, dict(type='bool-u8', name='blocking')], result='void', policy=POLICY),
        dict(id=first_id + 5, name='network.socket_is_connected', arguments=[handle], result='bool-u8', policy=POLICY),
        dict(id=first_id + 6, name='network.socket_pollin',
             arguments=[handle, dict(type='u16-le', range=[0, POLL_TIMEOUT_MAX_MS], name='timeout_ms')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 7, name='network.socket_pollout',
             arguments=[handle, dict(type='u16-le', range=[0, POLL_TIMEOUT_MAX_MS], name='timeout_ms')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 8, name='network.socket_accept', arguments=[handle],
             result='optional-nonzero-u32-le-handle', policy=POLICY),
        dict(id=first_id + 9, name='network.socket_recv',
             arguments=[handle, dict(type='u16-le', range=[1, TRANSFER_MAX], name='length')],
             result=dict(presence='bool-u8', length='u16-le',
                         data=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}'), policy=POLICY),
        dict(id=first_id + 10, name='network.socket_send',
             arguments=[handle, dict(type=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}', name='data')],
             result=dict(presence='bool-u8', length='u16-le'), policy=POLICY),
        dict(id=first_id + 11, name='network.socket_sendfile',
             arguments=[handle, dict(type='nonempty-nul-free-bytes-max255-zero-pad256', name='path')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 12, name='network.socket_connect',
             arguments=[handle, dict(type='nonempty-nul-free-bytes-max255-zero-pad256', name='address'),
                        dict(type='nonzero-u16-le', name='port')], result='bool-u8', policy=POLICY),
        dict(id=first_id + 13, name='network.socket_recv_from',
             arguments=[handle, dict(type='u16-le', range=[1, TRANSFER_MAX], name='length')],
             result=dict(presence='bool-u8', length='u16-le', peer_presence='bool-u8',
                         peer_address='host-order-u32-le-ipv4', peer_port='host-order-u16-le',
                         data=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}'), policy=POLICY),
        dict(id=first_id + 14, name='network.socket_is_pending', arguments=[handle],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 15, name='network.socket_reuse_address', arguments=[handle],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 16, name='network.socket_send_to',
             arguments=[handle,
                        dict(type=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}', name='data'),
                        dict(type='host-order-u32-le-ipv4', name='address'),
                        dict(type='host-order-u16-le', name='port')],
             result=dict(presence='bool-u8', length='u16-le'), policy=POLICY),
    ]
    return dict(methods=methods, transfer_max=TRANSFER_MAX, poll_timeout_max_ms=POLL_TIMEOUT_MAX_MS)


def rust(spec, notice):
    (open_, close, bind, listen, blocking, connected, pollin, pollout,
     accept, recv, send, sendfile, connect, recv_from, pending, reuse_address, send_to) = spec['methods']
    return notice + f'''use crate::{{result, Context, Error, Result}};
use core::num::NonZeroU32;

pub const SOCKET_TRANSFER_MAX: usize = {TRANSFER_MAX};
pub const SOCKET_POLL_TIMEOUT_MAX_MS: u16 = {POLL_TIMEOUT_MAX_MS};

/// An IPv4 address in the host-order representation used by ArduPilot.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Ipv4Address(pub u32);

impl Ipv4Address {{
    pub const fn octets(self) -> [u8; 4] {{ self.0.to_be_bytes() }}
}}

impl core::fmt::Display for Ipv4Address {{
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {{
        let octets = self.octets();
        write!(formatter, "{{}}.{{}}.{{}}.{{}}", octets[0], octets[1], octets[2], octets[3])
    }}
}}

/// The source of a received IPv4 datagram. The address and port are in host
/// order, matching `SocketAPM::last_recv_address` and the Lua socket API.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct Ipv4Endpoint {{
    pub address: u32,
    pub port: u16,
}}

impl Ipv4Endpoint {{
    pub const fn address_octets(&self) -> [u8; 4] {{ self.address.to_be_bytes() }}
}}

impl core::fmt::Display for Ipv4Endpoint {{
    fn fmt(&self, formatter: &mut core::fmt::Formatter<'_>) -> core::fmt::Result {{
        let octets = self.address_octets();
        write!(formatter, "{{}}.{{}}.{{}}.{{}}:{{}}",
               octets[0], octets[1], octets[2], octets[3], self.port)
    }}
}}

/// One successful bounded receive, with source metadata when the native
/// socket reports it.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct SocketReceive {{
    pub length: usize,
    pub peer: Option<Ipv4Endpoint>,
}}

/// A generation-checked socket owned by this module. Explicit close is useful
/// for prompt release; firmware closes every remaining socket after worker join.
#[derive(Debug, Eq, PartialEq)]
pub struct Socket {{ token: Option<NonZeroU32> }}

impl Socket {{
    fn token(&self) -> Result<NonZeroU32> {{ self.token.ok_or(Error::Invalid) }}

    fn call_bool(&self, context: &Context<'_>, id: u32) -> Result<bool> {{
        let input = self.token()?.get().to_le_bytes();
        let mut output = [0u8; 1];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw, id,
            input.as_ptr(), input.len() as u32, output.as_mut_ptr(), 1) }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    pub fn close(&mut self, context: &Context<'_>) -> Result {{
        let input = self.token()?.get().to_le_bytes();
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{close['id']}.slot(), input.as_ptr(), 4,
            core::ptr::null_mut(), 0) }})?;
        self.token = None;
        Ok(())
    }}

    pub fn bind(&self, context: &Context<'_>, address: &str, port: u16) -> Result<bool> {{
        if address.is_empty() || address.len() >= {PATH_SIZE} || address.as_bytes().contains(&0) || port == 0 {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; {4 + PATH_SIZE + 2}];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4..4 + address.len()].copy_from_slice(address.as_bytes());
        input[{4 + PATH_SIZE}..].copy_from_slice(&port.to_le_bytes());
        let mut output = [0u8; 1];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{bind['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), 1) }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    /// Start a connection. In logical blocking mode this waits at most 20 ms;
    /// in nonblocking mode `false` may mean the connection remains pending.
    pub fn connect(&self, context: &Context<'_>, address: &str, port: u16) -> Result<bool> {{
        if address.is_empty() || address.len() >= {PATH_SIZE} || address.as_bytes().contains(&0) || port == 0 {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; {4 + PATH_SIZE + 2}];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4..4 + address.len()].copy_from_slice(address.as_bytes());
        input[{4 + PATH_SIZE}..].copy_from_slice(&port.to_le_bytes());
        let mut output = [0u8; 1];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{connect['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), 1) }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    pub fn listen(&self, context: &Context<'_>, backlog: u8) -> Result<bool> {{
        if backlog == 0 {{ return Err(Error::Invalid); }}
        let mut input = [0u8; 5];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4] = backlog;
        let mut output = [0u8; 1];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{listen['id']}.slot(), input.as_ptr(), 5,
            output.as_mut_ptr(), 1) }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    /// Select logical blocking behavior. Descriptors remain nonblocking; when
    /// enabled each accept/send/receive waits at most 20 ms for readiness.
    pub fn set_blocking(&self, context: &Context<'_>, enabled: bool) -> Result {{
        let mut input = [0u8; 5];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4] = enabled as u8;
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{blocking['id']}.slot(), input.as_ptr(), 5,
            core::ptr::null_mut(), 0) }})
    }}

    pub fn is_connected(&self, context: &Context<'_>) -> Result<bool> {{
        self.call_bool(context, crate::imports::BINDING_{connected['id']}.slot())
    }}

    /// True while a nonblocking connection attempt remains in progress.
    pub fn is_pending(&self, context: &Context<'_>) -> Result<bool> {{
        self.call_bool(context, crate::imports::BINDING_{pending['id']}.slot())
    }}

    pub fn reuse_address(&self, context: &Context<'_>) -> Result<bool> {{
        self.call_bool(context, crate::imports::BINDING_{reuse_address['id']}.slot())
    }}

    fn poll(&self, context: &Context<'_>, id: u32, timeout_ms: u16) -> Result<bool> {{
        if timeout_ms > SOCKET_POLL_TIMEOUT_MAX_MS {{ return Err(Error::Invalid); }}
        let mut input = [0u8; 6];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4..].copy_from_slice(&timeout_ms.to_le_bytes());
        let mut output = [0u8; 1];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw, id,
            input.as_ptr(), 6, output.as_mut_ptr(), 1) }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    pub fn pollin(&self, context: &Context<'_>, timeout_ms: u16) -> Result<bool> {{
        self.poll(context, crate::imports::BINDING_{pollin['id']}.slot(), timeout_ms)
    }}

    pub fn pollout(&self, context: &Context<'_>, timeout_ms: u16) -> Result<bool> {{
        self.poll(context, crate::imports::BINDING_{pollout['id']}.slot(), timeout_ms)
    }}

    pub fn accept(&self, context: &Context<'_>) -> Result<Option<Socket>> {{
        let input = self.token()?.get().to_le_bytes();
        let mut output = [0u8; 4];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{accept['id']}.slot(), input.as_ptr(), 4,
            output.as_mut_ptr(), 4) }})?;
        Ok(NonZeroU32::new(u32::from_le_bytes(output)).map(|token| Socket {{ token: Some(token) }}))
    }}

    /// Receive at most 1024 bytes. `None` means the bounded readiness wait
    /// expired; `Some(0)` is orderly EOF.
    pub fn recv(&self, context: &Context<'_>, buffer: &mut [u8]) -> Result<Option<usize>> {{
        if buffer.is_empty() || buffer.len() > SOCKET_TRANSFER_MAX {{ return Err(Error::Invalid); }}
        let mut input = [0u8; 6];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4..].copy_from_slice(&(buffer.len() as u16).to_le_bytes());
        let mut output = [0u8; {TRANSFER_MAX + 3}];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{recv['id']}.slot(), input.as_ptr(), 6,
            output.as_mut_ptr(), output.len() as u32) }})?;
        if output[0] == 0 {{
            return if output[1..].iter().all(|byte| *byte == 0) {{ Ok(None) }} else {{ Err(Error::Host) }};
        }}
        if output[0] != 1 {{ return Err(Error::Host); }}
        let length = u16::from_le_bytes(output[1..3].try_into().unwrap()) as usize;
        if length > buffer.len() || output[3 + length..].iter().any(|byte| *byte != 0) {{
            return Err(Error::Host);
        }}
        buffer[..length].copy_from_slice(&output[3..3 + length]);
        Ok(Some(length))
    }}

    /// Receive at most 1024 bytes and retain optional UDP peer metadata.
    /// `None` means the bounded readiness wait expired; a zero length is EOF.
    pub fn recv_from(&self, context: &Context<'_>, buffer: &mut [u8]) -> Result<Option<SocketReceive>> {{
        if buffer.is_empty() || buffer.len() > SOCKET_TRANSFER_MAX {{ return Err(Error::Invalid); }}
        let mut input = [0u8; 6];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4..].copy_from_slice(&(buffer.len() as u16).to_le_bytes());
        let mut output = [0u8; {TRANSFER_MAX + 10}];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{recv_from['id']}.slot(), input.as_ptr(), 6,
            output.as_mut_ptr(), output.len() as u32) }})?;
        if output[0] == 0 {{
            return if output[1..].iter().all(|byte| *byte == 0) {{ Ok(None) }} else {{ Err(Error::Host) }};
        }}
        if output[0] != 1 {{ return Err(Error::Host); }}
        let length = u16::from_le_bytes(output[1..3].try_into().unwrap()) as usize;
        if length > buffer.len() || output[10 + length..].iter().any(|byte| *byte != 0) {{
            return Err(Error::Host);
        }}
        let peer = match output[3] {{
            0 if output[4..10].iter().all(|byte| *byte == 0) => None,
            1 => {{
                let address = u32::from_le_bytes(output[4..8].try_into().unwrap());
                let port = u16::from_le_bytes(output[8..10].try_into().unwrap());
                if address == 0 || port == 0 {{ return Err(Error::Host); }}
                Some(Ipv4Endpoint {{ address, port }})
            }}
            _ => return Err(Error::Host),
        }};
        buffer[..length].copy_from_slice(&output[10..10 + length]);
        Ok(Some(SocketReceive {{ length, peer }}))
    }}

    /// Send at most 1024 bytes. `None` means the bounded readiness wait
    /// expired; partial sends are returned for caller retry.
    pub fn send(&self, context: &Context<'_>, buffer: &[u8]) -> Result<Option<usize>> {{
        if buffer.is_empty() || buffer.len() > SOCKET_TRANSFER_MAX {{ return Err(Error::Invalid); }}
        let mut input = [0u8; {TRANSFER_MAX + 6}];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4..6].copy_from_slice(&(buffer.len() as u16).to_le_bytes());
        input[6..6 + buffer.len()].copy_from_slice(buffer);
        let mut output = [0u8; 3];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{send['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), 3) }})?;
        if output[0] == 0 {{
            return if output[1..] == [0, 0] {{ Ok(None) }} else {{ Err(Error::Host) }};
        }}
        if output[0] != 1 {{ return Err(Error::Host); }}
        let length = u16::from_le_bytes(output[1..].try_into().unwrap()) as usize;
        if length > buffer.len() {{ return Err(Error::Host); }}
        Ok(Some(length))
    }}

    /// Send one bounded datagram to a host-order IPv4 address and port.
    /// `None` means the bounded readiness wait expired.
    pub fn send_to(&self, context: &Context<'_>, buffer: &[u8], address: u32,
                   port: u16) -> Result<Option<usize>> {{
        if buffer.is_empty() || buffer.len() > SOCKET_TRANSFER_MAX {{ return Err(Error::Invalid); }}
        let mut input = [0u8; {TRANSFER_MAX + 12}];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4..6].copy_from_slice(&(buffer.len() as u16).to_le_bytes());
        input[6..10].copy_from_slice(&address.to_le_bytes());
        input[10..12].copy_from_slice(&port.to_le_bytes());
        input[12..12 + buffer.len()].copy_from_slice(buffer);
        let mut output = [0u8; 3];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{send_to['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), 3) }})?;
        if output[0] == 0 {{
            return if output[1..] == [0, 0] {{ Ok(None) }} else {{ Err(Error::Host) }};
        }}
        if output[0] != 1 {{ return Err(Error::Host); }}
        let length = u16::from_le_bytes(output[1..].try_into().unwrap()) as usize;
        if length > buffer.len() {{ return Err(Error::Host); }}
        Ok(Some(length))
    }}

    /// Transfer the file and connected socket to the firmware sendfile worker.
    /// On success this handle is invalidated; completion is asynchronous.
    pub fn sendfile(&mut self, context: &Context<'_>, path: &str) -> Result<bool> {{
        if path.is_empty() || path.len() >= {PATH_SIZE} || path.as_bytes().contains(&0) {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; {4 + PATH_SIZE}];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4..4 + path.len()].copy_from_slice(path.as_bytes());
        let mut output = [0u8; 1];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{sendfile['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), 1) }})?;
        match output[0] {{
            0 => Ok(false),
            1 => {{ self.token = None; Ok(true) }},
            _ => Err(Error::Host),
        }}
    }}
}}

impl crate::bindings::Network<'_, '_> {{
    pub fn open(&self, datagram: bool) -> Result<Socket> {{
        let input = [datagram as u8];
        let mut output = [0u8; 4];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{open_['id']}.slot(), input.as_ptr(), 1,
            output.as_mut_ptr(), 4) }})?;
        let token = NonZeroU32::new(u32::from_le_bytes(output)).ok_or(Error::Host)?;
        Ok(Socket {{ token: Some(token) }})
    }}
}}
'''
