# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Owned, pointer-free CRSF scripted-menu bindings."""

OWNER = 'AP_CRSF_Telem'
MENU = 'CRSFMenu'
PARAMETER = 'CRSFParameter'
MENU_NAME_MAX = 15
PARAMETER_DATA_MAX = 255
EVENT_PAYLOAD_MAX = 57
POLICY = ('v1:module-context-owned-menus,opaque-generation-tokens,no-pointer-wire,'
          'cleanup-after-worker-join,lua-restart-isolation,queued-pointer-prune-before-free,'
          'menu-name-nul-free-bytes-max15,parameter-bytes-max255,event-payload-max57,'
          'head-event-owner-validation,response-owner-validation,blocking-locks')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _alias(owner, alias, target):
    matches = [item for item in owner['aliases'] if item['alias'] == alias]
    expected = dict(name=target, alias=alias, depends=None, manual=True)
    if matches != [expected]:
        raise ValueError(f'unaudited {owner["name"]}.{alias} declaration')


def select(ir, first_id):
    crsf = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_crsf = dict(rename='crsf', depends='AP_CRSF_SCRIPTING_ENABLED', get=None, literal=False,
                         reference=False, semaphore=True, scheduler_semaphore=False,
                         semaphore_pointer=False)
    if crsf is None or any(crsf[key] != value for key, value in expected_crsf.items()):
        raise ValueError('unaudited AP_CRSF_Telem owner metadata')
    for alias, target in (('add_menu', 'lua_CRSF_add_root_menu'),
                          ('peek_menu_event', 'lua_CRSF_peek_menu_event'),
                          ('send_write_response', 'lua_CRSF_send_response')):
        _alias(crsf, alias, target)
    expected_methods = {
        'pop_menu_event': (('void', None, False, False, False, False, None), []),
        'send_response': (('boolean', None, False, False, False, False, None), []),
    }
    actual_methods = {
        method['name']: (_field_shape(method['return']), [_field_shape(arg) for arg in method['arguments']])
        for method in crsf['methods'] if method['name'] in expected_methods
    }
    if actual_methods != expected_methods:
        raise ValueError('unaudited AP_CRSF_Telem event methods')

    menu = next((item for item in ir['userdata'] if item['name'] == MENU), None)
    expected_menu = dict(rename=None, depends='AP_CRSF_SCRIPTING_ENABLED', get=None,
                         creation='lua_CRSF_new_menu', creation_manual=True, creation_args=1,
                         has_fields=False, has_enums=False, operations=0, operator_getter=None,
                         operators_manual=False, operator_names=[], literal=False, reference=False,
                         semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if menu is None or any(menu[key] != value for key, value in expected_menu.items()):
        raise ValueError('unaudited CRSFMenu metadata')
    _alias(menu, 'add_menu', 'lua_CRSF_add_menu')
    _alias(menu, 'add_parameter', 'lua_CRSF_add_parameter')
    expected_menu_methods = {
        'id': (('uint8_t', None, False, False, False, True, None), []),
        'name': (('string', None, False, False, False, False, None), []),
        'num_params': (('uint8_t', None, False, False, False, True, None), []),
    }
    actual_menu_methods = {
        method['name']: (_field_shape(method['return']), [_field_shape(arg) for arg in method['arguments']])
        for method in menu['methods'] if method['name'] in expected_menu_methods
    }
    if actual_menu_methods != expected_menu_methods:
        raise ValueError('unaudited CRSFMenu metadata methods')

    parameter = next((item for item in ir['userdata'] if item['name'] == PARAMETER), None)
    expected_parameter = dict(rename=None, depends='AP_CRSF_SCRIPTING_ENABLED', get=None,
                              creation=None, creation_manual=False, creation_args=0,
                              has_fields=False, has_enums=False, operations=0, operator_getter=None,
                              operators_manual=False, operator_names=[], literal=False, reference=False,
                              semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if parameter is None or any(parameter[key] != value for key, value in expected_parameter.items()):
        raise ValueError('unaudited CRSFParameter metadata')
    id_method = next((item for item in parameter['methods'] if item['name'] == 'id'), None)
    expected_id = ('uint8_t', None, False, False, False, True, None)
    if (id_method is None or id_method['rename'] is not None or id_method['depends'] is not None or
            id_method['deprecate'] is not None or _field_shape(id_method['return']) != expected_id or
            id_method['arguments']):
        raise ValueError('unaudited CRSFParameter.id declaration')
    _alias(parameter, 'data', 'lua_CRSF_param_data')
    return specification(first_id)


def specification(first_id):
    methods = [
        dict(id=first_id, name='crsf.add_menu', arguments=['menu-name'], result='optional-menu-token'),
        dict(id=first_id + 1, name='CRSFMenu.add_menu', arguments=['menu-token', 'menu-name'],
             result='optional-menu-token'),
        dict(id=first_id + 2, name='CRSFMenu.add_parameter',
             arguments=['menu-token', 'bytes-len-u8-max255-zero-pad255'], result='optional-parameter-token'),
        dict(id=first_id + 3, name='CRSFParameter.id', arguments=['parameter-token'], result='u8'),
        dict(id=first_id + 4, name='CRSFParameter.data', arguments=['parameter-token'],
             result='bytes-len-u8-max255-zero-pad255'),
        dict(id=first_id + 5, name='crsf.peek_menu_event', arguments=[],
             result='optional-owned-event-count-param-events-payload57'),
        dict(id=first_id + 6, name='crsf.pop_menu_event', arguments=['param-id', 'events'], result='bool-u8'),
        dict(id=first_id + 7, name='crsf.send_write_response',
             arguments=['bytes-len-u8-max255-zero-pad255'], result='bool-u8'),
        dict(id=first_id + 8, name='crsf.send_response', arguments=[], result='bool-u8'),
        dict(id=first_id + 9, name='crsf.get_menu_event', arguments=['event-mask-u8'],
             result='optional-owned-event-param-events-payload57'),
        dict(id=first_id + 10, name='CRSFMenu.id', arguments=['menu-token'], result='u8'),
        dict(id=first_id + 11, name='CRSFMenu.name', arguments=['menu-token'],
             result='utf8-len-u8-max15-zero-pad15'),
        dict(id=first_id + 12, name='CRSFMenu.num_params', arguments=['menu-token'], result='u8'),
    ]
    for method in methods:
        method['policy'] = POLICY
    return dict(methods=methods)


def rust(spec, notice):
    (add_root, add_menu, add_parameter, parameter_id, parameter_data,
     peek, pop, send_write, send, get_event, menu_id, menu_name, menu_num_params) = spec['methods']
    return notice + f'''use crate::{{result, Context, Error, Result}};
use core::num::NonZeroU32;

pub const CRSF_MENU_NAME_MAX: usize = {MENU_NAME_MAX};
pub const CRSF_PARAMETER_DATA_MAX: usize = {PARAMETER_DATA_MAX};
pub const CRSF_EVENT_PAYLOAD_MAX: usize = {EVENT_PAYLOAD_MAX};

#[derive(Debug, Eq, PartialEq)]
pub struct CrsfMenu {{ token: NonZeroU32 }}

#[derive(Debug, Eq, PartialEq)]
pub struct CrsfParameter {{ token: NonZeroU32 }}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CrsfParameterData {{ data: [u8; CRSF_PARAMETER_DATA_MAX], length: u8 }}

impl CrsfParameterData {{
    pub fn bytes(&self) -> &[u8] {{ &self.data[..self.length as usize] }}
}}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CrsfMenuName {{ data: [u8; CRSF_MENU_NAME_MAX], length: u8 }}

impl CrsfMenuName {{
    pub fn bytes(&self) -> &[u8] {{ &self.data[..self.length as usize] }}

    pub fn as_str(&self) -> Result<&str> {{
        core::str::from_utf8(self.bytes()).map_err(|_| Error::Host)
    }}
}}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct CrsfMenuEvent {{
    pub queue_depth: u8,
    pub param_id: u8,
    pub events: u8,
    payload: [u8; CRSF_EVENT_PAYLOAD_MAX],
    payload_length: u8,
}}

impl CrsfMenuEvent {{
    pub fn payload(&self) -> &[u8] {{ &self.payload[..self.payload_length as usize] }}
}}

fn crsf_name(name: &str) -> Result<[u8; 16]> {{
    let bytes = name.as_bytes();
    if bytes.len() > CRSF_MENU_NAME_MAX || bytes.contains(&0) {{ return Err(Error::Invalid); }}
    let mut output = [0u8; 16];
    output[0] = bytes.len() as u8;
    output[1..1 + bytes.len()].copy_from_slice(bytes);
    Ok(output)
}}

fn crsf_token(output: [u8; 4]) -> Option<NonZeroU32> {{
    NonZeroU32::new(u32::from_le_bytes(output))
}}

fn crsf_bool(output: u8) -> Result<bool> {{
    match output {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
}}

impl crate::bindings::Crsf<'_, '_> {{
    /// Add a top-level menu owned by this module instance.
    pub fn add_menu(&self, name: &str) -> Result<Option<CrsfMenu>> {{
        let input = crsf_name(name)?;
        let mut output = [0u8; 4];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{add_root['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), output.len() as u32) }})?;
        Ok(crsf_token(output).map(|token| CrsfMenu {{ token }}))
    }}

    /// Peek only when the global queue head belongs to this module.
    pub fn peek_menu_event(&self) -> Result<Option<CrsfMenuEvent>> {{
        let mut output = [0u8; 62];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{peek['id']}.slot(), core::ptr::null(), 0,
            output.as_mut_ptr(), output.len() as u32) }})?;
        match output[0] {{
            0 => Ok(None),
            1 if output[4] as usize <= CRSF_EVENT_PAYLOAD_MAX => {{
                let mut payload = [0u8; CRSF_EVENT_PAYLOAD_MAX];
                payload.copy_from_slice(&output[5..]);
                Ok(Some(CrsfMenuEvent {{ queue_depth: output[1], param_id: output[2],
                    events: output[3], payload_length: output[4], payload }}))
            }}
            _ => Err(Error::Host),
        }}
    }}

    /// Pop the same owned event that was observed by `crsf_peek_menu_event`.
    pub fn pop_menu_event(&self, event: &CrsfMenuEvent) -> Result<bool> {{
        let input = [event.param_id, event.events];
        let mut output = [0u8; 1];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{pop['id']}.slot(), input.as_ptr(), 2,
            output.as_mut_ptr(), 1) }})?;
        crsf_bool(output[0])
    }}

    pub fn send_write_response(&self, data: &[u8]) -> Result<bool> {{
        if data.len() > CRSF_PARAMETER_DATA_MAX {{ return Err(Error::Invalid); }}
        let mut input = [0u8; 1 + CRSF_PARAMETER_DATA_MAX];
        input[0] = data.len() as u8;
        input[1..1 + data.len()].copy_from_slice(data);
        let mut output = [0u8; 1];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{send_write['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), 1) }})?;
        crsf_bool(output[0])
    }}

    pub fn send_response(&self) -> Result<bool> {{
        let mut output = [0u8; 1];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{send['id']}.slot(), core::ptr::null(), 0,
            output.as_mut_ptr(), 1) }})?;
        crsf_bool(output[0])
    }}

    /// Consume the owned event at the queue head using the legacy filtered-event semantics.
    pub fn get_menu_event(&self, events: u8) -> Result<Option<CrsfMenuEvent>> {{
        let input = [events];
        let mut output = [0u8; 61];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{get_event['id']}.slot(), input.as_ptr(), 1,
            output.as_mut_ptr(), output.len() as u32) }})?;
        match output[0] {{
            0 => Ok(None),
            1 if output[3] as usize <= CRSF_EVENT_PAYLOAD_MAX => {{
                let mut payload = [0u8; CRSF_EVENT_PAYLOAD_MAX];
                payload.copy_from_slice(&output[4..]);
                Ok(Some(CrsfMenuEvent {{ queue_depth: 0, param_id: output[1],
                    events: output[2], payload_length: output[3], payload }}))
            }}
            _ => Err(Error::Host),
        }}
    }}
}}

impl CrsfMenu {{
    fn call_u8(&self, context: &Context<'_>, id: u32) -> Result<u8> {{
        let input = self.token.get().to_le_bytes();
        let mut output = [0u8; 1];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw, id,
            input.as_ptr(), 4, output.as_mut_ptr(), 1) }})?;
        Ok(output[0])
    }}

    pub fn id(&self, context: &Context<'_>) -> Result<u8> {{
        self.call_u8(context, crate::imports::BINDING_{menu_id['id']}.slot())
    }}

    pub fn name(&self, context: &Context<'_>) -> Result<CrsfMenuName> {{
        let input = self.token.get().to_le_bytes();
        let mut output = [0u8; 1 + CRSF_MENU_NAME_MAX];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{menu_name['id']}.slot(), input.as_ptr(), 4,
            output.as_mut_ptr(), output.len() as u32) }})?;
        if output[0] as usize > CRSF_MENU_NAME_MAX {{ return Err(Error::Host); }}
        let mut data = [0u8; CRSF_MENU_NAME_MAX];
        data.copy_from_slice(&output[1..]);
        Ok(CrsfMenuName {{ data, length: output[0] }})
    }}

    pub fn num_params(&self, context: &Context<'_>) -> Result<u8> {{
        self.call_u8(context, crate::imports::BINDING_{menu_num_params['id']}.slot())
    }}

    pub fn add_menu(&self, context: &Context<'_>, name: &str) -> Result<Option<CrsfMenu>> {{
        let encoded = crsf_name(name)?;
        let mut input = [0u8; 20];
        input[..4].copy_from_slice(&self.token.get().to_le_bytes());
        input[4..].copy_from_slice(&encoded);
        let mut output = [0u8; 4];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{add_menu['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), output.len() as u32) }})?;
        Ok(crsf_token(output).map(|token| CrsfMenu {{ token }}))
    }}

    pub fn add_parameter(&self, context: &Context<'_>, data: &[u8]) -> Result<Option<CrsfParameter>> {{
        if data.is_empty() || data.len() > CRSF_PARAMETER_DATA_MAX {{ return Err(Error::Invalid); }}
        let mut input = [0u8; 5 + CRSF_PARAMETER_DATA_MAX];
        input[..4].copy_from_slice(&self.token.get().to_le_bytes());
        input[4] = data.len() as u8;
        input[5..5 + data.len()].copy_from_slice(data);
        let mut output = [0u8; 4];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{add_parameter['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), output.len() as u32) }})?;
        Ok(crsf_token(output).map(|token| CrsfParameter {{ token }}))
    }}
}}

impl CrsfParameter {{
    pub fn id(&self, context: &Context<'_>) -> Result<u8> {{
        let input = self.token.get().to_le_bytes();
        let mut output = [0u8; 1];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{parameter_id['id']}.slot(), input.as_ptr(), 4,
            output.as_mut_ptr(), 1) }})?;
        Ok(output[0])
    }}

    pub fn data(&self, context: &Context<'_>) -> Result<CrsfParameterData> {{
        let input = self.token.get().to_le_bytes();
        let mut output = [0u8; 1 + CRSF_PARAMETER_DATA_MAX];
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{parameter_data['id']}.slot(), input.as_ptr(), 4,
            output.as_mut_ptr(), output.len() as u32) }})?;
        let length = output[0];
        let mut data = [0u8; CRSF_PARAMETER_DATA_MAX];
        data.copy_from_slice(&output[1..]);
        Ok(CrsfParameterData {{ data, length }})
    }}
}}
'''
