# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Lifecycle-owned I2C devices for native modules."""

OWNER = 'i2c'
DEVICE = 'AP_HAL::I2CDevice'
TRANSFER_MAX = 255
POLICY = ('v1:module-context-owned-devices,cleanup-after-worker-join,max4-devices-per-module,'
          'bus-0-to4,address-0-to128,transfer-max255,blocking-device-lock,'
          'copy-output-on-success,no-retained-module-pointers')


def _shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_owner = dict(rename=None, depends=None, get=None, literal=False, reference=False,
                          semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected_owner.items()):
        raise ValueError('unaudited i2c owner metadata')
    aliases = [item for item in owner['aliases'] if item['alias'] == 'get_device']
    if aliases != [dict(name='lua_get_i2c_device', alias='get_device', depends=None, manual=True)]:
        raise ValueError('unaudited i2c.get_device declaration')

    device = next((item for item in ir['ap_objects'] if item['name'] == DEVICE), None)
    expected_device = dict(rename=None, depends=None, get=None, creation=None, creation_manual=False,
                           creation_args=0, has_fields=False, has_enums=False, operations=0,
                           operator_getter=None, operators_manual=False, operator_names=[], literal=False,
                           reference=False, semaphore=False, scheduler_semaphore=False,
                           semaphore_pointer=True)
    if device is None or any(device[key] != value for key, value in expected_device.items()):
        raise ValueError('unaudited I2CDevice metadata')
    expected = {
        'set_retries': (('void', None, False, False, False, False, None),
                        [('uint8_t', None, False, False, False, False, ('0', '20'))]),
        'set_address': (('void', None, False, False, False, False, None),
                        [('uint8_t', None, False, False, False, True, None)]),
        'write_register': (('boolean', None, False, False, False, False, None),
                           [('uint8_t', None, False, False, False, True, None),
                            ('uint8_t', None, False, False, False, True, None)]),
    }
    if {method['name'] for method in device['methods']} != set(expected):
        raise ValueError('unaudited I2CDevice methods')
    for method in device['methods']:
        actual = (_shape(method['return']), [_shape(argument) for argument in method['arguments']])
        if (method['rename'] is not None or method['depends'] is not None or
                method['deprecate'] is not None or actual != expected[method['name']]):
            raise ValueError(f'unaudited I2CDevice.{method["name"]} declaration')
    expected_aliases = {
        ('AP_HAL__I2CDevice_read_registers', 'read_registers', None, True),
        ('AP_HAL__I2CDevice_transfer', 'transfer', None, True),
    }
    if {(item['name'], item['alias'], item['depends'], item['manual']) for item in device['aliases']} != expected_aliases:
        raise ValueError('unaudited I2CDevice manual aliases')
    return specification(first_id)


def specification(first_id):
    names = ['open', 'close', 'set_retries', 'set_address', 'write_register', 'read_registers', 'transfer']
    arguments = [
        [dict(type='u8', range=[0, 4], name='bus'), dict(type='u8', range=[0, 128], name='address'),
         dict(type='u32-le', name='bus_clock'), dict(type='bool-u8', name='use_smbus')],
        [dict(type='nonzero-u32-le', name='handle')],
        [dict(type='nonzero-u32-le', name='handle'), dict(type='u8', range=[0, 20], name='retries')],
        [dict(type='nonzero-u32-le', name='handle'), dict(type='u8', name='address')],
        [dict(type='nonzero-u32-le', name='handle'), dict(type='u8', name='register'),
         dict(type='u8', name='value')],
        [dict(type='nonzero-u32-le', name='handle'), dict(type='u8', name='first_register'),
         dict(type='u8', range=[1, TRANSFER_MAX], name='receive_length')],
        [dict(type='nonzero-u32-le', name='handle'),
         dict(type=f'bytes-max{TRANSFER_MAX}-zero-pad{TRANSFER_MAX}', name='send'),
         dict(type='u8', name='receive_length')],
    ]
    results = ['optional-nonzero-u32-le', 'void', 'void', 'void', 'bool-u8',
               f'optional-bytes-max{TRANSFER_MAX}', f'optional-bytes-max{TRANSFER_MAX}']
    return dict(methods=[dict(id=first_id + i, name=f'i2c.{name}', arguments=arguments[i],
                              result=results[i], policy=POLICY) for i, name in enumerate(names)],
                transfer_max=TRANSFER_MAX)


def rust(spec, notice):
    methods = spec['methods']
    ids = {method['name'].rsplit('.', 1)[1]: method['id'] for method in methods}
    return notice + f'''use core::num::NonZeroU32;
use crate::{{result, Context, Error, Result}};

pub const I2C_TRANSFER_MAX: usize = {TRANSFER_MAX};

pub struct I2cDevice {{ token: Option<NonZeroU32> }}

impl crate::bindings::I2c<'_, '_> {{
    pub fn get_device(&self, bus: u8, address: u8, bus_clock: u32,
                      use_smbus: bool) -> Result<Option<I2cDevice>> {{
        if bus > 4 || address > 128 {{ return Err(Error::Invalid); }}
        let mut input = [0u8; 7];
        input[0] = bus;
        input[1] = address;
        input[2..6].copy_from_slice(&bus_clock.to_le_bytes());
        input[6] = use_smbus as u8;
        let mut output = [0u8; 4];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{ids['open']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        Ok(NonZeroU32::new(u32::from_le_bytes(output)).map(|token| I2cDevice {{ token: Some(token) }}))
    }}
}}

impl I2cDevice {{
    fn token(&self) -> Result<NonZeroU32> {{ self.token.ok_or(Error::Invalid) }}

    fn setting(&self, context: &Context<'_>, value: u8, id: u32) -> Result {{
        let mut input = [0u8; 5];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4] = value;
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, id, input.as_ptr(), input.len() as u32,
                core::ptr::null_mut(), 0)
        }})
    }}

    pub fn set_retries(&self, context: &Context<'_>, retries: u8) -> Result {{
        if retries > 20 {{ return Err(Error::Invalid); }}
        self.setting(context, retries, crate::imports::BINDING_{ids['set_retries']}.slot())
    }}

    pub fn set_address(&self, context: &Context<'_>, address: u8) -> Result {{
        self.setting(context, address, crate::imports::BINDING_{ids['set_address']}.slot())
    }}

    pub fn write_register(&self, context: &Context<'_>, register: u8, value: u8) -> Result<bool> {{
        let mut input = [0u8; 6];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4] = register;
        input[5] = value;
        let mut output = [0u8; 1];
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{ids['write_register']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    pub fn read_registers(&self, context: &Context<'_>, first_register: u8,
                          receive: &mut [u8]) -> Result<bool> {{
        if receive.is_empty() || receive.len() > I2C_TRANSFER_MAX {{ return Err(Error::Invalid); }}
        let mut input = [0u8; 6];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4] = first_register;
        input[5] = receive.len() as u8;
        let mut output = [0u8; I2C_TRANSFER_MAX + 1];
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{ids['read_registers']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        match output[0] {{
            0 => Ok(false),
            1 => {{ receive.copy_from_slice(&output[1..1 + receive.len()]); Ok(true) }}
            _ => Err(Error::Host),
        }}
    }}

    pub fn transfer(&self, context: &Context<'_>, send: &[u8], receive: &mut [u8]) -> Result<bool> {{
        if send.len() > I2C_TRANSFER_MAX || receive.len() > I2C_TRANSFER_MAX {{ return Err(Error::Invalid); }}
        let mut input = [0u8; 4 + 2 + I2C_TRANSFER_MAX];
        input[..4].copy_from_slice(&self.token()?.get().to_le_bytes());
        input[4] = send.len() as u8;
        input[5] = receive.len() as u8;
        input[6..6 + send.len()].copy_from_slice(send);
        let mut output = [0u8; I2C_TRANSFER_MAX + 1];
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{ids['transfer']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        match output[0] {{
            0 => Ok(false),
            1 => {{ receive.copy_from_slice(&output[1..1 + receive.len()]); Ok(true) }}
            _ => Err(Error::Host),
        }}
    }}

    pub fn close(mut self, context: &Context<'_>) -> Result {{
        let input = self.token()?.get().to_le_bytes();
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{ids['close']}.slot(),
                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)
        }})?;
        self.token = None;
        Ok(())
    }}
}}
'''
