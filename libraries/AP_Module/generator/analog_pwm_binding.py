# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Opaque analog-input and lifecycle-owned PWM source handles."""

ANALOG_OWNER = 'hal.analogin'
ANALOG_SOURCE = 'AP_HAL::AnalogSource'
PWM_SOURCE = 'AP_HAL::PWMSource'
POLICY = ('v1:opaque-context-owned-handles,max4-each-per-module,analog-source-reused-after-cleanup,'
          'pwm-source-deleted-after-close-or-worker-join,no-retained-module-pointers')


def _shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _method(method):
    return (_shape(method['return']), tuple(_shape(argument) for argument in method['arguments']),
            method['rename'], method['depends'], method['deprecate'])


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == ANALOG_OWNER), None)
    channel = None if owner is None else next(
        (method for method in owner['methods'] if method['name'] == 'channel'), None)
    expected_channel = (('ap_object', ANALOG_SOURCE, False, False, False, False, None),
                        (('literal', 'ANALOG_INPUT_NONE', False, False, False, False, None),),
                        None, None, None)
    if (owner is None or owner['rename'] != 'analog' or
            owner['depends'] != '!defined(HAL_DISABLE_ADC_DRIVER)' or not owner['literal'] or
            channel is None or _method(channel) != expected_channel):
        raise ValueError('unaudited analog.channel declaration')

    analog = next((item for item in ir['ap_objects'] if item['name'] == ANALOG_SOURCE), None)
    expected_analog = {
        'set_pin': (('boolean', None, False, False, False, False, None),
                    (('uint8_t', None, False, False, False, True, None),), None, None, None),
        'voltage_average': (('float', None, False, False, False, False, None), (), None, None, None),
        'voltage_latest': (('float', None, False, False, False, False, None), (), None, None, None),
        'voltage_average_ratiometric': (
            ('float', None, False, False, False, False, None), (), None, None, None),
    }
    if (analog is None or analog['depends'] != '!defined(HAL_DISABLE_ADC_DRIVER)' or
            analog['aliases'] or {method['name']: _method(method) for method in analog['methods']} !=
            expected_analog):
        raise ValueError('unaudited AnalogSource declaration')

    pwm = next((item for item in ir['ap_objects'] if item['name'] == PWM_SOURCE), None)
    expected_pwm = {
        'set_pin': (('boolean', None, False, False, False, False, None),
                    (('uint8_t', None, False, False, False, True, None),
                     ('literal', '"Scripting"', False, False, False, False, None)), None, None, None),
        'get_pwm_us': (('uint16_t', None, False, False, False, False, None), (), None, None, None),
        'get_pwm_avg_us': (('uint16_t', None, False, False, False, False, None), (), None, None, None),
    }
    if (pwm is None or pwm['depends'] is not None or pwm['aliases'] or
            {method['name']: _method(method) for method in pwm['methods']} != expected_pwm):
        raise ValueError('unaudited PWMSource declaration')
    return specification(first_id)


def specification(first_id):
    names = [
        'analog.channel', 'AnalogSource.set_pin', 'AnalogSource.voltage_average',
        'AnalogSource.voltage_latest', 'AnalogSource.voltage_average_ratiometric',
        'pwm_source.open', 'pwm_source.close', 'PwmSource.set_pin', 'PwmSource.get_pwm_us',
        'PwmSource.get_pwm_avg_us',
    ]
    arguments = [
        [],
        [dict(type='nonzero-u32-le', name='handle'), dict(type='u8', name='pin')],
        [dict(type='nonzero-u32-le', name='handle')],
        [dict(type='nonzero-u32-le', name='handle')],
        [dict(type='nonzero-u32-le', name='handle')],
        [],
        [dict(type='nonzero-u32-le', name='handle')],
        [dict(type='nonzero-u32-le', name='handle'), dict(type='u8', name='pin')],
        [dict(type='nonzero-u32-le', name='handle')],
        [dict(type='nonzero-u32-le', name='handle')],
    ]
    results = ['optional-nonzero-u32-le', 'bool-u8', 'f32-le', 'f32-le', 'f32-le',
               'optional-nonzero-u32-le', 'void', 'bool-u8', 'u16-le', 'u16-le']
    return dict(methods=[dict(id=first_id + index, name=name, arguments=arguments[index],
                              result=results[index], policy=POLICY)
                         for index, name in enumerate(names)])


def rust(spec, notice):
    ids = {method['name']: method['id'] for method in spec['methods']}
    return notice + f'''use core::num::NonZeroU32;
use crate::{{result, Context, Error, Result}};

pub struct AnalogSource {{ token: NonZeroU32 }}
pub struct PwmSource {{ token: Option<NonZeroU32> }}

fn open(context: &Context<'_>, id: u32) -> Result<Option<NonZeroU32>> {{
    let mut output = [0u8; 4];
    result(unsafe {{ context.api.binding_call.unwrap()(context.raw, id, core::ptr::null(), 0,
        output.as_mut_ptr(), output.len() as u32) }})?;
    Ok(NonZeroU32::new(u32::from_le_bytes(output)))
}}

fn token_call(context: &Context<'_>, token: NonZeroU32, id: u32, output: &mut [u8]) -> Result {{
    let input = token.get().to_le_bytes();
    result(unsafe {{ context.api.binding_call.unwrap()(context.raw, id, input.as_ptr(), input.len() as u32,
        output.as_mut_ptr(), output.len() as u32) }})
}}

fn set_pin(context: &Context<'_>, token: NonZeroU32, pin: u8, id: u32) -> Result<bool> {{
    let mut input = [0u8; 5];
    input[..4].copy_from_slice(&token.get().to_le_bytes());
    input[4] = pin;
    let mut output = [0u8; 1];
    result(unsafe {{ context.api.binding_call.unwrap()(context.raw, id, input.as_ptr(), input.len() as u32,
        output.as_mut_ptr(), output.len() as u32) }})?;
    match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
}}

impl crate::bindings::Analog<'_, '_> {{
    pub fn channel(&self) -> Result<Option<AnalogSource>> {{
        open(self.context, crate::imports::BINDING_{ids['analog.channel']}.slot())
            .map(|token| token.map(|token| AnalogSource {{ token }}))
    }}
}}

impl crate::bindings::PwmSource<'_, '_> {{
    pub fn open(&self) -> Result<Option<PwmSource>> {{
        open(self.context, crate::imports::BINDING_{ids['pwm_source.open']}.slot())
            .map(|token| token.map(|token| PwmSource {{ token: Some(token) }}))
    }}
}}

impl AnalogSource {{
    pub fn set_pin(&self, context: &Context<'_>, pin: u8) -> Result<bool> {{
        set_pin(context, self.token, pin, crate::imports::BINDING_{ids['AnalogSource.set_pin']}.slot())
    }}

    fn voltage(&self, context: &Context<'_>, id: u32) -> Result<f32> {{
        let mut output = [0u8; 4];
        token_call(context, self.token, id, &mut output)?;
        Ok(f32::from_le_bytes(output))
    }}

    pub fn voltage_average(&self, context: &Context<'_>) -> Result<f32> {{
        self.voltage(context, crate::imports::BINDING_{ids['AnalogSource.voltage_average']}.slot())
    }}

    pub fn voltage_latest(&self, context: &Context<'_>) -> Result<f32> {{
        self.voltage(context, crate::imports::BINDING_{ids['AnalogSource.voltage_latest']}.slot())
    }}

    pub fn voltage_average_ratiometric(&self, context: &Context<'_>) -> Result<f32> {{
        self.voltage(context,
            crate::imports::BINDING_{ids['AnalogSource.voltage_average_ratiometric']}.slot())
    }}
}}

impl PwmSource {{
    fn token(&self) -> Result<NonZeroU32> {{ self.token.ok_or(Error::Invalid) }}

    pub fn set_pin(&self, context: &Context<'_>, pin: u8) -> Result<bool> {{
        set_pin(context, self.token()?, pin, crate::imports::BINDING_{ids['PwmSource.set_pin']}.slot())
    }}

    fn read(&self, context: &Context<'_>, id: u32) -> Result<u16> {{
        let mut output = [0u8; 2];
        token_call(context, self.token()?, id, &mut output)?;
        Ok(u16::from_le_bytes(output))
    }}

    pub fn get_pwm_us(&self, context: &Context<'_>) -> Result<u16> {{
        self.read(context, crate::imports::BINDING_{ids['PwmSource.get_pwm_us']}.slot())
    }}

    pub fn get_pwm_avg_us(&self, context: &Context<'_>) -> Result<u16> {{
        self.read(context, crate::imports::BINDING_{ids['PwmSource.get_pwm_avg_us']}.slot())
    }}

    pub fn close(mut self, context: &Context<'_>) -> Result {{
        let input = self.token()?.get().to_le_bytes();
        result(unsafe {{ context.api.binding_call.unwrap()(context.raw,
            crate::imports::BINDING_{ids['pwm_source.close']}.slot(), input.as_ptr(), input.len() as u32,
            core::ptr::null_mut(), 0) }})?;
        self.token = None;
        Ok(())
    }}
}}
'''
