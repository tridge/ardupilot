# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Owned EFI state and revalidated scripting-backend handle."""

STATE_WIRE_SIZE = 101
OWNER = 'AP_EFI'
BACKEND = 'AP_EFI_Backend'
STATE = 'EFI_State'
CYLINDER = 'Cylinder_Status'
DEPENDS = '(AP_EFI_SCRIPTING_ENABLED == 1)'
POLICY = ('v1:complete-owned-copy,no-retained-firmware-pointers,backend-index-revalidated-each-call,'
          'optional-f32-none-is-nan,ignition-voltage-none-is-minus-one,enum-ranges-validated')


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _method_shape(method):
    return (_field_shape(method['return']), tuple(_field_shape(arg) for arg in method['arguments']),
            method['rename'], method['depends'], method['deprecate'])


def _owner_metadata(owner, *, fields=False):
    keys = ('rename', 'depends', 'get', 'literal', 'reference', 'semaphore',
            'scheduler_semaphore', 'semaphore_pointer')
    result = tuple(owner[key] for key in keys)
    if fields:
        result += (owner['creation'], owner['creation_args'], owner['creation_manual'], owner['has_fields'],
                   owner['has_enums'], owner['operations'], owner['operator_getter'], owner['operators_manual'],
                   tuple(owner['operator_names']))
    return result


def _userdata_fields(owner):
    return {
        field['name']: (_field_shape(field['type']), field['read'], field['write'], field['rename'],
                        field['depends'], field['array_len'], field['valid_mask_name'], field['valid_mask_value'])
        for field in owner['fields']
    }


def select(ir, first_id):
    """Select EFI only when the audited Lua owner and value shapes match exactly."""
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected_owner = ('efi', DEPENDS, None, False, False, False, False, False)
    if (owner is None or _owner_metadata(owner) != expected_owner or owner['aliases'] or owner['fields']):
        raise ValueError('unaudited EFI owner metadata')
    expected_owner_methods = {
        'get_backend': ((('ap_object', BACKEND, False, False, False, False, None),
                         (('uint8_t', None, False, False, False, True, None),), None, None, None)),
        'get_state': ((('void', None, False, False, False, False, None),
                       (('userdata', STATE, False, True, False, False, None),), None, None, None)),
        'get_last_update_ms': ((('uint32_t', None, False, False, False, False, None),
                                (), None, None, None)),
    }
    actual_owner_methods = {method['name']: _method_shape(method) for method in owner['methods']}
    if len(owner['methods']) != len(expected_owner_methods) or actual_owner_methods != expected_owner_methods:
        raise ValueError('unaudited EFI owner methods')

    backend = next((item for item in ir['ap_objects'] if item['name'] == BACKEND), None)
    expected_value_metadata = (None, DEPENDS, None, False, False, False, False, False,
                               None, 0, False, False, False, 0, None, False, ())
    if (backend is None or _owner_metadata(backend, fields=True) != expected_value_metadata or
            backend['aliases'] or backend['fields']):
        raise ValueError('unaudited EFI backend metadata')
    expected_backend_methods = {
        'handle_scripting': (('boolean', None, False, False, False, False, None),
                             (('userdata', STATE, False, False, False, False, None),), None, None, None),
    }
    actual_backend_methods = {method['name']: _method_shape(method) for method in backend['methods']}
    if len(backend['methods']) != len(expected_backend_methods) or actual_backend_methods != expected_backend_methods:
        raise ValueError('unaudited EFI backend methods')

    common_float = (('float', None, False, False, False, True, None), True, True, None, None, None, None, None)
    expected_cylinder_fields = {name: common_float for name in (
        'ignition_timing_deg', 'injection_time_ms', 'cylinder_head_temperature',
        'cylinder_head_temperature2', 'exhaust_gas_temperature', 'exhaust_gas_temperature2',
        'lambda_coefficient')}
    common_value_metadata = (None, DEPENDS, None, False, False, False, False, False,
                             None, 0, False, True, False, 0, None, False, ())
    cylinder = next((item for item in ir['userdata'] if item['name'] == CYLINDER), None)
    if (cylinder is None or _owner_metadata(cylinder, fields=True) != common_value_metadata or
            cylinder['aliases'] or cylinder['methods'] or len(cylinder['fields']) != len(expected_cylinder_fields) or
            _userdata_fields(cylinder) != expected_cylinder_fields):
        raise ValueError('unaudited EFI cylinder state')

    expected_state_fields = {
        name: common_float for name in (
            'spark_dwell_time_ms', 'atmospheric_pressure_kpa', 'intake_manifold_pressure_kpa',
            'intake_manifold_temperature', 'coolant_temperature', 'oil_pressure', 'oil_temperature',
            'fuel_pressure', 'fuel_consumption_rate_cm3pm', 'estimated_consumed_fuel_volume_cm3',
            'ignition_voltage', 'throttle_out', 'pt_compensation')
    }
    for name in ('last_updated_ms', 'engine_speed_rpm'):
        expected_state_fields[name] = (('uint32_t', None, False, False, False, True, None),
                                       True, True, None, None, None, None, None)
    for name in ('engine_load_percent', 'throttle_position_percent', 'ecu_index'):
        expected_state_fields[name] = (('uint8_t', None, False, False, False, True, None),
                                       True, True, None, None, None, None, None)
    expected_state_fields['general_error'] = (('boolean', None, False, False, False, False, None),
                                              True, True, None, None, None, None, None)
    expected_state_fields['fuel_pressure_status'] = (
        ('enum', 'Fuel_Pressure_Status', False, False, False, False,
         ('Fuel_Pressure_Status::NOT_SUPPORTED', 'Fuel_Pressure_Status::ABOVE_NOMINAL')),
        True, True, None, None, None, None, None)
    expected_state_fields['cylinder_status'] = (('userdata', CYLINDER, False, False, False, False, None),
                                                True, True, None, None, None, None, None)
    state = next((item for item in ir['userdata'] if item['name'] == STATE), None)
    if (state is None or _owner_metadata(state, fields=True) != common_value_metadata or
            state['aliases'] or state['methods'] or len(state['fields']) != len(expected_state_fields) or
            _userdata_fields(state) != expected_state_fields):
        raise ValueError('unaudited EFI state')
    return specification(first_id)


def specification(first_id):
    cylinder = dict(name='EfiCylinderState', fields=[
        ['ignition_timing_deg', 'optional-f32-le,nan-unavailable'],
        ['injection_time_ms', 'optional-f32-le,nan-unavailable'],
        ['cylinder_head_temperature', 'optional-f32-le,nan-unavailable'],
        ['cylinder_head_temperature2', 'optional-f32-le,nan-unavailable'],
        ['exhaust_gas_temperature', 'optional-f32-le,nan-unavailable'],
        ['exhaust_gas_temperature2', 'optional-f32-le,nan-unavailable'],
        ['lambda_coefficient', 'optional-f32-le,nan-unavailable'],
    ])
    state = dict(name='EfiState', fields=[
        ['last_updated_ms', 'u32-le'], ['engine_state', 'EngineState-u8'], ['general_error', 'bool-u8'],
        ['crankshaft_sensor_status', 'CrankshaftSensorStatus-u8'],
        ['temperature_status', 'TemperatureStatus-u8'], ['fuel_pressure_status', 'FuelPressureStatus-u8'],
        ['oil_pressure_status', 'OilPressureStatus-u8'], ['detonation_status', 'DetonationStatus-u8'],
        ['misfire_status', 'MisfireStatus-u8'], ['debris_status', 'DebrisStatus-u8'],
        ['engine_load_percent', 'u8'], ['engine_speed_rpm', 'u32-le'],
        ['spark_dwell_time_ms', 'optional-f32-le,nan-unavailable'],
        ['atmospheric_pressure_kpa', 'optional-f32-le,nan-unavailable'],
        ['intake_manifold_pressure_kpa', 'optional-f32-le,nan-unavailable'],
        ['intake_manifold_temperature', 'optional-f32-le,nan-unavailable'],
        ['coolant_temperature', 'optional-f32-le,nan-unavailable'],
        ['oil_pressure', 'optional-f32-le,nan-unavailable'],
        ['oil_temperature', 'optional-f32-le,nan-unavailable'],
        ['fuel_pressure', 'optional-f32-le,nan-unavailable'],
        ['fuel_consumption_rate_cm3pm', 'optional-f32-le,nan-unavailable'],
        ['estimated_consumed_fuel_volume_cm3', 'optional-f32-le,nan-unavailable'],
        ['throttle_position_percent', 'u8'], ['ecu_index', 'u8'], ['spark_plug_usage', 'SparkPlugUsage-u8'],
        ['cylinder_status', cylinder], ['ignition_voltage', 'optional-f32-le,minus-one-unavailable'],
        ['throttle_out', 'optional-f32-le,nan-unavailable'],
        ['pt_compensation', 'optional-f32-le,nan-unavailable'],
    ])
    get_backend = dict(id=first_id, name='efi.get_backend',
                       arguments=[dict(type='u8', direction='in')],
                       result=dict(presence='bool-u8', handle='EfiBackend,index-revalidated'), policy=POLICY)
    handle = dict(id=first_id + 1, name='EfiBackend.handle_scripting',
                  arguments=[dict(type='u8', direction='revalidated-backend-index'),
                             dict(type=state, direction='in')], result='bool-u8', policy=POLICY)
    get_state = dict(id=first_id + 2, name='efi.get_state', arguments=[],
                     result=dict(type=state, direction='owned-out'), policy=POLICY)
    return dict(methods=[get_backend, handle, get_state])


def rust(spec, notice):
    get_backend, handle, get_state = spec['methods']
    return notice + f'''use crate::{{result, Context, Error, Result}};

macro_rules! efi_enum {{
    ($name:ident {{ $first:ident = $zero:expr, $($variant:ident = $value:expr),+ $(,)? }}) => {{
        #[derive(Clone, Copy, Debug, Eq, PartialEq)]
        #[repr(u8)]
        pub enum $name {{ $first = $zero, $($variant = $value),+ }}
        impl Default for $name {{ fn default() -> Self {{ Self::$first }} }}
        impl TryFrom<u8> for $name {{
            type Error = Error;
            fn try_from(value: u8) -> Result<Self> {{
                match value {{
                    $zero => Ok(Self::$first), $($value => Ok(Self::$variant),)+ _ => Err(Error::Host),
                }}
            }}
        }}
    }};
}}

efi_enum!(EngineState {{ Stopped = 0, Starting = 1, Running = 2, Fault = 3 }});
efi_enum!(CrankshaftSensorStatus {{ NotSupported = 0, Ok = 1, Error = 2 }});
efi_enum!(TemperatureStatus {{ NotSupported = 0, Ok = 1, BelowNominal = 2, AboveNominal = 3,
                              Overheating = 4, EgtAboveNominal = 5 }});
efi_enum!(FuelPressureStatus {{ NotSupported = 0, Ok = 1, BelowNominal = 2, AboveNominal = 3 }});
efi_enum!(OilPressureStatus {{ NotSupported = 0, Ok = 1, BelowNominal = 2, AboveNominal = 3 }});
efi_enum!(DetonationStatus {{ NotSupported = 0, NotObserved = 1, Observed = 2 }});
efi_enum!(MisfireStatus {{ NotSupported = 0, NotObserved = 1, Observed = 2 }});
efi_enum!(DebrisStatus {{ NotSupported = 0, NotDetected = 1, Detected = 2 }});
efi_enum!(SparkPlugUsage {{ Single = 0, FirstActive = 1, SecondActive = 2, BothActive = 3 }});

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EfiCylinderState {{
    pub ignition_timing_deg: Option<f32>,
    pub injection_time_ms: Option<f32>,
    pub cylinder_head_temperature: Option<f32>,
    pub cylinder_head_temperature2: Option<f32>,
    pub exhaust_gas_temperature: Option<f32>,
    pub exhaust_gas_temperature2: Option<f32>,
    pub lambda_coefficient: Option<f32>,
}}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct EfiState {{
    pub last_updated_ms: u32,
    pub engine_state: EngineState,
    pub general_error: bool,
    pub crankshaft_sensor_status: CrankshaftSensorStatus,
    pub temperature_status: TemperatureStatus,
    pub fuel_pressure_status: FuelPressureStatus,
    pub oil_pressure_status: OilPressureStatus,
    pub detonation_status: DetonationStatus,
    pub misfire_status: MisfireStatus,
    pub debris_status: DebrisStatus,
    pub engine_load_percent: u8,
    pub engine_speed_rpm: u32,
    pub spark_dwell_time_ms: Option<f32>,
    pub atmospheric_pressure_kpa: Option<f32>,
    pub intake_manifold_pressure_kpa: Option<f32>,
    pub intake_manifold_temperature: Option<f32>,
    pub coolant_temperature: Option<f32>,
    pub oil_pressure: Option<f32>,
    pub oil_temperature: Option<f32>,
    pub fuel_pressure: Option<f32>,
    pub fuel_consumption_rate_cm3pm: Option<f32>,
    pub estimated_consumed_fuel_volume_cm3: Option<f32>,
    pub throttle_position_percent: u8,
    pub ecu_index: u8,
    pub spark_plug_usage: SparkPlugUsage,
    pub cylinder_status: EfiCylinderState,
    pub ignition_voltage: Option<f32>,
    pub throttle_out: Option<f32>,
    pub pt_compensation: Option<f32>,
}}

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct EfiBackend {{ index: u8 }}

fn efi_float(output: &mut [u8], offset: usize, value: Option<f32>) -> Result {{
    if value.is_some_and(f32::is_nan) {{ return Err(Error::Invalid); }}
    output[offset..offset+4].copy_from_slice(&value.unwrap_or(f32::NAN).to_bits().to_le_bytes());
    Ok(())
}}

fn efi_read_float(input: &[u8], offset: usize) -> f32 {{
    f32::from_le_bytes(input[offset..offset + 4].try_into().unwrap())
}}

fn efi_optional_float(input: &[u8], offset: usize) -> Option<f32> {{
    let value = efi_read_float(input, offset);
    (!value.is_nan()).then_some(value)
}}

fn efi_decode(input: &[u8]) -> Result<EfiState> {{
    if input.len() != {STATE_WIRE_SIZE} || input[5] > 1 {{ return Err(Error::Host); }}
    let ignition_voltage = efi_read_float(input, 89);
    if ignition_voltage.is_nan() {{ return Err(Error::Host); }}
    Ok(EfiState {{
        last_updated_ms: u32::from_le_bytes(input[0..4].try_into().unwrap()),
        engine_state: input[4].try_into()?,
        general_error: input[5] != 0,
        crankshaft_sensor_status: input[6].try_into()?,
        temperature_status: input[7].try_into()?,
        fuel_pressure_status: input[8].try_into()?,
        oil_pressure_status: input[9].try_into()?,
        detonation_status: input[10].try_into()?,
        misfire_status: input[11].try_into()?,
        debris_status: input[12].try_into()?,
        engine_load_percent: input[13],
        engine_speed_rpm: u32::from_le_bytes(input[14..18].try_into().unwrap()),
        spark_dwell_time_ms: efi_optional_float(input, 18),
        atmospheric_pressure_kpa: efi_optional_float(input, 22),
        intake_manifold_pressure_kpa: efi_optional_float(input, 26),
        intake_manifold_temperature: efi_optional_float(input, 30),
        coolant_temperature: efi_optional_float(input, 34),
        oil_pressure: efi_optional_float(input, 38),
        oil_temperature: efi_optional_float(input, 42),
        fuel_pressure: efi_optional_float(input, 46),
        fuel_consumption_rate_cm3pm: efi_optional_float(input, 50),
        estimated_consumed_fuel_volume_cm3: efi_optional_float(input, 54),
        throttle_position_percent: input[58],
        ecu_index: input[59],
        spark_plug_usage: input[60].try_into()?,
        cylinder_status: EfiCylinderState {{
            ignition_timing_deg: efi_optional_float(input, 61),
            injection_time_ms: efi_optional_float(input, 65),
            cylinder_head_temperature: efi_optional_float(input, 69),
            cylinder_head_temperature2: efi_optional_float(input, 73),
            exhaust_gas_temperature: efi_optional_float(input, 77),
            exhaust_gas_temperature2: efi_optional_float(input, 81),
            lambda_coefficient: efi_optional_float(input, 85),
        }},
        ignition_voltage: (ignition_voltage != -1.0).then_some(ignition_voltage),
        throttle_out: efi_optional_float(input, 93),
        pt_compensation: efi_optional_float(input, 97),
    }})
}}

impl crate::bindings::Efi<'_, '_> {{
    pub fn get_backend(&self, index: u8) -> Result<Option<EfiBackend>> {{
        let input = [index];
        let mut output = [0u8; 1];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{get_backend['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        match output[0] {{ 0 => Ok(None), 1 => Ok(Some(EfiBackend {{ index }})), _ => Err(Error::Host) }}
    }}

    pub fn get_state(&self) -> Result<EfiState> {{
        let mut output = [0u8; {STATE_WIRE_SIZE}];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(self.context.raw,
                crate::imports::BINDING_{get_state['id']}.slot(),
                core::ptr::null(), 0, output.as_mut_ptr(), output.len() as u32)
        }})?;
        efi_decode(&output)
    }}
}}

impl EfiBackend {{
    pub fn handle_scripting(&self, context: &Context<'_>, state: &EfiState) -> Result<bool> {{
        if state.ignition_voltage.is_some_and(|value| value.is_nan() || value == -1.0) {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; {STATE_WIRE_SIZE + 1}];
        input[0] = self.index;
        input[1..5].copy_from_slice(&state.last_updated_ms.to_le_bytes());
        input[5..15].copy_from_slice(&[
            state.engine_state as u8, u8::from(state.general_error), state.crankshaft_sensor_status as u8,
            state.temperature_status as u8, state.fuel_pressure_status as u8, state.oil_pressure_status as u8,
            state.detonation_status as u8, state.misfire_status as u8, state.debris_status as u8,
            state.engine_load_percent,
        ]);
        input[15..19].copy_from_slice(&state.engine_speed_rpm.to_le_bytes());
        for (offset, value) in [
            (19, state.spark_dwell_time_ms), (23, state.atmospheric_pressure_kpa),
            (27, state.intake_manifold_pressure_kpa), (31, state.intake_manifold_temperature),
            (35, state.coolant_temperature), (39, state.oil_pressure), (43, state.oil_temperature),
            (47, state.fuel_pressure), (51, state.fuel_consumption_rate_cm3pm),
            (55, state.estimated_consumed_fuel_volume_cm3),
        ] {{ efi_float(&mut input, offset, value)?; }}
        input[59..62].copy_from_slice(&[
            state.throttle_position_percent, state.ecu_index, state.spark_plug_usage as u8,
        ]);
        for (offset, value) in [
            (62, state.cylinder_status.ignition_timing_deg),
            (66, state.cylinder_status.injection_time_ms),
            (70, state.cylinder_status.cylinder_head_temperature),
            (74, state.cylinder_status.cylinder_head_temperature2),
            (78, state.cylinder_status.exhaust_gas_temperature),
            (82, state.cylinder_status.exhaust_gas_temperature2),
            (86, state.cylinder_status.lambda_coefficient),
        ] {{ efi_float(&mut input, offset, value)?; }}
        input[90..94].copy_from_slice(&state.ignition_voltage.unwrap_or(-1.0).to_bits().to_le_bytes());
        efi_float(&mut input, 94, state.throttle_out)?;
        efi_float(&mut input, 98, state.pt_compensation)?;
        let mut output = [0u8; 1];
        result(unsafe {{
            context.api.binding_call.unwrap()(context.raw, crate::imports::BINDING_{handle['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}
}}
'''
