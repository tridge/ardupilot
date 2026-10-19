# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Safety-checked adapters for scriptable multicopter motor matrices."""

MATRIX_OWNER = 'AP_MotorsMatrix'
DYNAMIC_OWNER = 'AP_MotorsMatrix_Scripting_Dynamic'
SIX_DOF_OWNER = 'AP_MotorsMatrix_6DoF_Scripting'
FACTOR_TABLE = 'AP_MotorsMatrix_Scripting_Dynamic::factor_table'
MOTORS_OWNER = 'AP::motors()'
MATRIX_DEPENDS = 'APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI'
DYNAMIC_DEPENDS = ('(APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI) && '
                   'AP_MOTORS_FRAME_DYNAMIC_SCRIPTING_MATRIX_ENABLED')
SIX_DOF_DEPENDS = ('APM_BUILD_TYPE(APM_BUILD_ArduCopter) && '
                   'AP_MOTORS_FRAME_6DOF_SCRIPTING_ENABLED')
MATRIX_INIT_DEPENDS = ('(AP_MOTORS_FRAME_SCRIPTING_MATRIX_ENABLED || '
                       'AP_MOTORS_FRAME_6DOF_SCRIPTING_ENABLED || '
                       'AP_MOTORS_FRAME_DYNAMIC_SCRIPTING_MATRIX_ENABLED)')
MATRIX_THROTTLE_DEPENDS = 'AP_MOTORS_FRAME_SCRIPTING_MATRIX_ENABLED'
MOTORS_DEPENDS = ('AP_SCRIPTING_BINDING_MOTORS_ENABLED && '
                  '(APM_BUILD_TYPE(APM_BUILD_ArduPlane)||APM_BUILD_COPTER_OR_HELI)')
FACTOR_COUNT = 32
FRAME_STRING_MAX = 32
POLICY = ('v1:shared-global-motor-configuration,call-level-blocking-binding-lock,'
          'no-rollback-on-module-exit,no-retained-module-pointers,native-singletons-revalidated-each-call,'
          'indices-checked-against-firmware-motor-count,finite-factor-inputs,'
          'factor-table32-zero-above-firmware-count,frame-string-copy-max32-first-write-wins')
SIX_DOF_POLICY = POLICY + ',six-dof-complete-finite-factors,copter-only'


def _field_shape(field):
    return (field['kind'], field['name'], field['nullable'], field['reference'], field['pointer'],
            field['skip_check'], tuple(field['range']) if field['range'] else None)


def _method_shape(method):
    return (_field_shape(method['return']), tuple(_field_shape(arg) for arg in method['arguments']),
            method['rename'], method['depends'], method['deprecate'])


def _metadata(owner, *, fields=False):
    keys = ('rename', 'depends', 'get', 'literal', 'reference', 'semaphore',
            'scheduler_semaphore', 'semaphore_pointer')
    result = tuple(owner[key] for key in keys)
    if fields:
        result += (owner['creation'], owner['creation_args'], owner['creation_manual'], owner['has_fields'],
                   owner['has_enums'], owner['operations'], owner['operator_getter'], owner['operators_manual'],
                   tuple(owner['operator_names']))
    return result


def _shape(result, arguments=(), dependency=None):
    return (result, tuple(arguments), None, dependency, None)


def select(ir, first_id):
    """Fail closed unless all motor-configuration source contracts match the audit."""
    void = ('void', None, False, False, False, False, None)
    boolean = ('boolean', None, False, False, False, False, None)
    u8 = ('uint8_t', None, False, False, False, False, None)
    int8 = ('int8_t', None, False, False, False, False, None)
    float_skip = ('float', None, False, False, False, True, None)

    dynamic = next((item for item in ir['singletons'] if item['name'] == DYNAMIC_OWNER), None)
    expected_dynamic_metadata = ('Motors_dynamic', DYNAMIC_DEPENDS, None, False, False, False, False, False)
    expected_dynamic_methods = {
        'init': _shape(boolean, ((u8[0], None, False, False, False, False,
                                  ('0', 'AP_MOTORS_MAX_NUM_MOTORS')),)),
        'add_motor': _shape(void, (
            (u8[0], None, False, False, False, False, ('0', '(AP_MOTORS_MAX_NUM_MOTORS-1)')),
            (u8[0], None, False, False, False, False, ('0', 'AP_MOTORS_MAX_NUM_MOTORS')))),
        'load_factors': _shape(void, (
            ('userdata', FACTOR_TABLE, False, False, False, False, None),)),
    }
    if (dynamic is None or _metadata(dynamic) != expected_dynamic_metadata or dynamic['aliases'] or
            dynamic['fields'] or
            {method['name']: _method_shape(method) for method in dynamic['methods']} != expected_dynamic_methods):
        raise ValueError('unaudited dynamic motor matrix')

    matrix = next((item for item in ir['singletons'] if item['name'] == MATRIX_OWNER), None)
    expected_matrix_metadata = ('MotorsMatrix', MATRIX_DEPENDS, None, False, False, False, False, False)
    expected_matrix_names = {'add_motor_raw', 'get_lost_motor', 'get_thrust_boost', 'init', 'set_throttle_factor'}
    if (matrix is None or _metadata(matrix) != expected_matrix_metadata or matrix['aliases'] or matrix['fields'] or
            {method['name'] for method in matrix['methods']} != expected_matrix_names):
        raise ValueError('unaudited static motor matrix metadata')
    expected_matrix_methods = {
        'init': _shape(boolean, ((u8[0], None, False, False, False, False,
                                  ('0', 'AP_MOTORS_MAX_NUM_MOTORS')),), MATRIX_INIT_DEPENDS),
        'add_motor_raw': _shape(void, (
            (int8[0], None, False, False, False, False, ('0', '(AP_MOTORS_MAX_NUM_MOTORS-1)')),
            float_skip, float_skip, float_skip,
            (u8[0], None, False, False, False, False, ('0', 'AP_MOTORS_MAX_NUM_MOTORS')))),
        'set_throttle_factor': _shape(boolean, (
            (int8[0], None, False, False, False, False, ('0', '(AP_MOTORS_MAX_NUM_MOTORS-1)')),
            ('float', None, False, False, False, False, ('0', 'FLT_MAX'))), MATRIX_THROTTLE_DEPENDS),
    }
    actual_matrix_methods = {
        method['name']: _method_shape(method) for method in matrix['methods']
        if method['name'] in expected_matrix_methods
    }
    if actual_matrix_methods != expected_matrix_methods:
        raise ValueError('unaudited static motor matrix methods')

    six_dof = next((item for item in ir['singletons'] if item['name'] == SIX_DOF_OWNER), None)
    expected_six_dof_metadata = ('Motors_6DoF', SIX_DOF_DEPENDS, None, False, False, False, False, False)
    expected_six_dof_methods = {
        'init': _shape(boolean, ((u8[0], None, False, False, False, False,
                                  ('0', 'AP_MOTORS_MAX_NUM_MOTORS')),)),
        'add_motor': _shape(void, (
            (int8[0], None, False, False, False, False, ('0', '(AP_MOTORS_MAX_NUM_MOTORS-1)')),
            float_skip, float_skip, float_skip, float_skip, float_skip, float_skip,
            ('boolean', None, False, False, False, False, None),
            (u8[0], None, False, False, False, False, ('0', 'AP_MOTORS_MAX_NUM_MOTORS')))),
    }
    if (six_dof is None or _metadata(six_dof) != expected_six_dof_metadata or six_dof['aliases'] or
            six_dof['fields'] or
            {method['name']: _method_shape(method) for method in six_dof['methods']} != expected_six_dof_methods):
        raise ValueError('unaudited 6DoF motor matrix')

    table = next((item for item in ir['userdata'] if item['name'] == FACTOR_TABLE), None)
    expected_table_metadata = ('motor_factor_table', DYNAMIC_DEPENDS, None, False, False, False, False, False,
                               None, 0, False, True, False, 0, None, False, ())
    if (table is None or _metadata(table, fields=True) != expected_table_metadata or table['aliases'] or
            table['methods'] or {field['name'] for field in table['fields']} !=
            {'roll', 'pitch', 'yaw', 'throttle'}):
        raise ValueError('unaudited motor factor table metadata')
    for field in table['fields']:
        expected = (float_skip, True, True, None, None, 'AP_MOTORS_MAX_NUM_MOTORS', None, None)
        actual = (_field_shape(field['type']), field['read'], field['write'], field['rename'], field['depends'],
                  field['array_len'], field['valid_mask_name'], field['valid_mask_value'])
        if actual != expected:
            raise ValueError('unaudited motor factor table fields')

    motors = next((item for item in ir['singletons'] if item['name'] == MOTORS_OWNER), None)
    expected_motors_metadata = ('motors', MOTORS_DEPENDS, None, True, False, False, False, False)
    expected_motor_names = {
        'get_desired_spool_state', 'get_forward', 'get_interlock', 'get_lateral', 'get_pitch',
        'get_pitch_ff', 'get_roll', 'get_roll_ff', 'get_spool_state', 'get_throttle', 'get_throttle_in',
        'get_yaw', 'get_yaw_ff', 'set_external_limits', 'set_frame_string',
    }
    if (motors is None or _metadata(motors) != expected_motors_metadata or motors['aliases'] or motors['fields'] or
            {method['name'] for method in motors['methods']} != expected_motor_names):
        raise ValueError('unaudited motors owner metadata')
    frame_method = next(method for method in motors['methods'] if method['name'] == 'set_frame_string')
    if _method_shape(frame_method) != _shape(void, (('string', None, False, False, False, False, None),)):
        raise ValueError('unaudited motors frame string')
    return specification(first_id)


def specification(first_id):
    factor_table = dict(name='MotorFactorTable', fields=[
        ['roll', f'f32-le[{FACTOR_COUNT}],finite'],
        ['pitch', f'f32-le[{FACTOR_COUNT}],finite'],
        ['yaw', f'f32-le[{FACTOR_COUNT}],finite'],
        ['throttle', f'f32-le[{FACTOR_COUNT}],finite'],
    ])
    methods = [
        dict(id=first_id, name='MotorsDynamic.init',
             arguments=[dict(type='u8', direction='in', range=[0, 'firmware-motor-count'])],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 1, name='MotorsDynamic.add_motor',
             arguments=[dict(type='u8', direction='in', name='motor',
                             range=[0, 'firmware-motor-count-minus-one']),
                        dict(type='u8', direction='in', name='testing_order',
                             range=[0, 'firmware-motor-count'])],
             result='bool-u8,native-result-recovered-from-cpp', policy=POLICY),
        dict(id=first_id + 2, name='MotorsDynamic.load_factors',
             arguments=[dict(type=factor_table, direction='complete-owned-copy')],
             result='void', policy=POLICY),
        dict(id=first_id + 3, name='MotorsMatrix.init',
             arguments=[dict(type='u8', direction='in', range=[0, 'firmware-motor-count'])],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 4, name='MotorsMatrix.add_motor_raw',
             arguments=[dict(type='u8', direction='in', name='motor',
                             range=[0, 'firmware-motor-count-minus-one']),
                        dict(type='f32-le', direction='in', name='roll', float_input='finite'),
                        dict(type='f32-le', direction='in', name='pitch', float_input='finite'),
                        dict(type='f32-le', direction='in', name='yaw', float_input='finite'),
                        dict(type='u8', direction='in', name='testing_order',
                             range=[0, 'firmware-motor-count'])],
             result='void', policy=POLICY),
        dict(id=first_id + 5, name='MotorsMatrix.set_throttle_factor',
             arguments=[dict(type='u8', direction='in', name='motor',
                             range=[0, 'firmware-motor-count-minus-one']),
                        dict(type='f32-le', direction='in', name='factor',
                             range=[0, 'f32-max'], float_input='finite')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 6, name='Motors.set_frame_string',
             arguments=[dict(type=f'utf8-bytes-max{FRAME_STRING_MAX}-length-u8-zero-pad', direction='copied')],
             result='void', policy=POLICY),
        dict(id=first_id + 7, name='Motors6DoF.init',
             arguments=[dict(type='u8', direction='in', range=[0, 'firmware-motor-count'])],
             result='bool-u8', policy=SIX_DOF_POLICY),
        dict(id=first_id + 8, name='Motors6DoF.add_motor',
             arguments=[dict(type='u8', direction='in', name='motor',
                             range=[0, 'firmware-motor-count-minus-one']),
                        dict(type='f32-le', direction='in', name='roll', float_input='finite'),
                        dict(type='f32-le', direction='in', name='pitch', float_input='finite'),
                        dict(type='f32-le', direction='in', name='yaw', float_input='finite'),
                        dict(type='f32-le', direction='in', name='throttle', float_input='finite'),
                        dict(type='f32-le', direction='in', name='forward', float_input='finite'),
                        dict(type='f32-le', direction='in', name='right', float_input='finite'),
                        dict(type='bool-u8', direction='in', name='reversible', canonical=[0, 1]),
                        dict(type='u8', direction='in', name='testing_order',
                             range=[0, 'firmware-motor-count'])],
             result='void', policy=SIX_DOF_POLICY),
    ]
    return dict(methods=methods, factor_count=FACTOR_COUNT, frame_string_max=FRAME_STRING_MAX)


def rust(spec, notice):
    (dynamic_init, dynamic_add, dynamic_load, matrix_init, matrix_add, matrix_throttle, frame,
     six_dof_init, six_dof_add) = spec['methods']
    return notice + f'''// Shared motor configuration is mutated call by call and is not rolled back on module exit.

use crate::{{result, Error, Result}};

pub const MOTOR_FACTOR_COUNT: usize = {FACTOR_COUNT};
pub const MOTOR_FRAME_STRING_MAX: usize = {FRAME_STRING_MAX};

/// Complete copied factors for the maximum supported firmware motor count.
/// Entries above a target firmware's motor count must remain zero.
#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct MotorFactorTable {{
    pub roll: [f32; MOTOR_FACTOR_COUNT],
    pub pitch: [f32; MOTOR_FACTOR_COUNT],
    pub yaw: [f32; MOTOR_FACTOR_COUNT],
    pub throttle: [f32; MOTOR_FACTOR_COUNT],
}}

fn decode_bool(output: [u8; 1]) -> Result<bool> {{
    match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
}}

fn finite(value: f32) -> Result<u32> {{
    if value.is_finite() {{ Ok(value.to_bits()) }} else {{ Err(Error::Invalid) }}
}}

impl crate::bindings::MotorsDynamic<'_, '_> {{
    /// Mutates the shared dynamic mixer. Changes persist after this module exits.
    pub fn init(&self, expected_motors: u8) -> Result<bool> {{
        let input = [expected_motors];
        let mut output = [0u8; 1];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(
                self.context.raw, crate::imports::BINDING_{dynamic_init['id']}.slot(), input.as_ptr(), 1,
                output.as_mut_ptr(), 1)
        }})?;
        decode_bool(output)
    }}

    /// Adds to the shared dynamic mixer and returns the native acceptance result.
    pub fn add_motor(&self, motor: u8, testing_order: u8) -> Result<bool> {{
        let input = [motor, testing_order];
        let mut output = [0u8; 1];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(
                self.context.raw, crate::imports::BINDING_{dynamic_add['id']}.slot(), input.as_ptr(), 2,
                output.as_mut_ptr(), 1)
        }})?;
        decode_bool(output)
    }}

    /// Atomically copies all factors into the shared dynamic mixer. This is not rolled back.
    pub fn load_factors(&self, factors: &MotorFactorTable) -> Result {{
        let mut input = [0u8; 4 * MOTOR_FACTOR_COUNT * core::mem::size_of::<f32>()];
        let mut offset = 0;
        for field in [&factors.roll, &factors.pitch, &factors.yaw, &factors.throttle] {{
            for value in field {{
                input[offset..offset + 4].copy_from_slice(&finite(*value)?.to_le_bytes());
                offset += 4;
            }}
        }}
        result(unsafe {{
            self.context.api.binding_call.unwrap()(
                self.context.raw, crate::imports::BINDING_{dynamic_load['id']}.slot(),
                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)
        }})
    }}

}}

impl crate::bindings::MotorsMatrix<'_, '_> {{
    /// Initializes the shared static scriptable matrix. This is not rolled back.
    pub fn init(&self, expected_motors: u8) -> Result<bool> {{
        let input = [expected_motors];
        let mut output = [0u8; 1];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(
                self.context.raw, crate::imports::BINDING_{matrix_init['id']}.slot(), input.as_ptr(), 1,
                output.as_mut_ptr(), 1)
        }})?;
        decode_bool(output)
    }}

    pub fn add_motor_raw(&self, motor: u8, roll: f32, pitch: f32, yaw: f32,
                         testing_order: u8) -> Result {{
        let mut input = [0u8; 14];
        input[0] = motor;
        input[1..5].copy_from_slice(&finite(roll)?.to_le_bytes());
        input[5..9].copy_from_slice(&finite(pitch)?.to_le_bytes());
        input[9..13].copy_from_slice(&finite(yaw)?.to_le_bytes());
        input[13] = testing_order;
        result(unsafe {{
            self.context.api.binding_call.unwrap()(
                self.context.raw, crate::imports::BINDING_{matrix_add['id']}.slot(),
                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)
        }})
    }}

    pub fn set_throttle_factor(&self, motor: u8, factor: f32) -> Result<bool> {{
        if factor < 0.0 {{ return Err(Error::Invalid); }}
        let mut input = [0u8; 5];
        input[0] = motor;
        input[1..].copy_from_slice(&finite(factor)?.to_le_bytes());
        let mut output = [0u8; 1];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(
                self.context.raw, crate::imports::BINDING_{matrix_throttle['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), 1)
        }})?;
        decode_bool(output)
    }}

}}

impl crate::bindings::Motors6dof<'_, '_> {{
    /// Initializes the Copter-only shared 6DoF mixer. This is not rolled back.
    pub fn init(&self, expected_motors: u8) -> Result<bool> {{
        let input = [expected_motors];
        let mut output = [0u8; 1];
        result(unsafe {{
            self.context.api.binding_call.unwrap()(
                self.context.raw, crate::imports::BINDING_{six_dof_init['id']}.slot(), input.as_ptr(), 1,
                output.as_mut_ptr(), 1)
        }})?;
        decode_bool(output)
    }}

    /// Adds one complete set of finite factors to the Copter-only shared 6DoF mixer.
    /// The configuration persists after this module exits.
    pub fn add_motor(&self, motor: u8, roll: f32, pitch: f32, yaw: f32,
                     throttle: f32, forward: f32, right: f32,
                     reversible: bool, testing_order: u8) -> Result {{
        let mut input = [0u8; 27];
        input[0] = motor;
        for (index, value) in [roll, pitch, yaw, throttle, forward, right].iter().enumerate() {{
            let offset = 1 + index * 4;
            input[offset..offset + 4].copy_from_slice(&finite(*value)?.to_le_bytes());
        }}
        input[25] = u8::from(reversible);
        input[26] = testing_order;
        result(unsafe {{
            self.context.api.binding_call.unwrap()(
                self.context.raw, crate::imports::BINDING_{six_dof_add['id']}.slot(),
                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)
        }})
    }}

}}

impl crate::bindings::Motors<'_, '_> {{
    /// Copies a bounded label into firmware. Firmware accepts only its first frame label,
    /// and that allocation/configuration persists after this module exits.
    pub fn set_frame_string(&self, value: &str) -> Result {{
        if value.len() > MOTOR_FRAME_STRING_MAX || value.as_bytes().contains(&0) {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; MOTOR_FRAME_STRING_MAX + 1];
        input[0] = value.len() as u8;
        input[1..1 + value.len()].copy_from_slice(value.as_bytes());
        result(unsafe {{
            self.context.api.binding_call.unwrap()(
                self.context.raw, crate::imports::BINDING_{frame['id']}.slot(),
                input.as_ptr(), input.len() as u32, core::ptr::null_mut(), 0)
        }})
    }}
}}
'''
