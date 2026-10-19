# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Owned rangefinder state and revalidated rangefinder/proximity backends."""

RANGEFINDER_OWNER = 'RangeFinder'
RANGEFINDER_BACKEND = 'AP_RangeFinder_Backend'
RANGEFINDER_STATE = 'RangeFinder::RangeFinder_State'
PROXIMITY_OWNER = 'AP_Proximity'
PROXIMITY_BACKEND = 'AP_Proximity_Backend'
RANGEFINDER_DEPENDS = 'AP_RANGEFINDER_ENABLED'
PROXIMITY_DEPENDS = 'HAL_PROXIMITY_ENABLED == 1'
STATE_WIRE_SIZE = 13
POLICY = ('v1:non-owning-backend-index,revalidate-every-call,no-retained-firmware-pointers,'
          'complete-owned-rangefinder-state,enum-and-bool-validation,all-f32-bits,'
          'native-backend-return,no-lifecycle-rollback')


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


def _shape(result, arguments=(), rename=None):
    return (result, tuple(arguments), rename, None, None)


def select(ir, first_id):
    """Fail closed unless the audited Lua owners and method shapes are unchanged."""
    rangefinder = next((item for item in ir['singletons'] if item['name'] == RANGEFINDER_OWNER), None)
    expected_rangefinder_metadata = ('rangefinder', RANGEFINDER_DEPENDS, None, False, False, False, False, False)
    rangefinder_methods = {
        'distance_cm_orient', 'distance_orient', 'get_backend', 'get_pos_offset_orient',
        'ground_clearance_cm_orient', 'ground_clearance_orient', 'has_data_orient', 'has_orientation',
        'max_distance_cm_orient', 'max_distance_orient', 'min_distance_cm_orient', 'min_distance_orient',
        'num_sensors', 'signal_quality_pct_orient', 'status_orient',
    }
    if (rangefinder is None or _owner_metadata(rangefinder) != expected_rangefinder_metadata or
            rangefinder['aliases'] or rangefinder['fields'] or
            {method['name'] for method in rangefinder['methods']} != rangefinder_methods):
        raise ValueError('unaudited RangeFinder owner metadata')
    expected_get_backend = _shape(
        ('ap_object', RANGEFINDER_BACKEND, False, False, False, False, None),
        (('uint8_t', None, False, False, False, True, None),))
    actual_get_backend = next(method for method in rangefinder['methods'] if method['name'] == 'get_backend')
    if _method_shape(actual_get_backend) != expected_get_backend:
        raise ValueError('unaudited RangeFinder get_backend')

    object_metadata = (None, RANGEFINDER_DEPENDS, None, False, False, False, False, False,
                       None, 0, False, False, False, 0, None, False, ())
    rangefinder_backend = next(
        (item for item in ir['ap_objects'] if item['name'] == RANGEFINDER_BACKEND), None)
    expected_backend_names = {'distance', 'get_state', 'orientation', 'signal_quality_pct', 'status', 'type'}
    if (rangefinder_backend is None or _owner_metadata(rangefinder_backend, fields=True) != object_metadata or
            rangefinder_backend['fields'] or
            {method['name'] for method in rangefinder_backend['methods']} != expected_backend_names):
        raise ValueError('unaudited RangeFinder backend metadata')
    expected_backend_methods = {
        'distance': _shape(('float', None, False, False, False, False, None)),
        'get_state': _shape(
            ('void', None, False, False, False, False, None),
            (('userdata', RANGEFINDER_STATE, False, True, False, False, None),)),
        'signal_quality_pct': _shape(
            ('float', None, False, False, False, False, None), rename='signal_quality'),
        'type': _shape(('uint8_t', None, False, False, False, False, None)),
        'status': _shape(('uint8_t', None, False, False, False, False, None)),
        'orientation': _shape(('enum', 'Rotation', False, False, False, False, None)),
    }
    actual_backend_methods = {
        method['name']: _method_shape(method) for method in rangefinder_backend['methods']
        if method['name'] in expected_backend_methods
    }
    if actual_backend_methods != expected_backend_methods:
        raise ValueError('unaudited RangeFinder backend methods')
    expected_aliases = [dict(name='lua_range_finder_handle_script_msg', alias='handle_script_msg',
                             depends=None, manual=True)]
    if rangefinder_backend['aliases'] != expected_aliases:
        raise ValueError('unaudited RangeFinder handle_script_msg')

    state_metadata = ('RangeFinder_State', RANGEFINDER_DEPENDS, None, False, False, False, False, False,
                      None, 0, False, True, False, 0, None, False, ())
    state = next((item for item in ir['userdata'] if item['name'] == RANGEFINDER_STATE), None)
    expected_state_fields = {
        'distance_m': (('float', None, False, False, False, True, None),
                       True, True, 'distance', None, None, None, None),
        'signal_quality_pct': (('int8_t', None, False, False, False, True, None),
                               True, True, 'signal_quality', None, None, None, None),
        'voltage_mv': (('uint16_t', None, False, False, False, True, None),
                       True, True, 'voltage', None, None, None, None),
        'status': (('enum', 'RangeFinder::Status', False, False, False, False,
                    ('RangeFinder::Status::NotConnected', 'RangeFinder::Status::PoweredDown')),
                   True, True, None, None, None, None, None),
        'range_valid_count': (('uint8_t', None, False, False, False, True, None),
                              True, True, None, None, None, None, None),
        'last_reading_ms': (('uint32_t', None, False, False, False, True, None),
                            True, True, 'last_reading', None, None, None, None),
    }
    if (state is None or _owner_metadata(state, fields=True) != state_metadata or state['aliases'] or
            state['methods'] or _userdata_fields(state) != expected_state_fields):
        raise ValueError('unaudited RangeFinder state')

    proximity = next((item for item in ir['singletons'] if item['name'] == PROXIMITY_OWNER), None)
    expected_proximity_metadata = ('proximity', PROXIMITY_DEPENDS, None, False, False, False, False, False)
    proximity_methods = {
        'get_backend', 'get_closest_object', 'get_object_angle_and_distance', 'get_object_count',
        'get_status', 'num_sensors',
    }
    if (proximity is None or _owner_metadata(proximity) != expected_proximity_metadata or
            proximity['aliases'] or proximity['fields'] or
            {method['name'] for method in proximity['methods']} != proximity_methods):
        raise ValueError('unaudited Proximity owner metadata')
    expected_get_backend = _shape(
        ('ap_object', PROXIMITY_BACKEND, False, False, False, False, None),
        (('uint8_t', None, False, False, False, True, None),))
    actual_get_backend = next(method for method in proximity['methods'] if method['name'] == 'get_backend')
    if _method_shape(actual_get_backend) != expected_get_backend:
        raise ValueError('unaudited Proximity get_backend')

    proximity_object_metadata = (None, PROXIMITY_DEPENDS, None, False, False, False, False, False,
                                 None, 0, False, False, False, 0, None, False, ())
    proximity_backend = next((item for item in ir['ap_objects'] if item['name'] == PROXIMITY_BACKEND), None)
    if (proximity_backend is None or
            _owner_metadata(proximity_backend, fields=True) != proximity_object_metadata or
            proximity_backend['aliases'] or proximity_backend['fields']):
        raise ValueError('unaudited Proximity backend metadata')
    boolean = ('boolean', None, False, False, False, False, None)
    float_skip = ('float', None, False, False, False, True, None)
    expected_proximity_methods = {
        'handle_script_distance_msg': _shape(boolean, (float_skip, float_skip, float_skip, boolean)),
        'handle_script_3d_msg': _shape(
            boolean, (('userdata', 'Vector3f', False, False, False, False, None), boolean)),
        'type': _shape(('uint8_t', None, False, False, False, False, None)),
        'set_distance_min_max': _shape(boolean, (float_skip, float_skip)),
        'update_virtual_boundary': _shape(boolean),
    }
    actual_proximity_methods = {
        method['name']: _method_shape(method) for method in proximity_backend['methods']
    }
    if actual_proximity_methods != expected_proximity_methods:
        raise ValueError('unaudited Proximity backend methods')
    return specification(first_id)


def specification(first_id):
    rangefinder_handle = dict(type='u8,revalidated-rangefinder-backend-index',
                              direction='borrowed-receiver')
    proximity_handle = dict(type='u8,revalidated-proximity-backend-index',
                            direction='borrowed-receiver')
    optional_rangefinder = dict(presence='bool-u8', handle='RangeFinderBackend,index-revalidated')
    optional_proximity = dict(presence='bool-u8', handle='ProximityBackend,index-revalidated')
    state = dict(name='RangeFinderState', fields=[
        ['distance_m', 'f32-le,all-bits'], ['signal_quality_pct', 'i8'], ['voltage_mv', 'u16-le'],
        ['status', 'RangeFinderStatus-u8,range0-5'], ['range_valid_count', 'u8'],
        ['last_reading_ms', 'u32-le'],
    ])
    methods = [
        dict(id=first_id, name='RangeFinder.get_backend',
             arguments=[dict(type='u8', direction='in', name='index')],
             result=optional_rangefinder, policy=POLICY),
        dict(id=first_id + 1, name='RangeFinderBackend.type', arguments=[rangefinder_handle],
             result='u8', policy=POLICY),
        dict(id=first_id + 2, name='RangeFinderBackend.status', arguments=[rangefinder_handle],
             result='RangeFinderStatus-u8,range0-5', policy=POLICY),
        dict(id=first_id + 3, name='RangeFinderBackend.handle_script_distance',
             arguments=[rangefinder_handle, dict(type='f32-le', direction='in', float_input='all-f32-bits')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 4, name='RangeFinderBackend.handle_script_state',
             arguments=[rangefinder_handle, dict(type=state, direction='in')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 5, name='Proximity.get_backend',
             arguments=[dict(type='u8', direction='in', name='index')],
             result=optional_proximity, policy=POLICY),
        dict(id=first_id + 6, name='ProximityBackend.type', arguments=[proximity_handle],
             result='u8', policy=POLICY),
        dict(id=first_id + 7, name='ProximityBackend.handle_script_distance_msg',
             arguments=[proximity_handle,
                        dict(type='f32-le', direction='in', name='distance_m', float_input='all-f32-bits'),
                        dict(type='f32-le', direction='in', name='yaw_deg', float_input='all-f32-bits'),
                        dict(type='f32-le', direction='in', name='pitch_deg', float_input='all-f32-bits'),
                        dict(type='bool-u8', direction='in', name='push_to_boundary')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 8, name='ProximityBackend.set_distance_min_max',
             arguments=[proximity_handle,
                        dict(type='f32-le', direction='in', name='minimum', float_input='all-f32-bits'),
                        dict(type='f32-le', direction='in', name='maximum', float_input='all-f32-bits')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 9, name='ProximityBackend.update_virtual_boundary',
             arguments=[proximity_handle], result='bool-u8', policy=POLICY),
        dict(id=first_id + 10, name='RangeFinderBackend.get_state',
             arguments=[rangefinder_handle], result=state, policy=POLICY),
        dict(id=first_id + 11, name='RangeFinderBackend.orientation',
             arguments=[rangefinder_handle], result='Rotation-i32-le', policy=POLICY),
        dict(id=first_id + 12, name='ProximityBackend.handle_script_3d_msg',
             arguments=[proximity_handle, dict(type='Vector3f,three-f32-le', name='vector'),
                        dict(type='bool-u8', direction='in', name='push_to_boundary')],
             result='bool-u8', policy=POLICY),
    ]
    return dict(methods=methods, state_wire_size=STATE_WIRE_SIZE)


def rust(spec, notice):
    (rangefinder_get, rangefinder_type, rangefinder_status, rangefinder_distance, rangefinder_state,
     proximity_get, proximity_type, proximity_distance, proximity_min_max, proximity_boundary,
     rangefinder_get_state, rangefinder_orientation, proximity_3d) = spec['methods']
    return notice + f'''use crate::{{result, Context, Error, Result, Vector3f}};
use crate::enums::Rotation;

#[derive(Clone, Copy, Debug, Default, Eq, PartialEq)]
#[repr(u8)]
pub enum RangeFinderStatus {{
    #[default]
    NotConnected = 0,
    NoData = 1,
    OutOfRangeLow = 2,
    OutOfRangeHigh = 3,
    Good = 4,
    PoweredDown = 5,
}}

#[derive(Clone, Copy, Debug, Default, PartialEq)]
pub struct RangeFinderState {{
    pub distance_m: f32,
    pub signal_quality_pct: i8,
    pub voltage_mv: u16,
    pub status: RangeFinderStatus,
    pub range_valid_count: u8,
    pub last_reading_ms: u32,
}}

/// A non-owning backend index, resolved through the RangeFinder singleton on every call.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct RangeFinderBackend {{ index: u8 }}

/// A non-owning backend index, resolved through the Proximity singleton on every call.
#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct ProximityBackend {{ index: u8 }}

fn decode_bool(output: [u8; 1]) -> Result<bool> {{
    match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
}}

fn call_u8(context: &Context<'_>, id: u32, index: u8) -> Result<u8> {{
    let input = [index];
    let mut output = [0u8; 1];
    result(unsafe {{
        context.api.binding_call.unwrap()(context.raw, id, input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), output.len() as u32)
    }})?;
    Ok(output[0])
}}

fn decode_rangefinder_status(value: u8) -> Result<RangeFinderStatus> {{
    match value {{
        0 => Ok(RangeFinderStatus::NotConnected), 1 => Ok(RangeFinderStatus::NoData),
        2 => Ok(RangeFinderStatus::OutOfRangeLow), 3 => Ok(RangeFinderStatus::OutOfRangeHigh),
        4 => Ok(RangeFinderStatus::Good), 5 => Ok(RangeFinderStatus::PoweredDown),
        _ => Err(Error::Host),
    }}
}}

impl crate::bindings::Rangefinder<'_, '_> {{
    pub fn get_backend(&self, index: u8) -> Result<Option<RangeFinderBackend>> {{
        match call_u8(self.context, crate::imports::BINDING_{rangefinder_get['id']}.slot(), index)? {{
            0 => Ok(None), 1 => Ok(Some(RangeFinderBackend {{ index }})), _ => Err(Error::Host),
        }}
    }}
}}

impl crate::bindings::Proximity<'_, '_> {{
    pub fn get_backend(&self, index: u8) -> Result<Option<ProximityBackend>> {{
        match call_u8(self.context, crate::imports::BINDING_{proximity_get['id']}.slot(), index)? {{
            0 => Ok(None), 1 => Ok(Some(ProximityBackend {{ index }})), _ => Err(Error::Host),
        }}
    }}
}}

impl RangeFinderBackend {{
    pub fn sensor_type(&self, context: &Context<'_>) -> Result<u8> {{
        call_u8(context, crate::imports::BINDING_{rangefinder_type['id']}.slot(), self.index)
    }}

    pub fn status(&self, context: &Context<'_>) -> Result<RangeFinderStatus> {{
        decode_rangefinder_status(call_u8(
            context, crate::imports::BINDING_{rangefinder_status['id']}.slot(), self.index)?)
    }}

    pub fn get_state(&self, context: &Context<'_>) -> Result<RangeFinderState> {{
        let input = [self.index];
        let mut output = [0u8; {STATE_WIRE_SIZE}];
        result(unsafe {{
            context.api.binding_call.unwrap()(
                context.raw, crate::imports::BINDING_{rangefinder_get_state['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        Ok(RangeFinderState {{
            distance_m: f32::from_bits(u32::from_le_bytes([
                output[0], output[1], output[2], output[3],
            ])),
            signal_quality_pct: output[4] as i8,
            voltage_mv: u16::from_le_bytes([output[5], output[6]]),
            status: decode_rangefinder_status(output[7])?,
            range_valid_count: output[8],
            last_reading_ms: u32::from_le_bytes([
                output[9], output[10], output[11], output[12],
            ]),
        }})
    }}

    pub fn distance(&self, context: &Context<'_>) -> Result<f32> {{
        Ok(self.get_state(context)?.distance_m)
    }}

    pub fn signal_quality(&self, context: &Context<'_>) -> Result<f32> {{
        Ok(self.get_state(context)?.signal_quality_pct as f32)
    }}

    pub fn orientation(&self, context: &Context<'_>) -> Result<Rotation> {{
        let input = [self.index];
        let mut output = [0u8; 4];
        result(unsafe {{
            context.api.binding_call.unwrap()(
                context.raw, crate::imports::BINDING_{rangefinder_orientation['id']}.slot(),
                input.as_ptr(), 1, output.as_mut_ptr(), 4)
        }})?;
        Ok(Rotation(i32::from_le_bytes(output)))
    }}

    pub fn handle_script_distance(&self, context: &Context<'_>, distance_m: f32) -> Result<bool> {{
        let mut input = [0u8; 5];
        input[0] = self.index;
        input[1..].copy_from_slice(&distance_m.to_bits().to_le_bytes());
        let mut output = [0u8; 1];
        result(unsafe {{
            context.api.binding_call.unwrap()(
                context.raw, crate::imports::BINDING_{rangefinder_distance['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        decode_bool(output)
    }}

    pub fn handle_script_state(&self, context: &Context<'_>, state: &RangeFinderState) -> Result<bool> {{
        let mut input = [0u8; {STATE_WIRE_SIZE + 1}];
        input[0] = self.index;
        input[1..5].copy_from_slice(&state.distance_m.to_bits().to_le_bytes());
        input[5] = state.signal_quality_pct as u8;
        input[6..8].copy_from_slice(&state.voltage_mv.to_le_bytes());
        input[8] = state.status as u8;
        input[9] = state.range_valid_count;
        input[10..14].copy_from_slice(&state.last_reading_ms.to_le_bytes());
        let mut output = [0u8; 1];
        result(unsafe {{
            context.api.binding_call.unwrap()(
                context.raw, crate::imports::BINDING_{rangefinder_state['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        decode_bool(output)
    }}
}}

impl ProximityBackend {{
    pub fn sensor_type(&self, context: &Context<'_>) -> Result<u8> {{
        call_u8(context, crate::imports::BINDING_{proximity_type['id']}.slot(), self.index)
    }}

    pub fn handle_script_distance_msg(&self, context: &Context<'_>, distance_m: f32,
                                      yaw_deg: f32, pitch_deg: f32,
                                      push_to_boundary: bool) -> Result<bool> {{
        let mut input = [0u8; 14];
        input[0] = self.index;
        input[1..5].copy_from_slice(&distance_m.to_bits().to_le_bytes());
        input[5..9].copy_from_slice(&yaw_deg.to_bits().to_le_bytes());
        input[9..13].copy_from_slice(&pitch_deg.to_bits().to_le_bytes());
        input[13] = push_to_boundary as u8;
        let mut output = [0u8; 1];
        result(unsafe {{
            context.api.binding_call.unwrap()(
                context.raw, crate::imports::BINDING_{proximity_distance['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        decode_bool(output)
    }}

    pub fn handle_script_3d_msg(&self, context: &Context<'_>, vector: &Vector3f,
                                push_to_boundary: bool) -> Result<bool> {{
        let mut input = [0u8; 14];
        input[0] = self.index;
        input[1..13].copy_from_slice(&vector.encode());
        input[13] = push_to_boundary as u8;
        let mut output = [0u8; 1];
        result(unsafe {{
            context.api.binding_call.unwrap()(
                context.raw, crate::imports::BINDING_{proximity_3d['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), 1)
        }})?;
        decode_bool(output)
    }}

    pub fn set_distance_min_max(&self, context: &Context<'_>, minimum: f32,
                                maximum: f32) -> Result<bool> {{
        let mut input = [0u8; 9];
        input[0] = self.index;
        input[1..5].copy_from_slice(&minimum.to_bits().to_le_bytes());
        input[5..9].copy_from_slice(&maximum.to_bits().to_le_bytes());
        let mut output = [0u8; 1];
        result(unsafe {{
            context.api.binding_call.unwrap()(
                context.raw, crate::imports::BINDING_{proximity_min_max['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        decode_bool(output)
    }}

    pub fn update_virtual_boundary(&self, context: &Context<'_>) -> Result<bool> {{
        let input = [self.index];
        let mut output = [0u8; 1];
        result(unsafe {{
            context.api.binding_call.unwrap()(
                context.raw, crate::imports::BINDING_{proximity_boundary['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        decode_bool(output)
    }}
}}
'''
