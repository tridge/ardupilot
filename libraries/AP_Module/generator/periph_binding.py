# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""AP_Periph vehicle-notification state with an owned u64 wire value."""

OWNER = 'AP_Periph_FW'
POLICY = 'v1:read-singleton-state,u64-little-endian,no-retained-firmware-pointers'


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected = dict(rename='periph', depends='defined(HAL_BUILD_AP_PERIPH)', get=None,
                    literal=False, reference=False, semaphore=False,
                    scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited AP_Periph_FW metadata')
    method = next((item for item in owner['methods'] if item['name'] == 'get_vehicle_state'), None)
    if (method is None or method['return']['kind'] != 'userdata' or method['return']['name'] != 'uint64_t' or
            method['arguments'] or method['depends'] != 'AP_PERIPH_NOTIFY_ENABLED'):
        raise ValueError('unaudited AP_Periph_FW.get_vehicle_state')
    return specification(first_id)


def specification(first_id):
    return dict(methods=[dict(id=first_id, name='periph.get_vehicle_state', arguments=[],
                              result='u64-le', policy=POLICY)])


def rust(spec, notice):
    method = spec['methods'][0]
    return notice + f'''use crate::{{result, Result}};

impl crate::bindings::Periph<'_, '_> {{
    pub fn get_vehicle_state(&self) -> Result<u64> {{
        let mut output = [0u8; 8];
        result(unsafe {{ self.context.api.binding_call.unwrap()(
            self.context.raw, crate::imports::BINDING_{method['id']}.slot(), core::ptr::null(), 0,
            output.as_mut_ptr(), output.len() as u32)
        }})?;
        Ok(u64::from_le_bytes(output))
    }}
}}
'''
