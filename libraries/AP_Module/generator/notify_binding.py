# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Snapshot of named AP_Notify flags as a fixed u32 bitmask."""

import rust_calls

from call_model import U32
from call_model import Call
from call_model import Module

OWNER = 'AP_Notify'
# Bit n is FLAGS[n]. Append only: the order is the wire contract.
FLAGS = ('armed', 'flying', 'have_pos_abs', 'gps_glitching', 'ekf_bad', 'failsafe_battery',
         'failsafe_gcs', 'failsafe_radio', 'compass_cal_running', 'initialising',
         'pre_arm_check', 'pre_arm_gps_check')
POLICY = 'v1:flags-' + '-'.join(FLAGS) + ',bit-per-flag-lsb-first,unlocked-snapshot'


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    expected = dict(rename='notify', get=None, literal=False, reference=False,
                    semaphore=False, scheduler_semaphore=False, semaphore_pointer=False)
    if owner is None or any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited AP_Notify metadata')
    return specification(first_id, owner['depends'])


def specification(first_id, depends=None):
    return dict(depends=depends, methods=[dict(id=first_id, name='notify.get_flags', arguments=[],
                                               result='u32-le', policy=POLICY)])


def client_model(spec):
    method = spec['methods'][0]
    return Module((Call(method['name'], method['id'], result=U32, docs=(
        'Named AP_Notify flags packed as a bitmask; see NotifyFlags for the bits.',
        'Flags are read without a lock, so separate flags may come from adjacent updates.',
    )),))


def cpp(spec):
    method = spec['methods'][0]
    lines = [f'// {method["name"]}',
             'static __attribute__((noinline)) int32_t binding_notify_get_flags(',
             '    const uint8_t *, uint32_t, uint8_t *output, uint32_t)', '{']
    if spec['depends']:
        lines.append(f'#if {spec["depends"]}')
    lines += ['    uint32_t value = 0;']
    lines += [f'    value |= uint32_t(AP_Notify::flags.{name}) << {bit};' for bit, name in enumerate(FLAGS)]
    lines += ['    BindingWire::write(output, value, 4);', '    return AP_MODULE_STATUS_OK;']
    if spec['depends']:
        lines += ['#else', '    memset(output, 0, 4);', '    return AP_MODULE_STATUS_UNAVAILABLE;', '#endif']
    return lines + ['}', '']


def rust(spec, notice):
    constants = ''.join(f'    pub const {name.upper()}: u32 = 1 << {bit};\n' for bit, name in enumerate(FLAGS))
    return (rust_calls.emit(client_model(spec), notice) +
            '\n/// Bits of `notify().get_flags()`.\npub struct NotifyFlags;\n\nimpl NotifyFlags {\n' +
            constants + '}\n')
