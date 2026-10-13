# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Public binding contracts, independent of firmware dispatch numbering."""

import hashlib
import json
import math
import re
from pathlib import Path

import borrowed_binding
import libm_binding
import value_binding

DOMAIN = b'AP_Rust.binding.v1\n'
REVISION_PATH = Path(__file__).with_name('contract_revisions.json')


def read_revisions():
    revisions = json.loads(REVISION_PATH.read_text())
    if not isinstance(revisions, dict):
        raise ValueError('binding revisions must be an object')
    return revisions


def canonical(contract):
    return DOMAIN + json.dumps(contract, sort_keys=True, separators=(',', ':'), ensure_ascii=True).encode('ascii')


def digest(contract):
    # Interpret the first eight digest bytes as a little-endian integer. The
    # same u64 is rendered as 16 hex digits in ELF symbol names and manifests.
    return int.from_bytes(hashlib.sha256(canonical(contract)).digest()[:8], 'little')


def range_bound(bound):
    if re.fullmatch(r'[+-]?\d+', bound):
        return int(bound)
    if re.fullmatch(r'[+-]?(?:\d+(?:\.\d*)?|\.\d+)(?:[eE][+-]?\d+)?', bound):
        value = float(bound)
        if not math.isfinite(value):
            raise ValueError('non-finite numeric range bound')
        return int(value) if value.is_integer() else value
    return 'firmware-bound'


def describe(spec):
    revisions = read_revisions()
    values = {owner['name']: dict(name=owner['name'], fields=[[field, 'f32-le'] for field in owner['fields']])
              for owner in spec['math']['types']}
    scalars = dict(boolean='bool-u8', uint8_t='u8', uint16_t='u16-le', uint32_t='u32-le', float='f32-le',
                   int8_t='i8', int16_t='i16-le', int32_t='i32-le',
                   string='nonempty-nul-free-bytes-max16-zero-pad17',
                   cstring='nul-free-bytes-max255-zero-pad256,borrowed-call-duration', void='void')

    def value(kind):
        if kind in value_binding.TYPES:
            return value_binding.wire_type(kind)
        if kind in borrowed_binding.TYPES:
            return borrowed_binding.wire_type(kind)
        return values[kind] if kind in values else scalars[kind]

    entries = []

    def add(method, public, arguments, result, policy):
        revision = revisions.get(public, 1)
        if type(revision) is not int or not 0 < revision <= 0xffffffff:
            raise ValueError('invalid binding contract revision')
        contract = dict(name=public, revision=revision, encoding=spec['encoding'],
                        arguments=arguments, result=result, policy=policy)
        entries.append(dict(id=method['id'], hash=f'{digest(contract):016x}', contract=contract))

    def field_type(field):
        return dict(enum=field['enum'], wire='i32-le') if field['type'] == 'enum' else value(field['type'])

    for method in spec['methods']:
        arguments = []
        if method.get('receiver'):
            arguments.append(dict(type=value(method['receiver']), direction='borrowed-receiver,nonzero'))
        if method.get('value_receiver'):
            arguments.append(dict(type=value(method['value_receiver']), direction='inout'))
        for arg in method['arguments']:
            if arg['type'] == 'literal':
                arguments.append(dict(type='literal', direction='host-constant', expression=arg['expression']))
                continue
            item = dict(type=field_type(arg), direction='optional-out' if arg['output'] else 'in')
            if arg.get('reference'):
                item['direction'] = 'out'
            if arg.get('inout'):
                item['direction'] = 'inout'
            if arg.get('allow_nonfinite'):
                item['float_input'] = 'all-f32-bits'
            if 'range' in arg:
                # Numeric bounds are public. Dynamic firmware bounds remain
                # runtime checks; their private C++ expression is not an ABI.
                item['range'] = [range_bound(bound) for bound in arg['range']]
            arguments.append(item)
        policy = dict(encoding=spec['encoding'], lock_timeout_ms=spec['lock_timeout_ms'] if method['lock'] else None,
                      adapter=method.get('adapter'), outputs='copy-under-lock,zero-on-false')
        outputs = [arg for arg in method['arguments'] if arg['output']]
        if any(arg.get('reference') for arg in outputs):
            policy['outputs'] = 'copy-under-lock,always-present'
        elif len(outputs) > 1:
            policy['outputs'] = 'copy-under-lock,shared-presence-u8,zero-on-false'
        if method.get('value_receiver') or any(a.get('inout') for a in method['arguments']):
            policy['value_inputs'] = 'complete-copy,copyback-on-host-success-including-false,no-revalidation'
        add(method, method['api_name'], arguments, field_type(method['result']), policy)
        for alias in method['api_aliases']:
            add(method, alias, arguments, field_type(method['result']), policy)
    for method in spec['parameter']['methods']:
        arguments = [dict(type='Parameter', direction='opaque-receiver-le-u64')]
        arguments += [dict(type=value(arg['kind']), direction='optional-out' if arg['nullable'] else 'in')
                      for arg in method['arguments']]
        add(method, 'Parameter.' + method['name'], arguments, value('boolean'), spec['parameter']['policy'])
    layout = spec['parameter']['layout']
    add(layout, 'Parameter.__layout', [], dict(fields=[['size', 'u32-le'], ['alignment', 'u32-le']]), layout['policy'])
    for owner in spec['math']['types']:
        for method in owner['methods']:
            arguments = [dict(type=value(owner['name']), direction=method['receiver'])]
            arguments += [dict(type=value(arg['type']), direction=arg['direction']) for arg in method['arguments']]
            add(method, owner['name'] + '.' + method['name'], arguments, value(method['result']), spec['math']['policy'])
    for method in spec['libm']:
        arguments = [dict(type=value('float'), direction='in', float_input='all-f32-bits') for _ in range(method['arity'])]
        add(method, 'libm.' + method['name'], arguments, value('float'), libm_binding.POLICY)
    method = spec['file']
    add(method, method['name'], method['arguments'], method['result'], method['policy'])
    method = spec['log']
    add(method, method['name'], method['arguments'], method['result'], method['policy'])
    method = spec['strtof']
    add(method, method['name'], method['arguments'], method['result'], method['policy'])
    hashes = set()
    names = set()
    for entry in entries:
        if entry['hash'] in hashes or entry['contract']['name'] in names:
            raise ValueError('duplicate binding name or 64-bit contract hash collision')
        hashes.add(entry['hash'])
        names.add(entry['contract']['name'])
    if not set(revisions).issubset(names):
        raise ValueError('revision names an unknown public binding')
    return entries


def check_compatibility(previous, current):
    """Compare a released catalogue without reserving globally assigned IDs."""
    available = {entry['hash']: entry['contract'] for entry in current}
    for entry in previous:
        if entry['hash'] not in available:
            raise ValueError(f'incompatible binding: {entry["contract"]["name"]} ({entry["hash"]})')
        if available[entry['hash']] != entry['contract']:
            raise ValueError(f'64-bit contract hash collision with prior catalogue: {entry["hash"]}')


def emit(spec, notice):
    entries = describe(spec)
    rust = [notice, 'use crate::Import;', '']
    ids = set()
    for entry in entries:
        if entry['id'] in ids:
            continue  # source aliases call the same canonical SDK wrapper
        ids.add(entry['id'])
        rust += [f'#[link_section = ".ap_rust_import_{entry["hash"]}"]',
                 f'pub(crate) static BINDING_{entry["id"]}: Import = Import::new();', '']
    cpp = [notice, '#include "Bindings.h"', '#if AP_SCRIPTING_RUST_ENABLED', '', 'namespace AP_Rust', '{',
           'uint32_t binding_resolve(void *, uint64_t hash)', '{',
           '    static const struct {', '        uint64_t hash;', '        uint32_t id;', '    } bindings[] = {']
    for entry in sorted(entries, key=lambda entry: entry['hash']):
        cpp.append(f'        {{UINT64_C(0x{entry["hash"]}), {entry["id"]}U}},')
    cpp += ['    };', '    uint32_t low = 0;', '    uint32_t high = sizeof(bindings) / sizeof(bindings[0]);',
            '    while (low < high) {', '        const uint32_t middle = low + (high - low) / 2;',
            '        if (bindings[middle].hash < hash) {', '            low = middle + 1;', '        } else {',
            '            high = middle;', '        }', '    }',
            '    return low < sizeof(bindings) / sizeof(bindings[0]) && bindings[low].hash == hash ?',
            '           bindings[low].id : UINT32_MAX;', '}', '}', '#endif', '']
    return {'sdk/src/imports.rs': '\n'.join(rust), 'BindingImports_generated.cpp': '\n'.join(cpp),
            'generator/imports.json': json.dumps(entries, indent=2) + '\n'}
