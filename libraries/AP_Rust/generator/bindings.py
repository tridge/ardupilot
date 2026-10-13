# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Generate supported binding categories from the existing Lua generator's IR."""

import json
import os
import re
import subprocess
import tempfile
from collections import Counter
from pathlib import Path

import borrowed_binding
import file_binding
import libm_binding
import log_binding
import math_binding
import value_binding
import parameter_binding
import singleton_binding
import strtof_binding


ROOT = Path(__file__).resolve().parents[1]
LUA_GENERATOR = ROOT.parent / 'AP_Scripting/generator'
ENCODING = 'AP_Rust.bindings.v1:little-endian,IEEE754-f32,bool-u8,optional-presence-u8,exact-lengths'
LOCK_TIMEOUT_MS = 20
PARAM_ARGUMENTS = {
    'get': [('string', False), ('float', True)],
    'add_table': [('uint8_t', False), ('string', False), ('uint8_t', False)],
    'add_param': [('uint8_t', False), ('uint8_t', False), ('string', False), ('float', False)],
    'set_by_name': [('string', False), ('float', False)],
    'set_and_save_by_name': [('string', False), ('float', False)],
    'set_default_by_name': [('string', False), ('float', False)],
}
PARAM_RANGES = {'add_table': {0: ['0', '200'], 2: ['1', '63']},
                'add_param': {0: ['0', '200'], 1: ['1', '63']}}
PARAM_POLICY = 'v1:nonempty-nul-free-bytes-max16-zero-pad17,finite-inputs,checked-integer-range,persistent-tables'


def read_ir(description=None):
    with tempfile.TemporaryDirectory(prefix='ap-rust-bindings-') as work:
        work = Path(work)
        executable = work / 'generator'
        subprocess.run([os.environ.get('CC', 'cc'), '-std=c99', '-Wno-error=missing-field-initializers',
                        '-Wall', '-Werror', '-Wextra', str(LUA_GENERATOR / 'src/main.c'), '-o', str(executable)], check=True)
        output = work / 'bindings.json'
        subprocess.run([str(executable), '-i', str(description or LUA_GENERATOR / 'description/bindings.desc'),
                        '--dump-ir', str(output)], check=True)
        return json.loads(output.read_text())


def identifier(value):
    if not isinstance(value, str) or not re.fullmatch(r'[A-Za-z_][A-Za-z_0-9]*', value):
        raise ValueError(f'invalid identifier: {value}')
    return value


def policies():
    return json.loads((ROOT / 'generator/binding_policies.json').read_text())


def method_contract(owner, method):
    lock_fields = [('semaphore', 'semaphore'), ('scheduler-semaphore', 'scheduler_semaphore'),
                   ('semaphore-pointer', 'semaphore_pointer')]
    locks = [name for name, key in lock_fields if owner[key]]
    if len(locks) > 1:
        raise ValueError('conflicting-singleton-locks')
    lock = locks[0] if locks else None
    if method['deprecate']:
        raise ValueError('deprecated')
    name = identifier(method['name'])
    parameter = owner['name'] == 'AP_Param'
    if parameter and (name not in PARAM_ARGUMENTS or lock is not None or method['return']['kind'] != 'boolean' or
                      [(a['kind'], a['nullable']) for a in method['arguments']] != PARAM_ARGUMENTS[name] or
                      any(a['range'] != PARAM_RANGES.get(name, {}).get(i) for i, a in enumerate(method['arguments']))):
        raise ValueError('unaudited-parameter-adapter')
    result = singleton_binding.field(method['return'])
    args = [singleton_binding.field(a, argument=True, parameter=parameter) for a in method['arguments']]
    outputs = [a for a in args if a['output']]
    if outputs:
        references = [a.get('reference', False) for a in outputs]
        if any(references) and not all(references):
            raise ValueError('mixed-output-directions')
        if (all(references) and result['type'] != 'void') or (not any(references) and result['type'] != 'boolean'):
            raise ValueError('unsupported-output-result')
    public_owner = identifier(owner['rename'] or owner['name'])
    aliases = [a for a in owner['aliases'] if a['name'] == name and not a['manual']]
    if any(a['depends'] for a in aliases):
        raise ValueError('conditional-alias')
    public = public_owner + '.' + identifier(method['rename'] or name)
    item = dict(owner=owner['name'], method=name, rust_name=public.replace('.', '_').lower(), api_name=public,
                api_aliases=sorted(public_owner + '.' + identifier(a['alias']) for a in aliases),
                aliases=sorted((public_owner + '_' + a['alias']).lower() for a in aliases),
                owner_depends=owner['depends'], depends=method['depends'], lock=lock, result=result, arguments=args)
    for key in ('get', 'literal', 'reference'):
        if owner[key]:
            item[key] = owner[key]
    if parameter:
        item['adapter'] = PARAM_POLICY
    return item


def select(ir, selection, only=None):
    """Select by signature. `only` is a fail-closed subset hook for synthetic tests."""
    if ir['schema'] != 1:
        raise ValueError('unsupported Lua binding IR version')
    policy = policies()
    owners = {o['name']: o for o in ir['singletons']}
    if not set(policy['accessors']).issubset(owners):
        raise ValueError('stale accessor policy')
    for name, accessor in policy['accessors'].items():
        if not accessor['reason'] or any(owners[name][key] != accessor[key] for key in ('get', 'literal', 'reference')):
            raise ValueError('changed or unexplained accessor policy: ' + name)
    available = {o['name'] + '.' + m['name'] for o in ir['singletons'] + ir.get('ap_objects', []) for m in o['methods']}
    if not set(policy['excluded']).issubset(available) or not all(policy['excluded'].values()):
        raise ValueError('stale or unexplained binding exclusion')
    if not set(policy['effects']).issubset(available) or not all(policy['effects'].values()):
        raise ValueError('stale or unexplained binding effect')
    includes = {h['name']: h for h in ir['includes']}
    if not set(policy['headers']).issubset(includes):
        raise ValueError('stale header policy')
    if only and not {o + '.' + m for o, c in only.items() for m in c['methods']}.issubset(available):
        raise ValueError('missing requested binding')
    methods, coverage, names = [], [], set()
    for owner in sorted(ir['singletons'], key=lambda o: o['name']):
        owner_reports = {}
        for method in sorted(owner['methods'], key=lambda m: m['name']):
            key = owner['name'] + '.' + method['name']
            if only is not None and method['name'] not in only.get(owner['name'], {}).get('methods', []):
                continue
            report = dict(owner=owner['name'], name=method['name'], line=method['line'], kind='singleton-method')
            try:
                if key in policy['excluded']:
                    raise ValueError(policy['excluded'][key])
                if (owner['get'] or owner['literal'] or owner['reference']) and owner['name'] not in policy['accessors']:
                    raise ValueError('unaudited-accessor')
                item = method_contract(owner, method)
                effects = policy['accessors'].get(owner['name'], {}).get('effects')
                if effects:
                    item['getter_effects'] = effects
                if key in policy['effects']:
                    item['effects'] = policy['effects'][key]
            except ValueError as error:
                if only is not None:
                    raise ValueError(f'bindings.desc:{method["line"]}: {key}: {error}') from error
                report.update(status='unsupported', reason=str(error))
            else:
                for name in [item['rust_name']] + item['aliases']:
                    if name in names:
                        raise ValueError('duplicate Rust binding name: ' + name)
                    names.add(name)
                item['id'] = len(methods) + 1
                if only is not None:
                    header = includes[only[owner['name']]['include']]
                    item.update(include=header['name'], include_depends=header['depends'])
                methods.append(item)
                report.update(status='generated', api=item['api_name'])
            coverage.append(report)
            owner_reports[method['name']] = report
        for alias in sorted(owner['aliases'], key=lambda a: a['alias']):
            if alias['manual']:
                logger = owner['name'] == 'AP_Logger' and alias['alias'] == 'write'
                coverage.append(dict(owner=owner['name'], name=alias['alias'], kind='singleton-manual',
                                     status='generated' if logger else 'unsupported',
                                     reason='manual-logger-adapter' if logger else 'manual-lua'))
            elif alias['name'] in owner_reports:
                target = owner_reports[alias['name']]
                report = dict(owner=owner['name'], name=alias['alias'], kind='singleton-alias',
                              target=alias['name'], status=target['status'])
                if target['status'] != 'generated':
                    report['reason'] = target['reason']
                coverage.append(report)
    objects, object_reports = (borrowed_binding.select(ir, len(methods) + 1, method_contract, policy['effects'])
                               if only is None else ([], []))
    methods += objects
    coverage += object_reports
    values, value_reports = (value_binding.select(ir, len(methods) + 1, method_contract)
                             if only is None else ([], []))
    methods += values
    coverage += [r for r in value_reports if r['owner'] in value_binding.TYPES]
    parameter = parameter_binding.select(ir, selection['Parameter'], len(methods) + 1)
    math = math_binding.select(ir, selection, parameter['layout']['id'] + 1)
    libm = libm_binding.specification(max(m['id'] for o in math['types'] for m in o['methods']) + 1)
    file = file_binding.specification(max(m['id'] for m in libm) + 1)
    log = log_binding.specification(file['id'] + 1)
    strtof = strtof_binding.specification(log['id'] + 1)
    for owner in sorted(ir['userdata'], key=lambda o: o['name']):
        if only is None and owner['name'] in value_binding.TYPES:
            continue
        selected = next((o for o in math['types'] if o['name'] == owner['name']), None)
        owner_methods = parameter['methods'] if owner['name'] == 'Parameter' else selected['methods'] if selected else []
        generated = {m['name'] for m in owner_methods}
        generated.update(m['method'] for m in values if m['owner'] == owner['name'])
        for method in sorted(owner['methods'], key=lambda m: m['name']):
            included = method['name'] in generated
            coverage.append(dict(owner=owner['name'], name=method['name'], line=method['line'], kind='userdata-method',
                                 status='generated' if included else 'unsupported',
                                 reason='shared-parameter-adapter' if included and owner['name'] == 'Parameter' else
                                 'value-adapter' if included else 'userdata-ownership-or-mutation'))
        for alias in sorted(owner['aliases'], key=lambda a: a['alias']):
            coverage.append(dict(owner=owner['name'], name=alias['alias'], kind='userdata-alias',
                                 status='unsupported', reason='manual-lua' if alias['manual'] else 'userdata-alias'))
    headers = []
    for header in ir['includes']:
        if only is not None and header['name'] not in {c['include'] for c in only.values()}:
            continue
        override = policy['headers'].get(header['name'])
        if override:
            if not override['reason']:
                raise ValueError('unexplained header policy')
            if override['replace'] is None:
                continue
            header = dict(header, name=override['replace'])
        headers.append(header)
    report = dict(scope='Exported singleton/userdata/AP-object methods and aliases; globals, fields, enums, '
                        'constructors and operators are not counted',
                  lifecycle='Shared firmware state changes are not rolled back when a module exits; '
                            'resource registrations requiring ownership are explicitly excluded.',
                  summary=dict(Counter(c['kind'] + ':' + c['status'] for c in coverage)), entries=coverage)
    return dict(encoding=ENCODING, lock_timeout_ms=LOCK_TIMEOUT_MS, methods=methods, headers=headers,
                parameter=parameter, math=math, libm=libm, file=file, log=log, strtof=strtof, coverage=report)


def specification():
    return select(read_ir(), json.loads((ROOT / 'generator/bindings.json').read_text()))


def emit(contract):
    notice = '// SPDX-License-Identifier: GPL-3.0-or-later\n// Generated by generator/generate.py; do not edit.\n'
    cpp = [notice, '#include "Bindings.h"', '#if AP_SCRIPTING_RUST_ENABLED',
           '#include "BindingSupport.h"', '#include "HostApi.h"', '#include "BindingParam.h"',
           '#include "BindingFile.h"', '#include "BindingLog.h"',
           '#include <AP_Scripting/AP_Scripting_config.h>', '#include <AP_Vehicle/AP_Vehicle_Type.h>',
           '#include <AP_Scheduler/AP_Scheduler.h>', '#include <AP_Common/strtod.h>']
    header_lines = []
    for header in contract['headers']:
        if header['depends']:
            header_lines.append('#if ' + header['depends'])
        header_lines.append(f'#include <{header["name"]}>')
        if header['depends']:
            header_lines.append('#endif')
    # Lua emits the list in both its generated header and source. Some guards
    # become defined by headers later in the first pass (for example OSD).
    cpp += ['// Match the Lua generated header/source dependency passes.'] + header_lines + header_lines
    cpp += ['#include <AP_Scripting/Parameter.h>', '#include <new>', '#include <type_traits>',
            '', 'namespace AP_Rust', '{',
            f'static_assert(sizeof(Parameter) <= {parameter_binding.MAX_SIZE}, "Parameter layout size cap");',
            f'static_assert(alignof(Parameter) <= {parameter_binding.MAX_ALIGNMENT}, "Parameter layout alignment cap");',
            'static_assert(std::is_trivially_destructible<Parameter>::value, '
            '"Parameter layout-query requires trivial drop");', '']
    if any(m.get('literal') and m['owner'].startswith('hal.') for m in contract['methods']):
        cpp.insert(3, '#include <AP_HAL/AP_HAL.h>\nextern const AP_HAL::HAL &hal;')
    value_types = {f['type'] for m in contract['methods'] for f in [m['result']] + m['arguments']
                   if value_binding.is_value(f['type'])}
    value_types.update(m['value_receiver'] for m in contract['methods'] if m.get('value_receiver'))
    values = sorted(singleton_binding.TYPES[name][0] for name in value_types)
    imports = ', '.join(['result', 'Context', 'Error', 'Result'] + values)
    rust = [notice, f'use crate::{{{imports}}};', '', "impl Context<'_> {"]
    handles = {m['result']['type'] for m in contract['methods'] if m['result']['type'] in borrowed_binding.TYPES}
    cpp += borrowed_binding.cpp_helpers(handles)
    cpp += value_binding.cpp_helpers(value_types)
    for method in contract['methods']:
        cpp += singleton_binding.cpp(method)
        if not method.get('receiver') and not method.get('value_receiver'):
            rust += singleton_binding.rust(method)
    cpp += libm_binding.cpp(contract['libm'])
    cpp += strtof_binding.cpp(contract['strtof'])
    cpp += ['int32_t binding_dispatch(void *context, uint32_t id, const uint8_t *input, uint32_t input_size,',
            '                         uint8_t *output, uint32_t output_size)', '{', '    switch (id) {']
    for method in contract['methods']:
        cpp += [f'    case {method["id"]}:',
                f'        return binding_{method["id"]}(input, input_size, output, output_size);']
    cpp += parameter_binding.cpp_cases(contract['parameter'])
    for method in contract['libm']:
        cpp += [f'    case {method["id"]}:',
                f'        return binding_libm_{method["id"]}(input, input_size, output, output_size);']
    cpp += [f'    case {contract["file"]["id"]}:',
            '        return binding_file_read_at(input, input_size, output, output_size);']
    cpp += [f'    case {contract["log"]["id"]}:',
            '        return binding_log_write(input, input_size, output, output_size);']
    cpp += [f'    case {contract["strtof"]["id"]}:',
            f'        return binding_strtof_{contract["strtof"]["id"]}(input, input_size, output, output_size);']
    cpp += ['    default:', '        return binding_math_dispatch(id, input, input_size, output, output_size);',
            '    }', '}', '}', '#endif', '']
    rust += ['}', '']
    return {'Bindings_generated.cpp': '\n'.join(cpp), 'sdk/src/bindings.rs': '\n'.join(rust),
            'sdk/src/enums.rs': singleton_binding.rust_enums(contract['methods'], notice),
            'sdk/src/borrowed.rs': borrowed_binding.rust(
                [m for m in contract['methods'] if m.get('receiver')], notice, singleton_binding.rust),
            'sdk/src/values.rs': value_binding.rust(contract['methods'], notice, singleton_binding.rust),
            'sdk/src/libm.rs': libm_binding.rust(contract['libm'], notice),
            'sdk/src/strtof.rs': strtof_binding.rust(contract['strtof'], notice),
            'sdk/src/file.rs': file_binding.rust(contract['file'], notice),
            'sdk/src/log.rs': log_binding.rust(contract['log'], notice),
            'BindingLog_generated.h': log_binding.header(notice),
            'sdk/src/parameter.rs': parameter_binding.rust(contract['parameter'], notice),
            'sdk/src/math.rs': math_binding.rust(contract['math'], notice),
            'BindingMath_generated.cpp': math_binding.cpp(contract['math'], notice),
            'BindingConfig.h': notice + '#pragma once\n\n' +
            f'#define AP_RUST_FILE_PATH_SIZE {file_binding.PATH_SIZE}U\n' +
            f'#define AP_RUST_FILE_READ_MAX {file_binding.READ_MAX}U\n' +
            f'#define AP_RUST_BINDING_LOCK_TIMEOUT_MS {contract["lock_timeout_ms"]}U\n' +
            f'#define AP_RUST_BINDING_PARAMETER_LAYOUT {contract["parameter"]["layout"]["id"]}U\n' +
            ''.join(f'#define AP_RUST_BINDING_PARAMETER_{m["name"].upper()} {m["id"]}U\n'
                    for m in contract['parameter']['methods']) +
            ''.join(f'#define AP_RUST_BINDING_{o["name"].upper()}_{m["name"].upper()} {m["id"]}U\n'
                    for o in contract['math']['types'] for m in o['methods']),
            'generator/bindings_contract.json':
                json.dumps({k: v for k, v in contract.items() if k != 'coverage'}, indent=2) + '\n',
            'generator/coverage.json': json.dumps(contract['coverage'], indent=2) + '\n'}
