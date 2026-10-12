# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Audited native adapter for the same Parameter helper used by Lua."""

ARGUMENTS = {
    'init': [('string', False, False)],
    'get': [('float', True, False)],
    'configured': [],
    'set': [('float', False, True)],
    'set_and_save': [('float', False, True)],
    'set_default': [('float', False, True)],
}
POLICY = 'v2:Parameter,placement-init,host-layout,trivial-drop,receiver-le-u64,checked-f32'
MAX_SIZE = 16 * 1024 * 1024
MAX_ALIGNMENT = 4096


def layout_policy():
    return dict(ownership='module-heap,trivial-drop', size_min=1, size_max=MAX_SIZE,
                alignment='power-of-two,size-multiple', alignment_max=MAX_ALIGNMENT)


def select(ir, config, first_id):
    if config != {'include': 'AP_Scripting/AP_Scripting_helpers.h', 'methods': sorted(ARGUMENTS)}:
        raise ValueError('unsupported Parameter selection')
    if not any(item == {'name': config['include'], 'depends': None} for item in ir['includes']):
        raise ValueError('unaudited Parameter include')
    owner = next((item for item in ir['userdata'] if item['name'] == 'Parameter'), None)
    if owner is None:
        raise ValueError('missing Parameter userdata')
    expected = dict(rename=None, depends=None, get=None, creation='lua_new_Parameter', creation_manual=True,
                    creation_args=0, has_fields=False, has_enums=False, operations=0, operator_getter=None,
                    literal=False, reference=False, semaphore=False, scheduler_semaphore=False,
                    semaphore_pointer=False, aliases=[])
    if any(owner[key] != value for key, value in expected.items()):
        raise ValueError('unaudited Parameter owner metadata')
    methods = {method['name']: method for method in owner['methods']}
    result = []
    for name, arguments in sorted(ARGUMENTS.items()):
        method = methods.get(name)
        if method is None:
            raise ValueError(f'missing Parameter.{name}')
        expected_types = [('boolean', False, False)] + arguments
        types = [method['return']] + method['arguments']
        if (any(method[key] is not None for key in ('rename', 'depends', 'deprecate')) or
                len(types) != len(expected_types) or any(
                    value != dict(kind=kind, name=None, nullable=nullable, reference=False,
                                  pointer=False, skip_check=skip_check, range=None)
                    for value, (kind, nullable, skip_check) in zip(types, expected_types))):
            raise ValueError(f'bindings.desc:{method["line"]}: unaudited Parameter.{name} signature')
        result.append(dict(id=first_id + len(result), name=name, arguments=method['arguments'], result=method['return']))
    return dict(policy=POLICY, methods=result,
                layout=dict(id=first_id + len(result), policy=layout_policy()))


def cpp_cases(contract):
    lines = [f'    case {contract["layout"]["id"]}: {{ // Parameter::__layout',
             '        if (input_size != 0 || output_size != 8 || output == nullptr) {',
             '            return AP_RUST_STATUS_INVALID;', '        }',
             '        BindingWire::write(output, uint32_t(sizeof(Parameter)), 4);',
             '        BindingWire::write(output + 4, uint32_t(alignof(Parameter)), 4);',
             '        return AP_RUST_STATUS_OK;', '    }']
    for method in contract['methods']:
        name = method['name']
        setter = name.startswith('set')
        size = 25 if name == 'init' else (12 if setter else 8)
        output = 5 if name == 'get' else 1
        lines += [f'    case {method["id"]}: {{ // Parameter::{name}',
                  f'        if (input_size != {size} || output_size != {output} || input == nullptr || output == nullptr) {{',
                  '            return AP_RUST_STATUS_INVALID;', '        }', f'        memset(output, 0, {output});',
                  '        const uint64_t address = uint64_t(BindingWire::read(input, 4)) |',
                  '                                 (uint64_t(BindingWire::read(input + 4, 4)) << 32);',
                  '        if (address == 0 || address > UINTPTR_MAX || address % alignof(Parameter) != 0) {',
                  '            return AP_RUST_STATUS_INVALID;', '        }',
                  '        auto *parameter = reinterpret_cast<Parameter *>(uintptr_t(address));']
        if name == 'init':
            lines += ['        const char *name = BindingWire::read_string(input + 8);',
                      '        if (name == nullptr) {', '            return AP_RUST_STATUS_INVALID;', '        }',
                      '        parameter = new (parameter) Parameter();',
                      '        output[0] = parameter->init(name);']
        elif name == 'get':
            lines += ['        float value;', '        if (parameter->get(value)) {', '            output[0] = 1;',
                      '            BindingWire::write_float(output + 1, value);', '        }']
        elif setter:
            lines += ['        const float value = BindingWire::read_float(input + 8);',
                      '        if (!BindingParam::valid_value(parameter->type(), value)) {',
                      '            return AP_RUST_STATUS_INVALID;', '        }',
                      f'        output[0] = parameter->{name}(value);']
        else:
            lines += ['        output[0] = parameter->configured();']
        lines += ['        return AP_RUST_STATUS_OK;', '    }']
    return lines


def rust(contract, notice):
    ids = {method['name']: method['id'] for method in contract['methods']}
    lines = [notice, '''use crate::{result, Context, Error, Result};
use core::ptr::NonNull;

/// The same cached C++ Parameter helper used by Lua. It reads live firmware
/// values, not a snapshot. Backend parameter lifetimes and unsynchronized
/// scalar access have the same assumptions as Lua.
/// Storage belongs to the module heap; moving this wrapper does not move the
/// C++ object. Safe code cannot manufacture, share or outlive its context.
pub struct Parameter<'ctx, 'host> {
    context: &'ctx Context<'host>,
    storage: NonNull<u8>,
}

impl<'host> Context<'host> {
    /// Resolve a parameter by name, as with Lua's Parameter(name).
    /// A missing name returns None; allocation failure returns NoMemory.
    pub fn parameter<'ctx>(&'ctx self, name: &str) -> Result<Option<Parameter<'ctx, 'host>>> {
        validate_name(name)?;
        let mut layout = [0u8; 8];
        result(unsafe {''',
             '            self.api.binding_call.unwrap()(self.raw, '
             f'crate::imports::BINDING_{contract["layout"]["id"]}.slot(),',
             '''                core::ptr::null(), 0, layout.as_mut_ptr(), 8)
        })?;
        let size = u32::from_le_bytes(layout[..4].try_into().unwrap());
        let alignment = u32::from_le_bytes(layout[4..].try_into().unwrap());''',
             f'        if size == 0 || size > {MAX_SIZE} || !alignment.is_power_of_two() ||',
             f'            alignment > {MAX_ALIGNMENT} || size % alignment != 0 {{',
             '''            return Err(Error::Host);
        }
        let storage = NonNull::new(unsafe {
            self.api.allocate.unwrap()(self.raw, size, alignment)
        }).ok_or(Error::NoMemory)?;
        // Drop releases storage on init failure, including a concurrent stop.
        let mut parameter = Parameter { context: self, storage };
        if storage.as_ptr() as usize % alignment as usize != 0 {
            return Err(Error::Host);
        }
        if parameter.init(name)? { Ok(Some(parameter)) } else { Ok(None) }
    }
}

fn validate_name(name: &str) -> Result {
    if name.is_empty() || name.len() > 16 || name.as_bytes().contains(&0) {
        return Err(Error::Invalid);
    }
    Ok(())
}

impl Parameter<'_, '_> {
    fn call(&self, id: u32, input: &mut [u8], output: &mut [u8]) -> Result {
        input[..8].copy_from_slice(&(self.storage.as_ptr() as usize as u64).to_le_bytes());
        result(unsafe {
            self.context.api.binding_call.unwrap()(self.context.raw, id, input.as_ptr(),
                input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        })
    }

    /// Rebind this helper. A missing name clears it, just as Lua init does.
    pub fn init(&mut self, name: &str) -> Result<bool> {
        validate_name(name)?;
        let mut input = [0u8; 25];
        input[8..8 + name.len()].copy_from_slice(name.as_bytes());
        let mut output = [0u8; 1];''',
             f'        self.call(crate::imports::BINDING_{ids["init"]}.slot(), &mut input, &mut output)?;',
             '''        boolean(output[0])
    }

    /// Read the current value. An unbound helper returns None.
    pub fn get(&self) -> Result<Option<f32>> {
        let mut output = [0u8; 5];''',
             f'        self.call(crate::imports::BINDING_{ids["get"]}.slot(), &mut [0u8; 8], &mut output)?;',
             '''        if !boolean(output[0])? { return Ok(None); }
        Ok(Some(f32::from_le_bytes(output[1..5].try_into().unwrap())))
    }
''']
    for name in ('configured', 'set', 'set_and_save', 'set_default'):
        setter = name.startswith('set')
        lines.append(f'    /// `Parameter::{name}` from bindings.desc; same implementation as Lua.')
        if setter:
            lines.append('    /// Rejects non-finite and out-of-range integer inputs before C++ conversion.')
        if name == 'set_and_save':
            lines.append('    /// May wait for save IO; success does not imply durable storage.')
        lines += [f'    pub fn {name}(&self{", value: f32" if setter else ""}) -> Result<bool> {{',
                  f'        let mut input = [0u8; {12 if setter else 8}];']
        if setter:
            lines.append('        input[8..12].copy_from_slice(&value.to_le_bytes());')
        lines += ['        let mut output = [0u8; 1];',
                  f'        self.call(crate::imports::BINDING_{ids[name]}.slot(), &mut input, &mut output)?;',
                  '        boolean(output[0])', '    }', '']
    lines += ['''}

fn boolean(value: u8) -> Result<bool> {
    match value { 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }
}

impl Drop for Parameter<'_, '_> {
    fn drop(&mut self) {
        // The host asserts trivial destruction. Deallocation remains available
        // after stop, unlike binding calls, and runs before the worker joins.
        unsafe { self.context.api.deallocate.unwrap()(self.context.raw, self.storage.as_ptr()); }
    }
}
''']
    return '\n'.join(lines)
