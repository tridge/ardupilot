# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Bounded filesystem metadata, read and directory traversal APIs."""

OWNER = 'AP_Filesystem'
STAT = 'AP_Filesystem::stat_t'
PATH_SIZE = 256
READ_MAX = 1024
WRITE_MAX = 1024
NAME_MAX = 255
POLICY = ('v1:stateless-open-per-call,path-max255,virtual-and-local-backends,blocking-io,'
          'no-retained-descriptors,read-max1024,append-max1024,dir-index-u16,'
          'all-descriptors-closed-on-all-paths,fixed-zero-padded-wire')


def select(ir, first_id):
    owner = next((item for item in ir['singletons'] if item['name'] == OWNER), None)
    if owner is None or owner['rename'] != 'fs' or owner['depends'] is not None:
        raise ValueError('unaudited AP_Filesystem metadata')
    stat = next((item for item in owner['methods'] if item['name'] == 'stat'), None)
    if stat is None or stat['return']['kind'] != 'boolean' or len(stat['arguments']) != 2:
        raise ValueError('unaudited AP_Filesystem.stat')
    if (stat['arguments'][0]['kind'], stat['arguments'][1]['kind'], stat['arguments'][1]['name'],
            stat['arguments'][1]['nullable']) != ('string', 'userdata', STAT, True):
        raise ValueError('unaudited AP_Filesystem.stat arguments')
    format_method = next((item for item in owner['methods'] if item['name'] == 'format'), None)
    if (format_method is None or format_method['return']['kind'] != 'boolean' or
            format_method['arguments'] or format_method['depends'] != 'AP_FILESYSTEM_FORMAT_ENABLED'):
        raise ValueError('unaudited AP_Filesystem.format')
    value = next((item for item in ir['userdata'] if item['name'] == STAT), None)
    if value is None:
        raise ValueError('missing AP_Filesystem::stat_t')
    fields = {(field['name'], field['type']['kind'], field['read'], field['write'])
              for field in value['fields']}
    expected = {('size', 'uint32_t', True, False), ('mtime', 'uint32_t', True, False),
                ('mode', 'int32_t', True, False), ('atime', 'uint32_t', True, False),
                ('ctime', 'uint32_t', True, False)}
    if fields != expected:
        raise ValueError('unaudited AP_Filesystem::stat_t fields')
    return specification(first_id)


def specification(first_id):
    path = dict(type='nonempty-nul-free-bytes-max255-zero-pad256', name='path')
    methods = [
        dict(id=first_id, name='filesystem.stat', arguments=[path],
             result=dict(presence='bool-u8', size='u32-le', mtime='u32-le', directory='bool-u8'),
             policy=POLICY),
        dict(id=first_id + 1, name='filesystem.read_at',
             arguments=[path, dict(type='u32-le', name='offset'),
                        dict(type='u16-le', range=[1, READ_MAX], name='length')],
             result=dict(presence='bool-u8', length='u16-le',
                         data=f'bytes-max{READ_MAX}-zero-pad{READ_MAX}'), policy=POLICY),
        dict(id=first_id + 2, name='filesystem.dir_entry',
             arguments=[path, dict(type='u16-le', name='index')],
             result=dict(presence='bool-u8', length='u16-le',
                         name=f'utf8-max{NAME_MAX}-zero-pad{NAME_MAX}'), policy=POLICY),
        dict(id=first_id + 3, name='filesystem.truncate', arguments=[path], result='bool-u8', policy=POLICY),
        dict(id=first_id + 4, name='filesystem.append',
             arguments=[path, dict(type=f'bytes-max{WRITE_MAX}-zero-pad{WRITE_MAX}', name='data')],
             result='bool-u8', policy=POLICY),
        dict(id=first_id + 5, name='filesystem.unlink', arguments=[path], result='bool-u8', policy=POLICY),
        dict(id=first_id + 6, name='filesystem.format', arguments=[], result='bool-u8',
             policy=POLICY + ',requires-disarmed,asynchronous-backend-status-not-retained'),
    ]
    return dict(methods=methods, read_max=READ_MAX, write_max=WRITE_MAX, name_max=NAME_MAX)


def rust(spec, notice):
    stat, read, entry, truncate, append, unlink, format_ = spec['methods']
    return notice + f'''use crate::{{result, Error, Result}};

pub const FILESYSTEM_READ_MAX: usize = {READ_MAX};
pub const FILESYSTEM_WRITE_MAX: usize = {WRITE_MAX};

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub struct FileStat {{ pub size: u32, pub mtime: u32, pub is_directory: bool }}

#[derive(Clone, Debug, Eq, PartialEq)]
pub struct DirEntry {{ bytes: [u8; {NAME_MAX}], length: u8 }}

impl DirEntry {{
    pub fn as_str(&self) -> &str {{
        // Host output is validated during construction.
        unsafe {{ core::str::from_utf8_unchecked(&self.bytes[..self.length as usize]) }}
    }}
}}

fn path_wire(path: &str) -> Result<[u8; {PATH_SIZE}]> {{
    if path.is_empty() || path.len() >= {PATH_SIZE} || path.as_bytes().contains(&0) {{
        return Err(Error::Invalid);
    }}
    let mut output = [0u8; {PATH_SIZE}];
    output[..path.len()].copy_from_slice(path.as_bytes());
    Ok(output)
}}

impl crate::bindings::Fs<'_, '_> {{
    fn path_bool(&self, path: &str, id: u32) -> Result<bool> {{
        let input = path_wire(path)?;
        let mut output = [0u8; 1];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw, id,
            input.as_ptr(), input.len() as u32, output.as_mut_ptr(), 1) }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    pub fn stat(&self, path: &str) -> Result<Option<FileStat>> {{
        let input = path_wire(path)?;
        let mut output = [0u8; 10];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{stat['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), output.len() as u32) }})?;
        match output[0] {{
            0 if output[1..].iter().all(|byte| *byte == 0) => Ok(None),
            0 => Err(Error::Host),
            1 if output[9] <= 1 => Ok(Some(FileStat {{
                size: u32::from_le_bytes(output[1..5].try_into().unwrap()),
                mtime: u32::from_le_bytes(output[5..9].try_into().unwrap()),
                is_directory: output[9] != 0,
            }})),
            _ => Err(Error::Host),
        }}
    }}

    /// Read a bounded chunk. `None` means open failure; `Some(0)` is EOF.
    pub fn read_at(&self, path: &str, offset: u32,
                   buffer: &mut [u8]) -> Result<Option<usize>> {{
        if buffer.is_empty() || buffer.len() > FILESYSTEM_READ_MAX
            || u64::from(offset) + buffer.len() as u64 > i32::MAX as u64 {{
            return Err(Error::Invalid);
        }}
        let path = path_wire(path)?;
        let mut input = [0u8; {PATH_SIZE + 6}];
        input[..{PATH_SIZE}].copy_from_slice(&path);
        input[{PATH_SIZE}..{PATH_SIZE + 4}].copy_from_slice(&offset.to_le_bytes());
        input[{PATH_SIZE + 4}..].copy_from_slice(&(buffer.len() as u16).to_le_bytes());
        let mut output = [0u8; {READ_MAX + 3}];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{read['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), output.len() as u32) }})?;
        if output[0] == 0 {{
            return if output[1..].iter().all(|byte| *byte == 0) {{ Ok(None) }} else {{ Err(Error::Host) }};
        }}
        if output[0] != 1 {{ return Err(Error::Host); }}
        let length = u16::from_le_bytes(output[1..3].try_into().unwrap()) as usize;
        if length > buffer.len() || output[3 + length..].iter().any(|byte| *byte != 0) {{
            return Err(Error::Host);
        }}
        buffer[..length].copy_from_slice(&output[3..3 + length]);
        Ok(Some(length))
    }}

    /// Return the indexed directory entry. Each call opens and closes the
    /// directory, avoiding retained firmware pointers across module calls.
    pub fn dir_entry(&self, path: &str, index: u16) -> Result<Option<DirEntry>> {{
        let path = path_wire(path)?;
        let mut input = [0u8; {PATH_SIZE + 2}];
        input[..{PATH_SIZE}].copy_from_slice(&path);
        input[{PATH_SIZE}..].copy_from_slice(&index.to_le_bytes());
        let mut output = [0u8; {NAME_MAX + 3}];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{entry['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), output.len() as u32) }})?;
        if output[0] == 0 {{
            return if output[1..].iter().all(|byte| *byte == 0) {{ Ok(None) }} else {{ Err(Error::Host) }};
        }}
        if output[0] != 1 {{ return Err(Error::Host); }}
        let length = u16::from_le_bytes(output[1..3].try_into().unwrap()) as usize;
        if length > {NAME_MAX} || output[3 + length..].iter().any(|byte| *byte != 0) {{
            return Err(Error::Host);
        }}
        core::str::from_utf8(&output[3..3 + length]).map_err(|_| Error::Host)?;
        let mut bytes = [0u8; {NAME_MAX}];
        bytes.copy_from_slice(&output[3..]);
        Ok(Some(DirEntry {{ bytes, length: length as u8 }}))
    }}

    /// Create or truncate a file. No descriptor is retained after this call.
    pub fn truncate(&self, path: &str) -> Result<bool> {{
        self.path_bool(path, crate::imports::BINDING_{truncate['id']}.slot())
    }}

    /// Append one bounded chunk, creating the file if needed. Call truncate
    /// once before sequential appends when replacing a file.
    pub fn append(&self, path: &str, data: &[u8]) -> Result<bool> {{
        if data.is_empty() || data.len() > FILESYSTEM_WRITE_MAX {{ return Err(Error::Invalid); }}
        let path = path_wire(path)?;
        let mut input = [0u8; {PATH_SIZE + 2 + WRITE_MAX}];
        input[..{PATH_SIZE}].copy_from_slice(&path);
        input[{PATH_SIZE}..{PATH_SIZE + 2}].copy_from_slice(&(data.len() as u16).to_le_bytes());
        input[{PATH_SIZE + 2}..{PATH_SIZE + 2} + data.len()].copy_from_slice(data);
        let mut output = [0u8; 1];
        result(unsafe {{ self.context.api.binding_call.unwrap()(self.context.raw,
            crate::imports::BINDING_{append['id']}.slot(), input.as_ptr(), input.len() as u32,
            output.as_mut_ptr(), 1) }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}

    pub fn unlink(&self, path: &str) -> Result<bool> {{
        self.path_bool(path, crate::imports::BINDING_{unlink['id']}.slot())
    }}

    /// Start formatting the local filesystem. Firmware rejects this while armed.
    pub fn format(&self) -> Result<bool> {{
        let mut output = [0u8; 1];
        result(unsafe {{ self.context.api.binding_call.unwrap()(
            self.context.raw, crate::imports::BINDING_{format_['id']}.slot(),
            core::ptr::null(), 0, output.as_mut_ptr(), 1)
        }})?;
        match output[0] {{ 0 => Ok(false), 1 => Ok(true), _ => Err(Error::Host) }}
    }}
}}
'''
