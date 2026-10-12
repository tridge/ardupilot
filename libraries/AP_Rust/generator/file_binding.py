# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Bounded, stateless local-file reads for SD schedules (Lua io equivalent)."""

PATH_SIZE = 256
READ_MAX = 256
POLICY = ('v1:local-files-only,reject-leading-slashes-then-@,no-scheduler-lock,blocking-io,'
          'one-open-seek-read-close-per-call,no-retained-handles,not-atomic-between-calls,'
          'open-failure-is-none,seek-read-close-failure-is-host,past-eof-is-empty,zero-pad-output')


def specification(first_id):
    return dict(id=first_id, name='file.read_at',
                arguments=[dict(type='nonempty-nul-free-bytes-max255-zero-pad256', direction='in'),
                           dict(type='u32-le', direction='in', range=[0, 2147483647]),
                           dict(type='u16-le', direction='in', range=[1, READ_MAX])],
                result=dict(presence='bool-u8', length='u16-le', data=f'bytes-max{READ_MAX}-zero-pad{READ_MAX}'),
                policy=POLICY + ',offset-plus-count-le-INT32_MAX')


def rust(method, notice):
    return notice + f'''use crate::{{result, Context, Error, Result}};

impl Context<'_> {{
    /// Read up to 256 bytes from a local/SD file, opening and closing in this call.
    /// None means any open failure (missing, inaccessible, no card, out of memory,
    /// etc.), like Lua io.open's nil, not specifically "file absent".
    /// Some(0) is EOF, including offsets past EOF. Seek/read/close failures are Host.
    /// Virtual @ paths, empty paths/buffers and buffers larger than 256 are Invalid.
    /// Path strings are UTF-8, NUL-free, at most 255 bytes. offset+buffer.len() must
    /// fit i32. Only returned bytes are changed; errors/None leave the buffer alone.
    /// Blocking IO cannot be interrupted by stop; no scheduler lock is held.
    /// Not an atomic snapshot across calls. Cache startup data in your module heap;
    /// reopening every chunk can be expensive and uses transient firmware FS memory.
    pub fn file_read_at(&self, path: &str, offset: u32, buffer: &mut [u8]) -> Result<Option<usize>> {{
        if path.is_empty() || path.len() >= {PATH_SIZE} || path.as_bytes().contains(&0)
            || path.trim_start_matches('/').starts_with('@')
            || buffer.is_empty() || buffer.len() > {READ_MAX}
            || u64::from(offset) + buffer.len() as u64 > i32::MAX as u64
        {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; {PATH_SIZE + 6}];
        input[..path.len()].copy_from_slice(path.as_bytes());
        input[{PATH_SIZE}..{PATH_SIZE + 4}].copy_from_slice(&offset.to_le_bytes());
        input[{PATH_SIZE + 4}..].copy_from_slice(&(buffer.len() as u16).to_le_bytes());
        let mut output = [0u8; {READ_MAX + 3}];
        result(unsafe {{
            self.api.binding_call.unwrap()(self.raw, crate::imports::BINDING_{method['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        match output[0] {{
            0 => return Ok(None),
            1 => {{}},
            _ => return Err(Error::Host),
        }}
        let length = u16::from_le_bytes(output[1..3].try_into().unwrap()) as usize;
        if length > buffer.len() {{
            return Err(Error::Host);
        }}
        buffer[..length].copy_from_slice(&output[3..3 + length]);
        Ok(Some(length))
    }}
}}
'''
