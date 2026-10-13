# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Allocation-free firmware float conversion, with an end offset instead of a pointer."""


def specification(first_id):
    return dict(id=first_id, name='string.strtof',
                arguments=[dict(type='nul-free-bytes-max255-zero-pad256', direction='in')],
                result=dict(value='f32-le', consumed='u16-le'),
                policy='v1:ap_strtof,no-allocation,no-scheduler-lock,ascii-c-number-prefix,'
                       'no-conversion-is-zero-consumed,allow-nonfinite,ignore-errno,'
                       '19-significant-decimal-digits,rounding-not-guaranteed-at-halfway')


def cpp(method):
    return f'''static int32_t binding_strtof_{method['id']}(const uint8_t *input, uint32_t input_size,
                                    uint8_t *output, uint32_t output_size)
{{
    if (input_size != 256 || output_size != 6 || input == nullptr || output == nullptr) {{
        return AP_RUST_STATUS_INVALID;
    }}
    const char *text = BindingWire::read_cstring(input);
    if (text == nullptr) {{
        return AP_RUST_STATUS_INVALID;
    }}
    char *end;
    const float value = ap_strtof(text, &end);
    BindingWire::write_float(output, value);
    BindingWire::write(output + 4, uint16_t(end - text), 2);
    return AP_RUST_STATUS_OK;
}}
'''.splitlines()


def rust(method, notice):
    return notice + f'''use crate::{{result, Context, Error, Result}};

impl Context<'_> {{
    /// Allocation-free firmware ap_strtof: value and consumed byte count.
    /// Accepts C numeric prefixes (including whitespace, hex, NaN and infinity).
    /// No conversion returns (0.0, 0); overflow/underflow return inf/rounded zero
    /// or subnormals. errno is not exposed. Check the offset for trailing junk.
    /// Uses up to 19 significant decimal digits; rounding near halfway values
    /// is not guaranteed to match Rust's parser. Same implementation on all HALs.
    /// Input must be NUL-free and at most 255 bytes. No scheduler lock is held;
    /// stop is checked on every host call.
    pub fn strtof(&self, text: &str) -> Result<(f32, usize)> {{
        if text.len() > 255 || text.as_bytes().contains(&0) {{
            return Err(Error::Invalid);
        }}
        let mut input = [0u8; 256];
        input[..text.len()].copy_from_slice(text.as_bytes());
        let mut output = [0u8; 6];
        result(unsafe {{
            self.api.binding_call.unwrap()(self.raw, crate::imports::BINDING_{method['id']}.slot(),
                input.as_ptr(), input.len() as u32, output.as_mut_ptr(), output.len() as u32)
        }})?;
        let consumed = u16::from_le_bytes(output[4..6].try_into().unwrap()) as usize;
        if consumed > text.len() {{
            return Err(Error::Host);
        }}
        Ok((f32::from_le_bytes(output[..4].try_into().unwrap()), consumed))
    }}
}}
'''
