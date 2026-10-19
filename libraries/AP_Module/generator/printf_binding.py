# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Experimental typed packets for the HAL's shared printf formatter."""


def specification(first_id):
    return dict(id=first_id, name='string.format_printf',
                arguments=[dict(type='format:nul-free-bytes-max255-zero-pad256,count:u8,'
                                     'tags:u8[8],values:le-u64[8],strings:nul-terminated-pool256,'
                                     'tags:1=i32,2=u32,3=i64,4=u64,5=f64,6=string-offset-u16', direction='in')],
                result='length:u16-le,text:nul-free-utf8-max255-zero-terminated256',
                policy='v1:hal-snprintf-args,no-allocation,max8args,checked-format-types,'
                       'd/i/u/o/x/X/c/s/f/F/e/E/g/G,ll-integers,width<=255,precision<=60,'
                       'no-star/no-positional/no-p/no-n,ascii-nonzero-c,overflow-is-invalid,'
                       'firmware-float32-rendering')


def rust(method, notice):
    return notice + f'''use crate::{{PrintfArg, PrintfText, Result}};

impl crate::bindings::String<'_, '_> {{
    /// Experimental firmware printf, without Rust's numeric formatting code.
    /// Up to eight typed arguments; format is at most 255 bytes, and strings
    /// share a 256-byte pool including terminators. No NULs or allocation.
    /// Supports d/i/u/o/x/X,
    /// ASCII c, s, f/F/e/E/g/G, ll integers, literal widths <=255 and precision
    /// <=60. Other modifiers, *, positional arguments, p and n are rejected.
    /// Format/type mismatches, overflow and non-UTF-8 results return Invalid.
    /// Float output follows HAL float32 formatting, even for f64 arguments.
    pub fn format_printf(&self, format: &str, args: &[PrintfArg<'_>]) -> Result<PrintfText> {{
        crate::printf::format(self.context, crate::imports::BINDING_{method['id']}.slot(), format, args)
    }}
}}
'''
