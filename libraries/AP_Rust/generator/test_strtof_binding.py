# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise the generated packet adapter with the real allocation-free parser."""

import ctypes
import math
import os
import struct
import subprocess

import pytest

import bindings
import strtof_binding


@pytest.fixture(scope='module', params=['-O0', '-Os'])
def parser(tmp_path_factory, request):
    work = tmp_path_factory.mktemp('strtof-binding')
    hal = work / 'AP_HAL'
    hal.mkdir()
    (hal / 'Semaphores.h').write_text('''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t); void give(); }; }
''')
    (hal / 'AP_HAL_Boards.h').write_text('''#pragma once
#define CONFIG_HAL_BOARD 99
#define HAL_BOARD_CHIBIOS 10
#define HAL_BOARD_SITL 3
''')
    lines = ['#include "BindingSupport.h"', '#include "HostApi.h"', '#include <AP_Common/strtod.h>',
             'namespace AP_Rust {']
    lines += strtof_binding.cpp(strtof_binding.specification(1))
    lines += ['}', '''extern "C" int call(const uint8_t *in, unsigned ni, uint8_t *out, unsigned no) {
    return AP_Rust::binding_strtof_1(in, ni, out, no);
}''']
    # Compile the parser's one AP_Common helper verbatim, without pulling the
    # unrelated HAL-dependent utility functions into this host packet test.
    common = (bindings.ROOT.parent / 'AP_Common/AP_Common.cpp').read_text()
    start = common.index('bool hex_char_to_nibble(')
    end = common.index('\n}', start) + 2
    lines.append(common[start:end])
    source, binary = work / 'test.cpp', work / 'test.so'
    source.write_text('\n'.join(lines))
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', request.param,
                    '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-I' + str(work), '-I' + str(bindings.ROOT), '-I' + str(bindings.ROOT.parent),
                    str(source), str(bindings.ROOT.parent / 'AP_Common/strtod.cpp'),
                    '-o', str(binary)], check=True)
    lib = ctypes.CDLL(str(binary))
    lib.call.argtypes = [ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.call.restype = ctypes.c_int32
    return lib


@pytest.mark.parametrize('text,value,consumed', [
    (b'', 0.0, 0), (b'junk', 0.0, 0), (b'  ', 0.0, 0),
    (b'25', 25.0, 2), (b'-0', -0.0, 2), (b'+.125E+2', 12.5, 8),
    (b'  -2.5tail', -2.5, 6), (b'1e+', 1.0, 1), (b'0x1.8p2', 6.0, 7),
    (b'1e9999', math.inf, 6), (b'-1e-9999', -0.0, 8),
    (b'NaN', math.nan, 3), (b'-inf', -math.inf, 4),
    (b'1.401298464324817e-45', 2**-149, 21),
    (b'0' * 255, 0.0, 255),
    (b'1' * 255, math.inf, 255),
    (b'0' * 252 + b'1.5', 1.5, 255),
])
def test_conversion(parser, text, value, consumed):
    output = ctypes.create_string_buffer(6)
    assert parser.call(text.ljust(256, b'\0'), 256, output, 6) == 0
    actual, end = struct.unpack('<fH', output.raw)
    assert end == consumed
    if math.isnan(value):
        assert math.isnan(actual)
    else:
        assert struct.pack('<f', actual) == struct.pack('<f', value)


def test_invalid_packets(parser):
    good = b'25'.ljust(256, b'\0')
    output = ctypes.create_string_buffer(b'!!!!!!', 6)
    for packet in (b'1' * 256, b'1\0junk'.ljust(256, b'\0')):
        assert parser.call(packet, 256, output, 6) == -1
    for size in (0, 1, 255, 257):
        assert parser.call(good, size, output, 6) == -1
    for size in (0, 5, 7):
        assert parser.call(good, 256, output, size) == -1
    assert parser.call(None, 256, output, 6) == -1
    assert parser.call(good, 256, None, 6) == -1
    assert output.raw == b'!!!!!!'
