# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compare generated libm packets bit-for-bit with direct calls to the same libm."""

import ctypes
import os
import struct
import subprocess

import pytest

import bindings
import libm_binding


@pytest.fixture(scope='module', params=['-O0', '-O2'])
def libm(tmp_path_factory, request):
    work = tmp_path_factory.mktemp('libm-bindings')
    header = work / 'AP_HAL/Semaphores.h'
    header.parent.mkdir()
    header.write_text('''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { struct Semaphore { bool take(uint32_t); void give(); }; }
''')
    methods = libm_binding.specification(1)
    lines = ['#include "BindingSupport.h"', '#include "HostApi.h"', 'namespace AP_Rust {']
    lines += libm_binding.cpp(methods)
    lines += ['}', '''extern "C" int call(unsigned id, const uint8_t *in, unsigned ni, uint8_t *out, unsigned no) {
    switch (id) {''']
    lines += [f'    case {m["id"]}: return AP_Rust::binding_libm_{m["id"]}(in, ni, out, no);' for m in methods]
    lines += ['    default: return -1;', '    }', '}',
              'extern "C" uint32_t reference(unsigned id, uint32_t a_bits, uint32_t b_bits) {',
              '    float a, b, result;', '    memcpy(&a, &a_bits, 4); memcpy(&b, &b_bits, 4);', '    switch (id) {']
    for m in methods:
        args = 'a' if m['arity'] == 1 else 'a, b'
        lines.append(f'    case {m["id"]}: result = ::{m["name"]}({args}); break;')
    lines += ['    default: __builtin_trap();', '    }',
              '    uint32_t bits; memcpy(&bits, &result, 4); return bits;', '}']
    source, binary = work / 'libm.cpp', work / 'libm.so'
    source.write_text('\n'.join(lines))
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++11', request.param,
                    '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-I' + str(work), '-I' + str(bindings.ROOT), str(source), '-o', str(binary), '-lm'], check=True)
    lib = ctypes.CDLL(str(binary))
    lib.call.argtypes = [ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.call.restype = ctypes.c_int32
    lib.reference.argtypes = [ctypes.c_uint32] * 3
    lib.reference.restype = ctypes.c_uint32
    return lib


@pytest.mark.parametrize('method', libm_binding.specification(1), ids=lambda m: m['name'])
def test_libm_wire(libm, method):
    # Signed zero, subnormal, finite grid, infinities and quiet NaN payloads.
    values = [0, 0x80000000, 1, 0x80000001, 0x7f800000, 0xff800000, 0x7fc01234]
    values += [struct.unpack('<I', struct.pack('<f', v))[0] for v in (-100, -2, -1, -.125, .125, .5, 1, 2, 100)]
    output = ctypes.create_string_buffer(4)
    for a in values:
        for b in values if method['arity'] == 2 else [0]:
            packet = struct.pack('<I', a) + (struct.pack('<I', b) if method['arity'] == 2 else b'')
            assert libm.call(method['id'], packet, len(packet), output, 4) == 0
            assert struct.unpack('<I', output.raw)[0] == libm.reference(method['id'], a, b)
    for size in (0, 1, 3, method['arity'] * 4 + 1):
        assert libm.call(method['id'], packet, size, output, 4) == -1
    assert libm.call(method['id'], None, len(packet), output, 4) == -1
    assert libm.call(method['id'], packet, len(packet), None, 4) == -1
    assert libm.call(method['id'], packet, len(packet), output, 3) == -1
