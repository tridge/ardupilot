# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate persistent ONVIF client ownership and Rust emission."""

import copy
import ctypes
import os
import subprocess

import bindings
import onvif_binding as onvif
import pytest


def test_select_is_fail_closed():
    ir = bindings.read_ir()
    assert onvif.select(ir, 700) == onvif.specification(700)
    changed = copy.deepcopy(ir)
    owner = next(item for item in changed['singletons'] if item['name'] == onvif.OWNER)
    method = next(item for item in owner['methods'] if item['name'] == onvif.METHOD)
    method['arguments'][0]['pointer'] = True
    with pytest.raises(ValueError, match='unaudited AP_ONVIF.start'):
        onvif.select(changed, 700)


def test_contract_records_persistent_restart_semantics():
    method = onvif.specification(700)
    assert (method['id'], method['name'], method['result']) == (700, 'onvif.start', 'bool-u8')
    assert 'single-module-owner-claimed-before-start' in method['policy']
    assert 'persistent-client-allocation-not-released' in method['policy']
    assert 'blocking-firmware-network-call' in method['policy']


def test_emitted_rust_sdk_wire_and_validation(tmp_path):
    generated = onvif.rust(onvif.specification(700), '')
    assert "impl crate::bindings::Onvif<'_, '_>" in generated
    source = tmp_path / 'onvif.rs'
    source.write_text(r'''
use core::ffi::c_void;

#[derive(Clone, Copy, Debug, Eq, PartialEq)]
pub enum Error { Invalid, Host }
pub type Result<T = ()> = core::result::Result<T, Error>;
fn result(status: i32) -> Result { if status == 0 { Ok(()) } else { Err(Error::Host) } }
struct HostApi {
    binding_call: Option<unsafe extern "C" fn(*mut c_void, u32, *const u8, u32, *mut u8, u32) -> i32>,
}
pub struct Context<'a> { api: &'a HostApi, raw: *mut c_void }
pub mod bindings {
    use super::Context;
    pub struct Onvif<'context, 'api> { pub(crate) context: &'context Context<'api> }
}
impl<'api> Context<'api> {
    pub fn onvif(&self) -> bindings::Onvif<'_, 'api> { bindings::Onvif { context: self } }
}
pub(crate) struct Import(u32);
impl Import { pub(crate) fn slot(&self) -> u32 { self.0 } }
pub(crate) mod imports { use super::Import; pub(crate) static BINDING_700: Import = Import(700); }
mod generated {
''' + generated + r'''
}

#[cfg(test)]
mod tests {
    use super::{generated::*, Context, Error, HostApi};
    use core::ffi::c_void;

    struct Host { id: u32, calls: u32, input: [u8; 768], output: u8 }
    unsafe extern "C" fn call(raw: *mut c_void, id: u32, input: *const u8, ni: u32,
                               output: *mut u8, no: u32) -> i32 {
        let host = &mut *raw.cast::<Host>();
        host.id = id; host.calls += 1;
        host.input.copy_from_slice(core::slice::from_raw_parts(input, ni as usize));
        core::slice::from_raw_parts_mut(output, no as usize)[0] = host.output;
        0
    }

    #[test]
    fn exact_wire_and_validation() {
        let mut host = Host { id: 0, calls: 0, input: [0; 768], output: 1 };
        let api = HostApi { binding_call: Some(call) };
        let context = Context { api: &api, raw: (&mut host as *mut Host).cast() };
        assert_eq!(context.onvif().start("user", "pass", "http://camera"), Ok(true));
        assert_eq!(host.id, 700);
        assert_eq!(&host.input[..5], b"user\0");
        assert_eq!(&host.input[256..261], b"pass\0");
        assert_eq!(&host.input[512..526], b"http://camera\0");
        assert!(host.input[526..].iter().all(|byte| *byte == 0));
        let calls = host.calls;
        assert_eq!(context.onvif().start("bad\0user", "pass", "host"), Err(Error::Invalid));
        assert_eq!(context.onvif().start(&"x".repeat(256), "pass", "host"), Err(Error::Invalid));
        assert_eq!(host.calls, calls);
        host.output = 2;
        assert_eq!(context.onvif().start("user", "pass", "host"), Err(Error::Host));
    }
}
''')
    subprocess.run(['rustc', '--edition=2021', '--test', str(source), '-o', str(tmp_path / 'test')], check=True)
    subprocess.run([str(tmp_path / 'test')], check=True)


def _write_common_headers(tmp_path, enabled):
    headers = {
        'AP_HAL/AP_HAL_Boards.h': f'''#pragma once
#define AP_MODULE_ENABLED 1
#define ENABLE_ONVIF {1 if enabled else 0}
''',
        'AP_HAL/Semaphores.h': '''#pragma once
#include <stdint.h>
#define CLASS_NO_COPY(C) C(const C &) = delete; C &operator=(const C &) = delete
namespace AP_HAL { class Semaphore { public: bool take(uint32_t) { return true; } void give() {} }; }
using HAL_Semaphore = AP_HAL::Semaphore;
#define WITH_SEMAPHORE(sem) (void)(sem)
''',
        'AP_ONVIF/AP_ONVIF.h': '''#pragma once
#include <AP_HAL/AP_HAL_Boards.h>
#if ENABLE_ONVIF == 1
class AP_ONVIF {
public:
    static AP_ONVIF *get_singleton();
    bool start(const char *username, const char *password, const char *hostname);
};
#endif
''',
        'BindingConfig.h': '#pragma once\n',
    }
    for name, content in headers.items():
        path = tmp_path / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text(content)


@pytest.fixture
def onvif_host(tmp_path):
    _write_common_headers(tmp_path, True)
    source = tmp_path / 'wrapper.cpp'
    source.write_text(r'''
#include <AP_ONVIF/AP_ONVIF.h>
#include "BindingONVIF.h"
#include <string.h>
static AP_ONVIF onvif;
static bool start_result;
static uint32_t starts;
static char seen_hostname[256];
AP_ONVIF *AP_ONVIF::get_singleton() { return &onvif; }
bool AP_ONVIF::start(const char *, const char *, const char *hostname) {
    starts++;
    strncpy(seen_hostname, hostname, sizeof(seen_hostname) - 1U);
    return start_result;
}
extern "C" int32_t start(void *context, const uint8_t *input, uint32_t size,
                          uint8_t *output, uint32_t output_size) {
    return AP_Module::binding_onvif_start(context, input, size, output, output_size);
}
extern "C" void cleanup(void *context) { AP_Module::binding_onvif_cleanup(context); }
extern "C" void set_result(bool value) { start_result = value; }
extern "C" uint32_t start_count() { return starts; }
extern "C" const char *hostname_seen() { return seen_hostname; }
''')
    library = tmp_path / 'onvif.so'
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fPIC', '-shared',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT), str(source),
                    str(bindings.ROOT / 'BindingONVIF.cpp'), '-o', str(library)], check=True)
    lib = ctypes.CDLL(str(library))
    lib.start.argtypes = [ctypes.c_void_p, ctypes.c_void_p, ctypes.c_uint32, ctypes.c_void_p, ctypes.c_uint32]
    lib.cleanup.argtypes = [ctypes.c_void_p]
    lib.set_result.argtypes = [ctypes.c_bool]
    lib.hostname_seen.restype = ctypes.c_char_p
    return lib


def _packet(username=b'user', password=b'pass', hostname=b'http://camera'):
    fields = []
    for value in (username, password, hostname):
        assert len(value) <= 255 and b'\0' not in value
        fields.append(value + bytes(256 - len(value)))
    return b''.join(fields)


def test_host_claims_before_failed_start_and_releases_only_ownership(onvif_host):
    lib = onvif_host
    owner_a = ctypes.c_uint8()
    owner_b = ctypes.c_uint8()
    a = ctypes.addressof(owner_a)
    b = ctypes.addressof(owner_b)
    output = ctypes.create_string_buffer(1)
    packet = _packet()
    lib.set_result(False)
    assert lib.start(a, packet, len(packet), output, 1) == 0 and output.raw == b'\0'
    assert lib.start_count() == 1
    assert lib.start(b, packet, len(packet), output, 1) == -5
    assert lib.start_count() == 1
    lib.cleanup(a)
    lib.set_result(True)
    packet = _packet(hostname=b'http://new-camera')
    assert lib.start(b, packet, len(packet), output, 1) == 0 and output.raw == b'\1'
    assert lib.hostname_seen() == b'http://new-camera'
    malformed = bytearray(packet)
    malformed[10] = 1
    assert lib.start(a, bytes(malformed), len(malformed), output, 1) == -1


def test_host_compiles_with_onvif_disabled(tmp_path):
    _write_common_headers(tmp_path, False)
    subprocess.run([os.environ.get('CXX', 'c++'), '-std=c++17', '-Wall', '-Wextra', '-Werror', '-fsyntax-only',
                    '-I' + str(tmp_path), '-I' + str(bindings.ROOT),
                    str(bindings.ROOT / 'BindingONVIF.cpp')], check=True)
