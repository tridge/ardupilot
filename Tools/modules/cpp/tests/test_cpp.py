# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Compile both C++ target profiles and run Rust/C++ together in SITL."""

import subprocess

import apmod_debug
import pack
import pytest

from test_runtime import ROOT
from test_runtime import Vehicle

import build


@pytest.mark.parametrize('target', build.TARGETS)
@pytest.mark.parametrize('debug', [False, True])
def test_cpp_rebases_without_rust(tmp_path, target, debug, monkeypatch):
    monkeypatch.setenv('RUSTC', '/no/rust/installation')
    source = ROOT / 'libraries/AP_Module/tests/modules/lifecycle.cpp'
    first = build.build(source, tmp_path / 'first.apmod', target, 8192, debug=debug)
    second = build.build(source, tmp_path / 'second.apmod', target, 8192, base=0x24123000, debug=debug)
    assert first.read_bytes() == second.read_bytes()
    module = pack.unpack(first.read_bytes())
    assert {name for _, _, name in module['imports']} == {'ahrs.get_gyro', 'Vector3f.length', 'gcs.send_text'}
    assert module['contract']['heap_size'] == 4096
    if debug:
        symbols = apmod_debug.ModuleSymbols(first)
        assert not symbols.is_rust


def test_mixed_rust_cpp_sitl(tmp_path, monkeypatch):
    monkeypatch.setenv('AP_MODULE_LIFECYCLE', 'cleanup')
    images = {}
    for ext in ('cpp', 'rs'):
        source = ROOT / f'libraries/AP_Module/tests/modules/lifecycle.{ext}'
        image = build.build(source, tmp_path / f'{ext}.apmod', 'x86_64-unknown-linux-gnu', 8192)
        images[image.name] = image.read_bytes()
    vehicle = Vehicle(tmp_path / 'vehicle', images, {'SCR_ENABLE': 0})
    try:
        vehicle.start()
        vehicle.text('CPP ready')
        vehicle.text('Module: ready')
        for generation in range(1, 4):
            mark = len(vehicle.texts)
            vehicle.restart(generation, expected=2)
            vehicle.text('CPP ready', after=mark)
            vehicle.text('Module: ready', after=mark)
        vehicle.arm()
        vehicle.arm(False)
    finally:
        vehicle.stop()


@pytest.mark.parametrize('target', build.TARGETS)
def test_global_constructors_rejected(tmp_path, target):
    source = tmp_path / 'constructor.cpp'
    source.write_text('''#include <ap_module.hpp>
volatile unsigned initialized;
struct Init { Init() { initialized = 1; } } global;
static ap_module::Result<void> run(ap_module::Context &) { return {ap_module::Error::Ok}; }
AP_MODULE(run)
''')
    with pytest.raises((ValueError, subprocess.CalledProcessError)):
        build.build(source, tmp_path / 'constructor.apmod', target, 8192)


def test_allocations_and_parameter_raii_sitl(tmp_path):
    source = tmp_path / 'allocations.cpp'
    source.write_text('''#include <ap_module.hpp>
#include <new>
using namespace ap_module;
struct alignas(64) Aligned { uint32_t value; };
static Result<void> run(Context &c) {
    auto parameter = Parameter::create(c, "MOD_ENABLE");
    if (!parameter) { return {parameter.error}; }
    auto value = parameter.value.get();
    if (!value || !value.value.present || value.value.value != 1.0f) { return {Error::Host}; }
    auto *a = new Aligned{1234};
    auto *b = new uint32_t[10];
    if (reinterpret_cast<uintptr_t>(a) % 64 != 0 || a->value != 1234) { return {Error::Host}; }
    b[9] = 5678;
    delete a; delete[] b;
    auto *large = new (std::nothrow) uint8_t[65536];
    if (large != nullptr) { delete[] large; return {Error::Host}; }
    return c.text(6, "CPP allocations ok");
}
AP_MODULE(run)
''')
    image = build.build(source, tmp_path / 'allocations.apmod', 'x86_64-unknown-linux-gnu', 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {image.name: image.read_bytes()}, {'SCR_ENABLE': 0})
    try:
        vehicle.start()
        vehicle.text('CPP allocations ok')
        vehicle.text('Module: exited allocations.apmod')
        reports = [t for t in vehicle.texts if t.startswith('Module RAM: heap ')]
        assert 'used 0' in reports[-1], reports
        assert not any('panic' in t or 'returned -' in t for t in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()
