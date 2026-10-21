# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise the real GDB Python helper against a live SITL native module."""

import contextlib
import os
import shutil
import signal
import subprocess

from pathlib import Path

import pytest

import build

ROOT = Path(__file__).resolve().parents[3]


@pytest.mark.parametrize('module_count', [1, 2])
def test_gdb_sitl_symbols_and_reload(tmp_path, module_count):
    firmware = Path(os.environ.get('AP_MODULE_SITL_BINARY', ROOT / 'build/sitl/bin/arduplane'))
    debugger = os.environ.get('AP_MODULE_GDB', 'gdb')
    if not firmware.is_file() or not shutil.which(debugger):
        pytest.skip('requires Rust SITL firmware and Python-enabled GDB')
    modules = tmp_path / 'modules'
    modules.mkdir()
    module = build.build(ROOT / 'libraries/AP_Module/tests/modules/debug.rs', modules / 'debug.apmod',
                         'x86_64-unknown-linux-gnu', 16384, debug=True, opt_level='1')
    if module_count == 2:
        shutil.copyfile(module, modules / 'second.apmod')
    params = tmp_path / 'debug.parm'
    params.write_text('MOD_ENABLE 1\nMOD_HEAP_MAX 4096\nMOD_STACK_MAX 32768\nLOG_BACKEND_TYPE 0\n')
    commands = [
        'set pagination off', 'set confirm off', 'set debuginfod enabled off', 'set breakpoint pending on',
        f'source {ROOT / "Tools/modules/apmod_gdb.py"}', 'apmod printers', f'apmod symbols {module}',
        'break apmod_debug_tick', 'run', 'next', 'apmod list', 'print label', 'print *values',
        'print debug::APMOD_DEBUG_COUNT', 'print debug::APMOD_DEBUG_MAGIC', 'bt 8', 'next',
        'set language c++', "set var 'AP_Module::Runtime::_singleton'->_restart._value = 1",
        'disable 1', 'tbreak AP_Module::Module::start()', 'continue', 'apmod list',
        'break apmod_debug_tick', 'continue', 'next', 'print label', 'print debug::APMOD_DEBUG_COUNT',
        'apmod list',
        'apmod clear', 'quit',
    ]
    command = [debugger, '-nx', '--batch', str(firmware.resolve())]
    for expression in commands:
        command += ['-ex', expression]
    command += ['--args', str(firmware.resolve()), '--model', 'plane', '--serial0', 'none',
                '--serial1', 'none', '--serial2', 'none', '--speedup', '5', '--defaults', str(params)]
    log = tmp_path / 'gdb.log'
    with log.open('w') as stream:
        process = subprocess.Popen(command, cwd=tmp_path, stdout=stream, stderr=subprocess.STDOUT,
                                   start_new_session=True)
        try:
            process.wait(timeout=45)
        finally:
            with contextlib.suppress(ProcessLookupError):
                os.killpg(process.pid, signal.SIGKILL)
            process.wait()
    output = log.read_text()
    assert process.returncode == 0, output
    assert output.count('apmod: debug.apmod RX=') == 2, output
    assert output.count('apmod: second.apmod RX=') == (2 if module_count == 2 else 0), output
    assert 'apmod: removing breakpoint' in output, output
    assert '"apmod debug"' in output and 'Vec(size=3)' in output, output
    assert '305419896' in output, output
    assert 'debug.rs:' in output and 'debug::run' in output, output
    assert 'AP_Module::Module::run' in output, output
    assert 'lifecycle error' not in output and 'Cannot access memory' not in output, output
