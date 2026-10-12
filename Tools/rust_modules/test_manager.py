# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Run the native module manager on ChibiOS, including a spinning worker."""

import os
import re
import shutil
import subprocess

import pytest
from elftools.elf.elffile import ELFFile

import build
from test_threads import ROOT, gen_board, renode, renode_data, run  # noqa: F401


@pytest.mark.parametrize('ending', ['never', 'panic'])
def test_chibios_runtime_manager(tmp_path, renode, monkeypatch, ending):  # noqa: F811
    board = os.environ.get('OWNED_THREAD_BOARD', 'MatekH743')
    executable = ROOT / 'build' / board / 'examples/RustRuntime'
    if not executable.is_file():
        pytest.skip('Build examples/RustRuntime with --enable-rust-scripting')
    if not shutil.which('mcopy') or not shutil.which('mmd'):
        pytest.skip('Requires mtools')
    names = ['result', 'cycles', 'before', 'after', 'stack_free', 'stack_total',
             'manager_priority', 'worker_priority', 'manager_stack', 'worker_stack', 'timeouts', 'panics',
             'timeout_elapsed']
    with executable.open('rb') as stream:
        symbols = ELFFile(stream).get_section_by_name('.symtab')
        addresses = [symbols.get_symbol_by_name(f'rust_runtime_test_{name}')[0]['st_value'] for name in names]
        error_address = symbols.get_symbol_by_name('rust_runtime_test_error')[0]['st_value']
        addresses += list(range(error_address, error_address + 100, 4))
        memory_address = symbols.get_symbol_by_name('rust_runtime_test_memory')[0]['st_value']
        addresses += list(range(memory_address, memory_address + 256, 4))
    generated = gen_board.generate(ROOT, board, tmp_path / 'generated', uart_port=0, state_dir=tmp_path)
    data = renode_data.ensure_mcu_data(generated['mcu_type'])
    sdcard = run.make_fat_image(tmp_path / 'sdcard.img', 16 * 1024 * 1024, fat_bits=16)
    subprocess.run(['mmd', '-i', str(sdcard), '::/fixtures'], check=True)
    for name, mode in [('one', 'cleanup'), ('two', 'parameters'), ('end', ending)]:
        monkeypatch.setenv('AP_RUST_LIFECYCLE', mode)
        monkeypatch.setenv('AP_RUST_PARAM_MODE', 'renode')
        output = tmp_path / f'{name}.apmod'
        source = 'parameters.rs' if mode == 'parameters' else 'lifecycle.rs'
        build.build(ROOT / 'libraries/AP_Rust/tests/modules' / source, output,
                    'thumbv7em-none-eabihf', 8192)
        subprocess.run(['mcopy', '-i', str(sdcard), str(output), f'::/fixtures/{name}.apmod'], check=True)
    output = renode('''
$elf=@%s
$mcu_svd=@%s
$sdcard=@%s
$vector_base=0x%X
include @%s
emulation SetGlobalAdvanceImmediately true
emulation RunFor "40"
%s
''' % (executable.resolve(), data['svd'], sdcard, generated['app_base'], generated['resc'],
       '\n'.join(f'sysbus ReadDoubleWord 0x{address:X}' for address in addresses)))
    values = [int(value, 16) for value in re.findall(r'^0x[0-9A-Fa-f]{8}$', output, re.M)]
    (tmp_path / 'renode.log').write_text(output)
    assert len(values) == len(names) + 25 + 64, output
    result = dict(zip(names, values[:len(names)]))
    error_words = values[len(names):len(names)+25]
    result['error'] = b''.join(value.to_bytes(4, 'little') for value in error_words).split(b'\0')[0].decode()
    memory = b''.join(value.to_bytes(4, 'little') for value in values[len(names)+25:]).split(b'\0')[0].decode()
    print(f'{board} {ending}: {result}', flush=True)
    print(memory, flush=True)
    assert result['result'] == 0x600DF00D, (result, output)
    assert result['cycles'] == 20 and result['before'] == result['after'], result
    assert result['stack_free'] > 4096 and result['stack_total'] >= 16384, result
    assert result['manager_priority'] > result['worker_priority'], result
    fast_regions = {'STM32F405xx': (0x10000000, 0x10010000), 'STM32H743xx': (0x20000000, 0x20020000)}
    low, high = fast_regions[generated['mcu_type']]
    assert low <= result['manager_stack'] < high and low <= result['worker_stack'] < high, result
    assert result['timeouts'] == 1 and result['panics'] == int(ending == 'panic'), result
    assert 10000 <= result['timeout_elapsed'] < 12000, result
