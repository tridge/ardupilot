#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build a standalone no_std Rust module with the experimental ArduPilot SDK."""

import argparse
import json
import os
import subprocess
from pathlib import Path

import pack

HERE = Path(__file__).resolve().parent
SDK_ROOT = HERE.parents[1] / 'libraries/AP_Rust'
TARGETS = {
    'thumbv7em-none-eabihf': (1, 'cortex-m4'),
    'x86_64-unknown-linux-gnu': (2, 'x86-64'),
}


def build(source, output, target, stack_size, base=0x10000, cfg=()):
    compiler = os.environ.get('RUSTC', 'rustc')
    version = subprocess.check_output([compiler, '--version'], text=True)
    if not version.startswith('rustc 1.90.0 '):
        raise ValueError(f'expected Rust 1.90.0, got {version.strip()}')
    profile, cpu = TARGETS[target]
    subprocess.run([os.environ.get('PYTHON', 'python3'), str(SDK_ROOT / 'generator/generate.py'), '--check'], check=True)
    contract = json.loads((SDK_ROOT / 'sdk/abi.json').read_text())
    contract.update(cpu_profile=profile, stack_size=stack_size)
    pack.validate_contract(40 if profile == 1 else 62, contract['abi_version'], profile,
                           stack_size, bytes.fromhex(contract['fingerprint']))
    output.parent.mkdir(parents=True, exist_ok=True)
    library = output.with_name(f'lib{output.stem}_sdk.rlib')
    elf = output.with_suffix('.elf')
    common = [compiler, '--edition=2021', '--target', target, '-C', f'target-cpu={cpu}',
              '-C', 'opt-level=s', '-C', 'panic=abort', '-C', 'relocation-model=pic', '-C', 'codegen-units=1']
    subprocess.run(common + [str(SDK_ROOT / 'sdk/src/lib.rs'), '--crate-name', 'ap_rust_sdk', '--crate-type', 'rlib',
                             '-C', 'embed-bitcode=yes', '-o', str(library)], check=True)
    command = common + [str(source), '--extern', f'ap_rust_sdk={library}', '-C', 'lto=fat', '-o', str(elf),
                        '--check-cfg', 'cfg(test)', '--check-cfg', 'cfg(ap_rust_aerobatics_faults)']
    for option in cfg:
        command += ['--cfg', option]
    link_args = [f'-T{HERE / "module.ld"}', '--emit-relocs', f'--defsym=MODULE_BASE={base}']
    if profile == 2:
        command += ['-C', 'link-arg=-nostdlib', '-C', 'link-arg=-static', '-C', 'link-arg=-no-pie']
        link_args = ['-Wl,' + arg for arg in link_args]
    for argument in link_args:
        command += ['-C', f'link-arg={argument}']
    subprocess.run(command, check=True)
    output.write_bytes(pack.pack_elf(elf.read_bytes(), contract))
    return output


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--target', choices=TARGETS, required=True)
    parser.add_argument('--stack-size', type=int, required=True, help='declared module stack requirement in bytes')
    args = parser.parse_args()
    build(args.source, args.output, args.target, args.stack_size)


if __name__ == '__main__':
    main()
