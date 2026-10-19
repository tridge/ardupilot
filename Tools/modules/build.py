#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build a standalone no_std Rust module with the experimental ArduPilot SDK."""

import argparse
import json
import os
import re
import subprocess

from pathlib import Path

import pack

HERE = Path(__file__).resolve().parent
MIN_RUST_VERSION = (HERE / 'rust-toolchain-version').read_text().strip()
SDK_ROOT = HERE.parents[1] / 'libraries/AP_Module'
GENERATED_ROOT = HERE.parents[1] / 'build/modules/libraries/AP_Module'
TARGETS = {
    'thumbv7em-none-eabihf': (1, 'cortex-m4'),
    'x86_64-unknown-linux-gnu': (2, 'x86-64'),
}


def parse_rust_version(output):
    match = re.match(r'^rustc ([0-9]+)\.([0-9]+)\.([0-9]+)(?:[-+][^ ]+)?(?: |$)', output)
    if match is None:
        raise ValueError(f'unrecognised rustc version: {output.strip()}')
    return tuple(int(component) for component in match.groups())


def check_rust_version(output):
    minimum = tuple(int(component) for component in MIN_RUST_VERSION.split('.'))
    actual = parse_rust_version(output)
    if actual < minimum:
        raise ValueError(f'requires Rust {MIN_RUST_VERSION} or newer, got {output.strip()}')


def generate_bindings():
    subprocess.run([
        os.environ.get('PYTHON', 'python3'),
        str(SDK_ROOT / 'generator/generate.py'),
        '--output-dir', str(GENERATED_ROOT),
    ], check=True)
    return GENERATED_ROOT


def sdk_environment():
    generated = generate_bindings()
    return dict(os.environ, AP_MODULE_SDK_GENERATED=str(generated / 'sdk/src'))


def build(source, output, target, stack_size, base=0x10000, cfg=()):
    compiler = os.environ.get('RUSTC', 'rustc')
    version = subprocess.check_output([compiler, '--version'], text=True)
    check_rust_version(version)
    profile, cpu = TARGETS[target]
    environment = sdk_environment()
    contract = json.loads((GENERATED_ROOT / 'sdk/abi.json').read_text())
    contract.update(cpu_profile=profile, stack_size=stack_size)
    pack.validate_contract(40 if profile == 1 else 62, contract['abi_version'], profile,
                           stack_size, bytes.fromhex(contract['fingerprint']))
    output.parent.mkdir(parents=True, exist_ok=True)
    library = output.with_name(f'lib{output.stem}_sdk.rlib')
    elf = output.with_suffix('.elf')
    relocation = 'ropi-rwpi' if profile == 1 else 'pic'
    common = [compiler, '--edition=2021', '--target', target, '-C', f'target-cpu={cpu}',
              '-C', 'opt-level=s', '-C', 'panic=abort', '-C', f'relocation-model={relocation}',
              '-C', 'codegen-units=1']
    subprocess.run(common + [str(SDK_ROOT / 'sdk/src/lib.rs'), '--crate-name', 'ap_rust_sdk', '--crate-type', 'rlib',
                             '-C', 'embed-bitcode=yes', '-o', str(library)], check=True, env=environment)
    command = common + [str(source), '--extern', f'ap_rust_sdk={library}', '-C', 'lto=fat', '-o', str(elf),
                        '--check-cfg', 'cfg(test)', '--check-cfg', 'cfg(ap_rust_aerobatics_faults)']
    for option in cfg:
        command += ['--cfg', option]
    link_args = [f'-T{HERE / "module.ld"}', '--emit-relocs', f'--defsym=MODULE_BASE={base}']
    if profile == 1:
        link_args += ['--defsym=MODULE_DATA_ALIGNMENT=4']
    if profile == 2:
        command += ['-C', 'link-arg=-nostdlib', '-C', 'link-arg=-static', '-C', 'link-arg=-no-pie']
        link_args = ['-Wl,' + arg for arg in link_args]
    for argument in link_args:
        command += ['-C', f'link-arg={argument}']
    subprocess.run(command, check=True, env=environment)
    if profile == 2:
        # Some prebuilt compiler-builtins retain this undefined symbol after
        # their unwind sections are discarded. Strip it only if unreferenced;
        # objcopy and the packer still reject actual unresolved dependencies.
        subprocess.run([os.environ.get('OBJCOPY', 'objcopy'),
                        '--strip-symbol=rust_eh_personality', str(elf)], check=True, env=environment)
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
