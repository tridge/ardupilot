#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build a trusted native module from Rust or freestanding C++ source."""

import argparse
import json
import os
import subprocess
import warnings

from pathlib import Path

import pack

HERE = Path(__file__).resolve().parent
SDK_ROOT = HERE.parents[1] / 'libraries/AP_Module'
GENERATED_ROOT = HERE.parents[1] / 'build/modules/libraries/AP_Module'
TARGETS = {
    'thumbv7em-none-eabihf': (1, 'cortex-m4'),
    'x86_64-unknown-linux-gnu': (2, 'x86-64'),
}


def generate_bindings():
    subprocess.run([os.environ.get('PYTHON', 'python3'), str(SDK_ROOT / 'generator/generate.py'),
                    '--output-dir', str(GENERATED_ROOT)], check=True)
    return GENERATED_ROOT


def build(source, output, target, stack_size, base=0x10000, cfg=(), debug=False, opt_level='s', heap_size=4096):
    source, output = Path(source), Path(output)
    profile, _ = TARGETS[target]
    generate_bindings()
    contract = json.loads((GENERATED_ROOT / 'sdk/abi.json').read_text())
    contract.update(cpu_profile=profile, stack_size=stack_size, heap_size=heap_size)
    pack.validate_contract(40 if profile == 1 else 62, contract['abi_version'], profile,
                           stack_size, bytes.fromhex(contract['fingerprint']), heap_size)
    if heap_size > 32768:
        warnings.warn('initial heap exceeds the default MOD_HEAP_MAX=32768; configure firmware limits', stacklevel=2)
    if source.suffix == '.rs':
        from rust import build as backend
    elif source.suffix in ('.cpp', '.cc', '.cxx'):
        if cfg:
            raise ValueError('Rust cfg options are not supported for C++ modules')
        from cpp import build as backend
    else:
        raise ValueError('expected .rs, .cpp, .cc or .cxx module source')
    return backend.build(source, output, target, stack_size, base=base, cfg=cfg,
                         debug=debug, opt_level=opt_level, heap_size=heap_size)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('source', type=Path)
    parser.add_argument('output', type=Path)
    parser.add_argument('--target', choices=TARGETS, required=True)
    parser.add_argument('--stack-size', type=int, required=True, help='declared module stack requirement in bytes')
    parser.add_argument('--heap-size', type=int, required=True, help='initial module heap, capped by MOD_HEAP_MAX')
    parser.add_argument('--debug', action='store_true', help='retain DWARF and frame pointers in companion ELF')
    parser.add_argument('--opt-level', choices=('0', '1', '2', '3', 's', 'z'), default='s')
    args = parser.parse_args()
    build(args.source, args.output, args.target, args.stack_size,
          debug=args.debug, opt_level=args.opt_level, heap_size=args.heap_size)


if __name__ == '__main__':
    main()
