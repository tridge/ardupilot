#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate and display metadata from an ArduPilot Rust module."""

import argparse
import struct

from pathlib import Path

import pack

MACHINES = {
    40: 'ARM',
    62: 'x86-64',
}

PROFILES = {
    (40, 1): 'thumbv7em-none-eabihf',
    (62, 2): 'x86_64-unknown-linux-gnu',
}


def format_module(path, data, show_fixups=False):
    module = pack.unpack(data)
    magic = data[:8].decode('ascii')
    version = struct.unpack_from('<I', data, 8)[0]
    contract = module['contract']
    machine = module['machine']
    width = module['width']
    image_size = len(module['image'])
    memory_size = module['memory_size']
    rx_size = module['rx_size']
    rw_file_size = image_size - rx_size
    rw_memory_size = memory_size - rx_size
    rx_fixups = sum(offset < rx_size for offset, _ in module['fixups'])

    lines = [
        f'Module: {path}',
        f'Format: {magic} (version {version})',
        f'File size: {len(data)} bytes',
        f'Machine: {MACHINES.get(machine, "unknown")} ({machine})',
        f'Pointer width: {width * 8} bits',
    ]
    if contract is not None:
        profile = contract['cpu_profile']
        lines += [
            f'Target profile: {PROFILES.get((machine, profile), "unknown")} ({profile})',
            f'ABI version: {contract["abi_version"]}',
            f'API fingerprint: {contract["fingerprint"]}',
            f'Stack requirement: {contract["stack_size"]} bytes',
        ]
    else:
        lines.append('Runtime contract: none (relocation probe)')
    lines += [
        f'Initialized image: {image_size} bytes',
        f'Memory image: {memory_size} bytes ({memory_size - image_size} bytes zero-fill)',
        f'RX image: {rx_size} bytes',
        f'RW image: {rw_file_size} initialized, {rw_memory_size} memory '
        f'({rw_memory_size - rw_file_size} bytes zero-fill)',
        f'RX boundary: 0x{rx_size:08x}',
        f'RX alignment: {module["rx_alignment"]} bytes',
        f'RW alignment: {module["rw_alignment"]} bytes',
        f'Entry offset: 0x{module["entry"]:08x}',
        f'Relocations: {len(module["fixups"])} '
        f'({rx_fixups} RX, {len(module["fixups"]) - rx_fixups} RW)',
    ]
    if show_fixups and module['fixups']:
        lines.append('  SEG  OFFSET      TARGET')
        lines += [f'  {"RX" if offset < rx_size else "RW"}   0x{offset:08x}  0x{target:08x}'
                  for offset, target in module['fixups']]

    lines.append(f'Imports: {len(module["imports"])}')
    if module['imports']:
        lines.append('  HASH              SLOT        API')
        for binding_hash, offset, name in module['imports']:
            diagnostic_name = name if name is not None else '<not stored in this format>'
            lines.append(f'  {binding_hash:016x}  0x{offset:08x}  {diagnostic_name}')
    lines.append('Validation: OK')
    return '\n'.join(lines)


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('module', type=Path)
    parser.add_argument('--fixups', action='store_true', help='list relocation offsets and targets')
    args = parser.parse_args()
    try:
        data = args.module.read_bytes()
        output = format_module(args.module, data, args.fixups)
    except (OSError, ValueError) as error:
        parser.exit(1, f'{parser.prog}: {args.module}: {error}\n')
    print(output)


if __name__ == '__main__':
    main()
