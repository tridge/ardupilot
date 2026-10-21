#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Validate and display metadata from an ArduPilot Native module."""

import argparse
import io
import struct

from pathlib import Path

import pack

from elftools.common.exceptions import ELFError
from elftools.elf.elffile import ELFFile

MACHINES = {
    40: 'ARM',
    62: 'x86-64',
}

PROFILES = {
    (40, 1): 'thumbv7em-none-eabihf',
    (62, 2): 'x86_64-unknown-linux-gnu',
}


def file_breakdown(data, module, elf_data=None):
    runtime = module['contract'] is not None
    compact = data[:8] == pack.RUNTIME_MAGIC
    sizes = {'Container header': pack.RUNTIME_HEADER.size if runtime else pack.HEADER.size}
    if elf_data is None:
        sizes['Image (code/data/padding)'] = len(module['image'])
    else:
        try:
            rebuilt = pack.pack_elf(elf_data, module['contract'])
        except ELFError as error:
            raise ValueError(f'invalid companion ELF: {error}') from error
        if rebuilt != data:
            raise ValueError('companion ELF does not match module; use the matching ELF or --no-elf')
        elf = ELFFile(io.BytesIO(elf_data))
        sizes.update({'Code': 0, 'Read-only data': 0, 'Read-only metadata': 0, 'Initialized RW data': 0})
        for section in elf.iter_sections():
            if not section['sh_flags'] & 2 or section['sh_type'] == 'SHT_NOBITS':
                continue
            if section['sh_flags'] & 4:
                category = 'Code'
            elif section['sh_flags'] & 1:
                category = 'Initialized RW data'
            elif section.name == '.ARM.exidx':
                category = 'Read-only metadata'
            else:
                category = 'Read-only data'
            sizes[category] += section['sh_size']
        section_size = sum(value for key, value in sizes.items() if key != 'Container header')
        sizes['Image padding'] = len(module['image']) - section_size
    sizes['Relocation table'] = len(module['fixups']) * (pack.FIXUP.size if compact else pack.LEGACY_FIXUP.size)
    sizes['Import table'] = len(module['imports']) * pack.IMPORT.size
    sizes['Import names'] = len(data) - sum(sizes.values())
    return sizes


def format_module(path, data, show_fixups=False, elf_data=None, elf_path=None):
    module = pack.unpack(data)
    magic = data[:8].rstrip(b'\0').decode('ascii')
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
            f'Initial heap: {contract["heap_size"]} bytes',
        ]
    else:
        lines.append('Runtime contract: none (relocation probe)')
    lines.append('File breakdown:')
    if elf_data is not None:
        lines.append(f'  Section sizes from verified ELF: {elf_path or "<provided>"}')
    else:
        lines.append('  Code/data/padding split unavailable without matching ELF.')
    lines += [f'  {label}: {size} bytes' for label, size in file_breakdown(data, module, elf_data).items()]
    lines.append(f'  Total: {len(data)} bytes')
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
    elf_options = parser.add_mutually_exclusive_group()
    elf_options.add_argument('--elf', type=Path, help='matching unstripped ELF (default: MODULE with .elf suffix)')
    elf_options.add_argument('--no-elf', action='store_true', help='inspect only the module, ignoring any companion ELF')
    args = parser.parse_args()
    try:
        data = args.module.read_bytes()
        elf_path = args.elf or args.module.with_suffix('.elf')
        elf_data = None
        if not args.no_elf and (args.elf is not None or elf_path.is_file()):
            elf_data = elf_path.read_bytes()
        output = format_module(args.module, data, args.fixups, elf_data, elf_path)
    except (OSError, ValueError) as error:
        parser.exit(1, f'{parser.prog}: {args.module}: {error}\n')
    print(output)


if __name__ == '__main__':
    main()
