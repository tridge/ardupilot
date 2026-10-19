# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Linker GC, load-time import contracts and malformed container checks."""

import hashlib
import io
import struct
import subprocess
import zlib

from pathlib import Path

import apmod_info
import pack
import pytest

from elftools.elf.elffile import ELFFile

import build


@pytest.mark.parametrize('target,sha256', [
    ('thumbv7em', '63f44aa34990667067e4e5ae2c64c6e32de73430f15ca8de9eead5a36bc651f7'),
    ('x86_64', 'f2864f920f8d59b3aac36ddc7da3321e740f8b6d31e5b00ff84eb1dfdb1b4574'),
])
def test_frozen_probe_identity(target, sha256):
    root = Path(__file__).resolve().parents[3]
    data = bytes.fromhex((root / f'libraries/AP_Module/tests/compat/{target}.apmod.hex').read_text())
    assert hashlib.sha256(data).hexdigest() == sha256
    assert len(pack.unpack(data)['imports']) == 1


@pytest.fixture(params=list(build.TARGETS))
def one_import(tmp_path, request):
    source = tmp_path / 'one.rs'
    source.write_text('''#![no_std]
#![no_main]
use ap_rust_sdk::{Context, Result};
ap_rust_sdk::module!(run);
fn run(context: &Context<'_>) -> Result { let _ = context.arming().is_armed()?; Ok(()) }
#[allow(dead_code)]
fn unused(context: &Context<'_>) -> Result { let _ = context.ahrs().get_gyro()?; Ok(()) }
''')
    first = build.build(source, tmp_path / 'one.apmod', request.param, 8192)
    second = build.build(source, tmp_path / 'two.apmod', request.param, 8192, base=0x24123000)
    assert first.read_bytes() == second.read_bytes()
    return first.read_bytes()


def test_only_live_imports_survive(one_import):
    info = pack.unpack(one_import)
    assert len(info['imports']) == 1
    binding_hash, offset, name = info['imports'][0]
    assert binding_hash != 0 and offset >= info['rx_size']
    assert name == 'arming.is_armed'
    assert info['image'][offset:offset + 4] == b'\xff' * 4


def test_apmod_info(one_import):
    info = pack.unpack(one_import)
    binding_hash, offset, name = info['imports'][0]
    output = apmod_info.format_module(Path('one.apmod'), one_import)
    assert 'Module: one.apmod' in output
    assert 'Format: APRUST05 (version 5)' in output
    assert f'Pointer width: {info["width"] * 8} bits' in output
    assert f'Initialized image: {len(info["image"])} bytes' in output
    assert f'Memory image: {info["memory_size"]} bytes' in output
    assert f'RX image: {info["rx_size"]} bytes' in output
    assert 'Imports: 1' in output
    assert f'{binding_hash:016x}  0x{offset:08x}  {name}' in output
    assert output.endswith('Validation: OK')


@pytest.mark.parametrize('target', list(build.TARGETS))
def test_zero_imports(tmp_path, target):
    source = tmp_path / 'zero.rs'
    source.write_text('''#![no_std]
#![no_main]
use ap_rust_sdk::{Context, Result};
ap_rust_sdk::module!(run);
fn run(_: &Context<'_>) -> Result { Ok(()) }
''')
    image = build.build(source, tmp_path / 'zero.apmod', target, 8192)
    assert pack.unpack(image.read_bytes())['imports'] == []


def crc(data):
    data[76:80] = bytes(4)
    data[76:80] = struct.pack('<I', zlib.crc32(data))
    return data


def test_invalid_import_metadata(one_import):
    info = pack.unpack(one_import)
    records = pack.RUNTIME_HEADER.size + len(info['image']) + len(info['fixups']) * pack.FIXUP.size
    _, slot, _ = info['imports'][0]
    for field, value in [(56, 0), (56, 4097), (56, 0xffffffff),
                         (records + 8, 0), (records + 8, slot + 1),
                         (records + 8, len(info['image'])), (records + 8, 0xfffffffc),
                         (records + 12, 1), (records + 12, 0xffffffff),
                         (pack.RUNTIME_HEADER.size + slot, 0)]:
        bad = bytearray(one_import)
        struct.pack_into('<I', bad, field, value)
        with pytest.raises(ValueError):
            pack.unpack(crc(bad))
    bad = bytearray(one_import)
    bad[records] ^= 1
    with pytest.raises(ValueError, match='checksum'):
        pack.unpack(bad)
    bad = bytearray(one_import)
    bad[records + pack.IMPORT.size] = 0
    with pytest.raises(ValueError, match='import name'):
        pack.unpack(crc(bad))
    with pytest.raises(ValueError):
        pack.unpack(one_import[:-1])
    with pytest.raises(ValueError):
        pack.unpack(one_import + b'\0')


def test_stripped_elf_rejected(one_import, tmp_path):
    objcopy = 'arm-none-eabi-objcopy' if pack.unpack(one_import)['machine'] == 40 else 'objcopy'
    stripped = tmp_path / 'stripped.elf'
    subprocess.run([objcopy, '--strip-all', str(tmp_path / 'one.elf'), str(stripped)], check=True)
    with pytest.raises(ValueError, match='unstripped ELF symbol table required'):
        pack.pack_elf(stripped.read_bytes(), pack.unpack(one_import)['contract'])


def test_import_in_nobits_rejected(one_import, tmp_path):
    data = bytearray((tmp_path / 'one.elf').read_bytes())
    elf = ELFFile(io.BytesIO(data))
    index = next(index for index, section in enumerate(elf.iter_sections()) if section.name == '.data')
    # An import symbol may only identify initialized storage.
    struct.pack_into('<I', data, elf['e_shoff'] + index * elf['e_shentsize'] + 4, 8)
    with pytest.raises(ValueError, match='invalid import symbol'):
        pack.pack_elf(data, pack.unpack(one_import)['contract'])
