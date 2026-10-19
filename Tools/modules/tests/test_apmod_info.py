# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Account for every stored byte, using only verified ELF section boundaries."""

import io
import subprocess
import sys

from pathlib import Path

import apmod_info
import pack
import pytest

from elftools.elf.elffile import ELFFile
from test_modules import build as build_probe

import build

ROOT = Path(__file__).resolve().parents[3]
TOOL = ROOT / 'Tools/modules/apmod_info.py'


@pytest.fixture(scope='module', params=list(build.TARGETS))
def image(tmp_path_factory, request):
    directory = tmp_path_factory.mktemp('apmod-info')
    path = build.build(ROOT / 'libraries/AP_Module/examples/simple_loop.rs', directory / 'loop.apmod',
                       request.param, 8192)
    return path


def test_exact_section_and_container_breakdown(image):
    data = image.read_bytes()
    elf_data = image.with_suffix('.elf').read_bytes()
    module = pack.unpack(data)
    sizes = apmod_info.file_breakdown(data, module, elf_data)
    elf = ELFFile(io.BytesIO(elf_data))
    assert sizes['Code'] == elf.get_section_by_name('.text')['sh_size']
    assert sizes['Read-only data'] == elf.get_section_by_name('.rodata')['sh_size']
    allocated = sum(section['sh_size'] for section in elf.iter_sections()
                    if section['sh_flags'] & 2 and section['sh_type'] != 'SHT_NOBITS')
    assert sizes['Image padding'] == len(module['image']) - allocated
    assert sizes['Image padding'] > 0
    assert sizes['Container header'] == 80
    assert sizes['Relocation table'] == 4 * len(module['fixups'])
    assert sizes['Import table'] == 16 * len(module['imports'])
    assert sizes['Import names'] == sum(len(name) + 1 for _, _, name in module['imports'])
    assert sum(sizes.values()) == len(data)
    output = apmod_info.format_module(image, data, elf_data=elf_data)
    assert f'  Code: {sizes["Code"]} bytes' in output
    assert f'  Image padding: {sizes["Image padding"]} bytes' in output
    assert f'  Total: {len(data)} bytes' in output


def test_no_elf_does_not_guess_code_or_padding(image):
    data = image.read_bytes()
    sizes = apmod_info.file_breakdown(data, pack.unpack(data))
    assert 'Code' not in sizes and 'Image padding' not in sizes
    assert sum(sizes.values()) == len(data)
    assert 'split unavailable without matching ELF' in apmod_info.format_module(image, data)


def test_wrong_or_invalid_elf_rejected(image):
    data = image.read_bytes()
    elf_data = bytearray(image.with_suffix('.elf').read_bytes())
    elf = ELFFile(io.BytesIO(elf_data))
    elf_data[elf.get_section_by_name('.text')['sh_offset']] ^= 1
    with pytest.raises(ValueError, match='ELF does not match module'):
        apmod_info.format_module(image, data, elf_data=elf_data)
    with pytest.raises(ValueError, match='invalid companion ELF'):
        apmod_info.format_module(image, data, elf_data=b'not ELF')


def test_cli_companion_selection(image, tmp_path):
    def run(*args):
        return subprocess.run([sys.executable, str(TOOL), str(image), *map(str, args)],
                              text=True, capture_output=True)

    automatic = run()
    assert automatic.returncode == 0, automatic.stderr
    assert 'Section sizes from verified ELF:' in automatic.stdout
    without = run('--no-elf')
    assert without.returncode == 0, without.stderr
    assert 'split unavailable without matching ELF' in without.stdout
    explicit = run('--elf', image.with_suffix('.elf'))
    assert explicit.returncode == 0 and explicit.stdout == automatic.stdout
    missing = run('--elf', tmp_path / 'missing.elf')
    assert missing.returncode == 1
    assert 'missing.elf' in missing.stderr


def test_cli_without_companion_and_with_stale_companion(image, tmp_path):
    module = tmp_path / 'copy.apmod'
    module.write_bytes(image.read_bytes())
    command = [sys.executable, str(TOOL), str(module)]
    missing = subprocess.run(command, text=True, capture_output=True)
    assert missing.returncode == 0
    assert 'split unavailable without matching ELF' in missing.stdout
    elf_data = bytearray(image.with_suffix('.elf').read_bytes())
    elf = ELFFile(io.BytesIO(elf_data))
    elf_data[elf.get_section_by_name('.text')['sh_offset']] ^= 1
    module.with_suffix('.elf').write_bytes(elf_data)
    stale = subprocess.run(command, text=True, capture_output=True)
    assert stale.returncode == 1
    assert 'ELF does not match module' in stale.stderr
    ignored = subprocess.run(command + ['--no-elf'], text=True, capture_output=True)
    assert ignored.returncode == 0


@pytest.mark.parametrize('target', list(build.TARGETS))
def test_relocation_probe_breakdown(tmp_path, target):
    elf_data = build_probe(tmp_path, target)
    data = pack.pack_elf(elf_data)
    sizes = apmod_info.file_breakdown(data, pack.unpack(data), elf_data)
    assert sizes['Container header'] == 56
    assert sizes['Import table'] == sizes['Import names'] == 0
    assert sum(sizes.values()) == len(data)


@pytest.mark.parametrize('target', ['thumbv7em', 'x86_64'])
def test_frozen_module_container_breakdown(target):
    data = bytes.fromhex((ROOT / f'libraries/AP_Module/tests/compat/{target}.apmod.hex').read_text())
    sizes = apmod_info.file_breakdown(data, pack.unpack(data))
    assert sum(sizes.values()) == len(data)
    assert 'split unavailable' in apmod_info.format_module(Path('old.apmod'), data)
