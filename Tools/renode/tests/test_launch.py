#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Tests for the ArduPilot Renode launcher."""

import hashlib
import importlib.util
import io
import os
import socket
import tarfile

from pathlib import Path
from types import SimpleNamespace

import pytest

MODULE_PATH = Path(__file__).resolve().parents[1] / 'launch.py'
SPEC = importlib.util.spec_from_file_location('renode_launch', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
launch = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(launch)


def latest_metadata(package):
    return {
        'schema_version': 1,
        'renode_version': '1.16.1',
        'source': {'revision': '2a060779f4e2b87d1ae7238a041d858369818805'},
        'artifacts': [{
            'target': {
                'platform': 'linux',
                'architecture': 'x86_64',
                'runtime_identifier': 'linux-x64',
            },
            'packages': [package],
        }],
    }


def test_parse_metrics():
    metrics = launch.parse_metrics(
        'cpu PC; cpu PerformanceInMips; cpu ExecutedInstructions; '
        'emulation GetTimeSourceInfo\n'
        '0x08164B9C\n'
        '0x0000012C\n'
        '0x000000007F1BCE89\n'
        'Elapsed Virtual Time: 00:01:30.250000000\n'
        'Elapsed Host Time: 00:02:00.500000000\n'
        'Current load: 1.0\n'
        '(ardupilot) ')

    assert metrics == {
        'pc': 0x08164B9C,
        'mips': 300,
        'instructions': 0x7F1BCE89,
        'virtual_seconds': 90.25,
        'host_seconds': 120.5,
    }


def test_build_command_contains_selected_options():
    args = SimpleNamespace(
        monitor_port=12390,
        uart_port=5762,
        usbip_port=3240,
        renode='/opt/renode/renode',
        state_dir='/tmp/ardupilot-renode-launch-test',
    )
    launcher = launch.Launcher(args)
    launcher.board = 'CubeOrangePlus'
    launcher.firmware = str(
        launch.ROOT / 'build' / 'CubeOrangePlus' / 'bin' / 'arducopter')
    launcher.bootloader = 'none'
    launcher.cpu = min(os.sched_getaffinity(0))
    launcher.real_iomcu = True
    launcher.iomcu_force_update = True
    launcher.usb = True
    launcher.can = True
    launcher.can_base = 4
    launcher.ethernet = 'lo'

    command = launcher.build_command()

    assert command[:3] == [
        os.sys.executable,
        str(launch.HERE / 'run.py'),
        'CubeOrangePlus',
    ]
    assert command[command.index('--cpusel') + 1] == str(launcher.cpu)
    assert '--real-iomcu' in command
    assert '--iomcu-force-update' in command
    assert '--usb' in command
    assert command[command.index('--usbip-port') + 1] == '3240'
    assert '--can' in command
    assert command[command.index('--can-base') + 1] == '4'
    assert command[command.index('--ethernet-tap') + 1] == 'lo'


def test_force_iomcu_update_requires_real_iomcu():
    args = SimpleNamespace(
        monitor_port=12390,
        uart_port=5762,
        usbip_port=3240,
        renode=None,
        state_dir=None,
    )
    launcher = launch.Launcher(args)
    launcher.board = 'CubeOrangePlus'
    launcher.bootloader = 'none'
    launcher.iomcu_force_update = True

    with pytest.raises(ValueError, match='requires real IOMCU'):
        launcher.build_command()


def test_wait_port_free_ignores_closed_connection():
    server = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
    server.bind(('127.0.0.1', 0))
    port = server.getsockname()[1]
    server.listen()

    client = socket.create_connection(('127.0.0.1', port))
    connection, _address = server.accept()
    try:
        assert not launch.Launcher.wait_port_free(port, timeout=0.01)
    finally:
        # Closing the server side first leaves its local port in TIME_WAIT on
        # Linux.  There is no listener, so this must not prevent a restart.
        connection.close()
        client.recv(1)
        client.close()
        server.close()
    assert launch.Launcher.wait_port_free(port, timeout=0.01)


def test_select_linux_renode_package():
    package = {
        'filename': 'renode-test.linux-portable-dotnet.tar.gz',
        'sha256': 'a' * 64,
        'size': 123,
    }

    selected = launch.select_renode_package(
        latest_metadata(package), system='linux', machine='amd64')

    assert selected['filename'] == package['filename']
    assert selected['runtime_identifier'] == 'linux-x64'


def test_download_cache_is_reused_only_for_current_version(tmp_path):
    executable_data = b'#!/bin/sh\nexit 0\n'
    archive_stream = io.BytesIO()
    with tarfile.open(fileobj=archive_stream, mode='w:gz') as bundle:
        info = tarfile.TarInfo('renode-test/renode')
        info.mode = 0o755
        info.size = len(executable_data)
        bundle.addfile(info, io.BytesIO(executable_data))
    archive = archive_stream.getvalue()
    package = {
        'filename': 'renode-test.linux-portable-dotnet.tar.gz',
        'sha256': hashlib.sha256(archive).hexdigest(),
        'size': len(archive),
    }
    latest = latest_metadata(package)

    def open_archive(_request, timeout):
        assert timeout == 60
        return io.BytesIO(archive)

    executable, _metadata, downloaded = launch.install_current_renode(
        tmp_path, latest=latest, opener=open_archive)

    assert downloaded
    assert executable.read_bytes() == executable_data

    def no_download(_request, _timeout):
        raise AssertionError('current cache should not be downloaded again')

    cached, _metadata, downloaded = launch.install_current_renode(
        tmp_path, latest=latest, opener=no_download)
    assert not downloaded
    assert cached == executable

    newer = latest_metadata(package)
    newer['source']['revision'] = '3' * 40
    selected = launch.select_renode_package(
        newer, system='linux', machine='x86_64')
    assert launch.cached_renode(tmp_path, newer, selected) is None
