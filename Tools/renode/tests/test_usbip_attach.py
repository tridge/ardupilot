#!/usr/bin/env python3

# AP_FLAKE8_CLEAN

"""Tests for the Renode USB/IP attachment helper."""

import argparse
import importlib.util

from pathlib import Path
from unittest import mock

MODULE_PATH = Path(__file__).resolve().parents[1] / 'usbip_attach.py'
SPEC = importlib.util.spec_from_file_location('usbip_attach', MODULE_PATH)
assert SPEC is not None and SPEC.loader is not None
usbip_attach = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(usbip_attach)


def test_signal_detaches_active_port(tmp_path):
    args = argparse.Namespace(host='127.0.0.1', port=3240, busid='1-0',
                              install_rules=False)
    detached = []

    def request_stop(_delay):
        usbip_attach.request_stop(None, None)

    with mock.patch.object(usbip_attach, 'parse_args', return_value=args), \
            mock.patch.object(usbip_attach.signal, 'signal'), \
            mock.patch.object(usbip_attach, 'VHCI', tmp_path), \
            mock.patch.object(usbip_attach.os, 'access', return_value=True), \
            mock.patch.object(usbip_attach, 'attach', return_value=(4, '')), \
            mock.patch.object(usbip_attach, 'port_attached', return_value=True), \
            mock.patch.object(usbip_attach, 'detach', side_effect=detached.append), \
            mock.patch.object(usbip_attach.time, 'sleep', side_effect=request_stop):
        usbip_attach.main()

    assert detached == [4]


def test_malformed_vhci_status_port_is_ignored(tmp_path):
    status = tmp_path / 'status'
    status.write_text('hub port sta spd dev socket busid\n'
                      'hs not-a-port 006 00 0 0 1-0\n')
    with mock.patch.object(usbip_attach, 'VHCI', tmp_path):
        assert not usbip_attach.port_attached(0)
