# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Exercise parameter rejection, dynamic HTTP files and concurrent SYS snapshots."""

import http.client
import socket
import time

from pymavlink import mavutil
from test_runtime import ROOT
from test_runtime import Vehicle
from test_thread_info import FTPReader

import build


def test_lockdown_rejects_invalid_value_and_keeps_running(tmp_path):
    image = build.build(ROOT / 'libraries/AP_Module/applets/param_lockdown/main.rs',
                        tmp_path / 'lock.apmod', 'x86_64-unknown-linux-gnu', 8192)
    vehicle = Vehicle(tmp_path / 'vehicle', {'lock.apmod': image.read_bytes()}, {'SCR_ENABLE': 0})
    try:
        vehicle.start()
        vehicle.text('param-lockdown script loaded')
        vehicle.pump(0.2)
        vehicle.link.mav.param_set_send(1, 1, b'LOG_DISARMED', 200, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        reply = vehicle.until(lambda msg: msg.get_type() == 'PARAM_ERROR' and msg.param_id == 'LOG_DISARMED')
        assert reply.error == mavutil.mavlink.MAV_PARAM_ERROR_VALUE_OUT_OF_RANGE
        before = len(vehicle.texts)
        vehicle.link.mav.param_set_send(1, 1, b'LOG_DISARMED', 1, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        vehicle.text('param set applied', after=before)
        assert vehicle.parameter('LOG_DISARMED') == 1
        before = len(vehicle.texts)
        vehicle.link.mav.param_set_send(1, 1, b'PARAM_LOCK_ENAB', 0, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
        vehicle.text('param set applied', after=before)
        vehicle.pump(0.2)
        assert vehicle.parameter('ARMING_REQUIRE', 0) == 0
        assert not any('returned -' in text for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()


def test_webserver_dynamic_sys_files(tmp_path):
    with socket.socket() as available:
        available.bind(('127.0.0.1', 0))
        port = available.getsockname()[1]
    image = build.build(ROOT / 'libraries/AP_Module/applets/net_webserver/main.rs',
                        tmp_path / 'web.apmod', 'x86_64-unknown-linux-gnu', 16384, heap_size=65536)
    vehicle = Vehicle(tmp_path / 'vehicle', {'web.apmod': image.read_bytes()},
                      {'SCR_ENABLE': 0, 'MOD_HEAP_MAX': 65536})
    (vehicle.directory / 'example.txt').write_bytes(b'ordinary file\n')

    def get(path):
        deadline = time.monotonic() + 10
        while True:
            client = http.client.HTTPConnection('127.0.0.1', port, timeout=5)
            try:
                client.request('GET', path)
                response = client.getresponse()
                return response.status, response.getheader('Content-Length'), response.read()
            except ConnectionRefusedError:
                if time.monotonic() >= deadline:
                    raise
                vehicle.pump(0.1)
            finally:
                client.close()

    try:
        vehicle.start()
        vehicle.text('WebServer: starting on port 8080')
        assert vehicle.parameter('WEB_BIND_PORT', port) == port
        vehicle.restart(1)
        vehicle.text(f'WebServer: starting on port {port}')
        status, _, page = get('/')
        assert status == 200 and b'update_status()' in page
        status, _, listing = get('/mnt/@SYS/')
        assert status == 200 and b'modules.txt' in listing
        status, length, contents = get('/mnt/@SYS/modules.txt')
        assert status == 200 and length is None
        assert contents.startswith(b'ModulesV1\n') and b'web.apmod' in contents
        status, length, contents = get('/mnt/example.txt')
        assert status == 200 and int(length) == len(contents)
        assert contents == b'ordinary file\n'
    finally:
        vehicle.stop()


def test_concurrent_native_and_ftp_sys_snapshots(tmp_path):
    source = tmp_path / 'sys.rs'
    source.write_text('''#![no_std]
#![no_main]
use ap_rust_sdk::{Context, Error, Result};
ap_rust_sdk::module!(run);
fn run(context: &Context<'_>) -> Result {
    let mut bytes = [0u8; 256];
    let mut ready = false;
    while !context.stop_requested() {
        let length = context.fs().read_at("@SYS/modules.txt", 0, &mut bytes)?.ok_or(Error::Host)?;
        if !bytes[..length].starts_with(b"ModulesV1\\n") { return Err(Error::Host); }
        if !ready { context.text(6, "SYS ready")?; ready = true; }
        context.sleep_ms(10)?;
    }
    Ok(())
}
''')
    image = build.build(source, tmp_path / 'sys.apmod', 'x86_64-unknown-linux-gnu', 8192).read_bytes()
    vehicle = Vehicle(tmp_path / 'vehicle', {f'sys{i}.apmod': image for i in range(3)}, {'SCR_ENABLE': 0})
    try:
        vehicle.start()
        vehicle.until(lambda _: sum('SYS ready' in text for text in vehicle.texts) == 3)
        reader = FTPReader(vehicle)
        for _ in range(50):
            assert reader.read('@SYS/modules.txt').startswith('ModulesV1\n')
        assert not any('returned -' in text for text in vehicle.texts), vehicle.diagnostic()
    finally:
        vehicle.stop()
