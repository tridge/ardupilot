# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Vehicle-backed SITL checks for Rust loading, restart and Lua-style arming."""

import os
import socket
import subprocess
import time
import zlib
from pathlib import Path

import pytest
from pymavlink import mavutil

import build

ROOT = Path(__file__).resolve().parents[2]
SYSTEM_CHECK = 1 << 13
CHECKSUM_MASK = 0x007FFFFF


@pytest.fixture(scope='module')
def images(tmp_path_factory):
    directory = tmp_path_factory.mktemp('runtime-modules')
    result = {}
    for mode in ('cooperative', 'cleanup', 'return', 'panic', 'late_stop', 'flood'):
        previous = os.environ.get('AP_RUST_LIFECYCLE')
        os.environ['AP_RUST_LIFECYCLE'] = mode
        try:
            path = build.build(ROOT / 'libraries/AP_Rust/tests/modules/lifecycle.rs', directory / f'{mode}.apmod',
                               'x86_64-unknown-linux-gnu', 8192)
        finally:
            if previous is None:
                del os.environ['AP_RUST_LIFECYCLE']
            else:
                os.environ['AP_RUST_LIFECYCLE'] = previous
        result[mode] = path.read_bytes()
    return result


class Vehicle:
    def __init__(self, directory, files, extra, model='plane'):
        self.directory = directory
        self.directory.mkdir()
        self.modules = directory / 'rust'
        if files is not None:
            self.modules.mkdir()
            for name, data in files.items():
                (self.modules / name).write_bytes(data)
        settings = {'RUST_ENABLE': 1, 'RUST_HEAP_SIZE': 4096, 'RUST_STACK_SIZE': 65536,
                    'ARMING_REQUIRE': 1, 'LOG_BACKEND_TYPE': 0}
        settings.update(extra)
        self.defaults = directory / 'test.parm'
        self.defaults.write_text(''.join(f'{name} {value}\n' for name, value in settings.items()))
        self.process = None
        self.link = None
        self.log = None
        self.texts = []
        self.last_heartbeat = 0
        self.model = model

    def start(self):
        with socket.socket() as available:
            available.bind(('127.0.0.1', 0))
            port = available.getsockname()[1]
        binary = Path(os.environ.get('AP_RUST_SITL_BINARY', ROOT / 'build/sitl/bin/arduplane'))
        if not binary.is_file():
            pytest.skip('Build SITL Plane with AP_SCRIPTING_RUST_ENABLED=1')
        self.log = (self.directory / 'sitl.log').open('ab')
        self.process = subprocess.Popen([str(binary.resolve()), '--model', self.model, '--speedup', '5',
                                         '--base-port', str(port), '-I', '0', '--home', '-35.363261,149.165230,584,353',
                                         '--serial1', 'none', '--serial2', 'none',
                                         '--defaults', str(self.defaults)], cwd=self.directory,
                                        stdout=self.log, stderr=subprocess.STDOUT)
        deadline = time.monotonic() + 20
        while time.monotonic() < deadline:
            self.check_alive()
            try:
                self.link = mavutil.mavlink_connection(f'tcp:127.0.0.1:{port}', source_system=255)
                break
            except (ConnectionError, OSError):
                time.sleep(0.05)
        assert self.link is not None, self.diagnostic()
        self.until(lambda message: message.get_type() == 'HEARTBEAT', 20)
        assert self.parameter('RUST_ENABLE') in (0, 1)
        self.pump(5)  # let simulated home, sensors and startup calibration settle

    def stop(self):
        if self.link is not None:
            self.link.close()
            self.link = None
        if self.process is not None:
            self.process.terminate()
            try:
                self.process.wait(timeout=5)
            except subprocess.TimeoutExpired:
                self.process.kill()
                self.process.wait(timeout=5)
            self.process = None
        if self.log is not None:
            self.log.close()
            self.log = None

    def diagnostic(self):
        return '\n'.join(self.texts[-30:]) + '\n' + (self.directory / 'sitl.log').read_text(errors='replace')[-5000:]

    def check_alive(self):
        assert self.process.poll() is None, self.diagnostic()

    def receive(self):
        self.check_alive()
        if time.monotonic() - self.last_heartbeat > 1:
            self.link.mav.heartbeat_send(mavutil.mavlink.MAV_TYPE_GCS, mavutil.mavlink.MAV_AUTOPILOT_INVALID,
                                         0, 0, 0)
            self.last_heartbeat = time.monotonic()
        message = self.link.recv_match(blocking=True, timeout=0.1)
        if message is not None and message.get_type() == 'STATUSTEXT':
            self.texts.append(message.text)
        elif message is not None and message.get_type() == 'PARAM_ERROR':
            self.texts.append(str(message))
        return message

    def until(self, predicate, timeout=10):
        deadline = time.monotonic() + timeout
        while time.monotonic() < deadline:
            message = self.receive()
            if message is not None and predicate(message):
                return message
        raise AssertionError(self.diagnostic())

    def pump(self, seconds):
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            self.receive()

    def parameter(self, name, value=None):
        deadline = time.monotonic() + 10
        sent = 0
        while time.monotonic() < deadline:
            if time.monotonic() - sent > 1:
                if value is None:
                    self.link.mav.param_request_read_send(1, 1, name.encode(), -1)
                else:
                    self.link.mav.param_set_send(1, 1, name.encode(), value, mavutil.mavlink.MAV_PARAM_TYPE_REAL32)
                sent = time.monotonic()
            message = self.receive()
            if message is not None and message.get_type() == 'PARAM_VALUE' and message.param_id == name:
                return int(message.param_value)
        raise AssertionError(f'No parameter response for {name}\n{self.diagnostic()}')

    def wait_parameter(self, name, expected):
        deadline = time.monotonic() + 10
        while time.monotonic() < deadline:
            if self.parameter(name) == expected:
                return
            self.pump(0.1)
        raise AssertionError(f'{name} != {expected}\n{self.diagnostic()}')

    def text(self, fragment, after=0, timeout=10):
        if any(fragment in item for item in self.texts[after:]):
            return
        self.until(lambda msg: msg.get_type() == 'STATUSTEXT' and fragment in msg.text, timeout)

    def restart(self, value, expected=1):
        mark = len(self.texts)
        self.parameter('RUST_RESTART', value)
        self.text(f'Rust: loaded {expected} modules', after=mark)

    def checksums(self):
        self.parameter('RUST_DEBUG_OPTS', 32)
        self.wait_parameter('RUST_DEBUG_OPTS', 0)
        return self.parameter('RUST_LD_CSUM'), self.parameter('RUST_RUN_CSUM')

    def arm(self, armed=True, force=False, accepted=True):
        self.link.mav.command_long_send(1, 1, mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM, 0,
                                        int(armed), 21196 if force else 0, 0, 0, 0, 0, 0)
        result = self.until(lambda msg: msg.get_type() == 'COMMAND_ACK' and
                            msg.command == mavutil.mavlink.MAV_CMD_COMPONENT_ARM_DISARM).result
        if (result == mavutil.mavlink.MAV_RESULT_ACCEPTED) != accepted:
            self.pump(0.5)  # failure STATUSTEXT may follow the command acknowledgement
        assert (result == mavutil.mavlink.MAV_RESULT_ACCEPTED) == accepted, self.diagnostic()
        if accepted:
            self.until(lambda msg: msg.get_type() == 'HEARTBEAT' and
                       bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED) == armed)

    def worker(self, name):
        for task in Path(f'/proc/{self.process.pid}/task').iterdir():
            try:
                if (task / 'comm').read_text().strip() == name:
                    fields = (task / 'stat').read_text().rsplit(')', 1)[1].split()
                    return int(task.name), (int(fields[11]) + int(fields[12])) / os.sysconf('SC_CLK_TCK')
            except FileNotFoundError:
                pass
        return None

    def anonymous_mappings(self):
        writable = executable = 0
        for line in Path(f'/proc/{self.process.pid}/maps').read_text().splitlines():
            fields = line.split()
            if len(fields) != 5:  # exclude file mappings and named process/stack areas
                continue
            low, high = (int(value, 16) for value in fields[0].split('-'))
            if 'w' in fields[1] and 'x' not in fields[1]:
                writable += high - low
            if 'x' in fields[1] and 'w' not in fields[1]:
                executable += high - low
        return writable, executable


@pytest.fixture
def launch(tmp_path):
    vehicles = []

    def start(files=None, **extra):
        vehicle = Vehicle(tmp_path / f'vehicle-{len(vehicles)}', files, extra)
        vehicles.append(vehicle)
        vehicle.start()
        return vehicle

    yield start
    for vehicle in vehicles:
        vehicle.stop()


def checksum(data):
    return zlib.crc32(data) & CHECKSUM_MASK


def test_normal_exit_checksums_and_arming_bypasses(launch, images):
    vehicle = launch()
    assert vehicle.checksums() == (0, 0)
    vehicle.arm()
    vehicle.arm(False)
    vehicle.modules.mkdir()
    (vehicle.modules / 'one.apmod').write_bytes(images['return'])
    vehicle.restart(1)
    vehicle.text('Rust: exited one.apmod')
    assert vehicle.checksums() == (checksum(images['return']), 0)
    vehicle.arm()
    vehicle.arm(False)
    vehicle.parameter('RUST_LD_CSUM', checksum(images['return']) ^ 1)
    vehicle.arm(accepted=False)
    vehicle.parameter('RUST_DEBUG_OPTS', 16)
    vehicle.arm()
    vehicle.arm(False)
    vehicle.parameter('RUST_DEBUG_OPTS', 0)
    vehicle.parameter('ARMING_SKIPCHK', SYSTEM_CHECK)
    vehicle.arm()
    vehicle.arm(False)
    vehicle.parameter('ARMING_SKIPCHK', 0)
    vehicle.arm(force=True)
    vehicle.arm(False)
    (vehicle.modules / 'two.apmod').write_bytes(images['return'])
    vehicle.restart(2, expected=2)
    vehicle.pump(0.5)
    assert vehicle.checksums() == (0, 0)  # duplicate files cancel, including after restart


def test_armed_restart_panic_retention_and_cpu(launch, images):
    vehicle = launch({'one.apmod': images['cleanup'], 'two.apmod': images['cooperative']})
    expected = checksum(images['cleanup']) ^ checksum(images['cooperative'])
    assert vehicle.checksums() == (expected, expected)
    vehicle.arm()
    for request in range(1, 6):
        old = [vehicle.worker(name)[0] for name in ('one.apmod', 'two.apmod')]
        vehicle.restart(request, expected=2)
        for name in ('one.apmod', 'two.apmod'):
            assert vehicle.worker(name)[0] not in old
        assert vehicle.checksums() == (expected, expected)
    (vehicle.modules / 'one.apmod').write_bytes(images['panic'])
    (vehicle.modules / 'two.apmod').unlink()
    mark = len(vehicle.texts)
    vehicle.parameter('RUST_RESTART', 6)
    vehicle.text('panic; reboot required', after=mark)
    vehicle.until(lambda msg: msg.get_type() == 'HEARTBEAT' and
                  bool(msg.base_mode & mavutil.mavlink.MAV_MODE_FLAG_SAFETY_ARMED))
    before = vehicle.worker('one.apmod')
    assert before is not None
    vehicle.pump(2)
    after = vehicle.worker('one.apmod')
    assert before[0] == after[0] and after[1] - before[1] < 0.15
    vehicle.arm(False)
    vehicle.arm(accepted=False)
    vehicle.arm(force=True)
    mark = len(vehicle.texts)
    vehicle.parameter('RUST_RESTART', 7)
    vehicle.text('stop timeout', after=mark, timeout=15)
    assert vehicle.worker('one.apmod')[0] == before[0]
    assert not any('Rust: loaded' in text for text in vehicle.texts[mark:])
    vehicle.arm(False)


def test_failed_generation_cleanup_and_recovery(launch, images):
    vehicle = launch({'one.apmod': images['cooperative'], 'two.apmod': b'bad image'})
    vehicle.arm(accepted=False)
    assert vehicle.worker('one.apmod') is None  # no worker starts before all images validate
    (vehicle.modules / 'two.apmod').write_bytes(images['return'])
    vehicle.restart(1, expected=2)
    vehicle.pump(0.5)
    expected = (checksum(images['cooperative']) ^ checksum(images['return']), checksum(images['cooperative']))
    assert vehicle.checksums() == expected
    vehicle.arm()
    vehicle.arm(False)
    (vehicle.modules / 'three.apmod').write_bytes(images['return'])
    mark = len(vehicle.texts)
    vehicle.parameter('RUST_RESTART', 2)
    vehicle.text('module count', after=mark)
    vehicle.arm(accepted=False)
    assert vehicle.worker('one.apmod') is None


def test_saved_checksum_action_does_not_repeat_on_reboot(launch, images):
    vehicle = launch({'one.apmod': images['cooperative']}, RUST_RESTART=23)
    expected = checksum(images['cooperative'])
    assert vehicle.checksums() == (expected, expected)
    vehicle.pump(2)
    worker = vehicle.worker('one.apmod')[0]
    vehicle.pump(2)
    assert vehicle.worker('one.apmod')[0] == worker  # saved request did not request an extra reload
    vehicle.stop()
    (vehicle.modules / 'one.apmod').write_bytes(images['cleanup'])
    vehicle.start()
    vehicle.pump(2)
    assert vehicle.parameter('RUST_DEBUG_OPTS') == 0
    assert vehicle.parameter('RUST_LD_CSUM') == expected
    assert vehicle.parameter('RUST_RUN_CSUM') == expected
    assert expected != checksum(images['cleanup'])
    vehicle.arm(accepted=False)


def test_disabled_runtime_ignores_module_errors(launch):
    vehicle = launch({'bad.apmod': b'bad image'}, RUST_ENABLE=0)
    vehicle.arm()
    vehicle.arm(False)


def test_late_exit_and_coalesced_restarts(launch, images):
    vehicle = launch({'one.apmod': images['late_stop']})
    worker = vehicle.worker('one.apmod')[0]
    (vehicle.modules / 'one.apmod').write_bytes(images['cooperative'])
    mark = len(vehicle.texts)
    started = time.monotonic()
    for request in range(1, 7):
        vehicle.parameter('RUST_RESTART', request)
        vehicle.pump(0.2)
    vehicle.text('stop timeout', after=mark)
    # At speedup 5 the deadline is about two wall-clock seconds. Repeated
    # requests during Stopping must not keep pushing the deadline back.
    assert time.monotonic() - started < 3.5, vehicle.diagnostic()
    assert vehicle.worker('one.apmod')[0] == worker
    vehicle.text('Rust: exited one.apmod', after=mark)
    assert vehicle.worker('one.apmod') is None
    assert not any('Rust: loaded' in text for text in vehicle.texts[mark:])
    vehicle.arm(accepted=False)  # a late exit does not clear the timeout error
    vehicle.restart(7)
    vehicle.arm()
    vehicle.arm(False)


def test_directory_errors_and_text_queue_overflow(launch, images):
    vehicle = launch()
    vehicle.modules.write_bytes(b'not a directory')
    mark = len(vehicle.texts)
    vehicle.parameter('RUST_RESTART', 1)
    vehicle.text('module directory unavailable', after=mark)
    vehicle.arm(accepted=False)
    vehicle.modules.unlink()
    vehicle.modules.mkdir()
    (vehicle.modules / 'one.apmod').write_bytes(images['flood'])
    mark = len(vehicle.texts)
    vehicle.restart(2)
    vehicle.text('Rust: dropped', after=mark)
    vehicle.arm()
    vehicle.arm(False)


def test_restart_memory_mappings(launch, images):
    vehicle = launch({'one.apmod': images['cleanup'], 'two.apmod': images['cooperative']})
    request = 0
    for _ in range(10):  # warm the libc allocator and vehicle telemetry paths
        request += 1
        vehicle.restart(request, expected=2)
    baseline = vehicle.anonymous_mappings()
    for cycle in range(40):
        if cycle % 5 == 0:
            (vehicle.modules / 'two.apmod').write_bytes(b'bad image' * 32)
            request += 1
            mark = len(vehicle.texts)
            vehicle.parameter('RUST_RESTART', request)
            vehicle.text('load failed', after=mark)
            (vehicle.modules / 'two.apmod').write_bytes(images['cooperative'])
        request += 1
        vehicle.restart(request, expected=2)
        assert vehicle.anonymous_mappings() == baseline, vehicle.diagnostic()
    # This checks mapping reservations, not suballocations inside malloc arenas;
    # run it under Valgrind as well to catch leaked heap/metadata allocations.
