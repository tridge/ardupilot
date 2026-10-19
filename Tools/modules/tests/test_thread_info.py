# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Read SITL thread diagnostics over MAVFTP, including Rust worker reloads."""

import re
import struct

import pytest

from pymavlink import mavftp_op as ftp
from pymavlink.mavftp import FtpError
from test_runtime import Vehicle  # noqa: F401
from test_runtime import images  # noqa: F401


class FTPReader:
    def __init__(self, vehicle):
        self.vehicle = vehicle
        self.seq = 0
        self.session = 0

    def request(self, opcode, payload=b'', size=None, offset=0):
        op = ftp.FTP_OP(self.seq, self.session, opcode, len(payload) if size is None else size,
                        0, 0, offset, bytearray(payload))
        packet = op.pack()
        packet.extend(bytes(251 - len(packet)))
        self.vehicle.link.mav.file_transfer_protocol_send(0, 1, 1, packet)
        message = self.vehicle.until(lambda m: m.get_type() == 'FILE_TRANSFER_PROTOCOL'
                                     and m.payload[5] == opcode
                                     and struct.unpack_from('<H', bytes(m.payload))[0] == (self.seq + 1) % 65536)
        seq, self.session, result, length, _, _, _, offset = struct.unpack('<HBBBBBBI', bytes(message.payload[:12]))
        self.seq = (seq + 1) % 65536
        return result, bytes(message.payload[12:12 + length])

    def read(self, path):
        status, _ = self.request(ftp.OP_ResetSessions)
        assert status == ftp.OP_Ack
        status, payload = self.request(ftp.OP_OpenFileRO, path.encode() + b'\0')
        assert status == ftp.OP_Ack, payload
        length, = struct.unpack('<I', payload)
        contents = bytearray()
        try:
            while True:
                status, payload = self.request(ftp.OP_ReadFile, size=239, offset=len(contents))
                if status == ftp.OP_Nack:
                    assert payload == bytes([FtpError.EndOfFile]), payload
                    break
                assert status == ftp.OP_Ack and payload, (status, payload)
                contents.extend(payload)
                assert len(contents) <= length
        finally:
            status, _ = self.request(ftp.OP_TerminateSession)
            assert status == ftp.OP_Ack
        # @SYS advertises a placeholder stat size; EOF terminates the snapshot.
        assert 0 < len(contents) <= length
        return contents.decode()


@pytest.mark.parametrize('rust', [False, True])
def test_threads_over_ftp(tmp_path, images, rust):  # noqa: F811
    modules = {'diag.apmod': images['cleanup'], 'gone.apmod': images['return']} if rust else None
    vehicle = Vehicle(tmp_path / 'vehicle', modules, {'SCR_ENABLE': 1, 'RUST_ENABLE': int(rust)})
    try:
        vehicle.start()
        if rust:
            vehicle.text('Rust: exited gone.apmod')
        reader = FTPReader(vehicle)

        def snapshot():
            listing = reader.read('@SYS/threads.txt')
            print(listing, flush=True)
            assert listing.startswith('ThreadsV2\n')
            names = [line.split()[0] for line in listing.splitlines()[1:]]
            assert 'main' in names and 'Scripting' in names and 'FTP' in names
            assert 'gone.apmod' not in names
            # Default SITL libc stacks have no high-water mark instrumentation.
            assert re.search(r'^Scripting\s+PRI=\s*\d+ sp=\S+ STACK=\?/\d+', listing, re.M)
            if rust:
                assert names.count('RustMgr') == names.count('diag.apmod') == 1
                match = re.search(r'^diag.apmod\s+PRI=\s*0 sp=\S+ STACK=(\d+)/(\d+)$', listing, re.M)
                assert match, listing
                free, total = map(int, match.groups())
                assert 0 < free < total == 65536
            else:
                assert 'RustMgr' not in names
            return listing

        snapshot()
        if rust:
            for generation in range(1, 6):
                vehicle.restart(generation, expected=2)
                vehicle.pump(0.1)
                snapshot()
            (vehicle.modules / 'diag.apmod').unlink()
            (vehicle.modules / 'gone.apmod').unlink()
            vehicle.restart(6, expected=0)
            assert 'diag.apmod' not in reader.read('@SYS/threads.txt')
    finally:
        vehicle.stop()
