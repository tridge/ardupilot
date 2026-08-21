#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
"""Keep a Renode USB/IP device attached through firmware re-enumeration."""

import argparse
import os
from pathlib import Path
import shutil
import signal
import subprocess
import sys
import time


VHCI_RUN_DIR = Path("/run/vhci_hcd")
VHCI_STATUS = Path("/sys/devices/platform/vhci_hcd.0/status")
stop_requested = False


def parse_args():
    parser = argparse.ArgumentParser(
        description="attach a Renode USB/IP device and reattach after disconnects"
    )
    parser.add_argument("--host", default="127.0.0.1")
    parser.add_argument("--port", type=int, default=3240)
    parser.add_argument("--busid", default="1-0")
    return parser.parse_args()


def matching_port(args):
    try:
        records = VHCI_RUN_DIR.glob("port*")
    except OSError:
        return None
    expected = [args.host, str(args.port), args.busid]
    for record in records:
        try:
            fields = record.read_text(encoding="ascii").split()
        except (OSError, UnicodeError):
            continue
        if fields != expected:
            continue
        try:
            return int(record.name.removeprefix("port"))
        except ValueError:
            continue
    return None


def port_attached(port):
    try:
        lines = VHCI_STATUS.read_text(encoding="ascii").splitlines()[1:]
    except (OSError, UnicodeError, IndexError):
        return False
    for line in lines:
        fields = line.split()
        if len(fields) >= 3 and int(fields[1]) == port:
            return fields[2] == "006"
    return False


def attach(args):
    command = [
        "usbip",
        "--tcp-port",
        str(args.port),
        "attach",
        "--remote",
        args.host,
        "--busid",
        args.busid,
    ]
    result = subprocess.run(
        command,
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        text=True,
        check=False,
    )
    if result.returncode != 0:
        return None, result.stdout.strip()

    deadline = time.monotonic() + 5
    while time.monotonic() < deadline:
        port = matching_port(args)
        if port is not None and port_attached(port):
            return port, ""
        time.sleep(0.1)
    return None, "usbip attach succeeded but no vhci port appeared"


def detach(port):
    subprocess.run(
        ["usbip", "detach", "--port", str(port)],
        stdout=subprocess.DEVNULL,
        stderr=subprocess.DEVNULL,
        check=False,
    )


def request_stop(_signum, _frame):
    global stop_requested
    stop_requested = True


def main():
    args = parse_args()
    if os.geteuid() != 0:
        sys.exit("run this helper as root (for example, with sudo)")
    if shutil.which("usbip") is None:
        sys.exit("usbip is not installed")
    subprocess.run(["modprobe", "vhci_hcd"], check=True)

    signal.signal(signal.SIGINT, request_stop)
    signal.signal(signal.SIGTERM, request_stop)

    owned_attachment = False
    active_port = matching_port(args)
    last_error = None
    try:
        while not stop_requested:
            if active_port is not None and port_attached(active_port):
                time.sleep(0.2)
                continue

            if active_port is not None:
                print("USB/IP device disconnected; waiting to reattach", flush=True)
                active_port = None

            active_port, error = attach(args)
            if active_port is None:
                if error and error != last_error:
                    print("waiting for USB/IP export: %s" % error, flush=True)
                    last_error = error
                time.sleep(0.5)
                continue

            owned_attachment = True
            last_error = None
            print(
                "attached %s:%u/%s on vhci port %u"
                % (args.host, args.port, args.busid, active_port),
                flush=True,
            )
    finally:
        if owned_attachment:
            active_port = matching_port(args)
            if active_port is not None:
                detach(active_port)


if __name__ == "__main__":
    main()
