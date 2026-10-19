#!/usr/bin/env python3
# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Build the H743 RustRuntime example with a native module in ROMFS."""

import argparse
import subprocess

from pathlib import Path

import build

ROOT = Path(__file__).resolve().parents[2]
MODULE = ROOT / 'build/modules/romfs_runtime/romfs.apmod'
HWDEF = ROOT / 'Tools/modules/romfs-runtime.hwdef'


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--board', default='MatekH743', help='ChibiOS H7 board supported by the Renode fixture')
    parser.add_argument('--out', type=Path, default=ROOT / 'build/romfs-runtime', help='separate waf build directory')
    args = parser.parse_args()

    build.build(ROOT / 'Tools/modules/romfs_probe.rs', MODULE,
                'thumbv7em-none-eabihf', 8193)

    subprocess.run([str(ROOT / 'waf'), 'configure', '--board', args.board,
                    '--out', str(args.out), '--extra-hwdef', str(HWDEF)], cwd=ROOT, check=True)
    subprocess.run([str(ROOT / 'waf'), '--targets', 'examples/RustRuntime'], cwd=ROOT, check=True)
    print(args.out / args.board / 'examples/RustRuntime')


if __name__ == '__main__':
    main()
