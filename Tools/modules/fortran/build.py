# AP_FLAKE8_CLEAN
# SPDX-License-Identifier: GPL-3.0-or-later
"""Runtime-free GNU Fortran demo backend for x86-64 Linux/SITL and Thumb hard float."""

import os
import shutil
import subprocess

from pathlib import Path

from cpp import build as cpp

SDK = Path(__file__).resolve().parents[3] / 'libraries/AP_Module/sdk/fortran'


def build(source, output, target, stack_size, base=0x10000, cfg=(), debug=False, opt_level='s', heap_size=4096):
    if opt_level != 's':
        raise ValueError('Fortran demo modules currently require --opt-level=s')
    arm = target == 'thumbv7em-none-eabihf'
    default = 'arm-linux-gnueabihf-gfortran' if arm else 'gfortran'
    compiler = shutil.which(os.environ.get('FC', default))
    if compiler is None:
        raise ValueError(f'Fortran demo builds require GNU Fortran (set FC to {default})')
    version = subprocess.check_output([compiler, '--version'], text=True)
    triple = subprocess.check_output([compiler, '-dumpmachine'], text=True).strip()
    compatible = triple.startswith(('arm-', 'thumb-')) and 'eabi' in triple if arm else \
        triple.startswith('x86_64-') and 'linux' in triple
    if 'GNU Fortran' not in version or not compatible:
        raise ValueError(f'Fortran compiler target {triple} is incompatible with {target}')
    output.parent.mkdir(parents=True, exist_ok=True)
    module_dir = output.with_suffix('.fortran')
    module_dir.mkdir(exist_ok=True)
    common = [compiler, '-std=f2008', '-Os', '-fPIC', '-fno-stack-protector',
              '-ffunction-sections', '-fdata-sections', '-fno-unwind-tables', '-fno-asynchronous-unwind-tables',
              '-J' + str(module_dir), '-I' + str(module_dir)]
    arm_flags = ['-mcpu=cortex-m4', '-mthumb', '-mfpu=fpv4-sp-d16', '-mfloat-abi=hard']
    if arm:
        # GCC has no RWPI model. Reserve the C++ SDK's r9 base, and let the
        # packer reject GOT/RW references rather than accepting unsafe images.
        # Stack locals and PC-relative read-only constants need neither a GOT
        # nor a second relocation scheme. Mutable Fortran static data is excluded.
        common += arm_flags + ['-ffixed-r9']
    else:
        common += ['-march=x86-64', '-mtune=generic', '-mno-red-zone']
    if debug:
        common += ['-g', '-fno-omit-frame-pointer']
        if arm:
            # Clang's assembler does not accept GNU .loc "view" directives.
            common += ['-gno-variable-location-views', '-gdwarf-4']
    objects = []
    for index, path in enumerate((SDK / 'ap_module.f90', source)):
        obj = module_dir / f'{index}.o'
        if arm:
            # Use the existing Clang assembler, not Linux-target binutils.
            assembly = obj.with_suffix('.s')
            subprocess.run(common + ['-S', str(path), '-o', str(assembly)], check=True)
            subprocess.run([os.environ.get('CXX', 'clang++'), '--target=arm-none-eabi', *arm_flags,
                            '-c', str(assembly), '-o', str(obj)], check=True)
        else:
            subprocess.run(common + ['-c', str(path), '-o', str(obj)], check=True)
        objects.append(obj)
    if arm:
        gcc = os.environ.get('ARM_GCC', 'arm-none-eabi-gcc')
        library = Path(subprocess.check_output([gcc, *arm_flags, '-print-libgcc-file-name'], text=True).strip())
        if not library.is_file() or library.suffix != '.a':
            raise ValueError('ARM Fortran needs the Cortex-M4 hard-float libgcc.a (set ARM_GCC)')
        # Only referenced compiler helpers (e.g. integer/float conversion) are
        # extracted. Their sections/relocations undergo the same packer checks.
        objects.append(library)
    # Reuse the validated C++ entry/allocator, generated clients and packer.
    # Deliberately do not link libgfortran: runtime dependencies must fail here.
    return cpp.build(SDK / 'bridge.cpp', output, target, stack_size, base=base,
                     debug=debug, opt_level=opt_level, heap_size=heap_size, extra_objects=objects)
