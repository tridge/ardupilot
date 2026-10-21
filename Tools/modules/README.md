# Native module tools

These tools build, inspect and test ArduPilot's experimental Rust and C++ native
modules, plus a small Fortran talk demo. See the [runtime guide](../../libraries/AP_Module/README.md) for firmware
configuration and installation, and the [design](../../libraries/AP_Module/DESIGN.md)
for the container, ABI and resource-ownership contracts.

## Prerequisites

Install Python with `pyelftools`. Rust builds require Rust **1.98.1 or newer**. The minimum compiler
version is shared by the builder and CI through `rust/rust-toolchain-version`.
`RUSTC` selects a compiler other than the default `rustc`.

```sh
rustup update stable
rustup target add --toolchain stable thumbv7em-none-eabihf
python3 -m pip install pyelftools pytest
```

Supported targets are `thumbv7em-none-eabihf` for STM32 and
`x86_64-unknown-linux-gnu` for x86-64 SITL/Linux. Host builds need native
GCC/binutils; firmware builds and some tests also need the ArduPilot ARM
toolchain. Renode integration tests require `RENODE` to name an executable;
SD-card tests also require mtools. Compiler-dependent tests fail if their
compiler is missing or too old. Renode tests skip when their emulator or
firmware fixture is unavailable.

C++ builds need Clang 21+ and standalone LLD 21+, selected with `CXX` and `LLD`.
They do not invoke Rust or require a Rust installation. The C++ SDK is
freestanding C++17: local RAII and virtual dispatch work; exceptions, RTTI, TLS
and global constructors/destructors are rejected. Only `-Os` is currently
accepted for C++ modules, including with `--debug`.

Debugger integration tests use a real GDB process. For SITL, set
`AP_MODULE_SITL_BINARY` to a Plane ELF with C++ debug information and run
`tests/test_gdb_runtime.py`; `AP_MODULE_GDB` overrides native `gdb`. The test
checks one and two modules, Rust values, backtraces and reload cleanup.
For the firmware USB monitor under Renode:

```sh
AP_MODULE_USB_ELF=/path/to/CubeOrangePlus/arduplane \
AP_MODULE_ARM_GDB=/path/to/python-enabled/arm-none-eabi-gdb \
RENODE=/path/to/patched/renode \
    python3 -m pytest -q Tools/modules/tests/test_gdb_usb_runtime.py
```

The USB test launches its own emulator and sparse SD image; it does not attach
to a physical board or use kernel USB/IP drivers. Renode needs the guest
DebugMonitor and USB/IP patches described in `Tools/renode/patches/README.md`.
Both integration suites require GDB Python dependencies and the matching Rust
toolchain; absent optional firmware/emulator fixtures skip explicitly.

## Build and inspect

Run from the repository root:

```sh
Tools/modules/build.py \
    libraries/AP_Module/applets/aerobatics/main.rs \
    build/modules/airshow.apmod \
    --target thumbv7em-none-eabihf --stack-size 16384 --heap-size 32768
Tools/modules/apmod_info.py build/modules/airshow.apmod
Tools/modules/apmod_info.py --fixups build/modules/airshow.apmod
```

`build.py` dispatches `.rs` to `rust/build.py`, `.cpp`/`.cc`/`.cxx` to
`cpp/build.py`, and `.f90` to the demo `fortran/build.py`. Shared bindings are generated below
`build/modules/libraries/AP_Module/`. Rust uses fat LTO; C++ uses Clang/LLD.
Both package the linked ELF, retaining the companion `.elf` beside the output.
Rust also retains `lib<name>_sdk.rlib`; C++ retains its intermediate `.o` files.
`--heap-size` is required and declares initial heap capacity, checked against
`MOD_HEAP_MAX` on load. Armed growth is permitted only up to that cap.
`--stack-size` declares module stack use, not a measured bound. The firmware's
`MOD_STACK_MAX` must additionally cover the host-call margin: 4 KiB on Cortex-M
or 16 KiB on x86-64; backend overhead is additional.

`apmod_info.py` validates the file and reports its target, ABI, code/data sizes,
alignments, stack requirement, relocations and imported API names. It does not
check whether a particular firmware provides those APIs or has enough RAM.
It automatically uses a companion `.elf` to split file size into code, read-only
data, initialized RW data, alignment padding and container overhead. The ELF
must reproduce the module exactly; `--elf PATH` selects a different companion,
and `--no-elf` ignores it. Without an ELF, the tool reports the combined image
size and container overhead; section boundaries cannot be recovered from the
module alone. Padding means gaps between ELF sections and the RX/RW boundary,
not zero-valued instructions or constants within a section. Stack, heap and
zero-fill RAM are not stored in the file.

Change a module's resource declarations without rebuilding its code:

```sh
Tools/modules/apmod_info.py module.apmod --stack-size 16384 --heap-size 32768
```

Either option can be used alone. Sizes are bytes, from 256 through 16777216.
The tool validates the module and companion ELF before replacing the file in
place, updates its CRC and displays the new values. The ELF remains usable.
Changing declarations does not establish actual memory requirements: the stack
excludes the host-call margin and the heap is its initial capacity. Firmware
still enforces `MOD_STACK_MAX` and `MOD_HEAP_MAX`. The file checksum changes,
so refresh configured loaded/running checksum requirements before use.

`pack.py` is the lower-level ELF converter. Normal SDK builds should use
`build.py`, which supplies the generated ABI manifest and target profile.
Without a manifest, the converter emits a test-only relocation probe, not a
vehicle-loadable module. Unsupported sections, unresolved symbols and unsupported
instruction relocations are rejected rather than delegated to a firmware linker.

## Fortran talk demo (SITL and STM32H7)

The [Fortran simple loop](../../libraries/AP_Module/examples/simple_loop.f90)
reads gyro values, computes their magnitude in Fortran, and sends
`Fortran tick <seconds> gyro <magnitude>` once per second. The small SDK in
`libraries/AP_Module/sdk/fortran/` uses `ISO_C_BINDING` and the generated C++
clients; formatted text uses the firmware formatter, not Fortran I/O. The loader,
firmware, container and resource limits are identical for all three languages.

Install GNU Fortran (tested with gfortran 15.2), Clang 21+ and standalone LLD 21+.
`FC`, `CXX` and `LLD` select the compilers/linker. Rust is not needed to build or
run a Fortran module. From the repository root:

```sh
mkdir -p /tmp/fortran-demo/modules
Tools/modules/build.py libraries/AP_Module/examples/simple_loop.f90 \
    /tmp/fortran-demo/modules/fortran.apmod --target x86_64-unknown-linux-gnu \
    --stack-size 8192 --heap-size 4096
Tools/modules/apmod_info.py /tmp/fortran-demo/modules/fortran.apmod
Tools/autotest/sim_vehicle.py -v ArduPlane --waf-configure-arg=--enable-modules --use-dir /tmp/fortran-demo
```

In MAVProxy, set `MOD_ENABLE` to 1 and reboot (`param set MOD_ENABLE 1`, then
`reboot`). The Fortran module can run alongside Rust and C++ modules; increase
`MOD_MAX_MOD` before reboot if loading more than its default two modules.
Do not install modules in the repository's `modules/` submodule directory.

For STM32H7, use an ARM GNU Fortran cross compiler (tested with 15.2), for example
the `gfortran-arm-linux-gnueabihf` package. Although that compiler targets Linux
by default, the builder explicitly selects Cortex-M4-compatible Thumb hard float,
assembles with Clang and links without Linux startup, libc or libgfortran.
The ArduPilot `arm-none-eabi-gcc` toolchain supplies only referenced compiler
helpers from its Cortex-M4 hard-float `libgcc.a`; `ARM_GCC` overrides that compiler.

```sh
FC=arm-linux-gnueabihf-gfortran Tools/modules/build.py \
    libraries/AP_Module/examples/simple_loop.f90 build/modules/fortran_H7.apmod \
    --target thumbv7em-none-eabihf --stack-size 8192 --heap-size 4096
```

Copy `fortran_H7.apmod` to `/APM/modules/` on microSD and enable `MOD_ENABLE`.
SITL and STM32 require separately compiled images. No firmware loader changes
or additional runtime resolver are needed.

This is a runtime-free demo subset, not a complete generated Fortran SDK.
Only `-Os` is accepted; `--debug` retains Fortran/C++ DWARF. Fixed-size storage
and simple scalar/array arithmetic work. `PRINT`, `WRITE` (including internal
formatted writes), runtime-dependent allocation/intrinsics, and other uses of
libgfortran are not supplied and must fail the build. Avoid compiler-created
array temporaries requiring allocation. ARM Fortran uses a GOT-free PIC subset:
keep mutable variables in automatic stack locals, not `SAVE`, initialized locals,
`DATA` or mutable module globals. The packer rejects GOT/RW references it cannot
relocate safely. GCC reserves `r9` for the C++ bridge's RWPI static base; read-only
Fortran constants and code use PC-relative addressing. These restrictions do not
apply to the existing Rust/C++ backends. Flang 21 cannot lower ARM Fortran yet.

Run the package/rebase checks for both profiles, known-value maths and status
tests, and three-language SITL reload test with native and ARM GNU Fortran installed:

```sh
AP_MODULE_SITL_BINARY=/path/to/sitl/bin/arduplane ARM_FC=arm-linux-gnueabihf-gfortran \
    python3 -m pytest -q Tools/modules/fortran/tests
```

`ARM_FC` selects the cross compiler for tests without changing their native `FC`.
For H743 Renode SD/ROMFS execution and leak-free restart checks, build
`Tools/modules/build_romfs_runtime.py --language fortran` and run
`Tools/modules/tests/test_romfs_manager.py` with the matching
`AP_MODULE_ROMFS_BUILD_ROOT` and `RENODE` settings. The harness uses a known
vector rather than vehicle IMU hardware to check the Fortran calculation.

## Source-level debugging

Build the module and SDK with full debug information, keeping the matching
`.apmod` and `.elf` together on the host:

```sh
Tools/modules/build.py libraries/AP_Module/examples/simple_loop.rs \
    build/modules/simple_loop.apmod --target thumbv7em-none-eabihf \
    --stack-size 16384 --heap-size 4096 --debug --opt-level 1
```

`--debug` enables frame pointers and host-only DWARF, including debugger unwind
information. Debug sections and pretty-printer scripts are not packed into the
module. Frame pointers and a different `--opt-level` can change code size,
stack use and timing. The default optimisation level remains `s`.

Use a **Python-enabled** ARM GDB for STM32 or native GDB for SITL, with
`pyelftools` importable by its Python interpreter. Load the matching firmware
ELF with C++ debug information.

For example, configure and build SITL with:

```sh
CXXFLAGS=-g ./waf configure --board sitl --enable-modules --debug-symbols
./waf plane
```

On STM32, configure with `--debug-symbols --enable-USB-debug
--enable-modules` instead. Connect normally and use:

```gdb
source Tools/modules/apmod_gdb.py
apmod printers
apmod symbols build/modules/simple_loop.apmod
apmod list
info threads
break simple_loop::run
continue
bt
info locals
```

`apmod printers /path/to/rustc` selects the module's Rust toolchain; otherwise
it uses `RUSTC` or `rustc`. Install that toolchain's `rust-src` component to step
into `core`/`alloc`. The helper validates the companion ELF against the module,
matches the runtime's whole-file CRC, and maps RX and RW sections independently.
This supports separate ARM RAM allocations and RX in ROMFS. ARM RWPI static
locations are corrected in a temporary host ELF, never on the board. An ARM
unwinder handles the runtime's naked entry bridge.

Register multiple module files with `apmod symbols FILE.apmod ...`. Registration
can precede module startup; internal breakpoints follow `Module::start()` and
`Module::release()` without adding firmware hooks. Before release, code
breakpoints in that module are deleted and symbols removed, preventing stale
software-breakpoint writes into reused RAM. Recreate those breakpoints after
reload. Remove data watchpoints before reloading modules. `apmod clear` removes
the helper's loaded symbols and lifecycle hooks, including module code breakpoints.
Keep the helper enabled while module breakpoints are installed; clear it before
changing firmware. Optimised variables may still be unavailable.

For USB debugging with `--enable-USB-debug` firmware:

```sh
Tools/debug/gdb_usb.py build/CubeOrangePlus/bin/arduplane --port /dev/serial/by-id/DEVICE-if02 \
    --gdb /path/to/python-enabled/arm-none-eabi-gdb \
    --apmod build/modules/simple_loop.apmod
```

`--apmod` is repeatable and loads the helper. Rust pretty-printers are selected
only for Rust DWARF; C++ debugging does not invoke Rust tools. IDE and
`--serve` sessions can source the helper in their own GDB client instead.
No USB protocol or module-format change is needed. Debugging stops the vehicle;
use a disarmed bench setup, not an aircraft in flight.

## Tests

Shared Python tests live in `tests/`; language-specific tests live in
`rust/tests/` and `cpp/tests/`. Generator tests live with the native generator.
These toolchain/loader tests run without a vehicle firmware build:

```sh
python3 -m pytest -q Tools/modules/rust/tests/test_build.py \
    Tools/modules/tests/test_config.py \
    Tools/modules/tests/test_modules.py \
    Tools/modules/rust/tests/test_sdk.py \
    Tools/modules/tests/test_imports.py \
    Tools/modules/tests/test_split_alignment.py
python3 -m pytest -q libraries/AP_Module/generator
flake8 Tools/modules
```

Remaining tests exercise SITL vehicles or ChibiOS examples. Build Plane with
native modules enabled before running `tests/test_runtime.py` or the binding/applet tests.
Lua comparison tests also require Lua. `AP_MODULE_SITL_BINARY` selects a separately
built Plane executable. Do not replace it during a test: SITL executes it again
on reboot.

For the H743 SD runtime harness:

```sh
./waf configure --board MatekH743 \
    --extra-hwdef libraries/AP_Module/tests/enabled.hwdef
./waf --targets examples/ModuleRuntime
RENODE=/path/to/renode python3 -m pytest -q Tools/modules/tests/test_manager.py
```

The mixed ROMFS/SD harness uses a separate build directory:

```sh
Tools/modules/build_romfs_runtime.py
RENODE=/path/to/renode python3 -m pytest -q Tools/modules/tests/test_romfs_manager.py
```

`OWNED_THREAD_BOARD` selects a compatible board fixture; `AP_MODULE_BUILD_ROOT`
and `AP_MODULE_ROMFS_BUILD_ROOT` select existing example build roots. Tests for
`OwnedThread`, `ExecutableMemory`, `RustHeap`, `ModuleLoader`, `ModuleMath` and
`StackGuard` need the corresponding examples built first. H7 stack-guard tests
require a Renode build with the Cortex-M stacking-fault fix; they do not establish
real-hardware fault recovery or flight safety.

To exercise stack recovery with USB debugging enabled, build the CubeOrangePlus
examples and run both the unattached and attached-monitor tests. Use the patched
Renode described in [the USB debug guide](../renode/patches/README.md):

```sh
./waf configure --board CubeOrangePlus --enable-modules --enable-STACK_GUARD --enable-USB-debug --debug-symbols
./waf --targets examples/StackGuard,examples/ModuleRuntime
STACK_GUARD_BOARD=CubeOrangePlus RENODE=/path/to/renode \
    python3 -m pytest -q Tools/modules/tests/test_stack_guard.py
AP_STACK_USB_ELF=build/CubeOrangePlus/examples/StackGuard RENODE=/path/to/renode \
    python3 -m pytest -q Tools/modules/tests/test_stack_guard_usb.py
```

The attached tests check debugger probes against an active guard, repeated
overflow recovery, mutex release, FP use and refusal of unattributable or
critical-section faults. `AP_MODULE_BUILD_ROOT` selects a non-default build root
for the unattached tests; `STACK_GUARD_BOARD` defaults to `MatekH743`.

Vehicle autotests build and install modules through scoped helpers that refuse
to overwrite existing files. For example:

```sh
Tools/autotest/autotest.py build.Plane test.Plane.AerobaticsScripting_rust
```

Test sources/data remain beside the tools or under `libraries/AP_Module/tests/`.
Frozen fixtures in `libraries/AP_Module/tests/compat/` must not be regenerated
during routine SDK changes.
