# Native modules

`AP_Module` loads trusted native modules from SD card or ROMFS. Each module
has its own worker thread, heap and stack and uses generated bindings to call
ArduPilot APIs. Rust and freestanding C++ share one loader and ABI. Lua can be
enabled independently or alongside native modules.

See [DESIGN.md](DESIGN.md) for implementation contracts and the
[module tools guide](../../Tools/modules/README.md) for packaging and tests.

## Build the firmware

Native module support is included by default when `HAL_PROGRAM_SIZE_LIMIT_KB > 2048`,
including SITL. Runtime loading is disabled until `MOD_ENABLE` is set.

For a modules-only SITL Plane build:

```sh
./waf configure --board sitl --enable-modules --disable-lua-scripting
./waf plane
```

For STM32, select the board and keep or disable Lua as needed:

```sh
./waf configure --board CubeOrangePlus --enable-modules
./waf plane
```

The independent options are `--enable-modules` /
`--disable-modules` and `--enable-lua-scripting` /
`--disable-lua-scripting`. The older `--enable-scripting` /
`--disable-scripting` options select Lua. Disable both runtimes explicitly to
build without scripting. Extra hwdefs should use `AP_MODULE_ENABLED`
and `AP_SCRIPTING_LUA_ENABLED`; `AP_SCRIPTING_ENABLED` denotes shared support.

Supported module profiles are Cortex-M4-compatible Thumb-2 hard float on STM32
and x86-64 on SITL/Linux. Other Linux architectures need a separate target
profile; ARM Linux cannot use the STM32 module. Executable-memory support covers
STM32H743/H757 and F405. Small-board flash/RAM limits may prevent a complete
vehicle build with the desired features.

## Build a module

Install Rust **1.98.1 or newer**, Python `pyelftools`, and the target:

```sh
rustup update stable
rustup target add --toolchain stable thumbv7em-none-eabihf
python3 -m pip install pyelftools
```

`RUSTC` can select a compiler explicitly. The standalone builder compiles modules
directly, without a Cargo project. A minimal source file is:

```rust
#![no_std]
#![no_main]

use ap_rust_sdk::{Context, Result};

ap_rust_sdk::module!(run);

fn run(context: &Context<'_>) -> Result {
    loop {
        context.text(6, "Native module alive")?;
        context.sleep_ms(1000)?;
    }
}
```

Build an existing applet from the repository root:

```sh
Tools/modules/build.py \
    libraries/AP_Module/applets/aerobatics/main.rs \
    build/modules/airshow.apmod \
    --target thumbv7em-none-eabihf --stack-size 16384 --heap-size 32768
Tools/modules/apmod_info.py build/modules/airshow.apmod
```

Use `--target x86_64-unknown-linux-gnu` for x86-64 SITL/Linux. Modules are
target-specific. The stack declaration excludes the host-call margin, currently
4 KiB on Cortex-M and 16 KiB on x86-64. The declaration plus margin must fit
`MOD_STACK_MAX`; OS/guard overhead is additional. Measure the complete applet
and host-call stack usage; the declaration is not a stack proof.

Waf generates bindings below `build/<board>/libraries/AP_Module/` and rebuilds them
when inputs change. The module builder uses
`build/modules/libraries/AP_Module/`; Cargo SDK tests use Cargo's `OUT_DIR`.
Generated bindings are not checked into the source tree.

For C++, install Clang 21 or newer and standalone LLD 21 or newer (no Rust installation is required):

```sh
Tools/modules/build.py libraries/AP_Module/examples/simple_loop.cpp \
    build/modules/simple_cpp.apmod --target thumbv7em-none-eabihf \
    --stack-size 8192 --heap-size 4096
```

`CXX` selects Clang and `LLD` selects `ld.lld`. C++ modules use the freestanding
C++17 SDK in `sdk/cpp/`; exceptions, RTTI, TLS and automatic global constructors
or destructors are unsupported. Local RAII, virtual dispatch and module-owned
`new`/`delete` are supported. ARM C++ currently requires the default `-Os`;
`--debug` is supported, but other optimisation levels are rejected.

## Install and run

Copy `.apmod` files into `/APM/modules/` on SD card, or `./modules/` relative to the
SITL/Linux working directory. Set `MOD_ENABLE=1` and reboot. Check GCS messages
for loading, errors and memory reports before enabling application behavior.
The directory is not searched recursively.

For built-in modules, build the `.apmod` first and add an extra-hwdef entry:

```text
ROMFS modules/airshow.apmod build/modules/airshow.apmod
```

Pass that hwdef to `./waf configure --extra-hwdef <file>`, then build the firmware.
Alternatively put the module in `ROMFS_custom/modules/` before configuration.
The firmware build stores `.apmod` entries without compression and prelinks code
pointers. Other ROMFS files retain the normal compression policy.

STM32 can execute a version-6 ROMFS module directly from qualifying executable
flash, allocating only mutable data/BSS, heap and stack in RAM. SD modules also
copy code into executable RAM. SITL/Linux copy ROMFS code because their embedded
data is not an executable mapping. Ordinary `@ROMFS/` reads reconstruct the
original file, allowing a module to be copied to SD. Globally uncompressed ROMFS
builds retain canonical bytes and copy code rather than prelinking.

ROMFS and writable files are combined in basename order. ROMFS wins a duplicate
basename, with a warning on each load generation. Deduplication precedes
`MOD_MAX_MOD`; exceeding that limit fails loading rather than silently ignoring
modules. All images are prepared before workers start.

## Configuration and memory

Enable and memory/module-count changes require reboot.

| Parameter | Default | Purpose |
| --- | --- | --- |
| `MOD_ENABLE` | 0 | Enable module loading |
| `MOD_HEAP_MAX` | 32768 | Maximum backing heap capacity per module |
| `MOD_CODE_MAXSIZE` | 131072 | Per-module RX plus RW/BSS image limit |
| `MOD_STACK_MAX` | 32768 | Maximum declared stack plus host-call margin |
| `MOD_MAX_MOD` | 2 | Maximum unique modules, from 1 to 8 |
| `MOD_RESTART` | 0 | Change to request cooperative restart-all |
| `MOD_LD_CSUM` | -1 | Required loaded-file checksum; -1 disables |
| `MOD_RUN_CSUM` | -1 | Required running-file checksum; -1 disables |
| `MOD_DEBUG_OPTS` | 0 | Debug options described below |

The image budget rounds up to 4096 bytes; this does not pad stored ARM modules.
Only the actual image is allocated, with cache-line rounding for ARM RX RAM
and page rounding on POSIX. Larger ELF alignment requirements are preserved.
The HAL rounds stack requests to its minimum/alignment. Reports separate owned
RX, owned RW, stack reservations and heap usage. Borrowed ROMFS RX reports zero
owned RX bytes. Allocator bookkeeping, loader scratch, manager storage and
persistent firmware registrations are additional; heap usage is not total RAM.
The manager requests a separate 4 KiB stack on ARM or 32 KiB on x86-64.

`--heap-size` stores each module's initial heap request (default 4096). The loader
rejects requests over `MOD_HEAP_MAX`. Each worker receives its own declared stack
plus the host-call margin, not the entire `MOD_STACK_MAX` budget.

Like Lua, heaps can expand while armed, retaining 20 KiB of system memory.
Growth is bounded by `MOD_HEAP_MAX` and ten backing arenas. Capacity is charged
for all arenas, not just live allocations; freeing objects does not lower it.
Expansion warns with the module name and required initial heap, and latches a
pre-arm failure until reload/reboot without stopping the module or disarming.
Expanded capacity remains usable after disarm. Rebuild with a larger `--heap-size`
to change initial capacity; raise `MOD_HEAP_MAX` if needed. Fragmentation can
still prevent allocation. Capacity and arena-slot exhaustion are reported separately.

This pre-alpha cutover intentionally rejects old `APRUST` modules: rebuild all
modules and update install paths and parameter names. Parameter storage indices
are unchanged, so old saved values carry over. In particular the former initial
heap value now becomes a maximum; review limits before enabling modules.

`MOD_DEBUG_OPTS` bit 4 disables Rust pre-arm checks. Bit 5 saves both current
checksums and clears itself. Bit 6 disables heap expansion for the next module
generation. Checksums are 23-bit-masked XORs of whole-file CRC32 values, not
authentication; identical files can cancel. Normally returned modules stay in
the loaded checksum but leave the running checksum.

## Restart, errors and scheduling

Changing `MOD_RESTART` requests stop for all workers, joins them and reloads the
files, including while armed. SDK sleeps return `Error::Stopped` promptly;
propagate that result or poll `stop_requested()` and return normally so Rust
destructors run. Restart does not itself disarm the vehicle.

Workers are never forcibly cancelled. A ten-second stop timeout retains live
code, stacks, heaps and handles and prevents a new generation. Late exits can be
reaped, but another explicit restart request is needed to retry. Panic and
infallible allocation failure park the worker until reboot, not a whole-firmware
panic. Errors latch a Rust pre-arm failure until the next generation. Normal
return and an empty directory are not errors. SYSTEM-check and force-arm bypasses
follow Lua semantics.

Workers share scripting priority; the manager runs one priority above them on
priority-scheduled HALs, below flight/I/O threads. Normal ChibiOS configuration
has no equal-priority time slicing: a busy module can starve peer workers and
Lua. Use positive sleeps; `sleep_ms(0)` and stop polling do not yield.
Higher-priority threads can still preempt. POSIX behavior depends on policy:
`SCHED_OTHER` time-shares, whereas Linux FIFO has the same starvation concern.
There is no per-module CPU quota.

For load errors, use `apmod_info.py` and check the target, core ABI, imported API
name/hash and limits against the firmware. Known compiled-out APIs load but
return `Error::Unavailable`; unknown contracts reject loading. Formats and APIs
are experimental, so rebuilding against the current tree may be necessary.

## SDK and bindings

Bindings use Lua-style namespaces: `context.ahrs().get_gyro()` or
`context.vehicle().get_mode()`. Results use `Result` and optional outputs use
`Option`; no flat compatibility aliases are generated. `Context` cannot be sent
or shared across threads. Value types are copies, not shared Lua userdata;
methods calling firmware require a live context. Binding locks block and are
released before returning to Rust.

The generated `generator/coverage.json` records supported methods and exclusions
with reasons. Generated Rust docs describe bounds and side effects. Manual Lua
facilities have audited native adapters, not automatic translations. Native
bindings can differ in encoding, ownership and validation: general input strings
are UTF-8, at most 255 bytes, without NUL; parameter names are at most 16 bytes.

Use `context.parameter("NAME")` for repeated parameter access. Dynamic parameter
metadata/names are firmware-owned until reboot and share Lua's table limit.
Identical registration reuses metadata and restores saved values; unsaved changes
need not survive. Logger format registrations also persist until reboot. Keep
names and definitions stable across reloads.

Module exit cleans up owned resources but does not undo ordinary vehicle state
changes, such as modes, mission edits or RC/output overrides. Host calls can
block on devices or filesystem/parameter locks and cannot be interrupted by stop.
The module heap does not account for all host-owned queues/registry storage.
No module function pointers are registered with firmware.

The optional H7 MPU stack guard (`--enable-STACK_GUARD`) defaults off and is
compatible with USB debugging. It can recover a narrowly attributable stack
overflow, not arbitrary bad pointers, skipped guard pages or shared-state damage.
See the design for its limits.

## Tests

See the [tools guide](../../Tools/modules/README.md#tests) for Python,
SITL and Renode commands. C++ tests are under `tests/`, with standalone harnesses
under `examples/`. Simulation and compilation do not establish hardware timing,
cache correctness or flight safety.
