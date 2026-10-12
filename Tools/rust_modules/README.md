# Rust module relocation experiment

This is a preliminary offline prototype for the
[loadable module design](../../libraries/AP_Rust/DESIGN.md). It does not yet
load modules into ArduPilot, use AP_MultiHeap, create threads or expose vehicle
bindings. Its container and ABI are experimental. AI-assisted implementation,
reviewed with Claude; GPLv3 or later.

The converter links PIC code offline, resolves internal PC-relative references
and retains only absolute pointer fixups in the module file. Unknown sections,
undefined symbols and instruction fixups are rejected. x86-64 GOT entries
are converted only when each is the address of a defined image symbol; ARM
GOTs and dynamic-linking tables remain unsupported. In
particular, static Thumb MOVW/MOVT instructions are not supported. Fat LTO is
used to compile linked Rust bitcode with the module's relocation model.

Prerequisites are Python with pytest and pyelftools, Rust **1.90.0** with
`x86_64-unknown-linux-gnu` and `thumbv7em-none-eabihf`, native GCC/binutils,
`arm-none-eabi-gcc`, and Renode. The tests deliberately fail on a missing or
different compiler; Renode tests skip if RENODE does not name an installed
executable. These are opt-in toolchain tests, not dependency-free unit tests.

From the repository root:

```sh
RUSTC=/path/to/rustc RENODE=/path/to/renode \
    pytest -q Tools/rust_modules/test_modules.py
flake8 Tools/rust_modules/*.py
```

The tests link at two different addresses and compare normalized containers,
run two independently allocated native x86-64 copies with RX code and RW data,
and execute two copies under Renode's Cortex-M4 and Cortex-M7 models. They
exercise mutable static data, BSS, function pointers, strings, host calls and
single-precision floating point, Vec allocation through host callbacks and
core slice sorting. Host tests also inject allocation failure and verify
deallocation. Header corruption, truncation, invalid
fixups, load-address overflow and unsupported GOT/instruction relocations
must fail. Renode runs both the Python relocator output and the C++ Image
loader against the real packed input. It uses a minimal CPU/RAM harness; this does not
validate ChibiOS, SD-card loading, MPU policy, caches or real CPU performance.

The allocation callbacks are test allocators, not yet AP_MultiHeap. A larger
compiler-output corpus and ChibiOS integration remain required. The probe's
panic handler spins; this is only a harness and is not the planned SDK policy.

## Compatibility envelope

Without `--manifest`, the converter emits the original `APRUST01` relocation
probe format, accepted only by `Image::parse_probe()` when the test-only
`AP_RUST_LEGACY_PROBE_ENABLED` flag is enabled. A runtime must instead
call `Image::parse()` with its expected compatibility contract. That path
accepts only `APRUST03` with an exact core ABI version, CPU profile and 16-byte core
fingerprint, and a declared stack requirement no larger than its stack budget.
Both formats remain experimental. Binding imports are independent 64-bit API
hashes, so changes to unrelated bindings do not invalidate a module.

The `--manifest` JSON contains exactly `abi_version`, `cpu_profile`,
`stack_size` and `fingerprint` (32 hexadecimal digits). Profile 1 is ARM
Thumb-2 Cortex-M4-compatible hard float; profile 2 is baseline x86-64 Linux.
The pinned target builds used here provide those profiles. Metadata is a
trusted build declaration, not instruction-set analysis, a stack-depth proof,
authentication or a sandbox. The SDK build supplies this manifest from the
generated core API; users should not invent a matching fingerprint.

Version 3 has an 80-byte little-endian header: magic (8 bytes), followed by
32-bit version, machine, pointer width, alignment, initialized size, memory
size, RX boundary, entry offset, fixup count, ABI version, CPU profile and
stack requirement and import count; then the 16-byte core fingerprint and
32-bit CRC. Image bytes and absolute-pointer fixups follow, then 12-byte import
records (little-endian u64 hash, u32 slot offset), sorted by slot offset. At most
4096 imports are accepted; duplicate hashes/slots, RX or unaligned slots,
non-placeholder contents and overlap with pointer fixups are rejected.
Private `.ap_rust_import_<16 hex digits>` ELF sections contain one four-byte
UINT32_MAX placeholder each. The packer validates their RW placement; only live
sections survive linker GC. No exported static symbols or ARM GOT are needed.
Resolution patches slots before publishing the image. Unknown hashes reject it.
CRC covers the whole file with
the CRC field zeroed. Stack requirements must be 8-byte multiples from 256
bytes through 16 MiB. Version 1 remains a test-only probe format; experimental
version-2 runtime modules must be rebuilt for version 3.

## Vehicle autotests

SITL enables native Rust scripting by default (`HAL_PROGRAM_SIZE_LIMIT_KB > 2048`).
The Plane suite builds and flies the numeric AirShow using Rust 1.90.0 with the
`x86_64-unknown-linux-gnu` target installed. Select that compiler through `RUSTC`
if it is not your default `rustc`:

```sh
RUSTC=/path/to/rustc-1.90.0 Tools/autotest/autotest.py \
    build.Plane test.Plane.AerobaticsScripting_rust
```

The Plane `tests1a` CI job installs that pinned compiler. Local test runs do not
download toolchains automatically. Use `--uds` if the normal SITL TCP ports are
already in use.
The builder, autotest precondition and CI share `rust-toolchain-version`.
Do not rebuild or replace the running SITL binary during a test: SITL reboots by
executing that binary again. Use a separate build/worktree for concurrent builds.

Reusable `TestSuite` helpers:

- `install_rust_applet_context('aerobatics/main.rs', install_name='airshow.apmod')`
  builds a source under `libraries/AP_Rust/applets/` and installs it in `rust/`.
- `install_rust_test_module_context('math.rs')` builds a native test fixture;
  `install_rust_module_context(source)` accepts another source path.
- `build_rust_module(source, output, stack_size=16384, target='x86_64-unknown-linux-gnu')`
  exposes the build step for tests needing explicit artifacts or another target.
- `install_rust_file_context(source, install_name='trick72.txt', directory='scripts')`
  installs ancillary data (the default directory is `rust/`).
- `rust_scripting_restart()` requests cooperative reload and waits for the
  manager's new-generation message. Wait separately for applet-specific readiness.

Context installers refuse to overwrite existing files and remove their own files
on context cleanup, including failure cleanup. Intermediate build files are kept
in a temporary directory and removed. Tests must enable `RUST_ENABLE` and reboot
after installing, and configure image/heap/stack budgets; the module stack
requirement excludes the loader's host-callback margin. Removing a file does not
stop an already loaded worker: reboot after cleanup before running another test.
If a killed run leaves a file behind, installation raises `FileExistsError` with
its path. Check that file belongs to the interrupted test and move it aside or
remove it before retrying; the helper never deletes an unowned file.

## SDK modules

`build.py` builds a standalone source file against the generated core SDK,
pins Rust 1.90.0 and the CPU profile, and supplies the API manifest directly
from the checked-in generated identity. It writes the packed image plus
intermediate ELF and SDK archive beside the output. Example:

```sh
RUSTC=/path/to/rustc python3 Tools/rust_modules/build.py \
    Tools/rust_modules/sdk_probe.rs /tmp/sdk-probe.apmod \
    --target thumbv7em-none-eabihf --stack-size 8192
RUSTC=/path/to/rustc RENODE=/path/to/renode \
    python3 -m pytest -q Tools/rust_modules/test_sdk.py \
    libraries/AP_Rust/generator/test_generate.py
```

The SDK tests compare images linked at different bases, reject Send/Sync
contexts at compile time, and call the generated C API from actual Rust on
both Renode CPU models. Native tests in `test_rust_sdk` additionally use
AP_MultiHeap and two owned worker threads, and observe panic/OOM callbacks
in isolated child processes. This is not yet a card-loading module runtime.
