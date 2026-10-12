# Experimental Rust module components

The [design](DESIGN.md) describes the intended runtime. Currently implemented
are a bounded, aligned AP_MultiHeap adapter, a validated image copier and
relocator, HAL primitives for owned threads and executable memory, and a
single-module owner that connects those primitives to the SDK, and an
experimental vehicle loader/restart manager. The optional code defaults off.
Generated vehicle/parameter bindings and an
[RC AirShow demonstration](applets/aerobatics/README.md) are available. The
demonstration flies in SITL and its controller matches Lua on H743 under Renode;
it is not hardware-flight validated or a complete Lua-app replacement.
An initial no_std SDK provides generated core host calls and is exercised by
standalone modules, including native owned-thread tests.

The [offline tools](../../Tools/rust_modules/README.md) compile and package a
Rust probe. The C++ tests can execute that same probe using AP_MultiHeap on
x86-64 Linux. Renode tests run the C++ loader on Cortex-M4 and Cortex-M7, with
the packed input in emulated flash and two separate output RAM addresses.
Those loader tests are CPU harnesses, not ChibiOS vehicle tests. Separate
Renode tests run the real ChibiOS HAL examples for thread reclamation and
executable allocation. The code is AI-assisted and reviewed with Claude.

## Tests

`AP_SCRIPTING_ENABLED` enables shared scripting facilities when either
`AP_SCRIPTING_LUA_ENABLED` or `AP_SCRIPTING_RUST_ENABLED` is enabled. Lua
keeps its existing board defaults; Rust defaults off. Select the runtimes
independently with `--enable-lua-scripting` / `--disable-lua-scripting` and
`--enable-rust-scripting` / `--disable-rust-scripting`. The older
`--enable-scripting` / `--disable-scripting` options remain Lua aliases.
For a Rust-only SITL vehicle build:

```sh
./waf configure --board sitl --disable-lua-scripting --enable-rust-scripting
./waf plane
```

Use the runtime-specific macros in extra hwdefs. Existing explicit legacy
`AP_SCRIPTING_ENABLED` overrides still select Lua when no Lua choice is
given, but a disabled umbrella cannot be combined with an enabled runtime.

```sh
./waf configure --board sitl --extra-hwdef libraries/AP_Rust/tests/enabled.hwdef
./waf --targets tests/test_rust_heap,tests/test_rust_image
build/sitl/tests/test_rust_heap
AP_RUST_TEST_MODULE=/path/to/packed/x86-64/probe.apmod build/sitl/tests/test_rust_image
```

Without the extra hwdef the tests explicitly skip, because AP_SCRIPTING_RUST_ENABLED
defaults to zero. Without AP_RUST_TEST_MODULE, only the test executing a
compiler-built module skips; the synthetic image tests still run.
Set `AP_RUST_TEST_RUNTIME_MODULE` to a version-2 copy of the same probe to
also test execution through the strict compatibility parser. Its test-only
manifest uses ABI 1, profile 2, stack 4096 and fingerprint
`0102030405060708090a0b0c0d0e0f10`; this is not the future SDK API identity.

Heap usage includes allocation headers and alignment slack, but excludes
MultiHeap's backing-heap overhead. Heap creation is limited to 256 bytes
through 16 MiB with expansion disabled. Deallocation validates ownership
by walking a list, so its cost is linear in the live allocation count.
The module owner joins the worker before destroying its heap. Allocation and
deallocation stay available during cooperative stop, including from destructors.

The owned-thread API retains stacks until the OS thread is joined. POSIX
stack reservations include libc/TLS use; the usable callback stack is smaller.
Executable allocation uses dedicated pages on Linux/SITL, with an RX code
prefix and RW, non-executable data. ChibiOS currently accepts only H743 AXI
SRAM and F405 main SRAM. Publication cleans/invalidates D-cache, invalidates
I-cache where present, and issues barriers; it does not enforce per-image
MPU isolation. Renode does not prove cache behaviour on real silicon.

```sh
./waf configure --board MatekH743 --extra-hwdef libraries/AP_Rust/tests/enabled.hwdef
./waf --targets examples/OwnedThread,examples/ExecutableMemory
RENODE=/path/to/renode python3 -m pytest -q -s Tools/rust_modules/test_threads.py
```

Set `OWNED_THREAD_BOARD=KakuteF4` after building the same examples for that
board to exercise the F405 implementation. These tests check memory return
after 100 cycles and execute native functions from separately allocated images.

Image input must remain alive and immutable through loading. The caller owns
both buffers and must choose executable-capable RAM, publish cache changes
and set permissions before calling the entry. The version-3 compatibility
parser requires an exact core ABI version, CPU profile and core fingerprint,
and checks the declared stack requirement against the configured limit.
Untagged version-1 images are accepted only by the separate `parse_probe()`
test entry point, compiled only with `AP_RUST_LEGACY_PROBE_ENABLED=1` (included
in the test hwdef). The vehicle API is still incomplete;
neither format is yet a stable production module interface. Experimental
version-2 images must be rebuilt once for version 3.

## Initial SDK

`Vector3f` and `Quaternion` are allocation-free Rust `Copy` values with public
`f32` fields (`x/y/z` and `q1/q2/q3/q4`). Defaults match Lua: zero vector and
identity quaternion. Assignment copies the value; it does not alias another
Lua-style userdata object. `PartialEq` uses normal float equality, so NaNs are
unequal. The wire format explicitly encodes components, not C++ object layouts.

Math methods call the same AP_Math C++ methods/operators as Lua, with an explicit
`&Context` and a `Result`. Operators use named `add`, `sub`, and `mul` methods.
Mutating receivers/arguments copy back only on success. Inputs retain Lua's
unchecked float behavior, including NaNs and infinities. Normalizing a zero
vector divides by zero (NaNs or an enabled floating-point trap); normalizing a
zero quaternion raises a firmware internal error, fatal in SITL debug builds.
These are inherited behaviors, not native isolation guarantees. The Quaternion
calls retain Lua's `AP_AHRS_ENABLED` guard and return `Unavailable` when disabled.
`Vector3f::xy`/`Vector2f` are not exposed yet; ordinary Rust copying replaces
Lua's `Vector3f:copy()`.

AHRS exposes `ahrs_get_gyro()` as a `Vector3f`, and
`ahrs_get_velocity_ned()`, `ahrs_get_relative_position_ned_origin()` and
`ahrs_get_quaternion()` as optional value copies (all within `Result`). They
call the same C++ methods as Lua; the origin-relative getter uses the same
float adapter. `None` means no valid estimate. Each copy is made under the
AHRS semaphore, with the native binding's bounded wait returning `Error::Busy`.
Separate calls are not an atomic snapshot. Values require no module-heap
allocation and never borrow firmware storage. Missing AHRS returns
`Error::Unavailable`. No AP_AHRS changes are required.

`Tools/rust_modules/test_bindings_runtime.py` compares grounded Lua/Rust AHRS
samples in SITL before and after module restart. The Renode value harness also
loads `ahrs_unavailable.rs` to verify absent-AHRS handling on both Cortex-M
targets; it does not emulate a live estimator. Generator stand-in tests cover
valid/invalid estimates, bit-preserving copies, and lock contention.

The current math tests compare the dispatcher bit-for-bit with direct C++ calls
on the host, run a fixed Rust/Lua sequence against common tolerances in SITL,
and run the SDK module from SD on ChibiOS in Renode. Floating-point contraction
can differ by target/translation unit, so the on-target SDK checks use numeric
tolerances, not an x86 golden bitstream. They are not CPU benchmarks.

With a Lua+Rust SITL Plane build, run the value tests as follows:

```sh
./waf --targets tests/test_rust_math,tests/test_rust_sdk
build/sitl/tests/test_rust_math
RUSTC=/path/to/rustc python3 Tools/rust_modules/build.py \
    libraries/AP_Rust/tests/modules/math.rs /tmp/math.apmod \
    --target x86_64-unknown-linux-gnu --stack-size 8192
AP_RUST_TEST_MATH_MODULE=/tmp/math.apmod \
    build/sitl/tests/test_rust_sdk --gtest_filter=RustMathSDK.*
RUSTC=/path/to/rustc python3 -m pytest -q Tools/rust_modules/test_math_runtime.py -k 'not chibios'
```

For ChibiOS, build `examples/RustMath` with Rust enabled, then run the same
Python file with `-k chibios`, `RENODE=/path/to/renode`, and
`OWNED_THREAD_BOARD=MatekH743` (or `MatekF405`). The harness uses the real module
loader, AP_MultiHeap and worker threads, verifies zero module-heap peak usage,
and compares free memory before/after 20 reloads. The manager samples the HAL
thread registry while the worker waits at its final message, recording the
worker's fill-pattern stack high-water mark without formatting on that worker.
Its load-time scratch uses
the runtime's fast-memory policy; it is not charged to the module heap.

The category-expansion checkpoint builds all seven SITL vehicle targets, each
separately, with Lua enabled and disabled, and x86-64 Linux with Lua disabled.
That checkpoint used 1,612,920 bytes of H743 Plane flash (91,008 spare).
With enum/literal and string-input generation, H743 Plane with Lua+Rust uses
1,618,020 bytes (85,912 spare); the 13 libm adapters bring that to 1,618,832 bytes
(85,096 spare) with Lua still enabled. RC handle generation brings this to
1,620,056 bytes (83,872 spare). Complete Location/mission/Vector2f value adapters
and value inputs bring it to 1,636,900 bytes (67,032 spare), still with Lua enabled.
The bounded file reader brings this to 1,637,304 bytes (66,624 spare).
Rust-only libm growth may differ because
Lua already links these routines. Development targets 2 MiB boards, especially H7, for the
aerobatics demonstration. F405 is deferred: the earlier Rust-only Plane build
used 982,696 bytes (just 340 spare), before enum/literal expansion. Do not
silently remove public hashes to save space; disabled APIs must remain known
and return `Unavailable`. The current expansion adds 69,528 text bytes to the
Lua+Rust Plane SITL build compared with `79f9dacbddc`.

On both STM32 targets under Renode, the RTC/LED category probe passes 20 reloads
with zero module-heap use, unchanged available RAM, and 504 bytes of observed
worker stack use. The math probe uses 624 bytes. H743 Plane compiler stack-usage
reports show at most 64 bytes for a generated singleton adapter and 40 bytes
for the dispatcher. These are observations for the tested call paths, not a
bound on all downstream firmware methods or real-hardware validation. The
provisional host stack margin must still be validated for the full API before
hardware flight testing. The AirShow documentation describes its narrower
controller/flight tests and resource-comparison methodology.

The core ABI is generated from `generator/host_api.json`. Run
`python3 libraries/AP_Rust/generator/generate.py --check` to check the C header,
Rust declarations and ABI identity. This requires a host C compiler: the Lua
generator exports parsed metadata, so Rust does not duplicate its grammar.
ABI 3 keeps a single binding dispatcher but replaces global binding numbering
with hash imports. The fingerprint covers only the frozen core prefix (eight
C callbacks, status codes and layout). Appended callbacks keep that prefix
identity; old SDKs accept a larger host table. Incompatible prefix or entry
protocol changes require a new core ABI version.

`generator/contracts.py` hashes canonical public binding contracts with SHA-256,
taking the first eight digest bytes as a little-endian u64. Public names, input/
output directions, types and transitive wire layouts, optionality, policies and
an explicit semantic revision are included. Per-public-name revisions live in
`generator/contract_revisions.json`. Private C++ implementation names, Rust argument
names, dispatch numbers and feature guards are excluded. Declared enum type names
and fixed literal expressions are part of their public contracts: renaming a
literal's C++ constant conservatively changes its hash even if its value is unchanged.
Aliases retain their
own compatible public hashes. Generation rejects collisions. Compare an earlier
release catalogue with `generate.py --check --check-compatibility old-imports.json`.
This checks compatibility; it does not allocate IDs or serialize development.
A revision bump changes that public name's hash and rejects its old import.
Keeping old binaries working while changing behavior requires retaining the
old contract/implementation too, or introducing a new public API name. Aliases
have their own revisions. Bug fixes that preserve the contract need no bump.

Each used wrapper references a private four-byte import slot in a named ELF
section. The pinned fat-LTO/linker-GC build drops unused wrappers and imports.
The packer writes a 12-byte disk record per import (hash and slot offset).
Loading resolves and patches slots before publication. Calls use the resulting
firmware-private number with no hash lookup. The slots cost four RAM bytes per
used binding, within the image budget; the disk records are discarded with
load-time scratch. The firmware also stores a hash lookup table in flash.
Unknown imports reject loading and report the missing hash. Known but disabled
features retain their contracts and return `Error::Unavailable` at runtime.
Standalone loaders without a resolver can load only modules with no imports;
unresolved imports are no longer silently deferred to an Unavailable call.

The unchanged-binary compatibility tests renumber firmware dispatch slots and
remove an unused API, then execute the same compiled module; removing a used
API must fail loading. Cross-release compatibility is not yet a production
promise; these are experimental APIs, with ownership-sensitive bindings still
to be designed. Cached Parameter storage is queried from the firmware, so C++
size/alignment changes no longer require rebuilding modules using that protocol.
`tests/compat` holds frozen ABI-3 probe binaries (hex-encoded), their source and
the contract they require. Tests load these without recompilation in SITL and
on both Cortex-M targets. They were built with the pinned Rust 1.90.0 profile
and an 8192-byte declared stack. They are experimental compatibility checkpoints,
not a claim that ABI 3 or the remaining opaque-object APIs have been released.
The probes exercise one scalar math result, not optional outputs or Parameter.
Source provenance is `tests/compat/probe.rs` and the SDK in the commit introducing
these fixtures (parent `ee57af4141d`). They were built using
`Tools/rust_modules/build.py` with `--stack-size 8192` and respectively
`--target x86_64-unknown-linux-gnu` / `--target thumbv7em-none-eabihf`, with the
default link base. `Tools/rust_modules/test_imports.py` pins their SHA-256 values.
Do not regenerate them during routine SDK changes. A deliberate core ABI break
requires explicitly replacing the fixtures and documenting the lost compatibility.

The Python generator discovers singleton methods by signature category, not a
per-method selection list: 347 of 391 declarative singleton methods currently
generate in one pass. Adding another method with a supported signature requires
only changing `bindings.desc` and regenerating. Supported categories are void,
boolean, signed/unsigned 8/16/32-bit integers, float, and audited complete values
(Location, MissionItem, Vector2f, Vector3f and Quaternion). Nullable outputs share one presence flag and become
`Option<T>` or `Option<(T, U, ...)>`; void methods with `Ref` outputs return values
or tuples without a flag. For example, `context.rc_get_pwm(1)` returns
`Result<Option<u16>>`, and `context.led_get_rgb()` returns `Result<(u8, u8, u8)>`.
The generator preserves ranges, feature guards, aliases and all three Lua lock
styles. Generic float `skip_check` accepts all f32 bits, as Lua does; the existing
Parameter adapter retains its bounded names and finite-float policy.

Enum inputs/results use signed 32-bit wire values and distinct Rust newtypes in
`enums`. The IR does not yet export enum constants; use documented protocol or
parameter values and verify them against firmware in tests. Inputs retain Lua's
description range checks, with an additional underlying-type representability
check before casting. For unfixed C++ enums the description range must also fit
the enum's legal value range. Results outside i32 return `Error::Host` after the
host call has run. These representability checks are deliberately stricter than
Lua's casts. Enum reference outputs are not supported. Expression-valued range
bounds remain firmware runtime checks; narrowing a published range requires a
contract revision if it changes the API promise. Fixed literal arguments consume
no wire bytes or SDK argument and retain their position in the C++ call.

The enum/literal checkpoint tests disarmed SITL mode changes with SCRIPTING as
the reason, RC/servo queries alongside Lua, and restart of two workers running
pre-arm checks. The H743 Renode mount-enum probe passes 20 reloads with zero
module-heap use, exactly restored available RAM, and 432 bytes observed worker
stack use. It does not validate real flight timing or all downstream call paths.

General string inputs use a separate, allocation-free 256-byte wire field:
UTF-8 `&str`, at most 255 bytes, empty allowed, no embedded NUL, canonical zero
padding. Unlike Lua's arbitrary byte strings, oversized or embedded-NUL input
returns `Invalid`; it is not silently truncated by the SDK. Firmware still
applies its usual destination-specific truncation. Two-string calls reserve
512 bytes in the wrapper's input buffer. Parameter names retain their original
17-byte/nonempty protocol and hashes. Host methods must consume or copy strings
within the call, as required by Lua's collected strings; reviewers must audit
new string-taking methods for retention and concurrency.

Effects are recorded in generated documentation: display calls can block and
retain overrides, CRC scans cannot be interrupted by stop, and logger file
requests allocate firmware-owned queued filenames outside the module heap,
just as in Lua. That queue survives module exit and can grow without bound if
requests outpace logging. Persistent motor-frame/ONVIF initialization remains
excluded pending ownership/concurrency support. String results are not exposed.

The string probe matches Lua's text, named float/int/string telemetry and file
CRC in SITL across three loads. H743 Renode checks SD-file CRC over 20 reloads,
with zero module heap, exactly restored available RAM and 1,784 bytes observed
worker stack use. The Renode harness has no GCS singleton; live telemetry is
covered by SITL, not that stack measurement. Both maximum-length arguments of
the two-string wrapper are exercised, but not additional application formatting.

`Context::libm_*` provides individually hashed C float math for no_std modules:
`acosf`, `asinf`, `atan2f`, `ceilf`, `cosf`, `expf`, `floorf`, `fmodf`, `logf`,
`powf`, `sinf`, `sqrtf`, and `tanf`. These call the board's existing C library,
as Lua's `LUA_32BITS` math does, without linking another library into each module.
They allocate nothing and remain stop-gated `Result<f32>` calls. Non-finite
inputs and domain errors follow the board's floating-point environment, including
enabled traps; they are not converted to Rust errors or sanitized. Results are
not promised bit-identical across different boards/C libraries. These manual
standard-library adapters are separate from the `bindings.desc` coverage count.
Lua's integer `fmod`, integer-returning `floor`, radians/degrees conversion, and
NaN-sensitive min/max rules still need explicit handling in application ports.
Lua specializes exponent 2 as multiplication; use `x * x` for squaring, not
`libm_powf(x, 2.0)` when reproducing that behavior.
All 13 functions pass a same-board, bit-for-bit Lua comparison in SITL across a
restart. The H743 Renode probe performs 100 passes through all 13 and 20 module
reloads: zero module heap, exactly restored available RAM, and 936 bytes observed
worker stack use. The host packet tests compare against direct C calls at both
`-O0` and `-O2`, including special float values with traps disabled in that test.

Pointer-backed objects are selected by an explicit per-type lifetime strategy,
then their methods use the same signature codecs. The first strategy covers
all five `RC_Channel` methods and both channel queries. `RcChannel` is a copyable
nonzero channel index (four bytes, also when optional), not a C++ pointer or an
allocation. `rc_find_channel_for_option` and `rc_get_channel` return `Option`.
Calls such as `channel.norm_input(context)` require a live `Context` and look up
the channel again. A cached handle keeps the same channel when an RC option moves,
like Lua's cached object. Zero/out-of-range input tokens are `Invalid` before
locking; absent owners or channels are `Unavailable`. Unknown returned pointers
are host errors. No firmware layout or address is part of this protocol.

Native RC object calls take the scheduler semaphore, unlike Lua's unlocked
object methods. This serializes the `last_override_time`/`override_value` update
and shared override bookkeeping with the main loop and other native calls.
It does not protect existing unlocked Lua or other callers. Reads use the same
lock and can return `Busy`; callers must sleep/retry. `set_override` changes
shared pilot input: zero releases it, otherwise the configured timeout applies
(which can be disabled), and module exit does not undo it. Disabled overrides
and the override-enable channel silently ignore the request, as in Lua.

Disarmed 5x SITL comparisons cover normalized inputs, override release, a cached
channel after its option moves, and three restarts, with and without Lua.
Both sampled runs reported zero `Busy` results in 200 update attempts; this is
not a flight-load contention bound. The H743 Renode probe passes 20 reloads,
returning free memory exactly, with zero heap use and 416 bytes of worker stack.

Plain value types have explicit component wire layouts, never C++ object bytes.
Location's five flags include the hidden `loiter_ccw`; MissionItem includes all
15 MAVLink fields, including four not exposed by Lua. Hidden fields stay private
and survive get/modify/set cycles. Location fields are private: `lat()` reads
locally, while `set_lat(context, value)` validates and assigns through a generated
host adapter. The same applies to the other Lua-visible fields. In particular,
altitude bounds use the firmware macro, not a duplicated SDK constant. Default
values match the zeroed Lua constructors. Vector2f and visible MissionItem fields
are public and unchecked, like their Lua skip-check fields.

New value methods and value inputs conservatively use `&mut` and copy back all
state on host-call success, including `false` results and absent optional outputs.
Host errors leave Rust values unchanged. Fields produced by firmware are not
revalidated on reuse; Lua likewise validates field assignments, not existing
userdata. Two arguments cannot alias the same mutable Rust value; make a copy
when needed. Existing audited Vector3f/Quaternion methods keep their original
directions and hashes. Additional methods (including Vector3f copy/xy), new
value methods, field assignments and Vector2f operators are generated by category.
No value call allocates from the module heap.

Host size/offset assertions and source-member tests detect layout drift, including
Location flags and MAVLink extension fields that could otherwise fill padding
without changing `sizeof`. A new semantic member requires a wire-policy audit;
simply changing host layout must not change the public wire layout. Codec tests
cover hidden fields, zero defaults, false-result copyback and assignment bounds.
H743 Renode passes 20 value-probe reloads with exact free-memory recovery, zero
heap use and 648 bytes of worker stack. Lua-enabled disarmed SITL compares Location
arithmetic bit-for-bit with Lua; both variants read back a Rust-written mission
item over three loads.

`context.file_read_at(path, offset, buffer)` reads 1–256 bytes from a local/SD
file using AP_Filesystem, reopening and closing within each call. It returns
`Some(length)`, including `Some(0)` at/past EOF. `None` means **any open failure**,
not just a missing file: permission, no card, memory or file-table exhaustion
cannot be distinguished. This matches the nil result from Lua's `io.open`, without
its error-string detail; never treat it as permission to overwrite existing data.
Seek/read/close failures return `Host` and do not modify the caller's buffer.
Short reads are accumulated until the buffer fills or EOF is reached.

Paths are nonempty UTF-8, at most 255 bytes, with no NUL. A first component starting
with `@` (after any leading slashes) is rejected so virtual snapshot files are not
regenerated on every chunk. Ordinary `..` components are allowed, as in Lua; this
is not a filesystem sandbox. Offsets plus requested lengths must fit signed 32
bits. Calls do not form an atomic snapshot: do not replace a schedule during a
read; an applet can additionally compare file CRCs before and after reading.
Reopening per chunk is intended for small startup schedules, not streaming IO.
Cache data in the module's capped heap. FS-owned transient open-file allocations
are outside that heap, but no descriptor or buffer is retained by this adapter.

File IO holds no scheduler semaphore. It may block on filesystem locks/card
retries, and stop cannot interrupt it. The existing ten-second cooperative stop
timeout still applies; an old worker is never forcibly freed. The H743 real-FATFS
Renode probe performs 50 reads per load over 20 reloads, exceeding the 16-descriptor
table limit, with exact free-memory recovery, zero module-heap use and 1,760 bytes
of measured worker-stack use. This is not a worst-case stack bound. File-reading
modules should declare at least 8 KiB of module stack for this prototype; retain
the separate 4 KiB Cortex-M host margin and measure the complete applet call path.
Both POSIX SITL and FatFS test offsets inside, at and beyond EOF and directory
failures. Fault-injection tests cover seek/read/close failures and partial reads.

`context.log_write(name, labels, units, values)` writes dynamic DataFlash records
using the same AP_Logger registry as Lua. `LogValue` selects the Lua-supported
format codes (b/B/h/H/i/I/Q/f/c/C/e/E/L/M/n/N/Z); scaled integer variants do not
scale their input. Fixed byte strings are zero-padded. TimeUS is supplied by the
host. Names are 1–4 bytes, labels at most 57 bytes, and records contain 1–15 values
with at most 244 payload bytes. Labels must be nonempty and unique; optional
unit/multiplier strings have one valid ID per value. This stricter validation
avoids malformed records, including the one-byte DataFlash length overflow.

Keep names and metadata stable across calls, modules and Lua scripts. Metadata
and message IDs are shared firmware resources, outside the module heap, retained
until reboot and reused across reloads. Conflicting dynamic formats return Invalid
on hardware; SITL's existing logger raises config_error during lookup, as with Lua.
The registry does not prevent collisions with built-in log names: choose distinct
names. Success means best-effort submission, not storage durability. Logging
disabled or absent returns Unavailable. No scheduler semaphore is held.

SITL tests decode actual DataFlash files, including all supported types, exact
64-bit payloads, units, shared Lua/Rust formats and three module loads. The H743
Renode registry/record-assembly probe (without a storage backend) measures 1,416
bytes of stack, zero module-heap use, and 544 bytes of initial persistent firmware
allocation; free memory then recovers exactly across 20 reloads. This is not a
worst-case logging stack bound. Lua+Rust H743 Plane uses 1,638,980 flash bytes with
64,952 bytes free in this board's application partition at this checkpoint.
The logging-disabled H743 example also builds and runs in Renode. A full
logging-disabled SITL Plane build is blocked by pre-existing AP_CustomControl
logging and Plane::should_log references; no unrelated libraries were changed.

`generator/coverage.json` records every exported singleton/userdata IR method and
AP-object method/alias with its status and reason. Global aliases remain unexported;
fields, enums, constructors and operators are
explicitly outside the coverage count.
Unsupported categories currently include enum reference outputs, 64-bit scalars,
string outputs, additional value types, borrowed/resource
handles and manual Lua implementations. `generator/bindings.json` now only
selects audited Parameter and legacy math adapters; new plain-value methods use
conservative copyback instead of per-method mutation metadata. The full aerobatics
single-aircraft aerobatics API is covered; optional dual-aircraft MAVLink manual
bindings remain unsupported.
`generator/binding_policies.json` holds explicit lifecycle exclusions, header
substitutions and audited nonstandard accessors, each with a reason. Ordinary
new singleton methods need no entry there. Changed audited accessors fail
generation; new unaudited accessors appear as unsupported in the coverage report.

Bindings have the same shared-state effects as Lua. Mission changes, GPIO writes,
guided targets, LED overrides and policy switches are not undone on module exit.
In particular every OSD call, even a query, claims the display until the firmware
override timeout (currently two seconds without another call). This behavior is
documented in the generated methods; it does not imply per-module ownership.
The OSD getter takes its own untimed semaphore before argument validation and
the timed binding lock; neither a stop request nor the 20 ms lock timeout
interrupts that wait. Even a later-rejected call renews the display override.
Resource registrations without a reload-safe ownership policy remain excluded.
The full pre-expansion contract catalogue from `79f9dacbddc` is frozen in
`tests/compat/imports-v3-layout.json`; tests verify all its hashes still resolve.

The generated dispatcher validates exact buffer lengths, singleton availability
and static ranges before taking locks. A lock wait exceeding 20 ms returns
`Error::Busy`; callers can retry after sleeping. Propagating it out of `start`
instead reports a module error. Optional C++ results become `Ok(None)`, not
errors. Outputs are zeroed after successful buffer validation, including on
unavailable or busy calls. Input and output buffers must not overlap. Safe SDK
wrappers enforce their sizes and lifetimes; unsafe native pointers are trusted,
not sandbox-validated. Locks are released before returning to Rust.

### Parameters

`param_get`, `param_set`, `param_set_and_save`, `param_set_default`,
`param_add_table` and `param_add_param` follow the corresponding Lua methods.
Names/prefixes are nonempty, at most 16 bytes and contain no NUL. The wire uses
17-byte NUL-terminated, zero-padded fields, without an extra character-set
restriction. Setters reject NaN, infinity and values outside the destination
integer range; accepted fractional integer inputs truncate toward zero.
For example, 126.5 can be written to an int8 parameter (as 126), but 127.5
is rejected because it exceeds 127 before truncation.
Missing names return `Ok(None)` for reads or `Ok(false)` for setters.
As with Lua, callers should use printable ASCII names: control characters or
non-ASCII names can break parameter files and ground-station displays. Reads
return `f32`, which cannot represent every integer above 2^24 exactly.

Registration calls `AP_Param::add_table_persistent`, not Lua's legacy reset
path. Table metadata and copied names belong to the firmware until reboot,
outside the module's AP_MultiHeap. Identical registration reuses this storage.
Registration reloads saved values; unsaved changes can reset to defaults.
Existing indices cannot be renamed or the table grown within the same boot.
Ten table slots and keys 0–200 are shared with Lua. Prefix, capacity and
legacy/persistent ownership conflicts return `Ok(false)`, as do other
registration failures; the existing API does not distinguish their causes.
No module heap must remain alive just to retain a parameter table.

Writes have Lua's authority to change any named parameter. This is trusted
native code, not an access-control boundary. Registration writers are
serialized by AP_Param; ordinary parameter reads/writes retain the existing
firmware concurrency semantics, not general multi-core synchronization.
The runtime defaults list has its own reader/writer lock; reloads of external
defaults-file override arrays still follow existing initialization conventions.
By-name lookup scans the parameter table (twice for checked setters); these
calls suit occasional use. For repeated access, `context.parameter("NAME")`
returns `Result<Option<Parameter>>`. This wraps the same C++ `Parameter` helper
as Lua: `get()` reads the current value, and `init`, `set`, `set_and_save`,
`configured` and `set_default` call the same implementations. `init_by_info`
is not exposed yet because it allocates legacy-conversion parameter storage
with different ownership. A missing constructor name returns `None` rather
than raising Lua's error; a missing name passed to `init` returns false and
clears the binding. Invalid names leave an existing binding unchanged.

Each helper queries the firmware's size/alignment once at construction, then
allocates that storage from the module heap (currently 8 bytes on STM32,
16 bytes on x86-64), plus allocator bookkeeping. There is no extra query on
`get()` or `set()`. The SDK borrows its context,
keeps the C++ allocation at a fixed address across Rust moves, and frees it
on drop, including after stop and constructor failure. It cannot be sent or
shared across threads in safe Rust. Forgotten helpers are reclaimed with the
module heap after the worker joins. The wire carries a trusted native storage
pointer, not a sandbox-validated handle. The SDK never exposes that pointer
through a safe API. The host constructs the object and asserts trivial
destruction and the hashed layout protocol's limits (size at most 16 MiB,
power-of-two alignment at most 4096). These protocol caps match the module heap
limits and are part of the contract, not the current object's size.
Nontrivial destructors are not supported:
ordinary binding calls are stopped before Rust drops its wrappers, so those
would need a separate cleanup path. The initial fixed-layout Parameter hashes
are deliberately no longer supported; those experimental modules must rebuild.
This does not change the core ABI or unrelated math/binding hashes.

There are no new restrictions on parameter names or owner groups beyond the
existing native name encoding and numeric checks. Cached firmware pointers
have the same lifetime assumptions as Lua's helper; this does not solve
backend-object lifetime hazards or add synchronization of scalar values.

Saving queues the parameter object, not a value snapshot, and success does
not mean storage has been flushed. `param_set_and_save` and first-time table
registration can wait for the IO worker when the save queue is full; registration
and default updates can also wait for AP_Param's internal locks. An
in-flight call is not interruptible by module stop. A stalled worker therefore
causes the normal restart timeout/resource retention rather than forced
cancellation. The next host binding call checks stop before starting new work.
Firmware-owned metadata is not yet included in the manager's partial memory
accounting. `test_parameters_runtime.py` checks reload/reboot storage behavior;
the ChibiOS manager test also checks saved-value restoration and exact free
memory after repeated warm reloads. Existing experimental ABI-2 modules require
rebuilding for ABI 3; unrelated new bindings no longer force another rebuild.

Run `python3 -m pytest -q libraries/AP_Rust/generator` for generation and
dispatcher checks, and `Tools/rust_modules/test_bindings_runtime.py` with
pytest for vehicle readback/reload tests. The latter can compare Lua and Rust
simultaneously; set `AP_RUST_TEST_NO_LUA=1` for a Lua-disabled firmware build.

A module uses `#![no_std]`, `#![no_main]` and `ap_rust_sdk::module!(run)`:

```rust
fn run(context: &ap_rust_sdk::Context<'_>) -> ap_rust_sdk::Result {
    loop {
        context.text(6, "Rust module alive")?;
        context.sleep_ms(1000)?;
    }
}
```

The context cannot be sent or shared across threads. Sleep propagates stop
requests as `Error::Stopped`, so ordinary Rust locals are dropped on return.
Each loaded image can enter once. Its allocator calls the host with the
module's opaque context; the tests implement this with AP_MultiHeap. Fallible
allocation returns an error. Infallible allocation failure and explicit panic
reach the host panic callback without unwinding. Panic messages are bounded
and do not invoke user formatting: nonliteral formatted messages are omitted.
Truncated panic output is bytes, not necessarily valid UTF-8. Stack declarations
must leave space for the 192-byte panic buffer and the panic call path.

The host must keep its API/context alive, never unwind or retain pointers into
module data, and park panic-failed threads indefinitely rather than reaping
them. `Module` and the vehicle manager implement that ownership rule.
SDK callback tests terminate a child process after observing the
callback; module-owner tests additionally exercise actual parking and retention.

## Vehicle runtime

Enable `RUST_ENABLE` and reboot to load `*.apmod` files from `/APM/rust` on
SD-card targets, or `./rust` on SITL/Linux. Loading is filename-sorted and
validates all images before starting any workers. `RUST_MAX_MOD` bounds the
file count; extra files cause an error instead of being ignored.

`RUST_HEAP_SIZE`, `RUST_CODE_SIZE` and `RUST_STACK_SIZE` set separate
per-module limits at boot. RAM messages report individual reservations and
heap usage, not a complete physical-memory total: backing-allocator overhead,
manager storage and temporary file buffers are additional. These defaults
are experimental, not a recommendation for every board.
ChibiOS owned stacks prefer fast RAM, competing with other fast-memory users;
they can fall back to other heaps. Stacks need not be DMA-capable.

Change `RUST_RESTART` to stop and reload all modules, including while armed.
Workers must cooperate: no thread is forcibly cancelled. A ten-second stop
timeout retains live workers and prevents a new generation. Late exits are
reaped, but another explicit request is needed to retry. A panic parks its
worker until reboot. The manager has higher priority than workers on ChibiOS
and Linux FIFO; spinning modules can still starve Lua and other workers.

Pre-arm checks use the same SYSTEM-check and bypass paths as Lua. Normal
return and an empty directory are not failures. Optional `RUST_LD_CSUM` and
`RUST_RUN_CSUM` are masked XORs of loaded/running file CRC32 values, with -1
disabling each requirement. `RUST_DEBUG_OPTS` bit 4 bypasses Rust pre-arm
checks; bit 5 saves both checksums and clears itself in storage. Errors do not
automatically disarm an already armed vehicle.

`Tools/rust_modules/test_runtime.py` exercises the manager in a real SITL
Plane process. Its restart-memory test checks anonymous mapping reservations;
that alone does not detect suballocation leaks. ChibiOS manager tests live in
`Tools/rust_modules/test_manager.py`, separate from the module-owner harness.
Build `examples/RustRuntime` for MatekH743 (the default) or set
`OWNED_THREAD_BOARD=MatekF405` for its SD-card-equipped Cortex-M4 target.
Its GCS is a capture stub; the measured stack headroom is not a measurement
of the full vehicle's GCS queue path.
Neither simulation nor compilation establishes real-hardware flight safety.

Disabled-build accounting: compiling the vehicle and arming integration
before/after this manager with Rust disabled gives identical defined symbols
and symbol sizes. Earlier foundational changes on this branch increased the
observed Lua-only SITL Plane text from 4,879,884 to 4,880,172 bytes (+288),
with data (220,469) and BSS (245,800) unchanged. That aggregate earlier cost
is recorded separately; it is not a Rust-versus-Lua aerobatics benchmark.
The SDK includes conservative
volatile memory helpers to avoid libc imports; optimize only after measuring.

```sh
RUSTC=/path/to/rustc python3 Tools/rust_modules/build.py \
    Tools/rust_modules/sdk_probe.rs /tmp/sdk-probe.apmod \
    --target x86_64-unknown-linux-gnu --stack-size 8192
./waf --targets tests/test_rust_sdk
AP_RUST_TEST_SDK_MODULE=/tmp/sdk-probe.apmod build/sitl/tests/test_rust_sdk
```

Use `thumbv7em-none-eabihf` for the Cortex-M4-compatible hard-float image.
The stack declaration describes module use; host thread budgets must separately
cover backend overhead and margin. These declarations are not stack proofs.

## Module-owner lifecycle

`Module::load` validates compatibility and limits, relocates and publishes the
image, and releases its reference to the caller's input bytes before returning.
`start` creates the bounded heap and one scripting-priority worker. Prepare
all images before starting workers so ordinary heaps/stacks do not consume
scarce executable SRAM first. `request_stop` interrupts SDK
sleeps. `release` refuses to delete a live worker. A normal return can be joined
and freed; panic retains the image, heap, context and stack until reboot.
All manager-side methods must have one owner. The firmware text sink runs on
the worker, must be thread-safe, and must outlive the module.

Limits apply separately to the heap capacity, page-rounded image and reserved
stack. Known thread/guard and image-handle overhead, object metadata and heap
used/peak are reported separately. These are not complete physical RAM totals:
Unused backing-heap capacity, MultiHeap descriptors, backing-heap bookkeeping
and opaque OS overhead still need separate accounting. ChibiOS's default-heap
executable allocator reserves an additional 4 KiB per image so alignment
padding is owned, reported and reclaimable; fixed-region heaps do not need it.
The provisional host stack margin is 4 KiB on
Cortex-M and 16 KiB on x86-64; measure the complete binding stack before flight
use. The Cortex-M text sink must use at most 2 KiB and queue formatting elsewhere.

The Linux HAL unit suite tests two live modules, 100-cycle normal exit and
cleanup allocation, forgotten allocations, late/non-cooperative exits, panic
and infallible OOM retention, and low CPU use during parking:

```sh
for mode in cooperative return late panic oom cleanup leak never; do
    AP_RUST_LIFECYCLE="$mode" RUSTC=/path/to/rustc \
        python3 Tools/rust_modules/build.py \
        libraries/AP_Rust/tests/modules/lifecycle.rs /tmp/lifecycle/$mode.apmod \
        --target x86_64-unknown-linux-gnu --stack-size 8192
done
./waf configure --board linux --extra-hwdef libraries/AP_Rust/tests/enabled.hwdef
./waf --targets tests/test_rust_module
AP_RUST_TEST_MODULE_DIR=/tmp/lifecycle build/linux/tests/test_rust_module
```

The standalone `RustModule` ChibiOS example loads fixtures from an SD image,
executes 100 two-module stop/reload cycles with exact free-memory comparison,
then retains a panic-parked worker while its equal-priority harness continues.
It is a lifecycle harness, not the vehicle's boot loader or restart manager.

```sh
./waf configure --board MatekH743 --extra-hwdef libraries/AP_Rust/tests/enabled.hwdef
./waf --targets examples/RustModule
RUSTC=/path/to/rustc RENODE=/path/to/renode \
    python3 -m pytest -q -s Tools/rust_modules/test_lifecycle.py
```

Build `MatekF405` and set `OWNED_THREAD_BOARD=MatekF405` for the F405 SD-card
test (KakuteF4 has no SD card). These tests require mtools. Timing tests on
SITL must run with a simulated vehicle: the standalone example/unit-test clock
advances immediately instead of sleeping, so it cannot validate panic parking.
