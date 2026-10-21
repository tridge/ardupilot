# Native module design

`AP_Module` provides trusted native extensions with explicit image, heap, worker
and resource ownership. It is independent of Lua but shares scripting-level
APIs/facilities where appropriate.

See [README.md](README.md) for build/install instructions and operator settings;
this document describes the current implementation contracts.

## Trust and platform boundary

Modules execute with firmware privileges. Checksums detect corruption, not
authenticity, and Rust's type system is not an isolation boundary. Bad pointers,
arbitrary faults, unbounded host calls and locks held indefinitely can affect the
vehicle. Low priority and the optional stack guard do not make untrusted modules
safe. There is no general fault containment or per-module CPU quota.

Supported profiles are Thumb-2 Cortex-M4-compatible hard float on STM32 and
baseline x86-64 Linux for SITL/Linux. Rust builds require Rust 1.98.1 or newer;
C++ builds require Clang/standalone LLD and do not invoke Rust tools.
Thumb uses ROPI/RWPI; x86-64 uses PIC. Fat LTO applies that model to linked Rust
bitcode, including core/alloc. Modules use `no_std`, panic-abort semantics and
SDK memory helpers, without host libc or system startup imports.

## Binding generation and ABI

The Python Lua generator parses `bindings.desc` and exports versioned JSON IR.
The native generator consumes this IR rather than duplicating the grammar. It
generates firmware adapters, private dispatch IDs, Rust and C++ clients, manifests, contract
catalogues and coverage reports. Outputs live under build directories. Waf
tracks both generators' inputs; standalone and Cargo builds generate their own
outputs.

Singleton methods are selected by signature/type category. Audited policies
describe ownership, lifetime, nonstandard accessors and exclusions missing from
the Lua description. Values use explicit component wire layouts, not C++ object
bytes. Complete Location and mission-item copies preserve hidden semantic fields.
New value methods conservatively copy back mutable inputs on host success,
including false results; host errors leave client values unchanged.

Pointer-backed objects need a lifetime strategy: RC channels, for example, use
revalidated indices rather than exported C++ pointers. Resource adapters track
ownership against module contexts. Unsupported categories are recorded with
reasons in `generator/coverage.json`. Manual Lua functions require deliberate
native adapters.

The C-compatible entry receives a host API table and opaque context and returns
an `int32_t` status. Rust wraps its entry using `module!`; C++ uses `AP_MODULE`. C++
classes, Rust trait objects, slices and native strings do not cross the ABI.
Core calls use fixed-width scalars, explicit lengths and defined C layouts.

`generator/host_api.json` defines core ABI version 3: eight required callbacks
for allocation, deallocation, time, sleep, stop, text, panic and binding dispatch.
The 16-byte fingerprint covers that required prefix, status codes and layout.
Appended callbacks may preserve the prefix; incompatible prefix/entry changes
require a new core ABI version.

Each binding hash is the first eight SHA-256 digest bytes of its canonical
public contract, interpreted little-endian. Inputs include public names, types,
directions, transitive wire layouts, optionality, ranges, lock policy and semantic
revisions. Private implementation names, dispatch numbers, source order, guards
and unrelated APIs are excluded. Enum type names and fixed literal expressions
are included. Collisions fail generation. Semantic changes not expressed in the
contract need a revision in `contract_revisions.json`.

Each used wrapper references a four-byte ELF import placeholder. Linker garbage
collection removes unused wrappers/imports. The packer retains live hashes,
slots and diagnostic names. Loading resolves them to firmware-private `BindingId`
values before execution; calls do not look up hashes. Unknown imports reject
loading with their name/hash. Known compiled-out APIs retain contracts and return
`Unavailable`. Unrelated binding additions do not invalidate a module.

Adapters trust buffer sizes supplied by hash-compatible generated wrappers and
omit repeated size checks. Pointers are trusted, not sandbox-validated. The wire
is little-endian and input/output buffers must not overlap. Availability, ranges,
values and handle ownership are still checked. Common codecs avoid C++ padding.
Completely overwritten outputs are not pre-zeroed; absent/variable payloads are
initialized where required for canonical output.

Semaphore acquisition blocks. Locks are released before returning to Rust;
firmware does not call back into module code. SDK proxies borrow a context that
is neither `Send` nor `Sync`. Ordinary state changes retain Lua semantics and
are not rolled back on exit. A stop request cannot interrupt an in-flight host
call.

## Container and relocation

The offline packer links ELF, resolves image-local instruction/PC-relative
references and emits `.apmod`. Firmware is not an ELF linker. Unsupported
sections, undefined symbols, TLS, constructors, unwinding and unsupported
relocations are rejected. ARM GOT/dynamic-linking tables and static Thumb
MOVW/MOVT instruction patches are unsupported. x86-64 GOT entries must resolve
to defined image symbols.

The current runtime format is `APMOD06`, little-endian with an 84-byte header:

| Field | Encoding |
| --- | --- |
| Magic | 8 bytes, `APMOD06` followed by NUL |
| Version, ELF machine, pointer width | Three u32 values |
| Segment alignments | u32: low 16 bits RW, high 16 bits RX |
| Initialized size, memory size, RX boundary, entry offset | Four u32 values |
| Fixup count, ABI version, CPU profile, stack requirement, initial heap, import count | Six u32 values |
| Core fingerprint | 16 bytes |
| Container CRC32 | u32 |

Initialized image bytes follow, then sorted u32 pointer-fixup destination offsets,
sorted 16-byte import records (`u64 hash`, `u32 slot`, `u32 name offset`) and
canonical NUL-terminated API names. Fixup slots hold canonical image-relative
target offsets. Imports are sorted by slot; at most 4096 imports and 63-byte names
are accepted. Duplicate hashes/slots, invalid placeholders and overlaps with
pointer fixups are rejected.

The logical RX boundary is word-aligned on ARM and 4096-byte aligned on x86-64
for POSIX permission separation. Segment alignments are powers of two from
pointer width through 4096, derived from ELF requirements. RAM-loaded ARM RX
allocations isolate 32-byte cache lines and honor larger ELF alignment; ROMFS
payloads need only their ELF alignment. RW includes
initialized data and zero-filled BSS. Parsing checks bounds, overflow, entry,
fixup destinations/targets, imports, target, core compatibility and configured
budgets before publication. Stack declarations are trusted metadata, not a proof.

CRC32 covers the canonical file with its header CRC zeroed. Runtime loaded/running
checksums separately cover the complete canonical file including the stored CRC.
ROMFS prelinking preserves both identities by normalizing patched RX slots.

All old `APRUST` formats are rejected. `APMOD01` (NUL-padded) is a probe accepted
only by the separate test parser with `AP_MODULE_LEGACY_PROBE_ENABLED`.
Current SDK builds emit version 6; core ABI version 3 is separately versioned.
These are not released compatibility promises. Frozen `tests/compat/` fixtures
and catalogue checks detect unintended changes; do not routinely regenerate them.

## Executable memory and ROMFS

ARM ROPI/RWPI allows separate RX/RW bases. The entry trampoline saves/restores
`r9` and installs the module RW base. SD loading copies both segments. Absolute
RX-to-RW fixups are rejected; RX-to-RX pointer slots can be resolved by the
firmware linker. Target offsets preserve byte-pointer and Thumb-function bits.

ROMFS stores `.apmod` entries raw while other files follow normal compression
policy. Firmware generation prelinks ARM RX-to-RX slots and places entries in
executable flash, preserving payload alignment after the 84-byte header. Borrowing
requires HAL confirmation that the complete range is immutable executable flash;
filename or ROMFS membership alone is insufficient. RW/BSS is always copied,
fixed up and owned. Borrowed RX has firmware lifetime and is never written/freed.

`AP_ROMFS::find_uncompressed` exposes raw storage for loading. Ordinary filesystem
reads reconstruct canonical module bytes and validate integrity. Globally
uncompressed ROMFS builds do not prelink: they preserve existing borrowed-file
semantics and the runtime copies RX.

STM32 executable allocation selects instruction-capable SRAM, not arbitrary
malloc memory or F4 CCM. Publication performs cache maintenance/barriers, without
per-image MPU isolation. POSIX uses dedicated contiguous mappings for PIC: copy
RX/RW, publish the code prefix as RX and leave data RW/non-executable. SITL ROMFS
arrays are not executable and are copied. Emulation does not establish real
cache behavior.

## Module and generation ownership

`Image` parses/relocates without owning buffers. `Module` owns code, mutable data,
heap, host context and one worker. File source bytes can be released after load;
borrowed ROMFS RX remains valid for firmware lifetime. HAL owned threads retain
stack ownership until join/reap, unlike fire-and-forget threads.

`Runtime` discovers nonrecursive `*.apmod` files in ROMFS `modules/` and the writable
directory, deduplicates basenames with ROMFS precedence/warning, and sorts before
applying the module limit. All images are prepared before workers start so stacks
and heaps do not consume scarce executable RAM first. Partial startup failure
stops/reaps the new generation. The vehicle main loop never waits for worker exit.

Restart requests stop for all workers. SDK sleeps notice stop promptly; entry
functions return normally so destructors run. Bindings reject new work after
stop, while allocation/deallocation remain available within the heap budget
through cleanup. The manager joins returned workers before releasing code,
handles, data, heap or stack. Threads are never forcibly cancelled and no new
generation runs alongside live users of the old one.

A ten-second timeout retains the stopping generation and reports failure. Late
exits can be reaped; an explicit request retries loading. Panic/infallible OOM
invoke a host callback without unwinding through C++, mark failure and park until
reboot with resources retained. The SDK never uses whole-firmware panic or longjmp.

Host resources are scoped to the owning context and cleaned up only after users
join. No module function pointers are registered with firmware. DMA-sensitive
adapters use host storage or driver/filesystem bounce-buffer paths: worker stacks
need not be DMA-capable. Ordinary vehicle state is not restored implicitly.

## Memory, scheduling and health

Budgets independently cap backing heap capacity, RX plus RW/BSS image and worker stack.
The container declares an initial heap and module stack requirement. Admission
checks the initial heap against `MOD_HEAP_MAX` and declared stack plus host-call
margin against `MOD_STACK_MAX` before image allocation. Workers allocate only
those per-module requests. Armed expansion cannot exceed the heap cap.
Image limits round to 4096; actual allocation follows image size, not the limit.
HAL stack rounding and OS/guard overhead are reported separately. Host-call margin
is 4 KiB on Cortex-M and 16 KiB on x86-64, beyond the module declaration. Manager
stack requests are 4 KiB on ARM and 32 KiB on x86-64. Cortex-M text callbacks have
at most 2 KiB additional stack budget and queue formatting elsewhere.

The aligned AP_MultiHeap adapter serializes operations and tracks raw allocations
for aligned frees and reclamation. Free walks live allocations to check ownership.
Lua-style armed expansion retains a 20 KiB system reserve unless disabled for the
generation; it latches a warning/pre-arm error without stopping the worker.
Reports distinguish used/peak heap, capacity, owned RX/RW, stack/thread reservations
and metadata. Backing allocator, loader scratch, manager and persistent firmware
storage are additional, not a complete physical-memory total.

Parameter definitions and logger formats are firmware-owned until reboot, outside
module heaps. Identical parameter registration reuses metadata and restores saved
values; unsaved values can reset. Lua/Rust share dynamic-table limits. Cached
`Parameter` queries host size/alignment once, placement-constructs the shared
trivially-destructible helper in module-owned storage and frees it on drop.
C++ layout is not part of the public wire contract.

Workers use scripting priority; the manager is one priority above on priority
HALs, below vehicle/I/O work. ChibiOS's normal zero quantum means no peer
round-robin: busy workers can starve other modules and Lua. Positive sleeps
cooperate; zero sleeps and stop polling do not yield. Linux behavior depends on
`SCHED_OTHER` versus FIFO policy.

Health participates in SYSTEM pre-arm checks and existing disable/force-arm
bypasses. Errors latch until the next generation; empty directories/normal returns
are not failures. Loaded/running requirements are 23-bit-masked XORs of canonical
whole-file CRC32 values, reset each generation. Normally returned modules remain
loaded but no longer running. Armed restart follows the same ownership rules;
Rust errors never automatically disarm an already armed vehicle.

## Optional H7 stack-fault recovery

`HAL_STACK_GUARD_ENABLED` defaults off and is limited to supported H7 builds,
excluding bootloaders and IOMCU. Guarded workers have a 1 KiB-aligned
no-access area below the working area. MPU region 7 follows the guarded thread
at context switch; the null-pointer guard uses region 3.

USB debugging can be enabled in the same build. Its fault handler first repairs
guarded debugger probes, then attempts stack recovery before inspecting an
application exception frame or completing lazy FP stacking. A refused recovery
retains the normal debugger/crash path; incomplete frames never enter the monitor.
Recoverable overflows terminate the worker even while a debugger is attached;
break on `ap_stack_guard_recover` to inspect one before recovery.

Recovery requires a thread-mode PSP fault without suspended handler or interrupt
mask, with guard/allocation attribution from fault status and addresses, including
lazy FP state. Other faults follow normal crash handling. Recovery abandons damaged
frames, installs a fresh ChibiOS extended frame, notifies the owner through a
lock-free callback, releases owned mutexes with `chMtxUnlockAllS()` and exits with
`chThdExitS()` in one locked scheduler transaction. The manager joins before cleanup.

Forced mutex release cannot roll back partially completed host operations; binary
semaphores and non-mutex state are not reconstructed. A frame/write can skip the
entire guard. Arbitrary pointers and all nested/critical-section faults are not
contained. Recovery latches a Rust error without disarming. This narrow mechanism
does not replace stack measurement or provide general memory protection.
