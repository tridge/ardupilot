# Native FixedWing AirShow demonstration

This applet ports `AP_Scripting/applets/Aerobatics/FixedWing/plane_aerobatics.lua`.
The original controller and trajectories are by Matthew Hampsey, Andy Palmer and
Andrew Tridgell, with controller assistance from Paul Riseborough and testing by
Henry Wurzburg. This native port is AI-assisted (Codex, with independent Claude
reviews). It is experimental and not established as safe for hardware flight.

The ten-maneuver AirShow runs from RC or `NAV_SCRIPT_TIME` in Plane SITL.
RC tests include repeat execution, abort and module restart while controlling.
The controller also runs from SD on H743/ChibiOS under Renode and matches Lua
running on H743.
This is a single-aircraft, numeric-schedule demonstration, not a complete
replacement for every feature of the Lua applet.

## Running the demonstration

Build Plane with `--enable-modules`. Development targets boards with
2 MiB flash, particularly H743; F405 is not an acceptance target. Use Rust
1.98.1 or newer and install its `thumbv7em-none-eabihf` target for H743:

```sh
RUSTC=/path/to/rustc-1.98.1 Tools/modules/build.py \
    libraries/AP_Module/applets/aerobatics/main.rs /tmp/airshow.apmod \
    --target thumbv7em-none-eabihf --stack-size 16384
```

For SITL/Linux x86-64, build with `--target x86_64-unknown-linux-gnu` instead.
The image is CPU-profile specific. Put `airshow.apmod` in `APM/rust/` on SD
(`rust/` in SITL) and this directory's **numeric** `AirShow.txt` in
`APM/scripts/trick72.txt` (`scripts/trick72.txt` in SITL). Do not substitute the
original executable-Lua schedule, which the numeric parser rejects.

Schedule numbers are finite decimal tokens, at most 255 bytes each. Conversion
uses the firmware C library's `strtof()` through a hashed binding,
rather than embedding Rust's decimal parser and lookup tables in every module.
Whole-token validation rejects trailing junk, hexadecimal and non-finite values.
Rounding and locale follow the board's C library, as for Lua numbers. Conversion
may allocate firmware-owned memory outside the module heap.

Set `RUST_ENABLE=1`, `RUST_HEAP_SIZE=32768`, `RUST_CODE_SIZE=131072` and
`RUST_STACK_SIZE=32768` for the supplied H743 schedule, then reboot. Prefer a
48 KiB heap (`49152`) for x86-64 experiments or edited schedules. The tests
deliberately retain the smaller initial heap to check this specific schedule,
not arbitrary edits.
`RUST_DEBUG_OPTS` bit 6 disables Lua-style emergency expansion while armed;
without it, an undersized heap can expand and produce a warning/pre-arm error.
After the module registers its parameters,
set `TRIK_ENABLE=1` and change `RUST_RESTART` to reload. Set `TRIK1_ID=72`,
`RC7_OPTION=300` (action) and `RC9_OPTION=301` (selection). Channel 9 low selects
trick 1; channel 7 low aborts, middle selects and high executes. Execute only
after a fresh switch transition in a supported Plane scripting mode. A held
execute switch must not launch a maneuver after reload.

For AUTO missions, add `NAV_SCRIPT_TIME` with command 72 and timeout 0 (no
mission-item time limit). `TRIK_ENABLE` and RC scripting options are not needed.
The applet acknowledges completion with the mission command ID. As in Lua,
`AEROM_MIS_ANGLE=0` selects the bearing between the surrounding waypoints,
reversed when appropriate for the entry course; a nonzero value fixes the heading.
Waypoint searches skip non-location commands and are limited to 256 items,
including forward `DO_JUMP` traversal, to reject malformed/cyclic missions.
The next script item is preloaded while approaching it. A jump directly to an
uncached schedule may require loading inside Plane's one-second watchdog;
slow storage can cause that item to be skipped, with a diagnostic.

The ported built-in command IDs also accept Lua's four mission/RC arguments:
2 (Loop), 7 (Axial Roll), 8 (Rolling Circle), 9 (Half Cuban Eight),
10 (Half Reverse Cuban Eight), 12 (Humpty Bump), 14 (Scale Figure Eight),
15 (Immelmann Turn), 16 (Split-S), and 26 (Barrel Roll).
Loaded numeric schedules use their file's arguments, as Lua does; the mission
arguments do not scale or select a subsection of a loaded schedule.

Do **not** run the original Lua aerobatics applet concurrently: both own the
same parameter names and vehicle-control interface. Other Lua scripts may run
alongside Rust. Parameters use the same AP_Param tables, keys, defaults, live
`get()` behavior and saved-value semantics; no AP_Param changes are required.
The speed PI gains are captured at startup, as in Lua. Dynamic parameter tables
are firmware-owned outside either scripting heap and remain across reload.

The repeatable simulated-flight test configures the existing `plane-3d` model,
takes off, flies the complete schedule, compares maneuver order and checks path
error. It then repeats/aborts and restarts the native module while controlling:

```sh
MAVLINK20=1 RUSTC=/path/to/rustc-1.98.1 \
    python3 -m pytest -q -s Tools/modules/tests/test_aerobatics_flight.py
```

The test flies both engines separately with the same model. It is not a
hardware-flight procedure. Dual-aircraft time sync, unlisted built-in maneuvers,
stall-turn/rate overrides and arbitrary Lua maneuver functions remain unsupported.
The runner declines the dual-aircraft option; custom geometry requires rebuilding
the Rust module. The standard Plane tests `AerobaticsScripting_rust_mission` and
`AerobaticsScripting_rust_mission_control` cover AUTO AirShow completion, built-in
arguments, timeout, command replacement and mode exit with RC tricks disabled.
The built-in mission test flies Axial Roll; the AirShow tests exercise the other
ported geometries with fixed schedule arguments, not every built-in argument combination.

These are trusted native modules, not a sandbox. Compile-time Rust checks do not
provide instruction budgets, MPU isolation or flight-safety certification.
Transient pre-step contention skips a tick; command-delivery contention waits
for fresh observations next tick. A one-second local command deadline prevents
resuming after prolonged starvation. Ordinary failures release scripting
control. On module stop, host bindings reject further calls, so Plane's existing
one-second NavScript watchdog releases control, as with Lua restart. Invalid
finite/configuration checks are intentionally stronger than Lua's NaN checks.
Unlike Lua's retained task, the native runner discards its controller on mode
change or mission-item expiry/replacement, so stale commands cannot continue
into a new task. This demonstration does not add new disarm/failsafe policy.

`test_aerobatics_parameters.py` checks the live tables, storage indices, all
29 AEROM defaults, expansion to eleven tricks, the legacy acceleration/time
constant repairs, and saved values across reboot into the opposite engine.
`test_aerobatics_faults.py` injects Busy at read/delivery boundaries: brief
contention must recover; prolonged contention must stop at the local deadline;
an altitude abort must release even if its final delivery is Busy. Injection
requires the explicit test-only `ap_rust_aerobatics_faults` cfg and the source
under `tests/modules`; the ordinary image is checked for absence of its markers.

## Primitive and composition validation

The first acceptance stage covers math helpers, roll profiles, geometric path
components and alignment. `Tools/modules/tests/test_aerobatics.py` executes the
original Lua function definitions alongside the native module in disarmed SITL,
then compares 551 logged rows. Same-host results must match float32 bits; NaN
payloads are ignored. A narrow signed-zero allowance covers Lua integer
intermediates versus native float32 arithmetic. This policy does not permit
finite nonzero numeric differences.

The H743 test reads the actual Lua-produced values from SD through FATFS, caching
256-byte chunks, and checks the native results. It allows an absolute tolerance
of 2e-5 or relative tolerance of 2e-6 for different board/host C math libraries.
This is a cross-platform oracle, not Lua execution on H743. NaNs must correspond
to NaNs and infinities must compare equal; target comparisons ignore zero signs.
All 551 rows, including NaN-producing
roll extrapolation, are checked. Zero-vector/antiparallel quaternion normalization
is not a healthy-vehicle test: AP_Math reports an internal error for a zero
quaternion. Native tangent helpers require finite nonzero vectors whose rotation
quaternion has length at least `f32::EPSILON`, and a finite positive time step
with finite reciprocal; they return Invalid locally for invalid
inputs instead of triggering that firmware-wide flag. This is an intentional
invalid-input safety difference from the Lua helper, not a controller retuning.

The primitive H743 harness retains the five-second per-load deadline and exact
free-memory comparison across 20 reloads. Its emulated run time is derived from
the per-load deadline and cycle count, with a startup margin. It checks zero
live module-heap bytes after return; the roll sequence's peak allocation is
separately bounded to 128 bytes (80 bytes of data plus allocator overhead).
The 551-row run measured a 95-byte heap peak and 3,016 bytes of worker stack,
with exact free-memory recovery over all 20 reloads. All 11 earlier H743 binding
probes also passed their unchanged per-probe heap bounds and reload checks.
These probes do not measure vehicle execution timing or prove worst-case stack
depth.

Build SITL Plane with both scripting engines and `examples/RustMath` for
MatekH743, then run:

```sh
RUSTC=/path/to/rustc-1.98.1 RENODE=/path/to/renode \
    python3 -m pytest -q -s Tools/modules/tests/test_aerobatics.py
```

Without `RENODE` the H743 test skips explicitly. Host unit tests additionally
cover NaN ordering, the original asymmetric `sgn()` epsilon, sequence bounds,
zero-rate/duration and extrapolation behavior. No controller logic is changed to
make these edge cases look more conventional.

## AirShow composition

`AirShow.txt` is a numeric-only translation of the original executable-Lua
schedule through BarrelRoll. The ten maneuvers, nested paths, alignment, roll
references, orientation changes and throttle-boost attributes are compiled in Rust. The
parser deliberately rejects embedded Lua, unknown commands, timestamps and
rate-controller overrides. Custom maneuver functions require rebuilding the
module. It accepts at most 16 KiB and 256 commands; these are input bounds, not
a promise that every accepted schedule fits the configured module heap.

The retained path tree uses fallible allocation at construction and no
allocation during sampling. Re-entering a maneuver models Lua's single-entry
factory cache, including refreshing barrel-roll banking from live speed. This
policy covers AirShow; additional speed-dependent geometries need explicit
support. Maneuver messages are best effort, as in Lua. The optional `@` message
prefix and debug messages are not implemented at this stage.

`test_aerobatics_path.py` compares 501 full-schedule samples, changing speed and
entry rate and revisiting segments after lookahead. Position, roll, correction,
length, final orientation, next boundary and boost flag match Lua float32 bits;
all ten maneuver messages match in order. Messages are checked in SITL only;
the H743 harness has no GCS singleton. Build-only and full-sampling heap
peaks are identical. Deliberately undersized heaps fail without panic, recover
all module allocations, and permit subsequent reloads with smaller schedules.

The SITL construction peak is 20,542 bytes, an observed test peak rather than a
worst-case bound. H743 replay of the Lua-produced oracle checks worker-stack
usage and memory recovery over 20 reloads.
The H743 path test needs a larger harness heap than the small binding probes:

```sh
./waf configure --board MatekH743 --enable-modules \
    --extra-hwdef libraries/AP_Module/examples/RustMath/aerobatics.hwdef
./waf --targets examples/RustMath
RUSTC=/path/to/rustc-1.98.1 RENODE=/path/to/renode \
    python3 -m pytest -q -s Tools/modules/tests/test_aerobatics_path.py
```

Reconfigure without that extra hwdef before running the small binding probes;
the harness exports its heap limit and each test verifies the intended budget.

## Controller validation on SITL and H743

`test_aerobatics_controller.py` compares 512 synthetic observations for each of
three variants: nominal, full-path sweep with changing speed/airspeed/gyro, and
mirrored/scaled path with changed gains and disabled lookahead. The sweep forces
path progress to cover every maneuver; it is not a flight-dynamics simulation.
Each step compares 33 float32 values, including intermediate states. On each
platform, Rust matches the original Lua controller **bit-for-bit**, including
zero signs. Linux and H743 traces are not interchangeable: the nominal open-loop
stress trace exposes differences shared by Lua and Rust on each platform.

The H743 oracle runs the original Lua equations in a full Plane firmware, with
mock observations and outputs, and writes binary floats to the emulated SD card.
The native test then consumes that target-local oracle through real FATFS,
AP_MultiHeap and ChibiOS worker threads. There is no real actuator control in the
oracle. All three variants passed three reloads with exact free-memory recovery,
a 23,644-byte heap peak and a maximum 4,208-byte observed worker-stack high-water
mark, including the extra invalid-input/fallback probes. The H743 firmware used
GNU Arm Embedded GCC 10.2.1 and the modules used Rust 1.90.0.
The test also checks invalid inputs/configuration, waiting, completion, altitude
abort and repeat-start count handling. These are observed peaks, not universal
stack or execution-time bounds.

Build the Lua-oracle Plane first, then the controller harness without replacing
that Plane binary:

```sh
./waf configure --board MatekH743 --enable-modules \
    --default-parameters Tools/modules/aerobatics-oracle.parm \
    --extra-hwdef Tools/modules/aerobatics-oracle.hwdef
./waf plane
./waf configure --board MatekH743 --enable-modules \
    --extra-hwdef libraries/AP_Module/examples/RustMath/controller.hwdef
./waf --targets examples/RustMath
RUSTC=/path/to/rustc-1.98.1 RENODE=/path/to/renode \
    python3 -m pytest -q -s Tools/modules/tests/test_aerobatics_controller.py
```

The controller harness allows 20 seconds per load. Tests verify exported heap,
code, cycle and deadline settings and print ELF hashes for reproducibility.
The oracle firmware has embedded test defaults and must not be used for flight.
Reconfigure normally before building a deployable Plane. Renode verifies the
H743 software execution paths, not real-silicon cache behavior or CPU timing.

With the oracle Plane binary still present, `test_aerobatics_runner_h743.py`
also boots the **full runner**, loading `main.rs` from the emulated SD card.
Simulated GPS, compass, airspeed and IMU feed the real firmware; a small Lua
driver selects/starts/aborts the Rust trick and requests two reloads while
disarmed. This stationary fixture is not a H743 flight simulation. It recorded
772 controller updates over the twenty-second active window and zero rejected
backend writes in all 67 captured `DSF` statistics rows. It measured 24,817 peak
module-heap bytes and a 7,560-byte observed stack high-water mark from periodic
`STAK` samples. This runner is different from the controller probe above. The
image reservation was 81,948 bytes and the worker allocation 33,284 bytes,
including its configured 32,768-byte stack. Firmware-owned tables and the
shared manager are additional. With a 16 KiB logger buffer the test captured
only 79 update records; with 64 KiB it captured 772 and the statistics above.
The logger buffer is not charged to the module heap.

## Resource comparison

The flight test reports module unload counters (heap peak, image reservation,
thread reservation), Linux worker CPU time and DataFlash update timings. Rust's
`ARUN.Us` measures the controller; `ARUN.Total` measures the active update through
command delivery and ordinary logging, excluding sleep and its own timing-log
write. `ARUN.Flags` records path-rate and lookahead fallback use. Lua's
`SCR.Runtime` excludes garbage collection; worker CPU time includes it. Neither
elapsed-time field is a physical-H7 CPU benchmark.

Compare total RAM, not just the module heap: native code is loaded into RAM,
worker stacks and manager metadata are separate reservations, and persistent
parameter/logging metadata is outside both engines' heaps. Lua's logged
`SCR.Total_mem` is a before-GC sample, not a measurement of initial parsing's
peak. Results depend on the host load and should be reproduced on hardware
before drawing scheduling or worst-case resource conclusions.

One sequential SITL run (5x simulation speed, GCC 15.2.0 host firmware and Rust
1.90.0) measured 1.09 s Rust-worker CPU versus 2.09 s Lua-worker CPU over one
complete AirShow. Peak path errors were 6.58 m and 5.92 m respectively, within
the existing 15 m acceptance ceiling. Rust completed with a 32,768-byte heap
cap and a 31,221-byte allocation peak; Lua's largest logged before-GC sample
was 276,480 bytes in its 350,000-byte VM heap. The native image reservation was
110,616 bytes and its worker allocation 41,016 bytes (including a 32,768-byte
stack); overhead beyond the configured stack is platform-dependent. The shared
manager and persistent metadata are additional. This is a
measured host demonstration, not a promised speedup or a real-H7 benchmark.
