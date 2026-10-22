# Experimental Ada modules

Ada modules use the same container, loader and worker ABI as Rust and C++.
The initial SDK provides text output, cooperative sleep and stop requests.
It is not a complete generated Ada binding catalogue or a full Ada runtime.

## Toolchain

Use [GNAT LLVM](https://github.com/AdaCore/gnat-llvm), Clang/standalone LLD 21+
and a matching LLVM `llc` (tested with LLVM 21). `ADA` selects the
`llvm-gnat1` frontend; `LLVM_LLC`, `CXX` and `LLD` select the other tools.
Use GNAT's `ADA_INCLUDE_PATH` to select the toolchain's Ada runtime **source
specifications** for the target if they are not installed in its default
search path. The runtime library itself is not linked.
ARM builds also use the ArduPilot ARM GCC toolchain's `libgcc` for compiler
helpers such as M4 software double arithmetic; `ARM_GCC` can select its driver.

Ordinary GCC-based GNAT does not produce ARM ROPI/RWPI. Its PIC globals use
a GOT, which the module packer rejects. GNAT LLVM emits target-specific LLVM
IR; `llc` then generates PC-relative constants/code and r9-relative mutable
data, matching the existing Rust/C++ addressing model. No loader changes or
Ada support in the firmware are needed.

From the repository root, with the compiler environment set:

```sh
Tools/modules/build.py libraries/AP_Module/examples/simple_loop.adb \
    build/modules/ada_H7.apmod --target thumbv7em-none-eabihf \
    --cpu cortex-m7 --stack-size 8192 --heap-size 4096
Tools/modules/apmod_info.py build/modules/ada_H7.apmod
```

For x86-64 SITL, select its runtime source specifications, use
`--target x86_64-unknown-linux-gnu` and omit `--cpu`. For portable M4/M7 ARM
code use `--cpu cortex-m4`. Installation and firmware parameters follow the
[runtime guide](../../../libraries/AP_Module/README.md).

## Supported subset

The builder compiles one application `.adb` unit (with its companion `.ads`
where needed) and the small `AP_Module` SDK, not a GPR project. Export
`ap_module_ada_run` with the signature shown in
[simple_loop.ads](../../../libraries/AP_Module/examples/simple_loop.ads).
Do not retain its borrowed context after returning.

Static library-level data, fixed-size stack objects, scalar calculations and
ordinary subprogram calls are supported. Mutable static data resets on each
module load. Only `--opt-level=s` is currently accepted. `--debug` retains Ada
DWARF and the companion ELF; intermediate IR, objects and ALI files remain
beside the output in its `.ada/` directory.

The SDK configuration rejects tasking, allocators, implicit heap allocation,
finalization, secondary stacks, exception handlers, streams and dynamic
library-level elaboration. There is no binder, `libgnat`, standard I/O or
exception unwinder. Missing runtime functions fail at build/link time.

Bounds, range, arithmetic-overflow and assertion checks are retained. Supported
GNAT check failures call a non-returning helper that records the check name and
panics the module. They do not unwind or resume execution. The normal firmware
panic/retained-worker policy applies; this is not general fault isolation.

## Tests

With the toolchain installed:

```sh
python3 -m pytest -q Tools/modules/ada/tests
```

`ADA_NATIVE_INCLUDE_PATH` and `ADA_ARM_INCLUDE_PATH` let the tests switch runtime
source specifications between targets. Compiler-dependent cases explicitly skip
if GNAT LLVM is absent. SITL uses `AP_MODULE_SITL_BINARY`; H7 tests use `RENODE`
and `AP_MODULE_BUILD_ROOT` containing the `ModuleMath` and `ModuleRuntime`
examples. Tests cover link-address independence, mutable data, debug builds,
rejected runtime features, SITL reloads and deliberate runtime-check failures.
The H7 tests exercise the real ChibiOS loader, split RX/RW memory and worker
lifecycle, including repeated reloads and panic handling.
