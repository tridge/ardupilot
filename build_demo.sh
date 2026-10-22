#!/bin/bash
set -e

APPLET=libraries/AP_Module/applets/aerobatics
SINE=libraries/AP_Module/examples/sine
TARGET=thumbv7em-none-eabihf

# Local toolchain defaults can be overridden through the environment.
export LLD="${LLD:-/data/codex/apmodule-lld/usr/lib/llvm-21/bin/ld.lld}"
export ADA="${ADA:-/data/codex/apmodule-gnat-llvm/llvm-interface/bin/llvm-gnat1}"
export LLVM_LLC="${LLVM_LLC:-llc-21}"
export FC="${ARM_FC:-${FC:-/data/codex/apmodule-gfortran-arm15/root/usr/bin/arm-linux-gnueabihf-gfortran-15}}"
export ADA_INCLUDE_PATH="${ADA_ARM_INCLUDE_PATH:-${ADA_INCLUDE_PATH:-/data/codex/apmodule-gnat-arm14/usr/lib/gcc-cross/arm-linux-gnueabihf/14/adainclude}}"

build_pids=()
build_names=()

start_build()
{
    local name=$1
    shift
    "$@" &
    build_pids+=("$!")
    build_names+=("$name")
}

start_build "aerobatics rust" python3 Tools/modules/build.py "$APPLET/main.rs" aerobatics_rust_H7.apmod \
    --target "$TARGET" --cpu cortex-m7 --debug --opt-level 1 \
    --stack-size 16384 --heap-size 26000 "$@"

start_build "aerobatics ada" python3 "$APPLET/ada/build_airshow.py" aerobatics_ada_H7.apmod \
    --target "$TARGET" --cpu cortex-m7 --debug \
    --stack-size 16384 --heap-size 4096 "$@"

start_build "aerobatics f90" python3 "$APPLET/f90/build.py" aerobatics_f90_H7.apmod \
    --target "$TARGET" --cpu cortex-m7 --debug \
    --stack-size 16384 --heap-size 32768 "$@"

start_build "lua" python3 Tools/modules/lua/build.py lua-h7.apmod \
    --target "$TARGET" --cpu cortex-m7 --debug \
    --stack-size 24576 --heap-size 131072 "$@"

for source in rust/sine.rs f90/sine.f90 ada/sine.adb; do
    language=${source%%/*}
    start_build "sine $language" python3 Tools/modules/build.py "$SINE/$source" "sine_${language}_H7.apmod" \
        --target "$TARGET" --cpu cortex-m7 --debug \
        --stack-size 8192 --heap-size 4096 "$@"
done

build_status=0
for index in "${!build_pids[@]}"; do
    if ! wait "${build_pids[$index]}"; then
        printf 'Build failed: %s\n' "${build_names[$index]}" >&2
        build_status=1
    fi
done
if (( build_status != 0 )); then
    exit "$build_status"
fi

ls -ltr -- *.apmod
