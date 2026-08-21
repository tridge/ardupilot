#!/usr/bin/env bash

set -euo pipefail

script_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
usbip_patch="${script_dir}/../patches/usbip-device-state.patch"

destination="${1:-build/renode}"
branch="pr-arudpilot-am32-perf"

if [[ ! -d "${destination}/.git" ]]; then
    git clone --branch "${branch}" --single-branch --no-checkout \
        https://github.com/ArduPilot/renode.git "${destination}"
fi

destination="$(cd "${destination}" && pwd)"
git -C "${destination}" fetch origin "${branch}"
git -C "${destination}" checkout --detach FETCH_HEAD
git -C "${destination}" submodule sync --recursive
git -C "${destination}" submodule update --init --recursive
(
    cd "${destination}"
    ./perf_patches/apply.sh
    if git -C src/Infrastructure apply --check --reverse "${usbip_patch}" 2>/dev/null; then
        echo "already applied ${usbip_patch}"
    elif git -C src/Infrastructure apply --check "${usbip_patch}"; then
        git -C src/Infrastructure apply "${usbip_patch}"
        echo "applied ${usbip_patch}"
    else
        echo "FAILED to apply ${usbip_patch}" >&2
        exit 1
    fi
    ./build.sh --net
)

echo "Renode executable: ${destination}/renode"
