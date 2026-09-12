#!/bin/bash
set -e
cd -- "$(dirname -- "${BASH_SOURCE[0]}")"

# Keep local SITL links on Unix sockets. The stored NET_P1 parameters
# connect the vehicle directly to the buoy relay, preserving GCS sender IDs.
unset DISPLAY WAYLAND_DISPLAY TMUX STY ZELLIJ SITL_RITW_TERMINAL
exec nice ../../Tools/autotest/sim_vehicle.py --uds --no-extra-ports \
    --aircraft test -L ANUSC -D -f motorboat-skid "$@"
