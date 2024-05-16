#!/bin/bash

# This file is mainly used by workflows/cx_build_compare.yml to build the firmware for a specific board without bootloader and clean.

# Exit immediately if a command exits with a non-zero status
set -e

# The board to build for is passed as an argument to the script
BOARD=$1

if [ "$BOARD" == "CubeOrange" ] || [ "$BOARD" == "CarbonixCubeOrange" ] || [ "$BOARD" == "sitl" ]
then
    echo "Compiling Plane for $BOARD..."
    ./waf plane
else
    echo "Compiling AP_Periph for $BOARD..."
    ./waf AP_Periph
fi

echo "Build for $BOARD completed."
