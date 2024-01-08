#!/bin/bash

# Exit immediately if a command exits with a non-zero status
set -e

# The board to build for is passed as an argument to the script
BOARD=$1

if [ "$BOARD" == "CubeOrange" ] || [ "$BOARD" == "CarbonixCubeOrange" ] || [ "$BOARD" == "sitl" ]
then
    echo "Compiling Plane for $BOARD..."
    ./Tools/scripts/build_bootloaders.py "$BOARD"
    ./waf configure --board "$BOARD"
    ./waf plane
else
    echo "Compiling AP_Periph for $BOARD..."
    ./Tools/scripts/build_bootloaders.py "$BOARD"
    ./waf configure --board "$BOARD"
    ./waf AP_Periph
fi

echo "Build for $BOARD completed."
