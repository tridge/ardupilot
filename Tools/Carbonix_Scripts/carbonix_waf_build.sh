#!/bin/bash

# Exit if any command fails
set -e

# Check unstaged changes
if [[ $(git status --porcelain) ]]; then
    echo "Error: There are unstaged changes in the repository."
    exit 1
fi

# Check outdated submodules
# git submodule update --remote --quiet
# outdated_submodules=$(git submodule status | grep -e '-[[:alnum:]]' | awk '{print $2}')
# if [[ -n $outdated_submodules ]]; then
#     echo "The following submodules are not updated:"
#     echo "$outdated_submodules"
#     exit 1
# fi

echo "Running distclean..."
./waf distclean

main_boards=("CarbonixCubeOrange" "CubeOrange" "sitl")

for board in "${main_boards[@]}"; do
  echo "Compiling Plane for $board..."
  ./Tools/scripts/build_bootloaders.py "$board"
  ./waf configure --board "$board" --define=CARBOPILOT=1
  ./waf plane
done

periph_boards=("CarbonixF405" "Volanti-M1" "Volanti-M2" "Volanti-M3" "Volanti-M4" "Volanti-M5" "Volanti-LWing" "Volanti-RWing" "Volanti-LTail" "Volanti-RTail" "Ottano-M1" "Ottano-M2" "Ottano-M3" "Ottano-M4" "Ottano-M5" "Ottano-LWing" "Ottano-RWing" "Ottano-LTail" "Ottano-RTail")

for board in "${periph_boards[@]}"; do
  echo "Compiling AP_Periph for $board..."
  ./Tools/scripts/build_bootloaders.py "$board"
  ./waf configure --board "$board" --define=CARBOPILOT=1
  ./waf AP_Periph
done


echo "Script completed successfully."
