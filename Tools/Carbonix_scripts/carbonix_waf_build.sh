#!/bin/bash

# Exit if any command fails
set -e

echo "Running distclean..."
./waf distclean

main_boards=("CubeOrange" "CubeOrange-Volanti" "CubeOrange-Ottano" "sitl")
for board in "${main_boards[@]}"; do
  echo "Compiling ArduPlane for $board..."
  ./waf configure --board "$board" --define=CARBOPILOT=1
  ./waf plane
done
echo "Script completed successfully."

periph_boards=("CarbonixF405" "CarbonixF405-no-crystal")
for board in "${periph_boards[@]}"; do
  for file in libraries/AP_HAL_ChibiOS/hwdef/CarbonixCommon/cpn_params/*.parm; do
    # Extract the filename without the extension
    filename=$(basename -- "$file")
    filename="${filename%.*}"

    # Create extra hwdef file
    printf "undef CAN_APP_NODE_NAME\ndefine CAN_APP_NODE_NAME \"$board-$filename\"" > temp.hwdef
    
    # Compile AP_Periph for each board
    echo "Compiling AP_Periph for $board with $filename..."
    ./waf configure --board "$board" --define=CARBOPILOT=1 --extra-hwdef=temp.hwdef --default-parameters="$file"
    ./waf AP_Periph
    
    # Rename build outputs
    mkdir build/$board/bin/$filename
    # Move all the files (not folders) in build/$board/bin to build/$board/bin/$filename
    find build/$board/bin -maxdepth 1 -type f -exec mv {} build/$board/bin/$filename \;

    # Cleanup
    rm temp.hwdef
  done
done

echo "Script completed successfully."
