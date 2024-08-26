#!/bin/bash

[ -f eeprom.bin ] || {
    cp eeprom_base.bin eeprom.bin
}
../../Tools/autotest/sim_vehicle.py -D --console --map -l '50.37968622,25.77131554,220,330' -G --aircraft test $*
