#!/bin/bash

[ -f eeprom.bin ] || {
    cp eeprom_base.bin eeprom.bin
}
../../Tools/autotest/sim_vehicle.py -D --console --map -l '50.66487473,29.92565520,145,290' -G --aircraft test $*
