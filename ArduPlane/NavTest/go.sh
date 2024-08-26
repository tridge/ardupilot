#!/bin/bash

[ -f eeprom.bin ] || {
    cp eeprom_base.bin eeprom.bin
}
../../Tools/autotest/sim_vehicle.py -D --console --map -L Marulan -f quadplane -G --aircraft test $*
