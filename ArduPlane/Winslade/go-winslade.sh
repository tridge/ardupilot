#!/bin/bash

[ -f eeprom.bin ] || {
    cp eeprom_base.bin eeprom.bin
}
../../Tools/autotest/sim_vehicle.py -D --console --map -l ' -35.31603560,148.97679943,603,15' -f quadplane -G --aircraft test $*
