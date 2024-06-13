# hw definition file for processing by chibios_hwdef.py

include ../CubeOrange/hwdef.dat
include ./features.inc

# The Carbonix carrier board (CX13042008) uses uninverted logic for these pins,
# as opposed to how the CubePilot carrier hardware is designed. We have to
# redefine these pin assignments.
undef PB5
undef PB7
PB5 VDD_BRICK_VALID INPUT PULLUP
PB7 VDD_BRICK2_VALID INPUT PULLUP

# Set board heater target to 60C
undef HAL_IMU_TEMP_DEFAULT
define HAL_IMU_TEMP_DEFAULT 60

USE_BOOTLOADER_FROM_BOARD CubeOrange
