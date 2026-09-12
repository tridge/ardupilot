#!/bin/bash

#nice ../../Tools/autotest/sim_vehicle.py --aircraft test -L ANUSC -D -G -f motorboat-skid --console --map --out support.ardupilot.org:10099 $*
#nice ../../Tools/autotest/sim_vehicle.py --aircraft test -L ANUSC -D -G -f motorboat-skid --console --map --out 127.0.0.1:10099 $*

DEST=tcp:anusc.tridgell.net:20001
#DEST=anusc.tridgell.net:20001
#DEST=127.0.0.1:10001
nice ../../Tools/autotest/sim_vehicle.py --uds --aircraft test -L ANUSC -D -G -f motorboat-skid --console --map "$@"

