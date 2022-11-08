#!/bin/bash

for f in $*; do
    bname=$(basename $f .bin)
    dname=$(dirname $f)
    newname=$dname/$bname-glide.bin
    echo Creating $newname
    mavlogdump.py --meta -q -o "$newname" "$f" --condition='in_mission(SL2,MODE,RCOU,GPS)' --nottypes 'NK*,XK*' &
done
wait
