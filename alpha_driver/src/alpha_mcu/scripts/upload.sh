#!/bin/bash

## Reset pico
echo "Resetting PICO!"
echo "\$BTSC,1*1B" > /dev/ttyACM0

## wait until storage device pops up
echo "Waiting for storage device..."
while read i; do if [ "$i" = RPI-RP2 ]; then break; fi; done \
   < <(inotifywait  -e create,open --format '%f' --quiet /media/${USER}/ --monitor)

sleep 1

./elf2uf2/elf2uf2 alpha-mcu.elf alpha-mcu.uf2

cp alpha-mcu.uf2 /media/${USER}/RPI-RP2

echo "Done!"