#!/bin/bash
make
bash ../scripts/upload.sh -f firmware.bin -d downloads -u rodrigo -i raspberrypi.local
bash ../scripts/rprog_mcu.sh -f firmware.bin -d downloads -u rodrigo -i raspberrypi.local -m stm32flash -p /dev/serial0