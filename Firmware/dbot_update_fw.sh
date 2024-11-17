#!/bin/bash
make
bash ../scripts/upload.sh -f firmware.bin -d downloads -u pi -i dbot.local
bash ../scripts/rprog_mcu.sh -f firmware.bin -d downloads -u pi -i dbot.local -m stm32flash -p /dev/serial0