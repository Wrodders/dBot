#!/bin/bash

# Help function
function show_help {
    echo "Usage: $0 [-h] [-u USER] [-i HOSTNAME] [-p DEVICE]"
    echo
    echo "Options:"
    echo "  -u USER        Specify the username (default: pi)."
    echo "  -i HOSTNAME    Specify the hostname (default: dbot.local)."
    echo "  -p DEVICE      Specify the device for programming (default: /dev/serial0)."
    echo "  -h             Show this help message and exit."
    echo
    exit 0
}

# Default values
USER="pi"
HOSTNAME="dbot.local"
DEVICE="/dev/serial0"

# Parse command-line arguments
while [ "$#" -gt 0 ]; do
    case $1 in
        -u|--user) USER="$2";shift 2;;
        -i|--hostname) HOSTNAME="$2";shift 2;;
        -p|--device) DEVICE="$2";shift 2;;
        -h|--help)  show_help;;
        *) echo "Unknown parameter: $1"; show_help;;
    esac
done

# Build and upload firmware
make clean
make
bash ../scripts/upload.sh -f firmware.bin -d downloads -u "$USER" -i "$HOSTNAME"
bash ../scripts/rprog_mcu.sh -f firmware.bin -d downloads -u "$USER" -i "$HOSTNAME" -m stm32flash -p "$DEVICE"
