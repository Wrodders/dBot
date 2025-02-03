#!/bin/bash

# Exit immediately if a command fails
set -e

# Help function
function show_help {
    echo "Usage: $0 [-h] [-t TARGET] [-u USER] [-i HOSTNAME] [-p DEVICE]"
    echo
    echo "Options:"
    echo "  -t TARGET      Specify the target (default: stm32401cc)."
    echo "  -u USER        Specify the username (default: pi)."
    echo "  -i HOSTNAME    Specify the hostname (default: dbot.local)."
    echo "  -p DEVICE      Specify the device for programming (default: /dev/serial0)."
    echo "  -h             Show this help message and exit."
    echo
    exit 0
}

# Default values
TARGET="stm32f401cc"
USER="pi"
HOSTNAME="dbot.local"
DEVICE="/dev/serial0"

# Parse command-line arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        -t|--target) TARGET="$2"; shift 2;;
        -u|--user) USER="$2"; shift 2;;
        -i|--hostname) HOSTNAME="$2"; shift 2;;
        -p|--device) DEVICE="$2"; shift 2;;
        -h|--help) show_help;;
        *) echo "[ERROR] Unknown parameter: $1"; show_help;;
    esac
done

echo "[INFO] Building firmware for target: $TARGET"

# Build firmware with the specified target
make clean || { echo "[ERROR] make clean failed! Exiting."; exit 1; }
make TARGET="$TARGET" || { echo "[ERROR] make failed! Exiting."; exit 1; }

echo "[INFO] Uploading firmware..."
bash ../scripts/upload.sh -f firmware.bin -d downloads -u "$USER" -i "$HOSTNAME" || { echo "[ERROR] Firmware upload failed! Exiting."; exit 1; }

echo "[INFO] Programming MCU..."
bash ../scripts/rprog_mcu.sh -f firmware.bin -d downloads -u "$USER" -i "$HOSTNAME" -m stm32flash -p "$DEVICE" || { echo "[ERROR] MCU programming failed! Exiting."; exit 1; }

echo "[SUCCESS] Firmware successfully uploaded and MCU programmed!"
