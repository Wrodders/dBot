#!/bin/bash

# Function to display help message
show_help() {
    echo "Usage: $0 -u <username> -i <ip_address> -d <remote_directory> -f <file> -m <method> [-p <serial_port>]"
    echo ""
    echo "  -f <file>             Specify the binary file to flash"
    echo "  -d|--directory <remote_directory> Specify the remote directory (relative to home)"
    echo "  -u|--user <username>  Specify the username for SSH"
    echo "  -i|--ip <ip_address>  Specify the IP address of the Raspberry Pi"
    echo "  -m|--method <stlink|stm32flash> Specify the flashing method"
    echo "  -p|--port <serial_port> Specify the serial port (required for stm32flash)"
    echo "  --help                Show this help message"
    exit 0
}

# Function to flash using ST-Link
flash_stlink() {
    BIN_FILENAME=$1
    echo "Flashing $REMOTE_BIN_DIR/$BIN_FILENAME using ST-Link on Raspberry Pi..."
    ssh "$PI_USER@$PI_IP" << EOF
        sudo st-flash --hot-plug write "$REMOTE_BIN_DIR/$BIN_FILENAME" 0x8000000
        if [ $? -ne 0 ]; then
            echo "Error: Flashing failed."
            exit 1
        else
            echo "RPROG_MCU: Flashing successful."
        fi
EOF
}

# Function to flash using stm32flash
flash_stm32flash() {
    BIN_FILENAME=$1
    echo "Flashing $REMOTE_BIN_DIR/$BIN_FILENAME using stm32flash on Raspberry Pi..."
    echo "Entering FW update mode on the target..."
    ssh "$PI_USER@$PI_IP" << EOF
        echo "Entering FW update mode..."
        stty -F "$SERIAL_PORT" 115200
        echo '<BB5\n' > $SERIAL_PORT
        sleep 1
        stm32flash -R -w "$REMOTE_BIN_DIR/$BIN_FILENAME" -v "$SERIAL_PORT"
        if [ $? -ne 0 ]; then
            echo "Error: Flashing failed."
            exit 1
        else
            echo "RPROG_MCU: Flashing successful."
        fi
EOF
}

# Parse command line arguments
REMOTE_BIN_DIR=""
FILE=""
FLASH_METHOD=""
SERIAL_PORT=""

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -f|--file) FILE="$2"; shift ;;
        -d|--directory) REMOTE_BIN_DIR="$2"; shift ;;
        -u|--user) PI_USER="$2"; shift ;;
        -i|--ip) PI_IP="$2"; shift ;;
        -m|--method) FLASH_METHOD="$2"; shift ;;
        -p|--port) SERIAL_PORT="$2"; shift ;;
        --help) show_help ;;
        *) echo "Unknown parameter: $1"; show_help ;;
    esac
    shift
done

# Validate inputs
if [[ -z "$REMOTE_BIN_DIR" ]] || [[ -z "$PI_USER" ]] || [[ -z "$PI_IP" ]] || [[ -z "$FILE" ]] || [[ -z "$FLASH_METHOD" ]]; then
    echo "Error: All parameters (username, IP, directory, file, and method) must be specified."
    show_help
fi

# Check for serial port if using stm32flash
if [[ "$FLASH_METHOD" == "stm32flash" ]] && [[ -z "$SERIAL_PORT" ]]; then
    echo "Error: Serial port must be specified when using stm32flash."
    show_help
fi

# Flash the file using the specified method
case $FLASH_METHOD in
    stlink)
        flash_stlink "$FILE"
        ;;
    stm32flash)
        flash_stm32flash "$FILE"
        ;;
    *)
        echo "Invalid method: $FLASH_METHOD. Use 'stlink' or 'stm32flash'."
        show_help
        ;;
esac
