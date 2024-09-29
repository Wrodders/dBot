#!/bin/bash

# Hardcoded Raspberry Pi details
PI_USER="rodrigo"
PI_IP="192.168.0.39"  # Replace with the actual IP address of your Raspberry Pi
PI_USER_IP="$PI_USER@$PI_IP"
REMOTE_BIN_DIR="bin"

# Function to display help message
show_help() {
    echo "Usage: $0 -c <command> -f <file>"
    echo ""
    echo "Commands:"
    echo "  -c <command>       Specify command: 'upload', 'flash', or 'upload_flash'"
    echo "  -f <file>          Path to the binary file (for upload/upload_flash) or filename (for flash)"
    echo "  --help             Show this help message"
    exit 0
}

# Function to upload the binary file to the Raspberry Pi
upload_binary() {
    BIN_PATH=$1
    REMOTE_BIN_PATH="$REMOTE_BIN_DIR/$(basename $BIN_PATH)"

    echo "Uploading $BIN_PATH to $PI_USER_IP:$REMOTE_BIN_DIR"
    scp "$BIN_PATH" "$PI_USER_IP:$REMOTE_BIN_DIR/"
    if [ $? -ne 0 ]; then
        echo "Error: Failed to upload binary to Raspberry Pi."
        exit 1
    fi
    echo "Upload successful."
}

# Function to flash the binary onto the STM32 using ST-Link
flash_firmware() {
    BIN_FILENAME=$1
    echo "Flashing $REMOTE_BIN_DIR/$BIN_FILENAME using ST-Link on Raspberry Pi..."
    ssh "$PI_USER_IP" << EOF
        sudo st-flash write "$REMOTE_BIN_DIR/$BIN_FILENAME" 0x8000000
        if [ $? -ne 0 ]; then
            echo "Error: Flashing failed."
            exit 1
        else
            echo "Flashing successful."
        fi
EOF
}

# Function to upload and then immediately flash the binary
upload_and_flash() {
    upload_binary "$1"
    if [ $? -eq 0 ]; then
        BIN_FILENAME=$(basename "$1")
        flash_firmware "$BIN_FILENAME"
    else
        echo "Error: Upload failed. Flashing will not proceed."
        exit 1
    fi
}

# Parse command line arguments
COMMAND=""
FILE=""

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -c|--command) COMMAND="$2"; shift ;;
        -f|--file) FILE="$2"; shift ;;
        --help) show_help ;;
        *) echo "Unknown parameter: $1"; show_help ;;
    esac
    shift
done

# Validate inputs
if [[ -z "$COMMAND" ]] || [[ -z "$FILE" ]]; then
    echo "Error: Both command and file must be specified."
    show_help
fi

# Main logic to handle commands
case $COMMAND in
    upload)
        upload_binary "$FILE"
        ;;
    flash)
        flash_firmware "$FILE"
        ;;
    upload_flash)
        upload_and_flash "$FILE"
        ;;
    *)
        echo "Invalid command: $COMMAND. Use 'upload', 'flash', or 'upload_flash'."
        show_help
        ;;
esac
