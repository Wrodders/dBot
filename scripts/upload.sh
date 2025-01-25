#!/bin/bash

# Function to display help message
show_help() {
    echo "Usage: $0 -u <username> -i <ip_address> -d <remote_directory> -f <file1> [<file2> ... <fileN>]"
    echo ""
    echo "  -f <file1> [<file2> ... <fileN>] Path(s) to binary file(s) or folder(s)"
    echo "  -d|--directory <remote_directory> Specify the remote directory (relative to home)"
    echo "  -u|--user <username>      Specify the username for SSH"
    echo "  -i|--ip <ip_address>      Specify the IP address of the Raspberry Pi"
    echo "  --help                    Show this help message"
    exit 0
}

# Function to upload the binary file or folder to the Raspberry Pi
upload() {
    LOCAL_PATH=$1
    REMOTE_BIN_PATH="$REMOTE_BIN_DIR/$(basename "$LOCAL_PATH")"
    # Use the SSH command to prompt for password if necessary
    if [ -d "$LOCAL_PATH" ]; then
        echo "UPLOAD: Uploading folder $LOCAL_PATH to $PI_USER@$PI_IP:$REMOTE_BIN_PATH"
        scp -r "$LOCAL_PATH" "$PI_USER@$PI_IP:$REMOTE_BIN_PATH"
    else
        echo UPLOAD: "Uploading file $LOCAL_PATH to $PI_USER@$PI_IP:$REMOTE_BIN_PATH"
        scp "$LOCAL_PATH" "$PI_USER@$PI_IP:$REMOTE_BIN_PATH"
    fi

    if [ $? -ne 0 ]; then
        echo "Error: Failed to upload to Raspberry Pi."
        exit 1
    fi
}

# Parse command line arguments
REMOTE_BIN_DIR=""
FILE_LIST=()

while [[ "$#" -gt 0 ]]; do
    case $1 in
        -f|--file) shift; FILE_LIST+=("$1") ;;
        -d|--directory) REMOTE_BIN_DIR="$2"; shift ;;
        -u|--user) PI_USER="$2"; shift ;;
        -i|--ip) PI_IP="$2"; shift ;;
        --help) show_help ;;
        *) echo "Unknown parameter: $1"; show_help ;;
    esac
    shift
done

# Validate inputs
if [[ -z "$REMOTE_BIN_DIR" ]] || [[ -z "$PI_USER" ]] || [[ -z "$PI_IP" ]] || [[ ${#FILE_LIST[@]} -eq 0 ]]; then
    echo "Error: All parameters (username, IP, directory, and at least one file) must be specified."
    show_help
fi

# Create the full user@ip address
PI_USER_IP="$PI_USER@$PI_IP"

# Main logic to upload files
for FILE in "${FILE_LIST[@]}"; do
    upload "$FILE"
done

echo "UPLOAD: All files uploaded successfully."
exit 0
