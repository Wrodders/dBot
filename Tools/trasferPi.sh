#!/bin/bash

# Display help message
function display_help {
    echo "Usage: $0 <hostname> <local_path> [options]"
    echo "Transfer a file or folder to a Raspberry Pi Zero over SSH using rsync."
    echo "Arguments:"
    echo "  <hostname>          Hostname or IP address of the Raspberry Pi Zero."
    echo "  <source file path>  Path to the local file or folder to transfer."
    echo "Options:"
    echo "  -h, --help          Display this help message."
    echo "  -d, --destination   Destination folder on the Raspberry Pi Zero. Default: /home/pi."
    echo "Example:"
    echo "  $0 192.168.1.253 /path/to/local/file.txt"
    echo "  $0 192.168.1.253 /path/to/local/folder -d testFolder/"
    exit 1
}

if [ -z "$1" ]; then
    echo "Error: Hostname is mandatory."
    display_help
    exit 1
fi

PI_HOSTNAME="$1"
shift

if [ -z "$1" ]; then
    echo "Error: File path is mandatory."
    display_help
    exit 1
fi

# Set default values
PI_USERNAME="pi"
DEST_PATH="/home/pi/scripts"
PI_PORT=22  # Default SSH port

SRC_PATH="$1" # update source file path
shift

# Process optional destination file path
while [ "$#" -gt 0 ]; do
    case "$1" in
        -d|--destination)
            shift
            DEST_PATH="$DEST_PATH/$1"
            ;;
        *)
            echo "Error: Unknown option '$1'."
            display_help
            exit 1
            ;;
    esac
    shift
done

echo "Destination Path: $DEST_PATH"

# Check if the required number of arguments is provided
if [ -z "$SRC_PATH" ]; then
    display_help
fi

# Assign command line arguments to variables
BASENAME_SRC_PATH=$(basename "$SRC_PATH") # base name of source path
REMOTE_PATH="$DEST_PATH/$BASENAME_SRC_PATH" # path to the remote file including name

# Transfer the file or folder using rsync with SSH key authentication
rsync -e "ssh -p $PI_PORT" -avz "$SRC_PATH" "$PI_USERNAME"@"$PI_HOSTNAME":"$REMOTE_PATH"

# Check the exit status of the rsync command
if [ $? -eq 0 ]; then
    echo "Transferred: '$BASENAME_SRC_PATH' to '$REMOTE_PATH' at $PI_HOSTNAME successfully."
else
    echo "Error: Failed to transfer file or folder."
fi
