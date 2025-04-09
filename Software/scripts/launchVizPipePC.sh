#!/bin/bash
#--------------------------------------------------
#  @file    launchVisionPC.sh
#  @brief   Launches the test vision Pipeline on Mac. 
#  This version includes an option to enable debug mode for the vision pipeline.

# Function to clean up on exit
cleanup() {
    echo "Stopping all processes..."
    pkill -P $$  # Kill all child processes of this script
    exit 0
}

# Trap exit signals to clean up properly
trap cleanup SIGINT SIGTERM EXIT

FILE="recordings/recRaw7.mp4"

# Check if the file exists before continuing
if [[ ! -f "$FILE" ]]; then
    echo "Error: File $FILE does not exist!"
    exit 1
fi

PIPELINE="filesrc location=$FILE ! qtdemux ! queue ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink"

# Check if debug mode is enabled by the user
DEBUG_MODE=false

while [[ "$1" =~ ^- ]]; do
    case "$1" in
        --debug)
            DEBUG_MODE=true
            shift
            ;;
        *)
            echo "Unknown option $1"
            exit 1
            ;;
    esac
done

# Start the vision process with the pipeline
if [ "$DEBUG_MODE" = true ]; then
    echo "Starting vision process in debug mode..."
    ./build/vision "$PIPELINE" --debug
else
    echo "Starting vision process..."
    ./build/vision "$PIPELINE"
fi
