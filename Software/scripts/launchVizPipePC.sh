#!/bin/bash
#--------------------------------------------------
#  @file    launchVisionPC.sh
#  @brief   Launches the test vision Pipeline on Mac. 


# Function to clean up on exit
cleanup() {
    echo "Stopping all processes..."
    pkill -P $$  # Kill all child processes of this script
    exit 0
}

# Trap exit signals to clean up properly
trap cleanup SIGINT SIGTERM EXIT
FILE="recordings/recRaw7.mp4"


PIPELINE="filesrc location=$FILE ! qtdemux ! queue ! h264parse ! avdec_h264 ! videoconvert ! video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! videoconvert ! video/x-raw,format=GRAY8 ! appsink"

./build/vision "$PIPELINE"

