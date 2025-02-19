#!/bin/bash
#--------------------------------------------------
#  @file    launchVisionPC.sh
#  @brief   Launches the vision Pipeline on Mac. 



# Define named pipes
VIDEO_IN="/tmp/video_in"
VIDEO_OUT="/tmp/video_out"

# Ensure no old pipes exist
rm -f $VIDEO_IN $VIDEO_OUT

# Create named pipes
mkfifo $VIDEO_IN
mkfifo $VIDEO_OUT

# Function to clean up on exit
cleanup() {
    echo "Stopping all processes..."
    pkill -P $$  # Kill all child processes of this script
    rm -f $VIDEO_IN $VIDEO_OUT  # Remove named pipes
    exit 0
}

# Trap exit signals to clean up properly
trap cleanup SIGINT SIGTERM EXIT

# Start GStreamer video input pipeline (suppress stdout, keep stderr)
gst-launch-1.0 avfvideosrc do-timestamp=true ! videoconvert ! \
    videorate ! \
    video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! \
    filesink location=$VIDEO_IN > /dev/null &

# Start GStreamer video output pipeline
gst-launch-1.0 filesrc location=$VIDEO_OUT ! videoparse format=i420 width=640 height=480 ! \
    videoconvert ! autovideosink sync=false  > /dev/null &

# Run vision program in the foreground
./build/vision 

