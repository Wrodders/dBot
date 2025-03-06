#!/bin/bash
# -------------------------------------------------- 
#  @file    vizualizer.sh
#  @brief   Visualizes the video stream from the robot.
#           Optionally records the stream to a file (max 1 min).

echo "[VISION PIPELINE VISUALIZER]"
echo "----------------------------"

# Check if a device hostname is passed as an argument
if [ -z "$1" ]; then
    echo "Error: No device hostname provided."
    echo "Usage: $0 <device-hostname> [output-filename]"
    exit 1
fi

DEVICE_HOSTNAME=$1
OUTPUT_FILE=$2

# Determine the port based on the provided device hostname
case $DEVICE_HOSTNAME in
    "dbot")
        PORT=5001
        ;;
    "bot")
        PORT=5002
        ;;
    *)
        echo "[ERROR] Device hostname not recognized"
        exit 1
        ;;
esac

echo "Using port: $PORT for device: $DEVICE_HOSTNAME"

if [ -n "$OUTPUT_FILE" ]; then
    echo "Recording to file: $OUTPUT_FILE (limited to 1 minute)"
    sudo gst-launch-1.0 udpsrc port=$PORT ! \
        application/x-rtp, payload=96 ! \
        rtpjitterbuffer ! rtph264depay ! tee name=t \
        t. ! queue ! avdec_h264 ! videoconvert ! osxvideosink sync=false \
        t. ! queue ! h264parse ! splitmuxsink location="$OUTPUT_FILE" max-size-time=60000000000
else
    sudo gst-launch-1.0 udpsrc port=$PORT ! \
        application/x-rtp, payload=96 ! \
        rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert ! osxvideosink sync=false
fi
