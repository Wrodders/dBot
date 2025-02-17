#!/bin/bash
# -------------------------------------------------- 
#  @file    vizualizer.sh
#  @brief   Visualizes the video stream from the robot.
#  @date    2025-01-05
#  @version 1.0

echo "[VISION PIPELINE VISUALIZER]"
echo "----------------------------"

# Check if a device hostname is passed as an argument
if [ -z "$1" ]; then
    echo "Error: No device hostname provided."
    echo "Usage: $0 <device-hostname>"
    exit 1
fi

DEVICE_HOSTNAME=$1

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

# Start the video visualization process
sudo gst-launch-1.0 udpsrc port=$PORT ! \
    application/x-rtp, payload=96 ! \
    rtph264depay ! avdec_h264 ! videoconvert ! osxvideosink sync=false
