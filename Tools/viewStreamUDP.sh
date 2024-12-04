#!/bin/bash

# Check if GStreamer is installed
if ! command -v gst-launch-1.0 &> /dev/null; then
    echo "GStreamer is not installed. Please install it first."
    exit 1
fi

# Set the UDP port for streaming
PORT=5000

# GStreamer pipeline for receiving and displaying video
GST_PIPELINE="gst-launch-1.0 udpsrc port=$PORT ! application/x-rtp, media=video, clock-rate=90000, payload=96 ! rtpjpegdepay ! queue ! jpegdec ! videoconvert ! fpsdisplaysink"

FFPLAY="ffplay  udp://localhost:$PORT  -fflags nobuffer -flags low_delay -framedrop"

# Run GStreamer pipeline
echo "Starting GStreamer pipeline..."
$FFPLAY



