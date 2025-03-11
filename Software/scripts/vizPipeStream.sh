#!/bin/bash
# -------------------------------------------------- 
#  @file    vizualizer.sh
#  @brief   Visualizes the video stream from the robot.
#           Optionally records the stream to a file (max 1 min).
# --------------------------------------------------

# Display Help Function
function show_help {
    echo "Usage: $0 [-h] -d <device-hostname> [-o <output-filename>]"
    echo
    echo "Options:"
    echo "  -h                 Show this help message and exit"
    echo "  -d <hostname>      Specify the device hostname (required)"
    echo "  -o <file>          Specify an output filename for recording (optional, max 1 min)"
    echo
    echo "Supported Devices:"
    echo "  dbot -> Port 5001"
    echo "  bot  -> Port 5002"
    exit 0
}

# Default values
DEVICE_HOSTNAME=""
OUTPUT_FILE=""

# Parse command-line arguments
while getopts "hd:o:" opt; do
    case "$opt" in
        h) show_help ;;
        d) DEVICE_HOSTNAME="$OPTARG" ;;
        o) OUTPUT_FILE="$OPTARG" ;;
        ?) show_help ;;
    esac
done

# Validate required arguments
if [[ -z "$DEVICE_HOSTNAME" ]]; then
    echo "[ERROR] Device hostname is required!"
    show_help
fi

# Determine the port based on the device hostname
case "$DEVICE_HOSTNAME" in
    "dbot")
        PORT=5001
        ;;
    "bot")
        PORT=5002
        ;;
    *)
        echo "[ERROR] Unknown device hostname: $DEVICE_HOSTNAME"
        exit 1
        ;;
esac

echo "Using port: $PORT for device: $DEVICE_HOSTNAME"

# Check if recording is requested
if [[ -n "$OUTPUT_FILE" ]]; then
    echo "Recording enabled. Output file: $OUTPUT_FILE (Max 1 min)"
    
    # Prevent overwriting an existing file
    if [[ -f "$OUTPUT_FILE" ]]; then
        echo "[WARNING] File $OUTPUT_FILE already exists. Overwriting!"
    fi

    # Launch pipeline with tee element (view & record)
    sudo gst-launch-1.0 udpsrc port=$PORT ! \
        application/x-rtp, payload=96 ! \
        rtpjitterbuffer ! rtph264depay ! tee name=t \
        t. ! queue ! avdec_h264 ! videoconvert ! osxvideosink sync=false \
        t. ! queue ! h264parse ! mp4mux ! filesink location="$OUTPUT_FILE" -e
else
    # Launch only the video viewer
    sudo gst-launch-1.0 udpsrc port=$PORT ! \
        application/x-rtp, payload=96 ! \
        rtpjitterbuffer ! rtph264depay ! avdec_h264 ! videoconvert !  videoscale ! osxvideosink sync=false
fi
