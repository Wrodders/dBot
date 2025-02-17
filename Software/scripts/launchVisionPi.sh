#!/bin/bash
# Define named pipes
VIDEO_IN="/tmp/video_in"
VIDEO_OUT="/tmp/video_out"

# Ensure no old pipes exist
rm -f $VIDEO_IN $VIDEO_OUT

# Create named pipes
sudo mkfifo $VIDEO_IN
sudo mkfifo $VIDEO_OUT

# Function to clean up on exit
cleanup() {
    echo "Stopping all processes..."
    pkill -P $$  # Kill all child processes of this script
    rm -f $VIDEO_IN $VIDEO_OUT  # Remove named pipes
    exit 0
}

trap cleanup SIGINT SIGTERM EXIT

# Determine the port based on the hostname
HOSTNAME=$(hostname)
case $HOSTNAME in
    "dbot")
        PORT=5001
        ROT=5
        ;;
    "bot")
        PORT=5002
        ROT=5
        ;;
    *)
        echo "[ERROR] Device hostname not recognized"
        exit 1
        ;;
esac

echo "Using port: $PORT"

# Start the video input process
# Opens the camera using libcamerasrc, converts the video format to I420, and writes it to the named pipe
sudo gst-launch-1.0 libcamerasrc ! videoconvert ! videorate ! \
    videoflip method=$ROT ! \
    video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! \
    filesink location=$VIDEO_IN > /dev/null &

# Start the video output process to the selected port
# Reads the video from the named pipe, encodes it using x264, and sends it over UDP rtp
sudo gst-launch-1.0 filesrc location=$VIDEO_OUT blocksize=460800 do-timestamp=true ! \
     videoparse format=i420 width=640 height=480 framerate=30/1 ! \
     x264enc tune=zerolatency speed-preset=ultrafast bitrate=5000 key-int-max=30 ! \
     h264parse ! \
     'video/x-h264,profile=baseline,level=(string)4.1' ! \
     rtph264pay config-interval=-1 pt=96 ! \
     udpsink host=192.168.0.32 port=$PORT sync=false async=false > /dev/null &


# Start vision navigation process 
# Reads the video from the named pipe, processes it using the vision program, and writes the output to the other named pipe
./build/vision 

