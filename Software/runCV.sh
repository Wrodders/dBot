#!/bin/bash

# Default settings
SOURCE="camera"  # Default source is camera
UDP_ADDRESS="udp://127.0.0.1:5000"  # Default UDP address
CAMERA_INDEX=0  # Default camera index
WIDTH=640  # Default frame width
HEIGHT=480  # Default frame height

# Parse arguments
while [[ "$#" -gt 0 ]]; do
    case $1 in
        --source)
            SOURCE="$2"
            shift 2
            ;;
        --udp_address)
            UDP_ADDRESS="$2"
            shift 2
            ;;
        --camera)
            CAMERA_INDEX="$2"
            shift 2
            ;;
        --width)
            WIDTH="$2"
            shift 2
            ;;
        --height)
            HEIGHT="$2"
            shift 2
            ;;
        --help)
            echo "Usage: $0 [options]"
            echo "--source [camera|udp]    Set video source (default: camera)"
            echo "--udp_address [IP]       Set UDP address (default: udp://127.0.0.1:5000)"
            echo "--camera [index]         Set camera index (default: 0)"
            echo "--width [width]          Set video width (default: 640)"
            echo "--height [height]        Set video height (default: 480)"
            exit 0
            ;;
        *)
            echo "Unknown option $1"
            exit 1
            ;;
    esac
done

# Function to run FFmpeg with camera as source
run_camera_stream() {
    ffmpeg -f avfoundation -framerate 30 -video_size ${WIDTH}x${HEIGHT} -i "$CAMERA_INDEX" \
    -vf "format=gray" -f rawvideo -pix_fmt bgr24 -vcodec rawvideo - | ./main
}

# Function to run FFmpeg with UDP stream as source
run_udp_stream() {
    ffmpeg -i "$UDP_ADDRESS" -vf "format=gray" -f rawvideo -pix_fmt bgr24 -vcodec rawvideo - | ./main
}

# Run the appropriate stream based on source
if [[ "$SOURCE" == "camera" ]]; then
    echo "Starting camera stream..."
    run_camera_stream
elif [[ "$SOURCE" == "udp" ]]; then
    echo "Starting UDP stream from $UDP_ADDRESS..."
    run_udp_stream
else
    echo "Error: Invalid source. Please choose 'camera' or 'udp'."
    exit 1
fi
