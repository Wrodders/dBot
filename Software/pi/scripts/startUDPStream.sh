#!/bin/bash

RESOLUTION="360p"
HOST_IP="192.168.1.122"
PORT=5000

while [ "$#" -gt 0 ]; do
    case "$1" in
        -r|--resolution)
            RESOLUTION="$2"
            shift 2
            ;;
        -h|--host)
            HOST_IP="$2"
            shift 2
            ;;
        -p|--port)
            PORT="$2"
            shift 2
            ;;
        -help|--help)
            echo "Usage: $0 [OPTIONS]"
            echo "Options:"
            echo "  -r, --resolution RES     Set the resolution (120p, 360p, 720p). Default: 360p"
            echo "  -h, --host HOST_IP       Set the host IP address. Default: 192.168.1.22"
            echo "  -p, --port PORT          Set the port number. Default: 5000"
            echo "  -help, --help            Display this help message"
            exit 0
            ;;
        *)
            echo "Invalid argument: $1"
            exit 1
            ;;
    esac
done

RESWIDTH=""
RESHEIGHT=""

case "$RESOLUTION" in
    "120p")
        RESWIDTH=160
        RESHEIGHT=120
        ;;
    "360p")
        RESWIDTH=640
        RESHEIGHT=360
        ;;
    "720p")
        RESWIDTH=1280
        RESHEIGHT=720
        ;;
    *)
        echo "Invalid resolution. Supported resolutions: 120p, 360p, 720p"
        exit 1
        ;;
esac


GST_PIPELINE=" gst-launch-1.0 -v libcamerasrc ! video/x-raw, width=$RESWIDTH, height=$RESHEIGHT ! videoconvert ! jpegenc ! rtpjpegpay ! udpsink host=$HOST_IP port=$PORT sync=0"

LIBCAM_PIPE="libcamera-vid -t 0 --rotation 180 --autofocus-mode=manual --inline -o udp://$HOST_IP:$PORT" # THIS WORKS REALY WELL 


echo "Starting GStreamer pipeline with $RESWIDTH x $RESHEIGHT resolution, host: $HOST_IP, port: $PORT..."
$LIBCAM_PIPE



