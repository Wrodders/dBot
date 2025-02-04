#!/bin/bash
PROGRAM_NAME="LAUCH_CAMERA"
echo "[$PROGRAM_NAME][INFO] Launching camera. 360p @ 30fps"
sudo 'rpicam-vid -t 0 --camera 0 --nopreview --codec yuv420 --width 640 --height 360 --inline --listen -o - > ///tmp/video_out'