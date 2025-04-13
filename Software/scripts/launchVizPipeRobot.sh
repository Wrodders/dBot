#!/bin/bash
#--------------------------------------------------
#  @file    launchVisionPi.sh
#  @brief   Launches the vision test Pipeline on Raspberry Pi.

# Determine the port based on the hostname
HOSTNAME=$(hostname)
hostname
case $HOSTNAME in
    "dbot")
        PORT=5001 # Pi Zero 2W uses hardware encoder
        ROT=2 # Rotate 180 degrees
        sudo gst-launch-1.0 -vv libcamerasrc ! \
            videoconvert  ! \
            video/x-raw,format=I420,width=640,height=480,framerate=15/1 ! \
            v4l2h264enc ! 'video/x-h264,level=(string)4' !\
            h264parse ! queue! rtph264pay config-interval=1 pt=96 !\
            udpsink host=192.168.0.32 port=$PORT 
        ;;
    "bot") 
        cd /home/pi/prog/software

        PORT=5002 # Raspbeyr PI 5 uses software encoder
        ROT=5
        gst-launch-1.0 -v libcamerasrc ! \
            video/x-raw,format=I420,width=640,height=480,framerate=30/1 ! \
            videorate ! videoconvert ! video/x-raw,format=I420 ! queue ! \
            x264enc tune=zerolatency speed-preset=ultrafast quantizer=25 key-int-max=15 ! \
            h264parse ! rtph264pay config-interval=1 pt=96 ! udpsink host=192.168.0.32  port=5002
        ;;
    *)
        echo "[ERROR] Device hostname not recognized"
        exit 1
        ;;
esac




