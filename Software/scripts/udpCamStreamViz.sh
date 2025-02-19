#!/bin/bash
# -------------------------------------------------- 
#  @file    udpCamStreamViz.sh
#  @brief   Visualizes the video stream from the robot, over udp.


echo "[UDP CAM VIZ]Begin Camera Stream Vizulaization"

ffplay -i udp://192.168.0.39:5000 -fflags nobuffer -flags low_delay -framedrop