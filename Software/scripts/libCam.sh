#!/bin/bash
sudo libcamera-vid -t 0 --camera 0 --nopreview --autofocus-mode manual --codec yuv420 --width 640 --height 480 --inline --listen -o -
