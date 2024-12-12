#!/bin/bash

ffmpeg -re -stream_loop -1 -i output.mp4 -c:v libx264 -preset fast -f mpegts udp://localhost:5000
