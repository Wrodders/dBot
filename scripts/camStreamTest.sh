#!/bin/bash

echo "Begin Camera Stream Test"

ffplay -i udp://192.168.0.39:5000 -fflags nobuffer -flags low_delay -framedrop