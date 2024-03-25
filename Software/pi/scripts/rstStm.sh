#!/bin/bash

# Set GPIO pin 27 as output
gpio -g mode 27 out

# Toggle GPIO pin 27
gpio -g write 27 1
sleep 0.2  # Delay for 200 milliseconds
gpio -g write 27 0