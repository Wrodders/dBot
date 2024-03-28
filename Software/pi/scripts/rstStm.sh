#!/bin/bash

# Function to clean up GPIO and exit
cleanup_exit() {
    gpio mode 21 in
    exit
}

# Set pin 21 as input initially
gpio mode 21 in

# Parse command-line arguments
if [[ $1 == "hold" ]]; then
    # Set pin 21 as output low to simulate open drain
    gpio mode 21 out
    gpio write 21 0

    # Trap interrupt signals to cleanup and exit
    trap 'cleanup_exit' SIGINT SIGTERM

    # Loop indefinitely
    while true; do
        sleep 1
    done
else
    # Pull down pin 21 by setting it as output low
    gpio mode 21 out
    gpio write 21 0

    # Sleep for 1 second
    sleep 1

    # Set pin 21 back to input to float
    gpio mode 21 in
fi

# Clean up GPIO and exit
cleanup_exit
