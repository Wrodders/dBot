#!/bin/bash

# profiler.sh - Profiling script for C++ program using signals

# Initialize timestamps
start_read_time=0
end_read_time=0
start_process_time=0
end_process_time=0

# Record the time when signal is received
record_time() {
    local signal_name=$1
    local timestamp=$(date +%s%3N)  # Milliseconds timestamp
    echo "$signal_name received at $timestamp"
}

# Signal handlers
handle_read_signal() {
    record_time "SIGUSR1 (Start reading frame)"
    start_read_time=$(date +%s%3N)
}

handle_process_signal() {
    record_time "SIGUSR2 (End processing frame)"
    end_process_time=$(date +%s%3N)

    # Calculate and log the time difference between read and process
    if [ -n "$start_read_time" ] && [ -n "$end_process_time" ]; then
        processing_duration=$((end_process_time - start_read_time))
        echo "Processing time for frame: ${processing_duration}ms"
    fi
}

# Set up signal traps
trap handle_read_signal SIGUSR1
trap handle_process_signal SIGUSR2

# Main loop: Just wait for signals to process
echo "Profiler started. Waiting for signals..."
while true; do
    sleep 1
done
