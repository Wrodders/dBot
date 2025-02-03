#!/bin/bash

# Ensure correct usage
if [[ $# -lt 3 ]]; then
    echo "Usage: $0 <serial_port> <baud_rate> <filter_chars>"
    echo "Example: $0 /dev/ttyUSB0 115200 'A B C'"
    exit 1
fi

PORT="$1"
BAUD="$2"
shift 2  # Remove first two arguments, leaving only the filter characters

# Convert space-separated characters into grep pattern
FILTER_PATTERN="^<($(echo "$@" | sed 's/ /|/g'))"

# Validate filter input (ensure each argument is a single ASCII character)
for char in "$@"; do
    if [[ ${#char} -ne 1 ]]; then
        echo "[ERROR] Each filter must be a single ASCII character."
        exit 1
    fi
done

# Configure the serial port
stty -F "$PORT" "$BAUD" raw -echo 2>/dev/null
if [[ $? -ne 0 ]]; then
    echo "[ERROR] Could not configure serial port $PORT. Check permissions."
    exit 1
fi

# PROGRAM BEGIN #
echo "[INFO] TWSB DEBUG CONSOLE"
echo "-------------------------"
echo "[INFO] Listening on $PORT at $BAUD baud. Filtering messages with '<$FILTER'."
echo "[INFO] Type 'exit' to quit."
echo "-------------------------"
echo "[INFO] Listening on $PORT at $BAUD baud. Filtering messages with '$@'."
echo "[INFO] Type 'exit' to quit."

# Background listener process to filter incoming messages
( cat "$PORT" | grep -E --line-buffered "$FILTER_PATTERN" ) &  
LISTENER_PID=$!

# Command loop
while true; do
    echo -n "> "
    read CMD
    if [[ "$CMD" == "exit" ]]; then
        echo "[INFO] Exiting console."
        kill "$LISTENER_PID"
        exit 0
    fi
    echo -e "<$CMD" > "$PORT"
done
