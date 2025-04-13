#!/bin/bash
# -------------------------------------------------- 
#  @file    twsbConsole.sh
#  @brief   A simple console for debugging raw TWSB messages.

# Default values
DEFAULT_PORT="/dev/ttyUSB0"
DEFAULT_BAUD= "230400"
DEFAULT_FILTER="A B C D E"

# Assign arguments or use defaults
PORT="${1:-$DEFAULT_PORT}"
BAUD="${2:-$DEFAULT_BAUD}"
FILTER_CHARS="${@:3}"

# If no filter characters are provided, use the default
if [[ -z "$FILTER_CHARS" ]]; then
    FILTER_CHARS="$DEFAULT_FILTER"
fi

# Convert space-separated characters into grep pattern
FILTER_PATTERN="^<($(echo "$FILTER_CHARS" | sed 's/ /|/g'))"

# Validate filter input 
for char in $FILTER_CHARS; do
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
echo "[DEBUG_CONSOLE][INFO] TWSB DEBUG CONSOLE"
echo "-------------------------"
echo "[DEBUG_CONSOLE][INFO] Listening on $PORT at $BAUD baud. Filtering messages with '<$FILTER_CHARS>'."
echo "[DEBUG_CONSOLE][INFO] Type 'exit' to quit."
echo "-------------------------"

# Background listener process to filter incoming messages
( cat "$PORT" | grep -E --line-buffered "$FILTER_PATTERN" ) &  
LISTENER_PID=$!

# Command loop
while true; do
    echo -n "> "
    read CMD
    if [[ "$CMD" == "exit" ]]; then
        echo "[DEBUG_CONSOLE][INFO] Exiting console."
        kill "$LISTENER_PID"
        exit 0
    fi
    echo -e "<$CMD" > "$PORT"
done
