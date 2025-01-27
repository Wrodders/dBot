# Help function
function show_help {
    echo "Usage: $0 [-h] [-u USER] [-i HOSTNAME] [-p DEVICE]"
    echo
    echo "Options:"
    echo "  -u USER        Specify the username (default: pi)."
    echo "  -i HOSTNAME    Specify the hostname (default: dbot.local)."
    echo "  -h             Show this help message and exit."
    echo
    exit 0
}

# Default values
USER="pi"
HOSTNAME="bot.local"

# get software folder usign pwd 
# get the current directory




rsync -az . "$USER"@"$HOSTNAME":prog/software
rsync -az ../Firmware/pubrpc/mcuComs.h "$USER"@"$HOSTNAME":prog/software/inc

# Build the project on the remote

ssh "$USER"@"$HOSTNAME" << EOF

    cd prog/software
    bash buildSW.sh
EOF


