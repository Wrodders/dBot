#!/bin/bash

# Define the Raspberry Pi username and hostname
USER="pi"
HOST="dbot.local"

# Define the remote script and argument
REMOTE_SCRIPT="scripts/startUDPStream.sh"
REMOTE_ARGUMENT="-r 120p"

# SSH into the Raspberry Pi and execute the commands
ssh "$USER@$HOST" << EOF
    cd /home/pi/$REMOTE_SCRIPT || exit
    sudo bash startUDPStream.sh $REMOTE_ARGUMENT
EOF

# End of script
