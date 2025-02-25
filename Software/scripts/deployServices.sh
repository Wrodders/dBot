#!/bin/bash

echo "[DEPLOY_SERVICES] Available:"

sudo ls ~/prog/software/services/*.service

# Figure out hostname
hostname=$(hostname)
echo "Hostname: $hostname"

# Stop only the services we actually have
sudo systemctl stop twsb.service joystick.service vision.service 
#copy the services to /etc/systemd/system
sudo cp /home/pi/prog/software/services/twsb.service /etc/systemd/system/
sudo cp /home/pi/prog/software/services/joystick.service /etc/systemd/system/
sudo cp /home/pi/prog/software/services/vision.service /etc/systemd/system/


# Reload systemd to pick up new/updated units
sudo systemctl daemon-reload

# Enable & start twsb.service and joystick.service
sudo systemctl enable --now twsb.service
sudo systemctl enable --now joystick.service
sudo systemctl enable --now vision.service

# Conditionally enable/start cam or vision

echo "[DEPLOY_SERVICE] complete."