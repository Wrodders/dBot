#!/bin/bash
#--------------------------------------------------
#




# Copy service files
sudo cp ~/prog/software/services/*.service /etc/systemd/system/
# Reload systemd
sudo systemctl daemon-reload
#stop all services
sudo systemctl stop twsb.service joystick.service cam.service vision.service

# Enable services (start at boot)
sudo systemctl enable --now twsb.service joystick.service

hostname=$(hostname)
#if hostnam is dbot th euse cam serves else use vision service
if [ $hostname == "dbot" ]; then
    sudo cp ~/prog/software/services/cam.service /etc/systemd/system/
    sudo systemctl enable --now cam.service

else
    sudo cp ~/prog/software/services/vision.service /etc/systemd/system/
    sudo systemctl enable --now vision.service
fi

