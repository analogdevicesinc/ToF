#!/bin/bash

# call the newtwork-switch script from web to switch the network
SCRIPT_PATH="/home/analog/web-1.0.0/network-mode-switch.sh"

# run script with windows
sudo "$SCRIPT_PATH" 1 windows

# reboot the system
sudo reboot

