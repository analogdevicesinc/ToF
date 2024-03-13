#!/bin/bash
cd Workspace/ToF/build/apps/server

command="sudo systemctl stop network-gadget"
# Run the command with sudo and read the password from standard input
echo "analog" | sudo -S $command

./aditof-server
