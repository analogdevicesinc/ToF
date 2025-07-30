#!/bin/bash

# This script kills all instances of aditof-server started with sudo/root privileges.

echo "Killing all aditof-server processes running as root..."

# Find all PIDs of aditof-server owned by root and kill them
pids=$(ps -eo pid,user,comm | awk '$2=="root" && $3=="aditof-server" {print $1}')

if [ -z "$pids" ]; then
    echo "No aditof-server processes running as root found."
    exit 0
fi

echo "Found the following PIDs: $pids"
sudo kill -9 $pids

echo "All aditof-server processes killed."