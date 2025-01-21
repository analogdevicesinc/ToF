#!/bin/bash

# Script Name : change-permission.sh

# Description
#  - copies the current firmware in Workspace-<version>/Tools/Firmware_update_utility
#  - Runs the code to flash the firmware.
#  Usages : ./change-permission.sh <modify/view>

# Exit Codes:
# 0: Success
# 1: Some issue is there.
# 2: Invalid Choice

# Check if the correct number of arguments are provided
if [ "$#" -ne 1 ]; then
    echo "Usage: sudo $0 <modify/view>"
    exit 1
fi

USER_CHOICE=$1

# File to be modified
FILE="/boot/extlinux/extlinux.conf"

# Check the content of the append root line
append_line=$(sed -n '/label ADSD3500+ADSD3100/,/label/{/append root/p;}' "$FILE" | head -n 1)


if [ "$USER_CHOICE" == "view" ]; then
# Modify the content based on the condition
    if echo "$append_line" | grep -q "rootwait ro init=/sbin/reinit"; then
        echo "Permission is : RO"
    elif echo "$append_line" | grep -q "rootwait rw"; then
        echo "Permission is : RW"
    else 
        echo "Something went wrong!"
    fi
elif [ "$USER_CHOICE" == "modify" ]; then
    # Modify the content based on the condition
    if echo "$append_line" | grep -q "rootwait ro init=/sbin/reinit"; then
        sudo sed -i '/label ADSD3500+ADSD3100/,/label/{/append root/s|rootwait ro init=/sbin/reinit|rootwait rw|;}' "$FILE"
        echo "Change Permission from RO to RW"
    elif echo "$append_line" | grep -q "rootwait rw"; then
        sudo sed -i '/label ADSD3500+ADSD3100/,/label/{/append root/s|rootwait rw|rootwait ro init=/sbin/reinit|;}' "$FILE"
        echo "Change Permission from RW to RO"
    else 
        echo "Something went wrong!"
        exit 1
    fi
else
    echo "Invalid choice. Please enter 'modify' or 'view'."
    exit 2
fi
exit 0