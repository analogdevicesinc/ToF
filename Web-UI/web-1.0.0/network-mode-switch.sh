#!/bin/bash

# Script Name : network-mode-switch.sh

# Description
#  - copies the current firmware in Workspace-<version>/Tools/Firmware_update_utility
#  - Runs the code to flash the firmware.
#  Usages : ./network-mode-switch.sh <Workspace-<version>>

# Exit Codes:
# 0: Success
# 1: Some issue is there.
# 2: Invalid Choice

# Check if the correct number of arguments are provided
if [ "$#" -ne 2 ]; then
    echo "Usage: sudo $0 <version> <modify/check>"
    exit 1
fi

VERSION=$1
USER_CHOICE=$2

# File to be modified
CONFIG_FILE_1="/home/analog/Workspace-$VERSION/bin/usb-gadget.sh"
TEMP_FILE="/home/analog/Workspace-$VERSION/ToF/doc/usb-gadget.tmp"
CONFIG_FILE_2="/home/analog/Workspace-$VERSION/ToF/doc/usb-gadget.sh"

if [ "$USER_CHOICE" == "modify" ]; then
    # Check if both files exist
    if [ -f "$CONFIG_FILE_1" ] && [ -f "$CONFIG_FILE_2" ]; then
        # Swap the files using a temporary file
        mv "$CONFIG_FILE_1" "$TEMP_FILE"
        mv "$CONFIG_FILE_2" "$CONFIG_FILE_1"
        mv "$TEMP_FILE" "$CONFIG_FILE_2"
        echo "Files swapped successfully."
    else
        echo "One or both files not found: $CONFIG_FILE_1, $CONFIG_FILE_2"
    fi
fi

if [ "$USER_CHOICE" == "check" ]; then
    if grep -q "#create_rndis configs/c.1 rndis.0" $CONFIG_FILE_1 && grep -q "create_ecm configs/c.1 ecm.0" $CONFIG_FILE_1 ; then
        echo "ECM mode."
    else
        if grep -q "create_rndis configs/c.1 rndis.0" $CONFIG_FILE_1 && grep -q "#create_ecm configs/c.1 ecm.0" $CONFIG_FILE_1 ; then
            echo "RNDIS mode."
        else
            echo "Neither create_rndis nor create_ecm found."
        fi
    fi
fi