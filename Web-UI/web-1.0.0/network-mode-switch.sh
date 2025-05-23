#!/bin/bash

# Usage: sudo ./network-mode-switch.sh <version> <windows/ubuntu/check>

if [ "$#" -ne 2 ]; then
    echo "Usage: sudo $0 <version> <windows/ubuntu/check>"
    exit 1
fi

VERSION=$1
USER_CHOICE=$2

CONFIG_FILE_1="/home/analog/Workspace-$VERSION/bin/usb-gadget.sh"
TEMP_FILE="/home/analog/Workspace-$VERSION/ToF/doc/usb-gadget.tmp"
CONFIG_FILE_2="/home/analog/Workspace-$VERSION/ToF/doc/usb-gadget.sh"

# Function to detect current mode
detect_mode() {
    if grep -q "#create_rndis" "$CONFIG_FILE_1" && grep -q "create_ecm" "$CONFIG_FILE_1"; then
        echo "ecm"
    elif grep -q "create_rndis" "$CONFIG_FILE_1" && grep -q "#create_ecm" "$CONFIG_FILE_1"; then
        echo "rndis"
    else
        echo "unknown"
    fi
}

CURRENT_MODE=$(detect_mode)

if [ "$USER_CHOICE" == "windows" ]; then
    if [ "$CURRENT_MODE" == "rndis" ]; then
        echo "Already in RNDIS mode for Windows."
    elif [ "$CURRENT_MODE" == "ecm" ]; then
        mv "$CONFIG_FILE_1" "$TEMP_FILE"
        mv "$CONFIG_FILE_2" "$CONFIG_FILE_1"
        mv "$TEMP_FILE" "$CONFIG_FILE_2"
        echo "Switched to RNDIS mode for Windows."
        chmod +x CONFIG_FILE_1
    else
        echo "Unable to determine current mode."
    fi

elif [ "$USER_CHOICE" == "ubuntu" ]; then
    if [ "$CURRENT_MODE" == "ecm" ]; then
        echo "Already in ECM mode for Ubuntu."
    elif [ "$CURRENT_MODE" == "ecm" ]; then
        mv "$CONFIG_FILE_1" "$TEMP_FILE"
        mv "$CONFIG_FILE_2" "$CONFIG_FILE_1"
        mv "$TEMP_FILE" "$CONFIG_FILE_2"
        echo "Switched to ECM mode for Ubuntu."
        chmod +x CONFIG_FILE_1
    else
        echo "Unable to determine current mode."
    fi

else
    echo "Invalid choice. Use 'windows', 'ubuntu', or 'check'."
    exit 2
fi
