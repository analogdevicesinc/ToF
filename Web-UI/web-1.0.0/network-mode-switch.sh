#!/bin/bash

# Usage: sudo ./network-mode-switch.sh <version> <windows/ubuntu/check>

if [ "$#" -ne 2 ]; then
    echo "Usage: sudo $0 <version> <windows/ubuntu/check>"
    exit 1
fi

VERSION=$1
USER_CHOICE=$2

CONFIG_FILE_1="/home/analog/Workspace/bin/usb-gadget.sh"
CONFIG_FILE_3="/home/analog/Workspace/bin/"
TEMP_FILE="/home/analog/Workspace/requirements/usb-gadget.tmp"
CONFIG_FILE_2="/home/analog/Workspace/requirements/usb-gadget.sh"

# Check if both files exist
if [ ! -f "$CONFIG_FILE_1" ] || [ ! -f "$CONFIG_FILE_2" ]; then
    echo "Error: One or both config files do not exist."
    exit 1
fi


detect_mode() {
    rndis_enabled=$(grep -E '^[[:space:]]*create_rndis[[:space:]]+configs' "$CONFIG_FILE_1")
    ecm_enabled=$(grep -E '^[[:space:]]*create_ecm[[:space:]]+configs' "$CONFIG_FILE_1")
    rndis_commented=$(grep -E '^[[:space:]]*#[[:space:]]*create_rndis[[:space:]]+configs' "$CONFIG_FILE_1")
    ecm_commented=$(grep -E '^[[:space:]]*#[[:space:]]*create_ecm[[:space:]]+configs' "$CONFIG_FILE_1")

    if [[ -n "$ecm_enabled" && -n "$rndis_commented" ]]; then
        echo "ecm"
    elif [[ -n "$rndis_enabled" && -n "$ecm_commented" ]]; then
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
        cp "$CONFIG_FILE_1" "$TEMP_FILE"
        cp "$CONFIG_FILE_2" "$CONFIG_FILE_3"
        cp "$TEMP_FILE" "$CONFIG_FILE_2"
        echo "Switched to RNDIS mode for Windows."
        #chmod +x "$CONFIG_FILE_1"
    else
        echo "Unable to determine current mode."
    fi

elif [ "$USER_CHOICE" == "ubuntu" ]; then
    if [ "$CURRENT_MODE" == "ecm" ]; then
        echo "Already in ECM mode for Ubuntu."
    elif [ "$CURRENT_MODE" == "rndis" ]; then
        cp "$CONFIG_FILE_1" "$TEMP_FILE"
        cp "$CONFIG_FILE_2" "$CONFIG_FILE_3"
        cp "$TEMP_FILE" "$CONFIG_FILE_2"
        echo "Switched to ECM mode for Ubuntu."
        #chmod +x "$CONFIG_FILE_1"
    else
        echo "Unable to determine current mode."
    fi


else
    echo "Invalid choice. Use 'windows', 'ubuntu', or 'check'."
    exit 2
fi

# Clean up
rm -f "$TEMP_FILE"

exit 0
