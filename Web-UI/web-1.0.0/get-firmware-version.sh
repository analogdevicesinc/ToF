#!/bin/bash

# Script Name: get-firmware-version.sh
# Description:
#   - Parses a text file to extract the firmware version from the first 4 bytes of the fourth line.
#   - Removes leading zeros and formats the version as x.x.x.
# Usage: ./get-firmware-version.sh <Workspace-version>

# Check for exactly one argument
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <Workspace_version>"
    exit 1
fi

# Assign the first argument to WORKSPACE_VERSION
WORKSPACE_VERSION="$1"

# Construct the command with the extracted firmware version
cd /home/analog/Workspace-$WORKSPACE_VERSION/Tools/ctrl_app
COMMAND="./ctrl_app infile.txt > out.txt"

# Run the command
eval $COMMAND


FILE_PATH="out.txt"

# Check if the file exists
if [ ! -f "$FILE_PATH" ]; then
    echo "Error: File '$FILE_PATH' not found."
    exit 2
fi

# Extract the first 4 bytes of the fourth line
firmware_version=$(sed -n '4p' "$FILE_PATH" | cut -d' ' -f1-3)

formatted_version=$(echo $firmware_version | sed 's/\b0\+\([0-9]\)/\1/g')
formatted_version=$(echo $formatted_version | sed 's/ \+/./g')
formatted_version=$(echo $formatted_version | sed 's/\.\+/\./g')
formatted_version=$(echo $formatted_version | sed 's/\.$//g')

# # Display the formatted firmware version
# echo "Current Firmware Version: $formatted_version"
echo "$formatted_version"