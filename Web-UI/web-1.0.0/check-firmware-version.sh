#!/bin/bash

# Script Name: check-firmware-version.sh
# Description:
#   - checks the firmware version which is compatible with current workspace
#   - Removes leading zeros and formats the version as x.x.x.
# Usage: ./check-firmware-version.sh <Workspace-version>

# Check for exactly one argument
if [ "$#" -ne 1 ]; then
    echo "Usage: $0 <Workspace_version>"
    exit 1
fi

WORKSPACE_VERSION="$1"

# Path to the text file
FILE_PATH="/home/analog/Workspace/requirements/fm_version_requirements.txt"

# Read the firmware version from the file
FIRMWARE_VERSION=$(grep -oP 'firmware_version=\K[^\s]+' "$FILE_PATH")

# Check if the firmware version matches the expected version
echo "$FIRMWARE_VERSION"