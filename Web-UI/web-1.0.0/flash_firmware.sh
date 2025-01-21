#!/bin/bash

# Script Name : flash_firmware.sh

# Description
#  - copies the current firmware in Workspace-<version>/Tools/Firmware_update_utility
#  - Runs the code to flash the firmware.
#  Usages : ./flash_firmware.sh <workspace>

# Exit Codes:
# 0: Success
# 1: Incorrect usage (e.g., wrong number of arguments)
# 2: Specified directory does not exist
# 3: Target Workspace directory does not exist
# 4: Failed to remove existing Workspace symlink or symlink is not a link
# 5: Failed to create new Workspace symlink


# Check if the correct number of arguments are provided
if [ "$#" -ne 2 ]; then
    echo "Usage: sudo $0 <workspace> <firmware_version>"
    exit 1
fi

WORKSPACE=$1
FIRMWARE_VERSION=$2

BASE_DIRECTORY="/home/analog"

# Define the source and destination paths
SOURCE_DIR="$BASE_DIRECTORY/ADSD3500-firmware-$FIRMWARE_VERSION"
DEST_DIR="$BASE_DIRECTORY/Workspace-$WORKSPACE/Tools/Firmware_update_utility"

# Check if the source directory exists
if [ ! -d "$SOURCE_DIR" ]; then
    echo "Firmware version directory $SOURCE_DIR does not exist."
    exit 2
fi

# Copy the firmware files to the destination directory
cp -r "$SOURCE_DIR"/* "$DEST_DIR"

# echo "Firmware version $FIRMWARE_VERSION copied to $DEST_DIR successfully."

cd $DEST_DIR

COMMAND="./Firmware_Update  Fw_Update_$FIRMWARE_VERSION.bin"

# Run the script to flash the firmware
eval $COMMAND


# Define the target Workspace directory based on user input
TARGET_FOLDER="$BASE_DIRECTORY/ADSD3500-firmware-$FIRMWARE_VERSION"

# Check if the target Workspace directory exists
if [[ ! -d "$TARGET_FOLDER" ]]; then
    echo "Error: The target Workspace directory '$TARGET_FOLDER' does not exist."
    exit 3
fi

# Path to the Workspace symlink
SYMLINK_PATH="$BASE_DIRECTORY/ADSD3500-firmware"

# Remove the existing Workspace symlink if it exists
if [[ -L "$SYMLINK_PATH" ]]; then
    # echo "Removing existing Workspace symlink at '$SYMLINK_PATH'."
    rm "$SYMLINK_PATH"
    if [[ $? -ne 0 ]]; then
        echo "Error: Failed to remove existing Workspace symlink at '$SYMLINK_PATH'."
        exit 4
    fi
elif [[ -e "$SYMLINK_PATH" ]]; then
    # If Workspace exists but is not a symlink, do not remove
    echo "Error: '$SYMLINK_PATH' exists and is not a symbolic link. Cannot proceed."
    exit 4
fi

# Create a new Workspace symlink pointing to the target folder
# echo "Creating new Workspace symlink at '$SYMLINK_PATH' pointing to '$TARGET_FOLDER'."
ln -s "$TARGET_FOLDER" "$SYMLINK_PATH"
if [[ $? -ne 0 ]]; then
    echo "Error: Failed to create Workspace symlink at '$SYMLINK_PATH'."
    exit 5
fi

exit 0