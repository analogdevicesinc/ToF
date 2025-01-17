#!/bin/bash

# Script Name : nvm-read.sh

# Description
#  -  It takes a file name where the complete flash.bin from the imager module should be saved
#  Usages : ./nvm-read.sh <Workspace-version> <file-name>

# Exit Codes:
# 0: Success
# 1: NVM directory does not exists.

# Check if the correct number of arguments are provided
if [ "$#" -ne 2 ]; then
    echo "Usage: sudo $0 <Workspace-version> <file-name>"
    exit 1
fi

FILE_NAME=$2
VERSION=$1

BASE_DIRECTORY="/home/analog"

DEST_DIR="$BASE_DIRECTORY/Workspace-$VERSION/Tools/host_boot_tools/NVM_Utils"

# Check if the destination directory exists
if [ ! -d "$DEST_DIR" ]; then
    echo "NVM directory $DEST_DIR does not exist."
    exit 1
fi

cd $DEST_DIR

COMMAND="./NVM_READ $FILE_NAME"

# Run the script to read the NVM
eval $COMMAND

exit 0