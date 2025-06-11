#!/bin/bash

# Script Name: get-ui.sh
# Description:
#   - For a given directory, checks if a symbolic link web supplied from user exists.
#   - If it exists and points to '{web}-<version>', it extracts and prints <version>.
#   - If not, it prints appropriate messages.
# Usage: ./get-ui.sh <directory>

# Exit Codes:
# 0: Success
# 1: Incorrect usage
# 2: Symbolic link does not exist
# 3: Symbolic link exists but does not point to the expected target
# 4: Unexpected error

# Function to display usage instructions
print_help() {
    echo "Usage: $0 <directory>"
    echo "Provide the directory containing the 'web' symbolic link."
}

# Check for exactly one argument
if [ "$#" -ne 1 ]; then
    echo "Error: Incorrect number of arguments."
    print_help
    exit 1
fi

# Assign the first argument to DIRECTORY
DIRECTORY="$1"

# Expand ~ to home directory if DIRECTORY is ~
if [ "$DIRECTORY" == "~" ]; then
    DIRECTORY="$HOME"
fi

# Path to the symbolic link
# SYMLINK_PATH="${DIRECTORY}/${FOLDERNAME}"
SYMLINK_PATH="${DIRECTORY}/web"

# Check if the symbolic link exists
if [ -L "$SYMLINK_PATH" ]; then
    # Resolve the absolute path the symlink points to
    SYMLINK_TARGET=$(readlink -f "$SYMLINK_PATH")
    
    # Get the basename of the target directory
    TARGET_BASENAME=$(basename "$SYMLINK_TARGET")
    
    # Use regex to check if the target matches the pattern Workspace-<version>
    if [[ "$TARGET_BASENAME" =~ ^web-(.+)$ ]]; then
        VERSION="${BASH_REMATCH[1]}"
        echo "$VERSION"
        exit 0
    else
        echo "The symbolic link 'Workspace' does not point to a directory matching 'Workspace-<version>'."
        exit 3
    fi
else
    echo "No symbolic link named 'Workspace' found in '${DIRECTORY}'."
    exit 2
fi
