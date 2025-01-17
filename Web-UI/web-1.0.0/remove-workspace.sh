#!/bin/bash

# Script Name: remove-workspace.sh
# Description: 
#   - Removes the symbolic link ~/Workspace if it points to <directory>/Workspace-<version>.
#   - Moves the specified Workspace directory (<directory>/Workspace-<version>) to /tmp/Workspace-<version>-<GUID>.
# Usage: ./remove-workspace.sh <directory> <version>

# Exit Codes:
# 0: Success
# 1: Incorrect usage
# 2: Symbolic link does not exist
# 3: Symbolic link exists but does not point to the expected target
# 4: 'uuidgen' command not found
# 5: Failed to remove symbolic link
# 6: Workspace directory does not exist
# 7: Failed to move workspace directory

# Function to display usage instructions
print_help() {
    echo "Usage: $0 <directory> <version>"
    echo "Both <directory> and <version> parameters are mandatory."
}

# Check for exactly two arguments
if [ "$#" -ne 2 ]; then
    echo "Error: Incorrect number of arguments."
    print_help
    exit 1
fi

# Assign arguments to variables
DIRECTORY="$1"
VERSION="$2"

# Expand ~ to home directory if DIRECTORY is ~
if [ "$DIRECTORY" == "~" ]; then
    DIRECTORY="$HOME"
fi

# Construct the workspace directory name
WORKSPACE_DIR="Workspace-${VERSION}"

# Full path to the workspace directory
FULL_WORKSPACE_PATH="${DIRECTORY}/${WORKSPACE_DIR}"

# Path to the symbolic link named 'Workspace' in the user's home directory
SYMLINK_PATH="${HOME}/Workspace"

# Function to remove the symbolic link if it points to the expected target
remove_symlink_if_matches() {
    local symlink="$1"
    local expected_target="$2"

    if [ -L "$symlink" ]; then
        # Resolve the absolute path the symlink points to
        SYMLINK_TARGET=$(readlink -f "$symlink")

        if [ "$SYMLINK_TARGET" == "$expected_target" ]; then
            echo "Symbolic link '${symlink}' points to '${expected_target}'. Removing symbolic link."
            rm "$symlink"
            if [ $? -eq 0 ]; then
                echo "Symbolic link '${symlink}' removed successfully."
                return 0
            else
                echo "Error: Failed to remove symbolic link '${symlink}'."
                return 5
            fi
        else
            echo "Symbolic link '${symlink}' does not point to '${expected_target}'. No action taken."
            return 3
        fi
    else
        echo "No symbolic link named 'Workspace' found in '${HOME}'. No action taken."
        return 2
    fi
}

# Remove the symbolic link before moving the directory
remove_symlink_if_matches "$SYMLINK_PATH" "$FULL_WORKSPACE_PATH"
SYMLINK_STATUS=$?

if [ $SYMLINK_STATUS -eq 5 ] || [ $SYMLINK_STATUS -eq 3 ]; then
    exit $SYMLINK_STATUS
fi

# Check if the workspace directory exists
if [ -d "$FULL_WORKSPACE_PATH" ]; then
    echo "Found workspace directory: '${FULL_WORKSPACE_PATH}'"

    # Check if 'uuidgen' is available
    if ! command -v uuidgen &> /dev/null; then
        echo "Error: 'uuidgen' command not found. Please install it to generate GUIDs."
        exit 4
    fi

    # Generate a GUID
    GUID=$(uuidgen)
    echo "Generated GUID: ${GUID}"

    # Define the new directory name with GUID
    NEW_WORKSPACE_DIR="Workspace-${VERSION}-${GUID}"

    # Move the workspace directory to /tmp with the new name
    mv "$FULL_WORKSPACE_PATH" "/tmp/${NEW_WORKSPACE_DIR}"
    
    if [ $? -eq 0 ]; then
        echo "Successfully moved '${FULL_WORKSPACE_PATH}' to '/tmp/${NEW_WORKSPACE_DIR}'."
        exit 0
    else
        echo "Error: Failed to move '${FULL_WORKSPACE_PATH}' to '/tmp/${NEW_WORKSPACE_DIR}'."
        exit 7
    fi
else
    echo "Workspace directory '${FULL_WORKSPACE_PATH}' does not exist. No action taken."
    exit 6
fi
