#!/bin/bash

# Script Name: switch-UI.sh
# Description:
#   - switches the 'web' symlink to point to that version.
# Usage: ./switch-UI.sh <directory> [version]

# Exit Codes:
# 0: Success
# 1: Incorrect usage (e.g., wrong number of arguments)
# 2: Specified directory does not exist
# 3: No UI directories found in the specified directory
# 4: Invalid version format
# 5: Specified version does not exist
# 6: Target UI directory does not exist
# 7: Failed to remove existing UI symlink or symlink is not a link
# 8: Failed to create new UI symlink

# Function to display usage instructions
print_help() {
    echo "Usage: $0 <directory> <version>"
    echo "  <directory>: The base directory containing UI directories."
    echo "  <version>: The specific version to switch to."
}

# Check for at least one argument
if [[ $# -lt 2 ]]; then
    echo "Error: Insufficient arguments provided."
    print_help
    exit 1
fi

# Assign arguments to variables
BASE_DIRECTORY="$1"
USER_INPUT="$2"

# Expand ~ to home directory if BASE_DIRECTORY is ~
if [[ "$BASE_DIRECTORY" == "~" ]]; then
    BASE_DIRECTORY="$HOME"
fi

# Check if the base directory exists
if [[ ! -d "$BASE_DIRECTORY" ]]; then
    echo "Error: The specified directory '$BASE_DIRECTORY' does not exist."
    exit 2
fi

# Initialize an array to hold the versions
versions=()

# Loop through directories in the specified base directory
for dir in "$BASE_DIRECTORY"/web-*; do
    # Check if it's a directory and matches the desired format (UI-x.y.z)
    if [[ -d "$dir" && "$(basename "$dir")" =~ ^web-[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        # Extract the version part after the hyphen
        version="${dir##*-}"
        versions+=("$version")
    fi
done

# Check if any UI directories were found
if [[ ${#versions[@]} -eq 0 ]]; then
    echo "Error: No UI directories found in '$BASE_DIRECTORY'."
    exit 3
fi

# Validate the format of the user input (expecting x.y.z)
if [[ ! "$USER_INPUT" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
    echo "Error: Invalid version format '$USER_INPUT'. Expected format: x.y.z"
    exit 4
fi

# Check if the user-specified version exists in the versions array
version_exists=false
for version in "${versions[@]}"; do
    if [[ "$version" == "$USER_INPUT" ]]; then
        version_exists=true
        break
    fi
done

if [[ "$version_exists" == false ]]; then
    echo "Error: The version '$USER_INPUT' does not exist in '$BASE_DIRECTORY'."
    echo "Available versions:"
    for version in "${versions[@]}"; do
        echo "$version"
    done
    exit 5
fi

# Define the target UI directory based on user input
TARGET_FOLDER="$BASE_DIRECTORY/web-$USER_INPUT"

# Check if the target UI directory exists
if [[ ! -d "$TARGET_FOLDER" ]]; then
    echo "Error: The target UI directory '$TARGET_FOLDER' does not exist."
    exit 6
fi

# Path to the UI symlink
SYMLINK_PATH="$BASE_DIRECTORY/web"

# Remove the existing UI symlink if it exists
if [[ -L "$SYMLINK_PATH" ]]; then
    echo "Removing existing UI symlink at '$SYMLINK_PATH'."
    rm "$SYMLINK_PATH"
    if [[ $? -ne 0 ]]; then
        echo "Error: Failed to remove existing UI symlink at '$SYMLINK_PATH'."
        exit 7
    fi
elif [[ -e "$SYMLINK_PATH" ]]; then
    # If UI exists but is not a symlink, do not remove
    echo "Error: '$SYMLINK_PATH' exists and is not a symbolic link. Cannot proceed."
    exit 7
fi

# Create a new UI symlink pointing to the target folder
echo "Creating new UI symlink at '$SYMLINK_PATH' pointing to '$TARGET_FOLDER'."
ln -s "$TARGET_FOLDER" "$SYMLINK_PATH"
if [[ $? -ne 0 ]]; then
    echo "Error: Failed to create UI symlink at '$SYMLINK_PATH'."
    exit 8
fi

echo "UI has been successfully switched to version '$USER_INPUT'."
exit 0
