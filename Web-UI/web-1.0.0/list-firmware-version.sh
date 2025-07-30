#!/bin/bash

# Script Name: list-firmware-version.sh
# Description:
#   - For a given directory, lists available firmware versions.
#   - If a version is specified, switches the 'ADSD3500-firmware' symlink to point to that version.
# Usage: ./list-firmware-version.sh <directory>

# Exit Codes:
# 0: Success
# 1: Incorrect usage (e.g., wrong number of arguments)
# 2: Specified directory does not exist
# 3: No Firmware directories found in the specified directory
# 4: Failed to remove existing Firmware symlink or symlink is not a link
# 5: Failed to create new Firmware symlink

# Function to display usage instructions
print_help() {
    echo "Usage: $0 <directory> [version]"
    echo "  <directory>: The base directory containing Firmware directories."
}

# Check for at least one argument
if [[ $# -lt 1 ]]; then
    echo "Error: Insufficient arguments provided."
    print_help
    exit 1
fi

# Assign arguments to variables
BASE_DIRECTORY="$1"

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
for dir in "$BASE_DIRECTORY"/ADSD3500-firmware-*; do
    # Check if it's a directory and matches the desired format (ADSD3500-firmware-x.y.z)
    if [[ -d "$dir" && "$(basename "$dir")" =~ ^ADSD3500-firmware-[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
        # Extract the version part after the hyphen
        version="${dir##*-}"
        versions+=("$version")
    fi
done

# Check if any Firmware directories were found
if [[ ${#versions[@]} -eq 0 ]]; then
    echo "Error: No Firmware directories found in '$BASE_DIRECTORY'."
    exit 3
fi

for version in "${versions[@]}"; do
        echo "$version"
    done

exit 0
