#!/bin/bash

# Script Name: list-ui-version.sh
# Description:
#   - For a given directory, lists available Web-UI versions.
#   - If a version is specified, switches the 'web' symlink to point to that version.
# Usage: ./list-ui-version.sh <directory>

# Exit Codes:
# 0: Success
# 1: Incorrect usage (e.g., wrong number of arguments)
# 2: Specified directory does not exist
# 3: No UI directories found in the specified directory
# 4: Failed to remove existing UI symlink or symlink is not a link
# 5: Failed to create new UI symlink

# Function to display usage instructions
print_help() {
    echo "Usage: $0 <directory> [version]"
    echo "  <directory>: The base directory containing UI directories."
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
for dir in "$BASE_DIRECTORY"/web-*; do
    # Check if it's a directory and matches the desired format (web-x.y.z)
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

for version in "${versions[@]}"; do
        echo "$version"
    done

exit 0
