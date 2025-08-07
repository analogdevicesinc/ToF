#!/bin/bash

# Script Name: remove-workspace.sh
# Description:
#   - Lists available workspaces by version.
#   - Prevents deletion of the currently linked workspace unless a new target is selected.
#   - Updates symlink if needed and deletes the selected workspace.
# Usage: ./remove-workspace.sh





HOME="/home/analog"
SYMLINK_PATH="${HOME}/Workspace"

print_help() {
    echo "Usage: $0"
    echo "This script lists available workspaces and allows you to delete one."
}

if [[ "$1" != "y" ]]; then
  echo "Cancelled."
  exit 1
fi

# Get current symlink target
if [ -L "$SYMLINK_PATH" ]; then
    CURRENT_TARGET=$(readlink -f "$SYMLINK_PATH")
else
    echo "No symbolic link named 'Workspace' found in '${HOME}'."
    exit 2
fi

# List available workspaces
echo "Available workspaces:"
WORKSPACES=($(ls -d ${HOME}/Workspace-* 2>/dev/null))
if [ ${#WORKSPACES[@]} -eq 0 ]; then
    echo "No workspaces found."
    exit 4
fi

VERSIONS=()
for ws in "${WORKSPACES[@]}"; do
    version=$(basename "$ws" | cut -d'-' -f2-)
    VERSIONS+=("$version")
done

for i in "${!VERSIONS[@]}"; do
    echo "$((i+1)). ${VERSIONS[$i]}"
done

# Prompt user to select workspace to delete
read -p "Enter the number of the workspace you want to delete: " CHOICE
INDEX=$((CHOICE-1))

if [ "$INDEX" -lt 0 ] || [ "$INDEX" -ge "${#WORKSPACES[@]}" ]; then
    echo "Invalid selection."
    exit 5
fi

SELECTED_WORKSPACE="${WORKSPACES[$INDEX]}"

if [ "$SELECTED_WORKSPACE" == "$CURRENT_TARGET" ]; then
    echo "You selected the current workspace."
    echo ""
    echo "Please select a new workspace to point the symlink to before deletion."
    echo ""

    for i in "${!WORKSPACES[@]}"; do
        if [ "${WORKSPACES[$i]}" != "$CURRENT_TARGET" ]; then
            echo "$((i+1)). ${VERSIONS[$i]}"
        fi
    done

    read -p "Enter the number of the new workspace to point the symlink to: " NEW_CHOICE
    NEW_INDEX=$((NEW_CHOICE-1))

    if [ "${WORKSPACES[$NEW_INDEX]}" == "$CURRENT_TARGET" ] || [ "$NEW_INDEX" -lt 0 ] || [ "$NEW_INDEX" -ge "${#WORKSPACES[@]}" ]; then
        echo "Invalid selection or same as current."
        exit 6
    fi

    NEW_TARGET="${WORKSPACES[$NEW_INDEX]}"
    ln -sfn "$NEW_TARGET" "$SYMLINK_PATH"
    echo "Symlink updated to point to: ${VERSIONS[$NEW_INDEX]}"
fi

# Delete the selected workspace
rm -rf "$SELECTED_WORKSPACE"

if [ $? -eq 0 ]; then
    echo "Successfully deleted workspace version '${VERSIONS[$INDEX]}'."
    echo ""
    echo "Now system will reboot"
    sudo reboot
    exit 0
else
    echo "Error: Failed to delete workspace version '${VERSIONS[$INDEX]}'."
    exit 7
fi
