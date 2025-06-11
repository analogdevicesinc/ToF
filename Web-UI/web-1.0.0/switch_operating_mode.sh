#!/bin/bash

# Script to toggle or view default ROS environment in ~/.bashrc
# Usage:
#   ./switch_operating_mode.sh [noetic|humble|legacy]
#   ./switch_operating_mode.sh view

TARGET="$1"
BASHRC="/home/analog/.bashrc"
BACKUP="/home/analog/.bashrc.backup"

# Handle view command
if [[ "$TARGET" == "view" ]]; then
    if grep -q "source /opt/ros/noetic/setup.bash" "$BASHRC"; then
        echo "noetic"
    elif grep -q "source /opt/ros/humble/setup.bash" "$BASHRC"; then
        echo "humble"
    else
        echo "Legacy"
    fi
    exit 0
fi

# Validate input
if [[ "$TARGET" != "noetic" && "$TARGET" != "humble" && "$TARGET" != "legacy" ]]; then
    echo "Usage: $0 [noetic|humble|legacy|view]"
    exit 1
fi

TEMP_BASHRC=$(mktemp)

# Backup .bashrc
cp "$BASHRC" "$BACKUP"

# Remove existing ROS source lines
grep -vE "source /opt/ros/.*/setup.bash" "$BASHRC" > "$TEMP_BASHRC"

# Append the selected ROS version if not legacy
if [ "$TARGET" == "noetic" ]; then
    echo "source /opt/ros/noetic/setup.bash" >> "$TEMP_BASHRC"
elif [ "$TARGET" == "humble" ]; then
    echo "source /opt/ros/humble/setup.bash" >> "$TEMP_BASHRC"
fi

# Replace the original .bashrc
mv "$TEMP_BASHRC" "$BASHRC"

# Output result
if [ "$TARGET" == "legacy" ]; then
    echo "Removed ROS environment setup from $BASHRC (legacy mode)."
else
    echo "Updated $BASHRC to use ROS $TARGET by default."
fi

echo "Backup saved as $BACKUP"
