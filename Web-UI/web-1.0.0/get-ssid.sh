#!/bin/bash

# Path to the wpa_supplicant configuration file
config_file="/etc/wpa_supplicant/wpa_supplicant-wlan0.conf"

# Extract the ssid value
ssid=$(grep 'ssid=' "$config_file" | cut -d '"' -f 2)

# Check if ssid was found
if [ -z "$ssid" ]; then
    echo "SSID not found in the configuration file."
    exit 1
fi

# Use the ssid as the username
username="$ssid"

# Print the username
echo "$username"