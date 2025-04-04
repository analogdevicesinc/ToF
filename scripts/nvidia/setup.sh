#!/bin/bash

# List of required packages
# Update package list first
echo "Updating package list..."
sudo apt update

REQUIRED_PACKAGES=(
    upx
    jq
    cmake
    g++
    gcc
    libopencv-contrib-dev
    libopencv-dev
    libgl1-mesa-dev
    libglfw3-dev
    doxygen
    graphviz
    python3.10-dev
    python3-sphinx
)
# Check and install each package
for pkg in "${REQUIRED_PACKAGES[@]}"; do
    if dpkg -s "$pkg" &> /dev/null; then
        echo "$pkg is already installed ✅"
    else
        echo "$pkg is missing, installing... ⏳"
        sudo apt install -y "$pkg"
    fi
done
