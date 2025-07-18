#!/bin/bash

# Set locale to UTF-8
sudo apt update && sudo apt install locales -y
sudo locale-gen en_US en_US.UTF-8
sudo update-locale LC_ALL=en_US.UTF-8 LANG=en_US.UTF-8
export LANG=en_US.UTF-8

# Verify locale settings
locale

# Enable universe repository
sudo apt install software-properties-common -y
sudo add-apt-repository universe

# Install curl and add ROS 2 GPG key
sudo apt update && sudo apt install curl -y
sudo curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key -o /usr/share/keyrings/ros-archive-keyring.gpg

# Add ROS 2 repository
echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo $UBUNTU_CODENAME) main" | sudo tee /etc/apt/sources.list.d/ros2.list > /dev/null

# Install Python and ROS development tools
sudo apt update && sudo apt install -y \
  python3-flake8-docstrings \
  python3-pip \
  python3-pytest-cov \
  ros-dev-tools

# Install Python packages for linting and testing
python3 -m pip install -U \
   flake8-blind-except \
   flake8-builtins \
   flake8-class-newline \
   flake8-comprehensions \
   flake8-deprecated \
   flake8-import-order \
   flake8-quotes \
   "pytest>=5.3" \
   pytest-repeat \
   pytest-rerunfailures

# Install PCL and OpenCV libraries
sudo apt-get install -y libboost-all-dev libpcl1 python3-pcl pcl-tools python3-opencv
sudo apt install libpcl-dev -y

# Install additional ROS dependencies from file
if [ -f ros_dependencies.txt ]; then
    sudo apt-get install -y $(cat ros_dependencies.txt)
else
    echo "ros_dependencies.txt not found."
fi
