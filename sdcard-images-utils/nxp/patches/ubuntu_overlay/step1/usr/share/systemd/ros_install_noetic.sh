#!/bin/bash -eu

# The BSD License
# Copyright (c) 2020 Qbotics Labs Pvt Ltd
# Copyright (c) 2014 OROCA and ROS Korea Users Group

#set -x

name_ros_distro=noetic 
user_name=analog
echo "#######################################################################################################################"
echo ""
echo ">>> {Starting ROS Noetic Installation}"
echo ""
echo ">>> {Checking your Ubuntu version} "
echo ""
#Getting version and release number of Ubuntu
version=`lsb_release -sc`
relesenum=`grep DISTRIB_DESCRIPTION /etc/*-release | awk -F 'Ubuntu ' '{print $2}' | awk -F ' LTS' '{print $1}'`
echo ">>> {Your Ubuntu version is: [Ubuntu $version $relesenum]}"
#Checking version is focal, if yes proceed othervice quit
case $version in
  "focal" )
  ;;
  *)
    echo ">>> {ERROR: This script will only work on Ubuntu Focal (20.04).}"
    exit 0
esac

echo ""
echo ">>> {Ubuntu Focal 20.04 is fully compatible with Ubuntu Focal 20.04}"
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 1: Configure your Ubuntu repositories}"
echo ""
#Configure your Ubuntu repositories to allow "restricted," "universe," and "multiverse." You can follow the Ubuntu guide for instructions on doing this. 
#https://help.ubuntu.com/community/Repositories/Ubuntu

sudo add-apt-repository universe
sudo add-apt-repository restricted
sudo add-apt-repository multiverse

sudo apt update

echo ""
echo ">>> {Done: Added Ubuntu repositories}"
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 2: Setup your sources.list}"
echo ""

#This will add the ROS Noetic package list to sources.list 
sudo sh -c "echo \"deb http://packages.ros.org/ros/ubuntu ${version} main\" > /etc/apt/sources.list.d/ros-latest.list"

#Checking file added or not
if [ ! -e /etc/apt/sources.list.d/ros-latest.list ]; then
  echo ">>> {Error: Unable to add sources.list, exiting}"
  exit 0
fi

echo ">>> {Done: Added sources.list}"
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 3: Set up your keys}"
echo ""
echo ">>> {Installing curl for adding keys}"
#Installing curl: Curl instead of the apt-key command, which can be helpful if you are behind a proxy server: 
#TODO:Checking package is not working sometimes, so disabling it
#Checking curl is installed or not
#name=curl
#which $name > /dev/null 2>&1

#if [ $? == 0 ]; then
#    echo "Curl is already installed!"
#else
#    echo "Curl is not installed,Installing Curl"

sudo apt install -y curl
#fi

echo "#######################################################################################################################"
echo ""
#Adding keys
echo ">>> {Waiting for adding keys, it will take few seconds}"
echo ""
ret=$(curl -sSL 'http://keyserver.ubuntu.com/pks/lookup?op=get&search=0xC1CF6E31E6BADE8868B172B4F42ED6FBAB17C654' | sudo apt-key add -)

#Checking return value is OK
case $ret in
  "OK" )
  ;;
  *)
    echo ">>> {ERROR: Unable to add ROS keys}"
    exit 0
esac

echo ">>> {Done: Added Keys}"
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 4: Updating Ubuntu package index, this will take few minutes depend on your network connection}"
echo ""
sudo apt update
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 5: Install ROS, you pick how much of ROS you would like to install.}"
echo "     [1. Desktop-Full Install: (Recommended) : Everything in Desktop plus 2D/3D simulators and 2D/3D perception packages ]"
echo ""
echo "     [2. Desktop Install: Everything in ROS-Base plus tools like rqt and rviz]"
echo ""
echo "     [3. ROS-Base: (Bare Bones) ROS packaging, build, and communication libraries. No GUI tools.]"
echo ""
#Assigning default value as 1: Desktop full install
read -p "Enter your install (Default is 1):" answer 

case "$answer" in
  1)
    package_type="desktop-full"
    ;;
  2)
    package_type="desktop"
    ;;
  3)
    package_type="ros-base"
    ;;
  * )
    package_type="desktop-full"
    ;;
esac
echo "#######################################################################################################################"
echo ""
echo ">>>  {Starting ROS installation, this will take about 20 min. It will depends on your internet  connection}"
echo ""
sudo apt-get install -y ros-${name_ros_distro}-${package_type} 
echo ""
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 6: Setting ROS Environment, This will add ROS environment to .bashrc.}" 
echo ">>> { After adding this, you can able to access ROS commands in terminal}"
echo ""
echo "" >> /home/$user_name/.bashrc
echo "source /opt/ros/noetic/setup.bash" >> /home/$user_name/.bashrc
source /home/$user_name/.bashrc
sudo apt install -y python3-rosdep python3-rosinstall python3-rosinstall-generator python3-wstool build-essential
sudo rosdep init
rosdep update
echo ""
echo "#######################################################################################################################"
echo ">>> {Step 7: Testing ROS installation, checking ROS version.}"
echo ""
echo ">>> {Type [ rosversion -d ] to get the current ROS installed version}"
echo ""
echo "#######################################################################################################################"

