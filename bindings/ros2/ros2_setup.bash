#!/bin/bash

DISTRO_CODENAME=$( (lsb_release -sc || . /etc/os-release; echo ${VERSION_CODENAME} ) 2>/dev/null)

#determine ROS distribution
if [ $DISTRO_CODENAME = "bionic" ]
then
    ROS_DISTRO="crystal"
    #     echo crystal
elif [ $DISTRO_CODENAME = "focal" ]
then
    ROS_DISTRO="foxy"
    #     echo foxy
elif [ $DISTRO_CODENAME = "buster" ]
then
    ROS_DISTRO="humble"
    #     echo humble
elif [ $DISTRO_CODENAME = "jammy" ]
then
    ROS_DISTRO="humble"
    #     echo humble
fi

mkdir -p ${COLCON_WS}/src
cd ${COLCON_WS}/src
git clone https://github.com/analogdevicesinc/tof-ros2.git

cd ${COLCON_WS}
source /opt/ros/$ROS_DISTRO/setup.sh

colcon build --cmake-args -DADITOF_CMAKE_INSTALL_PREFIX=$1 -DADITOF_CMAKE_PREFIX_PATH=$2 -DLIBTOFI_LIBDIR_PATH=$3