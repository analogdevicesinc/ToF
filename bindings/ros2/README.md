# ROS2 Wrapper for Time of Flight SDK of Analog Devices&copy; (for Ubuntu 20.04)

## ROS2 Installation

- Install the recommended [ROS2 distribution](https://docs.ros.org/en/rolling/Releases.html) for your operating system**
  - [ROS Install page](https://docs.ros.org/en/foxy/Installation.html)

- In order to prepare the system to run the ROS wrapper in the general catkin workspace make sure to install correctly the following libraries:

## Library prerequisits

* Building ToF SDK:
- **Install the ADI ToF SDK library**
  - [Install SDK dependencies](https://github.com/analogdevicesinc/ToF/blob/master/doc/itof/linux_build_instructions.md)
  - Download and build the SDK, as well as enable ROS package building and specify the path towards the ROS HOME directory
```console
git clone https://github.com/analogdevicesinc/ToF
cd ToF
mkdir build && cd build
cmake -DWITH_NETWORK=1 -DWITH_ROS2=on -DROS_HOME_DIR="/home/${USER}/.ros" -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
sudo cmake --build . --target install
```
- **Build the aditof_roscpp package**
```console
sudo cmake --build . --target tof_ros2_package
```

## Starting camera node
- In the general ROS2 workspace run the following code:
```console
    cd ros2_ws
    source ./install/setup.bash
    ros2 run tof_ros2cpp tof_camera_node ip="10.42.0.1" config_file="config/config_walden_3500_nxp.json" use_depthCompute="true" mode=1
```
### Parameters
 [config_file:"<<b>path></b>"]
* Crosby with Pulsatrix: "```config/config_walden_3500_nxp.json```"
* Crosby: "```config/config_crosby_nxp.json```"
* Walden: "```config/config_walden_nxp.json```"

 [use_depthCompute] 
 - "true" for enabling Depth Compute libraries
 - "false" for disabling Depth Compute libraries 

 [mode]:
* 1 -> QMP mode of the camera
* 2 -> MP mode of the camera

## Published topics
1. Depth Compute enabled
   * tof_camera/ir
   * tof_camera/depth
   * tof_camera/raw
2. Depth Compute disabled
   * tof_camera/raw