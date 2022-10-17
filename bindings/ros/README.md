
# ROS Wrapper for the ADI ToF library

## Overview
This ROS package facilitates depth and IR data acquisition and processing for the Analog Devices depth cameras.

## Installation

- **Install the recommended [ROS distribution](http://wiki.ros.org/Distributions) for your operating system**
  - [ROS Install page](http://wiki.ros.org/ROS/Installation)
- **Make sure you have these ROS packages installed before building and running the examples**
  - [rviz](http://wiki.ros.org/rviz)\
    Run the command below for each package, replacing ROSDISTRO with the name of the ROS distribution you are using and PACKAGE with the name of the needed package.
  ```console
  sudo apt install ros-ROSDISTRO-PACKAGE
  ```
- **Install the ADI ToF SDK library**
  - [Install SDK dependencies](https://github.com/analogdevicesinc/ToF/blob/master/doc/itof/linux_build_instructions.md)
  - Download and build the SDK, as well as enable ROS package building and specify the path towards the ROS HOME directory
```console
git clone https://github.com/analogdevicesinc/ToF
cd ToF
mkdir build && cd build
cmake -DWITH_NETWORK=1 -DWITH_ROS=on -DROS_HOME_DIR="/home/${USER}/.ros" -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
sudo cmake --build . --target install
```
 - **Build the aditof_roscpp package**
  ```console
  sudo cmake --build . --target aditof_ros_package
```




## Usage
### Starting camera node

Before starting the camera node please run the following code-snippet: 
```console
    cd catkin_ws
    source devel/setup.bash
```


|                                 |          |   |
|---------------------------------|----------|---|
| With RQT Dynamic reconfigure    | USB      |`roslaunch aditof_roscpp camera_node_gui.launch config_file:="<path>"`   |
|                                 | Ethernet |`roslaunch aditof_roscpp camera_node_gui.launch ip:="10.42.0.1" config_file:="<path>"`   |
| Without RQT Dynamic reconfigure | USB      |`roslaunch aditof_roscpp camera_node.launch config_file:="<path>" mode:="1" use_depthCompute:="false"`   |
|                                 | Ethernet |`roslaunch aditof_roscpp camera_node.launch ip:="10.42.0.1" config_file:="<path>  mode:="1" use_depthCompute:="false"`   |


### Parameters:
 [config_file:"<<b>path></b>"]
* Crosby with Pulsatrix: "```config/config_walden_3500_nxp.json```"
* Crosby: "```config/config_crosby_nxp.json```"
* Walden: "```config/config_walden_nxp.json```"

 [use_depthCompute] 
 - `true` for enabling Depth Compute libraries
 - `false` for disabling Depth Compute libraries 

 [mode]:
* `1` -> QMP mode of the camera
* `2` -> MP mode of the camera

###  Dynamic reconfigure window:

    
 <p align="center"><img src="../../doc/img/ros_dynamic_reconfigure.png" /></p>
 

### Examples
  - Visualize point cloud in rviz
    ```console
    cd catkin_ws
    source devel/setup.bash
    roslaunch aditof_roscpp rviz_publisher.launch
    ```


## Published Topics
The aditof_camera_node publishes messages defined by the [sensor_msgs](http://wiki.ros.org/sensor_msgs) package on the following topics
- /aditof_roscpp/aditof_camera_info
- /aditof_roscpp/aditof_depth
- /aditof_roscpp/aditof_ir
- /aditof_roscpp/aditof_pcloud

## Update parameters at runtime using
Using the [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure) package, the aditof_ros_package offers the users the possibility to update the camera parameters

