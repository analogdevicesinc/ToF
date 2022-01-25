
# ROS Wrapper for the ADI ToF library

## Overview
This ROS package facilitates DEPTH and IR data acquisition and processing for the Analog Devices depth cameras.

## On Target
  
1.1 **Install the recommended [ROS distribution](http://wiki.ros.org/Distributions) for your operating system**
  - [ROS Install page](http://wiki.ros.org/ROS/Installation)
  - On target choose: [ROS Melodic Distro](http://wiki.ros.org/melodic/Installation/Ubuntu) and install the ROS-Base (Bare Bones) package
**Install the ADI ToF SDK library**
  - [Install SDK dependencies](https://github.com/analogdevicesinc/aditof-sdk-rework/blob/6c7fb376aeec73a21ab177adf297c5781bcbd544/doc/linux/build_instructions.md#installing-the-dependencies)

  1.2 Download the SDK from the repository \
  1.3 Copy the following files to the specific location after installing the ROS package:
  
  ```console
  sudo cp <libs folder direcory>/libtofi_config.so $ROS_ROOT/../../lib/.
  sudo cp <libs folder direcory>/libtofi_compute.so $ROS_ROOT/../../lib/.
  sudo cp -r <config folder directory> $HOME/.ros/
  ```
  where "libs folder direcory" is the location of the folder where the depth_compute files are stored and the "config folder directory" is the directory where the neccessary config files are strored for the camera. \


  1.4 After this, build the SDK, as well as enable ROS package building 

```console
git clone https://github.com/analogdevicesinc/aditof-sdk-rework
cd aditof-sdk-rework
mkdir build && cd build
cmake -DNXP=1 -DWITH_ROS=1 -DUSE_ITOF=1 -DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" .. 
sudo cmake --build . --target install
sudo cmake --build . --target aditof_ros_package
```
1.5 In order to start the camera node, please use the following example:
```console
cd catkin_ws
source devel/setup.bash
export ROS_MASTER_URI=http://<TARGET_IP>:11311
export ROS_IP=<TARGET_IP>
roslaunch aditof_roscpp camera_node.launch
```
where the TARGET_IP is the IP address of target device.\

In order to enable the DEPHT_COMPUTE library use the following command line to start the ROS node (by default, without specified argument the depth_compute library is disabled):
```console
roslaunch aditof_roscpp camera_node.launch use_depth_library:="true"

```

## On Host
 2.1 **Install the recommended [ROS distribution](http://wiki.ros.org/Distributions) for your operating system**
  - [ROS Install page](http://wiki.ros.org/ROS/Installation)

 2.2 After installing run the following commands:
 ```console
cd catkin_ws
source devel/setup.bash
export ROS_MASTER_URI=http://<TARGET_IP>:11311
export ROS_IP=<HOST_IP>
```
where the TARGET_IP is the IP address of the device on which is running the camera node and the HOST_IP is the IP address of the users computer where you want to access the topics.


## Published Topics
The aditof_camera_node publishes messages defined by the [sensor_msgs](http://wiki.ros.org/sensor_msgs) package on the following topics
- /aditof_roscpp/aditof_depth 
- /aditof_roscpp/aditof_ir

