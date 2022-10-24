
# ROS Wrapper for Time of Flight SDK of Analog Devices

## ROS Wrapper exists in an external repository for general package implementations: [External repository](https://github.com/rbudai98/tof_roscpp)

# 1. Install ROS

Install the recommended [ROS distribution](http://wiki.ros.org/Distributions) for your operating system**
  - [ROS Install page](http://wiki.ros.org/ROS/Installation)

# 2. ToF dependency

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

# 3. Usage

In directory ```catkin_ws/src/``` clone the repository:

```console
git clone https://github.com/rbudai98/tof_roscpp.git
```

After cloning the repository in the ``catkin_ws/ run the following command:
 
```console
catkin_make
source devel/setup.bash
```

### Starting the camera node 

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


### Published Topics
The aditof_camera_node publishes messages defined by the [sensor_msgs](http://wiki.ros.org/sensor_msgs) package on the following topics
- /aditof_roscpp/aditof_camera_info
- /aditof_roscpp/aditof_depth
- /aditof_roscpp/aditof_ir
- /aditof_roscpp/aditof_pcloud

### Update parameters at runtime using
Using the [dynamic_reconfigure](http://wiki.ros.org/dynamic_reconfigure) package, the aditof_ros_package offers the users the possibility to update the camera parameters
