# ROS2 Wrapper for Time of Flight SDK of Analog Devices&copy; (for Ubuntu 20.04)

## ROS2 Installation

- Install the recommended [ROS2 distribution](https://docs.ros.org/en/rolling/Releases.html) for your operating system**
  - [ROS Install page](https://docs.ros.org/en/foxy/Installation.html)

- In order to prepare the system to run the ROS wrapper in the general catkin workspace make sure to install correctly the following libraries:

## Library prerequisits

* ToF libraries:
```console
wget https://swdownloads.analog.com/cse/aditof/tof_deb_pkg/tof_roscpp.deb
sudo dpkg -i tof_roscpp.deb
```

- After preparing the neccessary libraries open the general ```ros_ws``` workspace and in the ```src```  and run 
```console
git clone https://github.com/rbudai98/tof_ros2cpp.git
```
and in the main ros2_ws directory run the: ```colcon build``` commad.
- <b>NOTE: </b>The config files from the cloned repository must be copied by the Cmake of the project into the root of the ros2_ws directory.


## Starting camera node
- In the general ROS2 workspace run the following code:
```console
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

