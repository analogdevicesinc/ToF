# ROS2 Wrapper for Time of Flight SDK of Analog Devices

## ROS 2 Wrapper exists in an external repository for general package implementations: [External repository](https://github.com/rbudai98/tof_ros2cpp)

# 1. Install ROS2

- Install the recommended [ROS2 distribution](https://docs.ros.org/en/rolling/Releases.html) for your operating system**
  - [ROS Install page](https://docs.ros.org/en/foxy/Installation.html)

- In order to prepare the system to run the ROS wrapper in the general catkin workspace make sure to install correctly the following libraries:

# 2. ToF dependency


- [Install SDK dependencies](https://github.com/analogdevicesinc/ToF/blob/master/doc/itof/linux_build_instructions.md)

```console
git clone https://github.com/analogdevicesinc/ToF
cd ToF
mkdir build && cd build
```
* In case you have the Depth-Compute library for the SDK, use the following command:
```console
cmake -DWITH_NETWORK=1 -DWITH_ROS2=on -DROS_HOME_DIR="/home/${USER}/.ros" -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DUSE_DEPTH_COMPUTE_STUBS=0 ..
```
* Otherwise, if the Depth-Compute library is not added, plese use the following command:
```console
cmake -DWITH_NETWORK=1 -DWITH_ROS2=on -DROS_HOME_DIR="/home/${USER}/.ros" -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DUSE_DEPTH_COMPUTE_STUBS=1 ..
```

```console
sudo cmake --build . --target install
```
- **Build the aditof_roscpp package**
```console
sudo cmake --build . --target tof_ros2_package
```

# 3. Usage

In directory ```ros2_ws/src/``` clone the repository:

```console
git clone https://github.com/rbudai98/tof_ros2cpp.git
```

After cloning the repository in the ``ros2_ws/ run the following command:
 
```console
colcon build
source devel/setup.bash
```

### Starting camera node
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

