
# ROS Wrapper for Time of Flight SDK of Analog Devices

## ROS Wrapper exists in an external repository for general package implementations: [External repository](https://github.com/analogdevicesinc/tof-ros)

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

For usage information please refere to ```section 3``` from: [ROS Usage](https://github.com/analogdevicesinc/tof-ros#3-usage)