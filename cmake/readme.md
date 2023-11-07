# 3D Time of Flight : cmake

#### Overview
This folder contains helper cmake.in files for the project and also config files for other cmake projects to use in order to find the SDK.

#### Using the CMake build system
In order to build the SDK with cmake, we first need to have all the dependencies installed: glog, protobuf, libwebsockets ( [Check instructions for your system](https://github.com/analogdevicesinc/ToF/tree/master/doc) ).

After creating a build folder and moving into it `mkdir -p build && cd build`, we can run cmake.

`cmake -D<option> <path_to_ToF>`

which will generate all the necessary recipes for building and installing. Useful cmake options are:

| \<option\> | value | default | description |
| --------- | ----------- | ----------- | ----------- |
| WITH_EXAMPLES | on/off | on | Build the examples. |
| WITH_DOC | on/off | off | Build the doxygen documentation. |
| WITH_PYTHON | on/off | off | Build the python bindings. |
| WITH_OPENCV | on/off | off | Build the opencv bindings. |
| WITH_OPEN3D | on/off | off | Build the open3D bindings. |
| WITH_ROS | on/off | off | Build the ROS bindings. |
| WITH_ROS2 | on/off | off | Build the ROS2 bindings. |
| WITH_NETWORK | on/off | off | Build the network interface for a Linux or Windows host build; use **off** for building on the target. |
| USE_DEPTH_COMPUTE_STUBS | on/off | off | Use depth compute stubs instead of the depth compute libraries. On the target set to **on**. On the host set tof **off**. |
| USE_DEPTH_COMPUTE_OPENSOURCE | on/off | off | When **on** the open source radial to XYZ (point cloud) generation is used. Otherwise the closed source partial depth compute library is required. |
| USE_DEPTH_COMPUTE_ON_TARGET | on/off | on | When on depth compute is performed on the target. This is specific to the eval kit. | 
| WITH_GLOG_DEPENDENCY | on/off | on | Build the sdk with GLOG dependency. Used for logging. |
| WITH_PROTOBUF_DEPENDENCY | on/off | on | Build the sdk with Protobuf dependency. Used for serializing the messages sent over network. |
| WITH_COMMAND_LINE_TOOLS | on/off | on | Build the command line tools. |
| CMAKE_PREFIX_PATH | \<path\> | Empty | Specifies a path which will be used by the FIND_XXX() commands. |
| CMAKE_INSTALL_PREFIX | \<path\> |  /usr/local on UNIX, c:/Program Files on Windows | Installation directory used by `cmake install`. |
| PYTHON_EXECUTABLE | \<path\> | Path to default python executable used | Specify which python executable should be used for building the python bindings. |


#### Building and Installing 

To build the sdk the following command is used:

`cmake --build . [--config <config>] [--target <target>]`

Where `<config>` is the build type: `Debug, Release ...` and target is one of the following:

| \<target\> | description |
| --------- | ----------- |
| install | Install the SDK in the system |
| doc | Build the doxygen documentation |
| copy-dll-bindings | Copy the necessary dll files in the bindings build folder (Only on Windows) |

Example: Consider a user that has the dependencies for the project installed in specific folders in `/opt`: `/opt/glog`, `/opt/protobuf`, `/opt/websockets`, and that wants to install the SDK in `/opt/aditof`, with examples on and all the possible bindings enabled. The following set of commands will do:
```
cd ToF
mkdir build && cd build
cmake -DWITH_PYTHON=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DCMAKE_INSTALL_PREFIX="/opt/aditof" ..
sudo cmake --build . --target install
```

The sdk can also be built with the network interface
```
cmake -DWITH_NETWORK=on ..
```
When building with this option, Websockets is required as a dependency of the sdk

After installing you should run `ldconfig` to update the links/cache that the dynamic loader uses.

#### Using the SDK with other CMake projects
To use the SDK in your own project, simply add this two lines to your projects CMakeLists.txt and your good to go:
```
find_package(aditof 1.0.0 REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE aditof::aditof)
```

Instead of `aditof::aditof` you could use `${aditof_LIBRARIES}`:
```
find_package(aditof 1.0.0 REQUIRED)
target_link_libraries(${PROJECT_NAME} PRIVATE ${aditof_LIBRARIES})
``` 

The prerequisite for this to work is to have the SDK installed in the system. If the path for the dependencies of the SDK, or even the SDK is not in the default `PATH` you might need to add a `CMAKE_PREFIX_PATH` that points to the dependencies/sdk when running the cmake command for your project.

Example: Consider a user that has the dependencies for the project installed in specific folders in `/opt`: `/opt/glog`, `/opt/protobuf`, `/opt/websockets` and the SDK in `/opt/aditof` and wants to use the SDK with another project. In the CMakeLists.txt of the project the two lines from above are required to be added and the cmake command should specify

```
-DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets;/opt/aditof"
```

#### Additional notes on the CMake options

##### WITH_PYTHON = ON

Install Python such that the python executable can be accessed from the command-line interface. Python 3.7 is required.

Tested with Python 3.7.9 64-bit.

##### WITH_DOC = ON

Install doxygen, ensure the doxygen exectuable is in the PATH.

https://www.doxygen.nl/download.html#srcbin 

Tested with doxygen 1.9.4.

Install GraphViz, ensure the GraphViz executable to the PATH.

https://graphviz.org/download/

Tested with GraphViz 4.0.0.

##### WITH_OPENCV = ON

Download and install OpenCV 4.6.0 from to https://opencv.org/releases/. 

It is important the OpenCV_DIR environment variable be set:
* For a permananet addition for the current and future shells (note close and reopen shell): **setx OpenCV_DIR C:\dev\opencv\build\x64\vc15\lib**
* For a temporary addition for the current shell: **set OpenCV_DIR=C:\dev\opencv\build\x64\vc15\lib**

##### WITH_OPEN3D = ON

Download and install open3d-devel-windows-amd64-0.15.1.zip from https://github.com/isl-org/Open3D/releases.

It is important the Open3D_DIR environment variable be set:
* For a permananet addition for the current and future shells (note close and reopen shell): **setx Open3D_DIR C:\dev\open3d\CMake**
* For a temporary addition for the current shell: **set Open3D_DIR=C:\dev\open3d\CMake**

##### WITH_ROS = ON

See [here](../bindings/ros/README.md) for more details.

##### WITH_ROS2 = ON

See [here](../bindings/ros2/README.md) for more details.

##### USE_DEPTH_COMPUTE_STUBS = ON

Building with the Depth Compute Libraries

If USE_DEPTH_COMPUTE_STUBS is set to OFF in CMakeLists.txt the depth compute libraries (.lib and .dll) files must be provided for the build. The depth compute libraries are included in the evaluation platform release. From the ADTF3175x installation path, navigate to the folder **depth_compute_installer**, then execute the TOF_Depth_CompteEngine_Windows installer. From here on we will assume the depth compute installer is TOF_DepthComputeEngine_Windows-Rel3.0.0.exe.

Assuming the installation path for the depth compute installation was not changed, navigate to *C:\Analog Devices\TOF_DepthComputeEngine_Windows-Rel3.0.0\lib*.

One more assumption, you have cloned the ToF repo into a folder called *ToF*.

On the same level as the folder *ToF*, create a folder called libs. Copy the files from *C:\Analog Devices\TOF_DepthComputeEngine_Windows-Rel3.0.0\lib* into *libs*.

Where you should now have the following.

```console
C:.
+---libs
|       tofi_compute.dll
|       tofi_compute.lib
|       tofi_config.dll
|       tofi_config.lib
|       tofi_processor.obj
|
\---ToF
    |   .clang-format
    |   .clangformatignore
    |   .gitignore
    |   appveyor.yml
    |   CMakeLists.txt
    |   ...
```
