# Overview

## Build scripts to be ran on Windows.

**setup_project.bat** is a script that downloads and installs the dependencies and builds the sdk.
By default the sdk will be built in folder: **current_script_path\build** using the **Visual Studio 16 2019** generator and the **Release** configuration.
The dependencies (glog, protobuf and websockets) will be downloaded and installed in **current_script_path\deps\installed**.
Please use option -h or --help for further reference on how to change the defaults.

## Pre-requisites

* CMake (Windows installer can be downloaded from: https://cmake.org/download/)

## Usage
Open Windows PowerShell then cd to the location of the script. For example:
```console
cd C:\Users\user\ToF\scripts\windows
```
Get the help menu for more details on how to use the script:
```console
.\setup_project.bat --help
```
Run the script with default parameters:
```console
.\setup_project.bat
```

Run the script but specify different custom settings. Also, by skipping the '-y' parameter, the script will give a short summary of what actions will do and ask permision to continue or to stop.
```console
.\setup_project.bat -b myBuild -d myDeps -i myInstalledDeps -g "Visual Studio 16 2019" -c Release
```

Once the build is finished, navitage to the newly created **build** folder.

In the build folder open **adi_tof_project.sln" with the flavour of Visual Studio specified via **setup_project.bat**.

## Manual build
Alternatively, the SDK and/or its dependencies can be manually built. Instructions can be found [here](../../doc/itof/windows_build_instructions.md).


# Understanding the CMake Options

* WITH_EXAMPLES: Build the examples.
* WITH_DOC: Builds documentation generated with doxygen. doxygen must be in the PATH.
* WITH_PYTHON: Builds the Python bindings. The Python 3.7 executable must be in the PATH.
* WITH_OPENCV: Builds the OpenCV C++ bindings.
* WITH_OPEN3D: Builds the Open3D C++ bindings.
* WITH_ROS: Builds the ROS1 C++ bindings.
* WITH_NETWORK: Enables network access to the device when set to **ON**, USB UVC when set to **OFF**.
* WITH_ITOF: Do NOT change, leave **ON**.
* USE_DEPTH_COMPUTE_STUBS: Used when building with stubs in place of the depth compute libraries.

## Additional notes on the CMake options

### WITH_PYTHON = ON

Install Python such that the python executable can be accessed from the command-line interface. Python 3.7 is required.

Tested with Python 3.7.9 64-bit.

### WITH_DOC = ON

Install doxygen, ensure the doxygen exectuable is in the PATH.

https://www.doxygen.nl/download.html#srcbin 

Tested with doxygen 1.9.4.

Install GraphViz, ensure the GraphViz executable to the PATH.

https://graphviz.org/download/

Tested with GraphViz 4.0.0.

### WITH_OPENCV = ON

Download and install OpenCV 4.6.0 from to https://opencv.org/releases/. 

It is important the OpenCV_DIR environment variable be set:
* For a permananet addition for the current and future shells (note close and reopen shell): **setx OpenCV_DIR C:\dev\opencv\build\x64\vc15\lib**
* For a temporary addition for the current shell: **set OpenCV_DIR=C:\dev\opencv\build\x64\vc15\lib**

### WITH_OPEN3D = ON

Download and install open3d-devel-windows-amd64-0.15.1.zip from https://github.com/isl-org/Open3D/releases.

It is important the Open3D_DIR environment variable be set:
* For a permananet addition for the current and future shells (note close and reopen shell): **setx Open3D_DIR C:\dev\open3d\CMake**
* For a temporary addition for the current shell: **set Open3D_DIR=C:\dev\open3d\CMake**

### WITH_ROS = ON

See [here](../../bindings/ros/README.md) for more details.

### USE_DEPTH_COMPUTE_STUBS = ON

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
