# Analog Devices 3D ToF software suite 

## Overview
The **ADI ToF SDK** is a cross platform library for the Analog Devices depth cameras based on the ADSD3100 TOF Signal Processor. It provides support for data processing on the embedded processor platform as well as for USB, Ethernet, Offline or Wi-Fi to connect to a host computer. This flexibility enables evaluation across a wide range of use cases and environments.

The SDK provides an API to control the ToF camera, AB stream and depth data. Windows support are built into the SDK as well as sample code and wrappers for various languages including Python and C/C++.

License : [![License](https://img.shields.io/badge/license-MIT-blue.svg)](LICENSE)
Platform details : [![Hardware](https://img.shields.io/badge/hardware-wiki-green.svg)]()

## Supported Host Platforms

For more details on building the SDK on a host platform please check the **User Guide** specified below:.

| Operating System | Documentation | GitHub main status | Downloads |
| --------- | ----------- | ----------- | ----------- |
| Windows | [Build Instructions](scripts/windows) | [![Build status](https://dev.azure.com/AnalogDevices/3DToF-rework/_apis/build/status/analogdevicesinc.ToF?branchName=main)](https://dev.azure.com/AnalogDevices/3DToF-rework/_build?view=runs&branchFilter=3310) | [![ToF evaluation kit installer](https://img.shields.io/badge/release-ToF_evaluation_kit_installer-blue.svg)](https://github.com/analogdevicesinc/ToF/releases/latest) |
| Linux | [Build Instructions](doc/itof/linux_build_instructions.md) | [![Build status](https://dev.azure.com/AnalogDevices/3DToF-rework/_apis/build/status/analogdevicesinc.ToF?branchName=main)](https://dev.azure.com/AnalogDevices/3DToF-rework/_build?view=runs&branchFilter=3310) | [![ToF evaluation kit installer](https://img.shields.io/badge/release-ToF_evaluation_kit_installer-blue.svg)](https://github.com/analogdevicesinc/ToF/releases/latest) |

### Requirements

Host Platform: 

* Windows 10 or
* Ubuntu 20.04 or
* Ubuntu 22.04

Other
* Python 3.10

## Supported Embedded Platforms

For more details on building the SDK on an embedded platform please check the **User Guide** specified below:
### ITOF camera
| Operating system | Evaluation boards | Documentation | GitHub main status |
| --------- | ----------- | ----------- | ----------- |
| NXP | [EVAL-ADTF3175D-NXZ](https://wiki.analog.com/resources/eval/user-guides/eval-adtf3175d-nxz) | [Build instructions](doc/itof/nxp_build_instructions.md) | [![Build status](https://dev.azure.com/AnalogDevices/3DToF-rework/_apis/build/status/analogdevicesinc.ToF?branchName=main)](https://dev.azure.com/AnalogDevices/3DToF-rework/_build?view=runs&branchFilter=3310) |

[How to write the SD card image onto the SD card?](doc/sdcard_burn.md)

## SDK Documentation

From an architectural point of view, the SDK consists of two layers. One layer is the high level API that allows clients to easily grab a camera object, configure it and request frames. The other layer is the low level API which exposes the interface through which low level operations can be made to the camera hardware.

For more details about the SDK check the links below:

[Building and installing the SDK](cmake/)

## SDK Examples
| Example | Language | Description |
| --------- | ------------- | ----------- |
| tof-viewer | <a href="examples/tof-viewer"> C++ </a> | Graphical User interface for visualising stream from depth camera |
| data-collect | <a href="examples/data_collect"> C++ </a> | A command line application that takes in command line input arguments (like number of frames, mode to be set, folder location to save frame data) and captures the frames and stores in path provided |
| first-frame | <a href="examples/first-frame"> C++ </a> <br> <a href="bindings/python/examples/first_frame"> Python </a> | An example code that shows the steps required to get to the point where camera frames can be captured. |
| low_level_example | <a href="bindings/python/examples/low_level_example"> Python</a> | A simple example of how to get access to the low-level API of the camera. |
| ROS2 CPP Wrapper | <a href="bindings/ros2/"> ROS2/C++</a> | ROS2 binding that publishes topics |

## Other Examples
| Example | Language | Description |
| --------- | ------------- | ----------- |
| ROS2 Application | <a href="https://github.com/analogdevicesinc/adi_3dtof_adtf31xx"> C++ </a> | A more extensive ROS2 example based on the ADI ToF SDK. |
| Stitching Algorithm | <a href="https://github.com/analogdevicesinc/adi_3dtof_image_stitching"> C++ </a> | A stiching algorithm using ADI ToF data. |

## Directory Structure
| Directory | Description |
| --------- | ----------- |
| apps | Applications specific to various targets and hosts |
| bindings | SDK bindings to other languages |
| ci | Useful scripts for continuous integration |
| cmake | Helper files for cmake |
| dependencies | Contains third-party and owned libraries |
| doc | Documentation |
| drivers | Holds drivers for nxp and nvidia |
| examples | Example code for the supported programming languages |
| scripts | Useful development scripts |
| sdcard-images-utils | Things required to build a SD card image for targets |
| libaditof | Submodule with SDK source code |
| tools | Standalone applications |

---
**Known issues**

On a PC where Linux kernel version 5.13 is installed, a USB connected camera cannot be detected. The v4l device name that we expect is not being set properly. However it works for older and newer versions. 

---
