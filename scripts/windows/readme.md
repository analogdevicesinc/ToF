## Overview

Build scripts to be ran on Windows.

**setup_project.bat** is a script that downloads and installs the dependencies and builds the sdk.
By default the sdk will be built in folder: **current_script_path\build** using the **Visual Studio 16 2019** generator and the **Release** configuration.
The dependencies (glog, protobuf and websockets) will be downloaded and installed in **current_script_path\deps\installed**.
Please use option -h or --help for further reference on how to change the defaults.

## Pre-requisites

* CMake (Windows installer can be downloaded from: https://cmake.org/download/)

## Usage
Open Windows PowerShell then cd to the root folder of the repository. For example:
```console
cd C:\Users\user\ToF
```
Get the help menu for more details on how to use the script:
```console
.\scripts\windows\setup_project.bat --help
```
Run the script with default parameters:
```console
.\scripts\windows\setup_project.bat
```

## Manual build
Alternatively, the SDK and/or its dependencies can be manually built. Instructions can be found [here](doc/itof/windows_build_instructions.md).
