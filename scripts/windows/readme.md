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


## Understanding the CMake Options

See [here](../../cmake/readme.md) for more details.
