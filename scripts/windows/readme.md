## Overview

Build scripts to be ran on Windows.

By default the sdk will be built in folder: **current_script_path\build** using the **Visual Studio 15 2017 Win64** generator and the **Release** configuration.
The dependencies (glog, protobuf and websockets) will be downloaded and installed in **current_script_path\deps\installed**.
Please use option -h or --help for further reference on how to change the defaults.

## Pre-requisites

* CMake (Windows installer can be downloaded from: https://cmake.org/download/)
