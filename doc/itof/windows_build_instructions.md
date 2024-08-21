# Windows Build Instructions

## Building the SDK only

### Pre-requisites
* Install MS Visual Studio 16 2019
* Install MS .NET Framework 4.5
* CMake

### Installing the dependencies
* CMake

Windows installer can be downloaded from: https://cmake.org/download/

## Building the SDK with examples

See [here](../../cmake/readme.md) for details on the cmake options.

### Additional pre-requisites
* None

### Installing the additional dependencies
* None

### Build SDK with examples and in Visual Studio
- Generate the VisualStudio solution
```console
git clone --branch v5.0.0 https://github.com/analogdevicesinc/ToF
cd ToF
git submodule update --init
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="C:\projects\ToF\deps\glog\build_0_6_0\local_path\glog;C:\projects\ToF\deps\protobuf\build_3_9_0\local_path\protobuf;C:\projects\ToF\deps\libwebsockets\build_3_1\local_path\websockets" -G "Visual Studio 16 2019" -DWITH_EXAMPLES=on ..
```
- Open 'adi_tof_project.sln' generated in 'ToF\build' in MS Visual Studio 2019
- Select 'Release' build
- Application binaries are created in 'ToF\build\example\aditof-demo\Release' directory


## SDK with bindings

- Please check the readme files for each type of binding in the [bindings directory](../../bindings).


## Building the SDK in Visual Studio using the script

### Pre-requisites
* None

### Steps to build the SDK
- Run the script located in: 'ToF\scripts\windows' with the suitable configuration parameters. <br>
The following example runs the script with generator **Visual Studio 16 2019** and configuration **Release**. The sdk will be built in folder: **current_script_path\build** and the dependencies will be installed in folder: **current_script_path\deps\installed**.
Use parameter *-h* or *--help* for more information regarding the parameters. 
```
.\setup_project.bat -g "Visual Studio 16 2019" -c Release
```
- Open 'adi_tof_project.sln' generated in **current_script_path\build** in MS Visual Studio.

- Select the configuration to match the configuration used in the script. (e.g.,**Release**) for build.
![Display Image](/doc/img/configuration_VS.PNG)

- The sdk Dynamic-link library (aditof.dll) is created in: **current_script_path\build\sdk\Release**.

- Build the application.
![Display Image](/doc/img/build_VS.PNG)

## Generate doxygen documentation

Requirements:
* Doxygen
* Graphviz

In order to generate the doxygen documentation you must compile the sdk in the following way:
```console
cmake -DCMAKE_PREFIX_PATH="C:\projects\ToF\deps\glog\build_0_6_0\local_path\glog;C:\projects\ToF\deps\protobuf\build_3_9_0\local_path\protobuf;C:\projects\ToF\deps\libwebsockets\build_3_1\local_path\websockets;C:\projects\ToF\opencv\build" -G "Visual Studio 16 2019" -DOPENSSL_INCLUDE_DIRS="C:\OpenSSL-Win64\include" -DWITH_DOC=on ..
cmake --build . --target doc -j 4
```
After compilation, the documentation can be found at this path:
```console
build\doc\doxygen_doc\html\index.html
```
