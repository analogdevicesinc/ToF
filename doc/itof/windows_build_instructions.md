# Windows Build Instructions


## Building the SDK only

### Pre-requisites
* Install MS Visual Studio 16 2019
* Install MS .NET Framework 4.5
* CMake
* Glog v0.3.5
* Libwebsockets v3.1
* Protocol Buffers v3.9.0
* Depth Compute

### Installing the dependencies
* CMake

Windows installer can be downloaded from: https://cmake.org/download/

* Glog:
```console
git clone --branch v0.3.5 --depth 1 https://github.com/google/glog
cd glog
mkdir build_0_3_5 && cd build_0_3_5
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=./local_path/glog -G "Visual Studio 16 2019" ..
cmake --build . --target install --config Debug
cmake --build . --target install --config Release
```

* Libwebsockets:
```console
git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_3_1 && cd build_3_1
cmake -DLWS_WITH_SSL=OFF -DCMAKE_INSTALL_PREFIX=./local_path/websockets -G "Visual Studio 16 2019" ..
cmake --build . --target install --config Debug
cmake --build . --target install --config Release
```

* Protobuf:
```console
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0 && cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -Dprotobuf_MSVC_STATIC_RUNTIME=OFF -DCMAKE_INSTALL_PREFIX=./local_path/protobuf -G "Visual Studio 16 2019" ../cmake
cmake --build . --target install --config Debug
cmake --build . --target install --config Release
```

* Depth Compute:

The Depth Compute library is required to build the SDK and run any application based on the SDK.
Based on the following scenarios you will be able to do the following:

1. You have the **development package** (tofi_compute.dll, tof_compute.lib, tofi_config.dll, tofi_config.lib) for the Depth Compute library. With it you will be able to build the SDK. You just need to put the files mentioned above in a directory called 'libs' which should be placed at the same level as the repository directory.

    If you prefer to keep the files somewhere else, you can tell cmake where to find them using the cmake variable: LIBTOFI_LIBDIR_PATH

2. You have the **distribution package** (tofi_compute.dll, tofi_config.dll) for the Depth Compute library. With it you won't be able to build the SDK unless you turn on the  USE_DEPTH_COMPUTE_STUBS cmake option. This will create an empty Depth Compute library allowing the build of SDK to complete. To use the real Depth Compute, just replace the stubs generated during build (tofi_compute.dll, tofi_config.dll) with the ones from the distribution package.

3. You have no Depth Compute packages. In this case you can still build the SDK by enabling the USE_DEPTH_COMPUTE_STUBS option but you won't have access to the functionality that Depth Compute provides.

### Download and build SDK only
* Follow below steps to download the SDK, generate MS Visual Studio project and build it directly from command line
```console
git clone https://github.com/analogdevicesinc/aditof-sdk-rework
cd aditof-sdk-rework
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="C:\projects\aditof-sdk\deps\glog\build_0_3_5\local_path\glog;C:\projects\aditof-sdk\deps\protobuf\build_3_9_0\local_path\protobuf;C:\projects\aditof-sdk\deps\libwebsockets\build_3_1\local_path\websockets" -G "Visual Studio 16 2019" -DWITH_EXAMPLES=off ..
cmake --build . --config Release
```

* Or if you will be building the Depth Compute stubs the cmake command should be:
```console
cmake -DCMAKE_PREFIX_PATH="C:\projects\aditof-sdk\deps\glog\build_0_3_5\local_path\glog;C:\projects\aditof-sdk\deps\protobuf\build_3_9_0\local_path\protobuf;C:\projects\aditof-sdk\deps\libwebsockets\build_3_1\local_path\websockets" -G "Visual Studio 16 2019" -DWITH_EXAMPLES=off -DUSE_DEPTH_COMPUTE_STUBS=on ..
```

## Building the SDK with examples

### Additional pre-requisites
* None

### Installing the additional dependencies
* None

### Build SDK with examples and in Visual Studio
- Generate the VisualStudio solution
```console
cd aditof-sdk-rework
mkdir build
cd build
cmake -DCMAKE_PREFIX_PATH="C:\projects\aditof-sdk\deps\glog\build_0_3_5\local_path\glog;C:\projects\aditof-sdk\deps\protobuf\build_3_9_0\local_path\protobuf;C:\projects\aditof-sdk\deps\libwebsockets\build_3_1\local_path\websockets" -G "Visual Studio 16 2019" -DWITH_EXAMPLES=on ..
```
- Open 'adi_tof_project.sln' generated in 'aditof-sdk-rework\build' in MS Visual Studio 2019
- Select 'Release' build
- Application binaries are created in 'aditof-sdk-rework\build\example\aditof-demo\Release' directory


## SDK with bindings

- Please check the readme files for each type of binding in the [bindings directory](../../bindings).


## Building the SDK in Visual Studio using the script

### Pre-requisites
* None

### Steps to build the SDK
- Run the script located in: 'aditof-sdk-rework\scripts\windows' with the suitable configuration parameters. <br>
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
