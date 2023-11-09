# Linux Build Instructions

**Please note, use the applicable tag, when cloning, and release version, when getting the latest depth compute library files. As of writing, version 4.3.0 of the release is available as well as tag v4.3.0.** 

If you building for a ARM64 Linux host you will need the ARM64 libraries. These are available in the installation package. See

Note, all actities below are assuming to be done in a base folder. Where the **libs**, and its contents, folder is called out.

```
base folder
├── glog
├── libs
│   ├── libtofi_compute.so
│   └── libtofi_config.so
├── libwebsockets
├── protobuf
└── ToF
```

You can use the evaluation folder as a reference by over writing the files with the executables and binaries built.

The following variables must be exported for execution of the examples:
```
export LD_LIBRARY_PATH=.:$LD_LIBRARY_PATH
# Only if the OpenCL depth compute libraries are being used.
export OverrideDefaultFP64Settings=1
export IGC_EnableDPEmulation=1 
```

## Building the SDK only

See [here](../../cmake/readme.md) for details on the cmake options.

Download the latest Linux installer - as of writing v4.4.0.
https://github.com/analogdevicesinc/ToF/releases

* mkdir libs
* cd libs
* cp ~/Analog\ Devices/TOF_Evaluation_Ubuntu_ADTF3175D-Rel4.3.0/bin_22.04/libaditofi\* .

### Pre-requisites
* CMake
* OpenGL
* Glog v0.6.0
* Libwebsockets v3.1
* Protocol Buffers v3.9.0

### Installing the dependencies
* CMake:
```console
sudo apt install cmake
```

* OpenGL:
```console
sudo apt install libgl1-mesa-dev libglfw3-dev
```

* Glog:
```console
pushd .
git clone --branch v0.6.0 --depth 1 https://github.com/google/glog
cd glog
mkdir build_0_6_0 && cd build_0_6_0
cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
sudo cmake --build . --target install
popd
```

* Libwebsockets:
```console
pushd .
git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets
cd libwebsockets
mkdir build_3_1 && cd build_3_1
cmake -DLWS_WITH_SSL=OFF -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets ..
sudo cmake --build . --target install
popd
```

* protobuf:
```console
pushd .
git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
cd protobuf
mkdir build_3_9_0 && cd build_3_9_0
cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake
sudo cmake --build . --target install
cd ../..
popd
```


### Download and build SDK only

Please note, ensure you are using the intended branch.

```console
git clone --branch v4.4.0 --depth 1 https://github.com/analogdevicesinc/ToF
cd ToF
mkdir build && cd build
cmake -DWITH_EXAMPLES=off -DWITH_NETWORK=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
make -j4
```

## SDK with examples

### Additional pre-requisites
* OpenCV

### Installing the additional dependencies
* OpenCV:
```console
sudo apt install libopencv-contrib-dev
sudo apt install libopencv-dev
```

### Build SDK with examples

Please note, ensure you are using the intended branch.

```console
git clone --branch v4.4.0 --depth 1 https://github.com/analogdevicesinc/ToF
cd ToF
mkdir build && cd build
cmake -DWITH_EXAMPLES=on -DWITH_NETWORK=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..
make -j4
```

## SDK with bindings

- Please check the readme files for each type of binding in the [bindings directory](../../bindings).

## Generate doxygen documentation

Requirements:
* Doxygen
* Graphviz

```console
sudo apt-get install doxygen graphviz
```

In order to generate the doxygen documentation you must compile the sdk in the following way:
```console
cmake -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets;/opt/opencv" -DWITH_DOC=on ..
make -j4 doc
```
After compilation, the documentation can be found at this path:
```console
build/doc/doxygen_doc/html/index.html
```
