# Linux Build Instructions

**Please note, use the applicable tag, when cloning, and release version when getting the latest depth compute library files for the embedded system. As of writing, version 5.0.0 of the release is available as well as tag v5.0.0.**

## Building the SDK only

### Pre-requisites
* CMake

### Installing the dependencies
* CMake:
```console
sudo apt install cmake
```

### Download and build SDK only

Please note, ensure you are using the intended branch.

```console
git clone --branch v5.0.0 --depth 1 https://github.com/analogdevicesinc/ToF
cd ToF
git submodule update --init
mkdir build && cd build
cmake -DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

See [here](../../cmake/readme.md) for details on the cmake options.

## SDK with examples

### Additional pre-requisites
* OpenCV
* OpenGL

### Installing the additional dependencies
* OpenCV:
```console
sudo apt install libopencv-contrib-dev
sudo apt install libopencv-dev
```

* OpenGL:
```console
sudo apt install libgl1-mesa-dev libglfw3-dev
```

### Build SDK with examples

Please note, ensure you are using the intended branch.

```console
git clone --branch v5.0.0 --depth 1 https://github.com/analogdevicesinc/ToF
cd ToF
git submodule update --init
mkdir build && cd build
cmake -DWITH_EXAMPLES=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DCMAKE_BUILD_TYPE=Release ..
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
cmake -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets;/opt/opencv" -DWITH_DOC=on -DCMAKE_BUILD_TYPE=Release ..
make -j4 doc
```
After compilation, the documentation can be found at this path:
```console
build/doc/doxygen_doc/html/index.html
```
