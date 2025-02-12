# Linux Host Build Instructions

**Please note, use the applicable tag, when cloning, and release version when getting the latest depth compute library files for the embedded system. As of writing, version 6.0.0 of the release is available as well as tag v6.0.0.**

## Building the SDK only

### Pre-requisites
* CMake
* g++
* Python 3 - note, we are assuming Python 3.8 in this document, change as needed for your setup
* OpenCV - for the examples
* OpenGL - for the examples
* Doxygen - for documentation generation
* Graphviz - for documentation generation

#### Installing the pre-requisites
```console
sudo apt update
sudo apt install cmake g++ \
     libopencv-contrib-dev libopencv-dev \
     libgl1-mesa-dev libglfw3-dev \
     doxygen graphviz
```

For Linux builds install the necessary version of Python dev libraries. For example for Ubuntu 24.04 with Python 3.12 as the default Python:
```console
sudo apt install python3.12-dev
```

### Building the SDK with Example

Please note, ensure you are using the intended branch.

See [here](../../cmake/readme.md) for details on the cmake options.

Choose the branch as needed. In our example below we are using the branch/tag v6.0.0:
* --branch v6.0.0

To build:
* the examples add the CMake option: -DWITH_EXAMPLES=on
* the documentation add the CMake optionL -DWITH_DOC=on 

```console
git clone --branch v6.0.0 https://github.com/analogdevicesinc/ToF
cd ToF
git submodule update --init --recursive
mkdir build && cd build
cmake -DCMAKE_BUILD_TYPE=Release ..
make -j
```

Note, the documentation can be found at this path:
```console
build/doc/doxygen_doc/html/index.html
```
