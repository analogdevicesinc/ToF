# NXP Build Instructions

## Dependencies

Please note, the libs folder below must contain libtofi_compute.so and libtofi_config.so. This folder and these two files are on the SD card image that is a part of the eval kit.

These are the ARM64 depth compute libraries. 

```
base folder
├── libs
│   ├── libtofi_compute.so
│   └── libtofi_config.so
└── ToF
```

Other dependencies are already installed on the NXP.

## Building the SDK only

See [here](../../cmake/readme.md) for details on the cmake options.

### Pre-requisites
* CMake
* Depth Compute

### Installing the dependencies
* CMake:
```console
sudo apt install cmake
```

* Depth Compute:

The Depth Compute library is required to build the SDK and run any application based on the SDK.
Based on the following scenarios you will be able to do the following:

1. You have the **arm development package for nxp** (libtofi_compute.so, libtofi_config.so) for the Depth Compute library. With it you will be able to build the SDK. The files should be included in the SD card image (~\Workspace\libs). You just need to put the files mentioned above in a directory called 'libs' which should be placed at the same level as the repository directory.

    If you prefer to keep the files somewhere else, you can tell cmake where to find them using the cmake variable: LIBTOFI_LIBDIR_PATH

2. You have no Depth Compute packages. In this case you can still build the SDK by enabling the USE_DEPTH_COMPUTE_OPENSOURCE option but you won't have access to the functionality that Depth Compute provides.

### Download and build SDK only

Please note, ensure you are using the intended branch.

```console
git clone --branch v6.1.0  https://github.com/analogdevicesinc/ToF
cd ToF
git submodule update --init
mkdir build && cd build
cmake -DNXP=1 -DWITH_EXAMPLES=off -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DCMAKE_BUILD_TYPE=Release ..
make -j4
```

### SDK with examples

Please note, ensure you are using the intended branch.

```console
git clone --branch v6.1.0  https://github.com/analogdevicesinc/ToF
cd ToF
git submodule update --init
mkdir build && cd build
cmake -DNXP=1 -DWITH_EXAMPLES=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" -DCMAKE_BUILD_TYPE=Release ..
make -j4
```


