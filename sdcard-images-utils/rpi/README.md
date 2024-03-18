# Analog Devices's Raspberry Pi 4b CMOS TOF build scripts

## Introduction
Main intention of this scripts set is to build environment for Raspberry Pi 4b (64bit) based TOF product evaluation.

The build script provides ready to use images that can be deployed on microSD.

The buildsystem is capable to create one target:

  1. Linux Kernel image based ADI Linux Kernel. (Intented for development)

     [ADI Linux Kernel Branch]: https://github.com/analogdevicesinc/linux/tree/rpi-5.10.y

## Dependencies

Dependencies that must be installed: Please do an **sudo apt-get update** first.

* Bison: **sudo apt install bison**
* Flex: **sudo apt install flex**
* gcc aarch64: **sudo apt install gcc-aarch64-linux-gnu**
* libssl-dev: **sudo apt install libssl-dev**
* debootstrap: **sudo apt install debootstrap**
* zlib dev: **sudo apt install zlib1g-dev**
* device tree compiler: **sudo apt install device-tree-compiler**
* bc: **sudo apt install bc**

## Build with host tools
Simply running ./create_package.sh.