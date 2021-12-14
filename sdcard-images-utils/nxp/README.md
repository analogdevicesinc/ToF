# Analog Devices's i.MX8MP CMOS TOF build scripts

## Introduction
Main intention of this scripts set is to build environment for i.MX8MP based TOF product evaluation.

The build script provides ready to use images that can be deployed on microSD or eMMC.

The buildsystem is capable to create two different targets:
		1. Buildroot based initramfs (Intented for production)
		2. Ubuntu based ROOTFS (Intented for development)

Some BASH environment variables supported to change the building behavior.
User can specify local mirror location to speed up the fetch of packages and git sources.

* DISTRO_MIRROR is used to specify the mirror location of ports.ubuntu.com.
* NXP_GIT_LOCATION is used to specify the root location of git repositories for NXP firmware, kernel, bootloader and tools.
* TOF_GIT_LOCATION is used to specify the location of git repository of TOF SDK.

## Build with host tools
Simply running ./runme.sh, it will check for required tools, clone and build images and place results in images/ directory.
The selection of target is performed using "BUILD_TYPE" variable from runme.sh script. Valid options are: ubuntu | buildroot

## Deploying
For SD card bootable images, plug in a microSD and run the following, where sdX is the location of the SD card got mounted on your machine -

`sudo dd if=images/microsd-<hash>.img of=/dev/sdX`

Take care S3 boot switches to be configured accordingly.

## SD card creator
To write a image into a sdcard, run

`create_sd.sh images/microsd-xxxxxxx.img /dev/sdX`

The package hdparm is required to notify the partition change to the kernel. Please install it with your package management system.
