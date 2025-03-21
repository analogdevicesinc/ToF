#!/usr/bin/env bash

set -ex

if [[ $EUID > 0 ]]; then
	echo "This script must be run as root user"
	echo 'usage: ./runme.sh <tof_branch_name>'
        exit 1

fi

if [ "$#" -ne 1 ]; then
	echo 'usage: ./runme.sh <tof_branch_name>'
        exit 1
fi


### Command Line args
BRANCH=$1
SDK_VERSION="0.0.0" # NOTE: The 'apply_patch.sh' script in the patch would update the correct software version. TODO: Remove SDK Version from here.

echo ${BRANCH}

if [ -z ${BRANCH} ]; then
	echo 'usage: ./runme.sh <tof_branch_name>'
	exit 1
fi

ROOTDIR=`pwd`
BR_COMMIT=`git log -1 --pretty=format:%h`

function configure_toolchain()
{
	sudo apt-get update
	sudo apt install -y bison flex python3-sphinx gcc-aarch64-linux-gnu libssl-dev qemu qemu-user-static debootstrap zlib1g-dev device-tree-compiler bc
	mkdir -p build
	cd $ROOTDIR/build
	echo "Download the toolchain"
	wget https://developer.download.nvidia.com/embedded/L4T/bootlin/aarch64--glibc--stable-final.tar.gz
	cd $ROOTDIR/build
	mkdir -p aarch64--glibc--stable-final
	echo "Extract the toolchain"
	tar -xvf aarch64--glibc--stable-final.tar.gz -C aarch64--glibc--stable-final
	rm -rf aarch64--glibc--stable-final.tar.gz
}


function download_bsp_source()
{
	echo "Download and Extract the NVIDIA Jetson Linux 36.4.3 BSP Driver package"
	wget -q -O- https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/release/Jetson_Linux_R36.4.3_aarch64.tbz2 | tar xj
	echo "Clone the NVIDIA L4T linux kernel sources"
	cd Linux_for_Tegra/
	pushd .
}

function download_sample_rootfs()
{
	echo "Download and Extract the sample root file system"
	cd rootfs
	wget -q -O- https://developer.nvidia.com/downloads/embedded/l4t/r36_release_v4.3/release/Tegra_Linux_Sample-Root-Filesystem_R36.4.3_aarch64.tbz2 | tar xj
	popd

}

function install_flash_prerequisites()
{
	echo "Install the additional packages required for flashing"
	sudo ./apply_binaries.sh
	sudo ./tools/l4t_flash_prerequisites.sh
	pushd .

}

function download_linux_kernel()
{
	echo "Clone the NVIDIA L4T_36_4_3 linux kernel sources"
	cd source
	./source_sync.sh -t jetson_36.4.3
	popd
	echo "Build environmental setup Completed for Jetpack 6.2 !!!"
}

function apply_git_format_patches()
{
        COMPONENTS="nv-public kernel-jammy-src nvidia-oot"
		cd $ROOTDIR/build/Linux_for_Tegra
                for i in $COMPONENTS; do
                        if [ "x$i" == "xnv-public" ]; then

                                echo "Applying nv-public directory\n\n"
                                pushd .
                                cd $ROOTDIR/build/Linux_for_Tegra/source/hardware/nvidia/t23x/nv-public/
                                git am $ROOTDIR/patches/hardware/nvidia/t23x/nv-public/*.patch
                                popd

                        elif [ "x$i" == "xkernel-jammy-src" ]; then
                                echo "Applying kernel-jammy-src directory patches\n\n"
                                pushd .
                                cd $ROOTDIR/build/Linux_for_Tegra/source/kernel/kernel-jammy-src/
                                git am $ROOTDIR/patches/kernel/kernel-jammy-src/*.patch
                                popd

                        elif [ "x$i" == "xnvidia-oot" ]; then
                                echo "Applying nvidia/drivers directory patches\n\n"
                                pushd .
                                cd $ROOTDIR/build/Linux_for_Tegra/source/nvidia-oot/drivers/
                                git am $ROOTDIR/patches/nvidia-oot/drivers/*.patch
                                popd

                        else
                                echo "Invalid component"
                        fi
                done
}

function build_kernel_Image()
{
	echo "Build Linux kernel Image, modules and dts\n"

	pushd .
	BUILD_DIR=$ROOTDIR/build/Linux_for_Tegra/source
	cd $BUILD_DIR

	export INSTALL_MOD_PATH=$BUILD_DIR/modules
	export KERNEL_HEADERS=$BUILD_DIR/kernel/kernel-jammy-src
	export CROSS_COMPILE=$ROOTDIR/build/aarch64--glibc--stable-final/bin/aarch64-linux-

	echo "Creating Output Directories"
	mkdir -p $BUILD_DIR/modules/boot
	mkdir -p $BUILD_DIR/modules/dtb

	echo "Build kernel"
	make -C kernel

	echo "Build in tree kernel modules"
	sudo -E make install -C kernel

	echo "Build out of tree kernel modules"
	make modules
	sudo -E make modules_install

	echo "Build DTB"
	make dtbs
	cp -rf $BUILD_DIR/kernel-devicetree/generic-dts/dtbs/*.dtb  $BUILD_DIR/modules/dtb
	cp -rf $BUILD_DIR/kernel-devicetree/generic-dts/dtbs/*.dtbo $BUILD_DIR/modules/dtb

	echo "Copy kernel Image"
	cp $ROOTDIR/build/Linux_for_Tegra/source/modules/boot/Image $ROOTDIR/build/Linux_for_Tegra/rootfs/boot/Image.fgv2

	echo "Copy DTB"
	cp -r $ROOTDIR/build/Linux_for_Tegra/source/modules/dtb/* $ROOTDIR/build/Linux_for_Tegra/kernel/dtb/
	cp -r $ROOTDIR/build/Linux_for_Tegra/source/modules/dtb/* $ROOTDIR/build/Linux_for_Tegra/rootfs/boot/

	echo "Copy kernel modules"
	cp -r $ROOTDIR/build/Linux_for_Tegra/source/modules/lib/modules/5.15.148-tegra $ROOTDIR/build/Linux_for_Tegra/rootfs/lib/modules
}

function apply_ubuntu_overlay()
{
	echo "Updating dhcpd.conf"
        cp $ROOTDIR/patches/ubuntu_overlay/opt/nvidia/l4t-usb-device-mode/dhcpd.conf $ROOTDIR/build/Linux_for_Tegra/rootfs/opt/nvidia/l4t-usb-device-mode/

	 # Set the MTU size to 15000 by default
        echo "Configuring the MTU size to 15000 by default"
        cp $ROOTDIR/patches/ubuntu_overlay/etc/NetworkManager/conf.d/10-ignore-interface.conf $ROOTDIR/build/Linux_for_Tegra/rootfs/etc/NetworkManager/conf.d/
        cp $ROOTDIR/patches/ubuntu_overlay/etc/systemd/network/10-rndis0.network $ROOTDIR/build/Linux_for_Tegra/rootfs/etc/systemd/network/

	echo "Updating the usb device mode configuration"
        cp $ROOTDIR/patches/ubuntu_overlay/opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode-*.sh  $ROOTDIR/build/Linux_for_Tegra/rootfs/opt/nvidia/l4t-usb-device-mode/

	echo "Copy the network gadget service file"
	cp $ROOTDIR/patches/ubuntu_overlay/etc/systemd/system/network-gadget.service $ROOTDIR/build/Linux_for_Tegra/rootfs/etc/systemd/system

	echo "Remove the default dtb overlay file"
	sed -i 's/,tegra234-p3768-0000+p3767-0000-dynamic.dtbo/tegra234-p3767-camera-p3768-adsd3500.dtbo/g' $ROOTDIR/build/Linux_for_Tegra/p3768-0000-p3767-0000-a0.conf
}

function clone_sdk_dependencies()
{
        echo "Clone the ADITOF Target SDK dependencies"
        pushd .
        cd $ROOTDIR/build/Linux_for_Tegra/rootfs/usr/local/
        mkdir Workspace && cd Workspace
        git clone --branch v0.6.0 --depth 1 https://github.com/google/glog
        git clone --branch v3.1-stable --depth 1 https://github.com/warmcat/libwebsockets
        git clone --branch v3.9.0 --depth 1 https://github.com/protocolbuffers/protobuf
        git clone --branch $BRANCH https://github.com/analogdevicesinc/ToF.git
        popd
}

function sw_version_info()
{
       touch $ROOTDIR/build/Linux_for_Tegra/rootfs/boot/sw-versions
       SW_VERSION_FILE=$ROOTDIR/build/Linux_for_Tegra/rootfs/boot/sw-versions
       echo -n "SDK    Version : " >> $SW_VERSION_FILE ; echo "$SDK_VERSION" >> $SW_VERSION_FILE
       echo -n "Branch Name    : " >> $SW_VERSION_FILE ; echo "$BRANCH" >> $SW_VERSION_FILE
       echo -n "Branch Commit  : " >> $SW_VERSION_FILE ; echo "$BR_COMMIT" >> $SW_VERSION_FILE
       echo -n "Build  Date    : " >> $SW_VERSION_FILE ; date >> $SW_VERSION_FILE
       echo    "Kernel Version : 5.15.148-tegra" >> $SW_VERSION_FILE
       echo -n "Tegra  Release : " >> $SW_VERSION_FILE ; cat $ROOTDIR/build/Linux_for_Tegra/rootfs/etc/nv_tegra_release >> $SW_VERSION_FILE
}

function main()
{
	configure_toolchain
	download_bsp_source
	download_sample_rootfs
	install_flash_prerequisites
	download_linux_kernel
	apply_git_format_patches
	build_kernel_Image
	apply_ubuntu_overlay
	clone_sdk_dependencies
	sw_version_info
}

main
