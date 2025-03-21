#!/bin/bash
set -ex

if [[ $EUID > 0 ]]; then
	echo "This script must be run as root user"
	echo 'usage: ./create_patch_image.sh'
        exit 1

fi

if [ "$#" -ne 2 ]; then
        echo 'usage: ./create_patch_image.sh <sdk_version> <tof_branch_name>'
        exit 1
fi


### Command Line args

SDK_VERSION=$1
BRANCH=$2

echo ${SDK_VERSION}
echo ${BRANCH}

if [ -z ${SDK_VERSION} ]; then
        echo 'usage: ./create_patch_image.sh <sdk_version> <tof_branch_name>'
        exit 1
fi

if [ -z ${BRANCH} ]; then
        echo 'usage: ./create_patch_image.sh <sdk_version> <tof_branch_name>'
        exit 1
fi


ROOTDIR=`pwd`
BR_COMMIT=`git log -1 --pretty=format:%h`

PATCH_DIR=$ROOTDIR/build/NVIDIA_ToF_ADSD3500_REL_PATCH_$(date +"%d%b%y")
mkdir -p $PATCH_DIR

function apply_git_format_patches()
{
	COMPONENTS="nv-public kernel-jammy-src nvidia-oot"
	cd $ROOTDIR/build/Linux_for_Tegra
	for i in $COMPONENTS; do
		if [ "x$i" == "xnv-public" ]; then

			echo "Applying nv-public directory\n\n"
			pushd .
			cd $ROOTDIR/build/Linux_for_Tegra/source/hardware/nvidia/t23x/nv-public/
			git reset --hard origin/l4t/l4t-r36.4.2
			git am $ROOTDIR/patches/hardware/nvidia/t23x/nv-public/*.patch
			popd

		elif [ "x$i" == "xkernel-jammy-src" ]; then
			echo "Applying kernel-jammy-src directory patches\n\n"
			pushd .
			cd $ROOTDIR/build/Linux_for_Tegra/source/kernel/kernel-jammy-src/
			git reset --hard jetson_36.4.3
			git am $ROOTDIR/patches/kernel/kernel-jammy-src/*.patch
			popd

		elif [ "x$i" == "xnvidia-oot" ]; then
			echo "Applying nvidia/drivers directory patches\n\n"
			pushd .
			cd $ROOTDIR/build/Linux_for_Tegra/source/nvidia-oot/drivers/
			git reset --hard origin/l4t/l4t-r36.4.2
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

	rm -rf modules
	make clean
	cd $KERNEL_HEADERS
	make distclean
	cd $BUILD_DIR

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
	cp $ROOTDIR/build/Linux_for_Tegra/source/modules/boot/Image $PATCH_DIR/Image.fgv2

	echo "Copy DTB"
	cp -r $ROOTDIR/build/Linux_for_Tegra/source/modules/dtb/* $ROOTDIR/build/Linux_for_Tegra/rootfs/boot/
	cp -r $ROOTDIR/build/Linux_for_Tegra/source/modules/dtb   $PATCH_DIR

	echo "Copy kernel modules"
	cp -r $ROOTDIR/build/Linux_for_Tegra/source/modules/lib/modules/5.15.148-tegra $ROOTDIR/build/Linux_for_Tegra/rootfs/lib/modules
	cd $ROOTDIR/build/Linux_for_Tegra/source/modules
	tar --owner root --group root -cjf $BUILD_DIR/kernel_supplements.tbz2 lib/modules
	cd -
	mv $BUILD_DIR/kernel_supplements.tbz2 $PATCH_DIR
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

function sw_version_info()
{
	rm -rf $ROOTDIR/build/Linux_for_Tegra/rootfs/boot/sw-versions
	touch $ROOTDIR/build/Linux_for_Tegra/rootfs/boot/sw-versions
	SW_VERSION_FILE=$ROOTDIR/build/Linux_for_Tegra/rootfs/boot/sw-versions
	echo -n "SDK    Version : " >> $SW_VERSION_FILE ; echo "$SDK_VERSION" >> $SW_VERSION_FILE
	echo -n "Branch Name    : " >> $SW_VERSION_FILE ; echo "$BRANCH" >> $SW_VERSION_FILE
	echo -n "Branch Commit  : " >> $SW_VERSION_FILE ; echo "$BR_COMMIT" >> $SW_VERSION_FILE
	echo -n "Build  Date    : " >> $SW_VERSION_FILE ; date >> $SW_VERSION_FILE
	echo    "Kernel Version : 5.15.148-tegra" >> $SW_VERSION_FILE
	echo -n "Tegra  Release : " >> $SW_VERSION_FILE ; cat $ROOTDIR/build/Linux_for_Tegra/rootfs/etc/nv_tegra_release >> $SW_VERSION_FILE
	cp $SW_VERSION_FILE $PATCH_DIR
}

function clone_sdk()
{
	pushd .
	cd $PATCH_DIR
	sudo git clone --branch $BRANCH git@github.com:adi-innersource/tofi-fgv2.git
	cd $ROOTDIR/build/Linux_for_Tegra/rootfs/usr/local/Workspace/tofi-fgv2/
	sudo git checkout main
	sudo git pull
	popd
}

function main()
{
	echo "Creating the system image patch file"
	apply_git_format_patches
	build_kernel_Image
	apply_ubuntu_overlay
	clone_sdk
	sw_version_info
	cp $ROOTDIR/scripts/system_upgrade/apply_overlay.sh $PATCH_DIR
	cd $ROOTDIR/build
	zip -r "NVIDIA_ToF_ADSD3500_REL_PATCH_$(date +"%d%b%y").zip" NVIDIA_ToF_ADSD3500_REL_PATCH_*
	mv *.zip $ROOTDIR
	rm -rf $PATCH_DIR
	echo "System image patch "NVIDIA_ToF_ADSD3500_REL_PATCH_$(date +"%d%b%y").zip" file created successfully"
}

main
