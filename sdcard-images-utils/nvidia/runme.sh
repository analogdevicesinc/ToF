#!/bin/bash
set -ex

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
	make install -C kernel

	echo "Build out of tree kernel modules"
	make modules
	make modules_install

	echo "Build DTB"
	make dtbs
	cp -rf $BUILD_DIR/kernel-devicetree/generic-dts/dtbs/*.dtb  $BUILD_DIR/modules/dtb
	cp -rf $BUILD_DIR/kernel-devicetree/generic-dts/dtbs/*.dtbo $BUILD_DIR/modules/dtb

	echo "Copy kernel Image"
	cp $ROOTDIR/build/Linux_for_Tegra/source/modules/boot/Image $PATCH_DIR/

	echo "Copy DTB"
	cp -r $ROOTDIR/build/Linux_for_Tegra/source/modules/dtb   $PATCH_DIR

	echo "Copy kernel modules"
	cd $ROOTDIR/build/Linux_for_Tegra/source/modules
	tar --owner root --group root -cjf $BUILD_DIR/kernel_supplements.tbz2 lib/modules
	cd -
	mv $BUILD_DIR/kernel_supplements.tbz2 $PATCH_DIR
	popd
}

function copy_ubuntu_overlay()
{
	cp -rf $ROOTDIR/patches/ubuntu_overlay $PATCH_DIR

}

function sw_version_info()
{
	touch sw-versions
	SW_VERSION_FILE=sw-versions
	echo -n "SDK    Version : " >> $SW_VERSION_FILE ; echo "$SDK_VERSION" >> $SW_VERSION_FILE
	echo -n "Branch Name    : " >> $SW_VERSION_FILE ; echo "$BRANCH" >> $SW_VERSION_FILE
	echo -n "Branch Commit  : " >> $SW_VERSION_FILE ; echo "$BR_COMMIT" >> $SW_VERSION_FILE
	echo -n "Build  Date    : " >> $SW_VERSION_FILE ; date >> $SW_VERSION_FILE
	echo    "Kernel Version : 5.15.148-tegra" >> $SW_VERSION_FILE
	mv $SW_VERSION_FILE $PATCH_DIR
}

function clone_sdk()
{
	pushd .
	cd $PATCH_DIR
	git clone --branch $BRANCH https://github.com/analogdevicesinc/ToF.git
	cd ToF
	git submodule update --init --recursive
	popd
}

function main()
{
	configure_toolchain
	download_bsp_source
	download_linux_kernel
	apply_git_format_patches
	build_kernel_Image
	copy_ubuntu_overlay
	clone_sdk
	sw_version_info
	cp $ROOTDIR/scripts/system_upgrade/apply_patch.sh $PATCH_DIR
	cd $ROOTDIR/build
	zip -r "NVIDIA_ToF_ADSD3500_REL_PATCH_$(date +"%d%b%y").zip" NVIDIA_ToF_ADSD3500_REL_PATCH_*
	mv *.zip $ROOTDIR
	rm -rf $PATCH_DIR
	echo "System image patch "NVIDIA_ToF_ADSD3500_REL_PATCH_$(date +"%d%b%y").zip" file created successfully"
}

main
