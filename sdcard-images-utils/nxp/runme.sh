#!/bin/bash
set -ex

### Command Line args

SDK_VERSION=$1
BRANCH=$2
KERNEL_VERSION=$3

echo ${SDK_VERSION}
echo ${BRANCH}
echo ${KERNEL_VERSION}

if [ -z ${SDK_VERSION} ]; then
	echo 'usage: ./runme.sh <sdk_version> <ToF_Branch> <kernel_version>'
	exit 1
fi

if [ -z ${BRANCH} ]; then
	echo 'usage: ./runme.sh <sdk_version> <ToF_Branch> <kernel_version>'
	exit 1
fi

if [ -z ${KERNEL_VERSION} ]; then
        echo 'usage: ./runme.sh <sdk_version> <ToF_Branch> <kernel_version>'
	echo 'Available Kernel Versions are 5.10.72 and 5.15.71'
        exit 1
fi

### General setup
NXP_REL=rel_imx_5.4.70_2.3.0

if [ "${KERNEL_VERSION}" == "5.10.72" ]; then
	KERNEL_NXP_REL=lf-5.10.72-2.2.0
	KERNEL_ID=kernel_5_10_72
elif [ "${KERNEL_VERSION}" == "5.15.71" ]; then
	KERNEL_NXP_REL=lf-5.15.71-2.2.2
	KERNEL_ID=kernel_5_15_71
else
	KERNEL_NXP_REL=lf-5.10.72-2.2.0
	KERNEL_ID=kernel_5_10_72
fi

#rel_imx_5.4.70_2.3.0
UBOOT_NXP_REL=imx_v2020.04_5.4.70_2.3.0
#rel_imx_5.4.24_2.1.0
#imx_v2020.04_5.4.24_2.1.0
BUILDROOT_VERSION=2020.11.2
BUILD_TYPE="ubuntu" # valid options: ubuntu | buildroot
###
SHALLOW=${SHALLOW:false}
if [ "x$SHALLOW" == "xtrue" ]; then
        SHALLOW_FLAG="--depth 500"
fi

REPO_PREFIX=`git log -1 --pretty=format:%h`
NXP_GIT_LOCATION=${NXP_GIT_LOCATION:=https://github.com/nxp-imx}
export ARCH=arm64
export CROSS_COMPILE=aarch64-linux-gnu-
ROOTDIR=`pwd`

COMPONENTS="imx-atf uboot-imx linux-imx imx-mkimage"
mkdir -p build
for i in $COMPONENTS; do
	if [[ ! -d $ROOTDIR/build/$i ]]; then
		cd $ROOTDIR/build/
		if [ "x$i" == "xuboot-imx" ]; then
			CHECKOUT=$UBOOT_NXP_REL
		elif [ "x$i" == "xlinux-imx" ]; then
			CHECKOUT=$KERNEL_NXP_REL
		else
			CHECKOUT=$NXP_REL
		fi
		git clone ${SHALLOW_FLAG} ${NXP_GIT_LOCATION}/$i -b $CHECKOUT
		cd $i
		if [[ -d $ROOTDIR/patches/$i/ && "x$i" == "xlinux-imx" ]]; then
                        git am $ROOTDIR/patches/$i/${KERNEL_ID}/*.patch
		elif [[ -d $ROOTDIR/patches/$i/ ]]; then
			git am $ROOTDIR/patches/$i/*.patch
		fi
	fi
done

if [[ ! -d $ROOTDIR/build/firmware ]]; then
	cd $ROOTDIR/build/
	mkdir -p firmware
	cd firmware
	wget https://www.nxp.com/lgfiles/NMG/MAD/YOCTO/firmware-imx-8.10.bin
	bash firmware-imx-8.10.bin --auto-accept
	cp -v $(find . | awk '/train|hdmi_imx8|dp_imx8/' ORS=" ") ${ROOTDIR}/build/imx-mkimage/iMX8M/
fi

# Build linux
echo "*** Building Linux kernel"
cd $ROOTDIR/build/linux-imx
cp $ROOTDIR/configs/adi_imx_v8_defconfig arch/arm64/configs/
make adi_imx_v8_defconfig
make -j32 Image dtbs modules
if [[ -d modules ]]; then
	rm -R modules
fi
mkdir modules
make modules_install INSTALL_MOD_PATH=./modules
tar cvf modules.tar -C modules .
rm -R modules

# Build ROOTFS
if [[ $BUILD_TYPE == "buildroot" ]]; then
	if [[ ! -d $ROOTDIR/build/buildroot ]]; then
		cd $ROOTDIR/build
		git clone ${SHALLOW_FLAG} https://github.com/buildroot/buildroot -b $BUILDROOT_VERSION
	fi

	# Build buildroot
	echo "*** Building buildroot"
	cd $ROOTDIR/build/buildroot
	cp $ROOTDIR/configs/buildroot_defconfig configs/imx8mp_hummingboard_pulse_defconfig
	make imx8mp_hummingboard_pulse_defconfig
	make

	export CROSS_COMPILE=$ROOTDIR/build/buildroot/output/host/bin/aarch64-linux-
	ROOTFS_IMAGE=$ROOTDIR/build/buildroot/output/images/rootfs.ext4
elif [[ $BUILD_TYPE == "ubuntu" ]]; then
	#Build ubuntu rootfs
	echo "*** Building ubuntu rootFS"
	mkdir -p $ROOTDIR/build/ubuntu
	bash $ROOTDIR/deboot.sh $BRANCH
	ROOTFS_IMAGE=$ROOTDIR/build/ubuntu/rootfs.ext4
fi

# Build ATF
echo "*** Building ATF"
cd $ROOTDIR/build/imx-atf
make -j32 PLAT=imx8mp bl31
cp build/imx8mp/release/bl31.bin $ROOTDIR/build/imx-mkimage/iMX8M/

# Build u-boot
echo "*** Building u-boot"
cd $ROOTDIR/build/uboot-imx/
make imx8mp_solidrun_defconfig
make -j 32
set +e
cp -v $(find . | awk '/u-boot-spl.bin$|u-boot.bin$|u-boot-nodtb.bin$|.*\.dtb$|mkimage$/' ORS=" ") ${ROOTDIR}/build/imx-mkimage/iMX8M/
cp tools/mkimage ${ROOTDIR}/build//imx-mkimage/iMX8M/mkimage_uboot
set -e

# Bring bootlader all together
echo "*** Creating boot loader image"
unset ARCH CROSS_COMPILE
cd $ROOTDIR/build/imx-mkimage/iMX8M
sed "s/\(^dtbs = \).*/\1imx8mp-solidrun.dtb/;s/\(mkimage\)_uboot/\1/" soc.mak > Makefile
make clean
make flash_evk SOC=iMX8MP

# Create disk images
echo "*** Creating disk images"
mkdir -p $ROOTDIR/images/tmp/
cd $ROOTDIR/images
dd if=/dev/zero of=tmp/part1.fat32 bs=1M count=148
env PATH="$PATH:/sbin:/usr/sbin" mkdosfs tmp/part1.fat32

IMG=microsd-${REPO_PREFIX}.img
if [ -n $SDK_VERSION ]; then
	IMG=microsd-${SDK_VERSION}-${REPO_PREFIX}.img
fi

echo "sd_img_ver	$IMG" > $ROOTDIR/images/sw-versions
echo "kernel		$KERNEL_NXP_REL" >> $ROOTDIR/images/sw-versions
echo "u-boot		$UBOOT_NXP_REL" >> $ROOTDIR/images/sw-versions

echo "MENU TITLE ADITOF boot options" > $ROOTDIR/images/extlinux.conf
echo "TIMEOUT 10" >> $ROOTDIR/images/extlinux.conf
echo "DEFAULT ADSD3500+ADSD3100" >> $ROOTDIR/images/extlinux.conf
echo "label ADSD3100" >> $ROOTDIR/images/extlinux.conf
echo "        linux ../Image" >> $ROOTDIR/images/extlinux.conf
echo "        fdt ../imx8mp-adi-tof-noreg.dtb" >> $ROOTDIR/images/extlinux.conf
if [[ $BUILD_TYPE == "buildroot" ]]; then
	echo "        initrd ../rootfs.cpio.uboot" >> $ROOTDIR/images/extlinux.conf
else
	echo "        append root=/dev/mmcblk1p2 rootwait ro init=/usr/lib/init_resize.sh" >> $ROOTDIR/images/extlinux.conf
fi
echo "label ADSD3500+ADSD3100" >> $ROOTDIR/images/extlinux.conf
echo "        linux ../Image" >> $ROOTDIR/images/extlinux.conf
echo "        fdt ../imx8mp-adi-tof-adsd3500.dtb" >> $ROOTDIR/images/extlinux.conf
if [[ $BUILD_TYPE == "buildroot" ]]; then
	echo "        initrd ../rootfs.cpio.uboot" >> $ROOTDIR/images/extlinux.conf
else
	echo "        append root=/dev/mmcblk1p2 rootwait ro init=/usr/lib/init_resize.sh" >> $ROOTDIR/images/extlinux.conf
fi
echo "label ADSD3500+ADSD3030" >> $ROOTDIR/images/extlinux.conf
echo "        linux ../Image" >> $ROOTDIR/images/extlinux.conf
echo "        fdt ../imx8mp-adi-tof-adsd3030.dtb" >> $ROOTDIR/images/extlinux.conf
if [[ $BUILD_TYPE == "buildroot" ]]; then
	echo "        initrd ../rootfs.cpio.uboot" >> $ROOTDIR/images/extlinux.conf
else
	echo "        append root=/dev/mmcblk1p2 rootwait ro init=/usr/lib/init_resize.sh" >> $ROOTDIR/images/extlinux.conf
fi
echo "label ADSD3500-SPI+ADSD3100" >> $ROOTDIR/images/extlinux.conf
echo "        linux ../Image" >> $ROOTDIR/images/extlinux.conf
echo "        fdt ../imx8mp-adi-tof-adsd3500-spi.dtb" >> $ROOTDIR/images/extlinux.conf
if [[ $BUILD_TYPE == "buildroot" ]]; then
	echo "        initrd ../rootfs.cpio.uboot" >> $ROOTDIR/images/extlinux.conf
else
	echo "        append root=/dev/mmcblk1p2 rootwait ro init=/usr/lib/init_resize.sh" >> $ROOTDIR/images/extlinux.conf
fi
echo "label ADSD3500-SPI+ADSD3030" >> $ROOTDIR/images/extlinux.conf
echo "        linux ../Image" >> $ROOTDIR/images/extlinux.conf
echo "        fdt ../imx8mp-adi-tof-adsd3030-spi.dtb" >> $ROOTDIR/images/extlinux.conf
if [[ $BUILD_TYPE == "buildroot" ]]; then
	echo "        initrd ../rootfs.cpio.uboot" >> $ROOTDIR/images/extlinux.conf
else
	echo "        append root=/dev/mmcblk1p2 rootwait ro init=/usr/lib/init_resize.sh" >> $ROOTDIR/images/extlinux.conf
fi


mmd -i tmp/part1.fat32 ::/extlinux
mcopy -i tmp/part1.fat32 $ROOTDIR/images/sw-versions ::/sw-versions
mcopy -i tmp/part1.fat32 $ROOTDIR/images/extlinux.conf ::/extlinux/extlinux.conf
mcopy -i tmp/part1.fat32 $ROOTDIR/build/linux-imx/arch/arm64/boot/Image ::/Image
mcopy -s -i tmp/part1.fat32 $ROOTDIR/build/linux-imx/arch/arm64/boot/dts/freescale/imx8mp-adi*.dtb ::/
if [[ $BUILD_TYPE == "buildroot" ]]; then
	mcopy -s -i tmp/part1.fat32 $ROOTDIR/build/buildroot/output/images/rootfs.cpio.uboot ::/
fi
dd if=/dev/zero of=${IMG} bs=1M count=6001
dd if=$ROOTDIR/build/imx-mkimage/iMX8M/flash.bin of=${IMG} bs=1K seek=32 conv=notrunc
env PATH="$PATH:/sbin:/usr/sbin" parted --script ${IMG} mklabel msdos mkpart primary 2MiB 150MiB mkpart primary 150MiB 6000MiB
dd if=tmp/part1.fat32 of=${IMG} bs=1M seek=2 conv=notrunc
dd if=${ROOTFS_IMAGE} of=${IMG} bs=1M seek=150 conv=notrunc
echo -e "\n\n*** Image is ready - images/${IMG}"
