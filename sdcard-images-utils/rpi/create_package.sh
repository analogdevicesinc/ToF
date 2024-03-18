cd linux
KERNEL=kernel8
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- bcm2711_defconfig
make -j8 ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- Image modules dtbs

# 32-Bit
#sudo cp mnt/fat32/$KERNEL.img mnt/fat32/$KERNEL-backup.img
#sudo cp arch/arm/boot/zImage mnt/fat32/$KERNEL.img
# Choose one of the following based on the kernel version
  # For kernels up to 6.4:
#  sudo cp arch/arm/boot/dts/*.dtb mnt/fat32/
  # For kernel 6.5 and above:
#  sudo cp arch/arm/boot/dts/broadcom/*.dtb mnt/fat32/
#sudo cp arch/arm/boot/dts/overlays/*.dtb* mnt/fat32/overlays/
#sudo cp arch/arm/boot/dts/overlays/README mnt/fat32/overlays/
#sudo umount mnt/fat32
#sudo umount mnt/ext4

#64-Bit
#KERNEL=kernel8
#sudo cp mnt/fat32/$KERNEL.img mnt/fat32/$KERNEL-backup.img
cp arch/arm64/boot/Image package/$KERNEL.img
cp arch/arm64/boot/dts/broadcom/*.dtb package/
cp arch/arm64/boot/dts/overlays/*.dtb* package/overlays/
cp arch/arm64/boot/dts/overlays/README package/overlays/
make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- INSTALL_MOD_PATH=package/ext4 modules_install
rm -rf package.tar.gz
tar -czvf package.tar.gz package/*

#sudo umount mnt/fat32
#sudo umount mnt/ext4
#make ARCH=arm64 CROSS_COMPILE=aarch64-linux-gnu- INSTALL_MOD_PATH=package/ext4 modules_install
#tar -czvf package.tar.gz package/*
