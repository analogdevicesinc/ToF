KERNEL=kernel8
rm -rf package
tar -vxf package.tar.gz package/
#sudo cp mnt/fat32/$KERNEL.img mnt/fat32/$KERNEL-backup.img
cp package/$KERNEL.img /boot/firmware/$KERNEL.img
cp package/*.dtb /boot/firmware/
cp package/overlays/*.dtb* /boot/firmware/overlays/
cp package/overlays/README /boot/firmware/overlays/

cp -rf package/ext4/lib/modules/* /lib/modules/