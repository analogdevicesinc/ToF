sudo mkdir /mnt/ramdisk 2> /dev/null
sudo mount -t tmpfs -o rw,size=1G tmpfs /mnt/ramdisk
df -h | grep ramdisk
