#! /bin/bash

help()
{
	echo "Usage: ./`basename $0` image_image sdcard_dev_name" 
	exit 0
}

IMAGE=$1
DISK=$2
RFS_PART=2

if [ "x$1" == "x"  -o "x$2" == "x" ]; then
    help
    exit -1
fi

echo "`basename $0` $1 $2"

sudo dd if=/dev/zero of=${DISK} bs=1M count=3
sudo dd if=${IMAGE} of=${DISK} bs=1M status=progress

sudo sync
sudo hdparm -z ${DISK}
echo "Resize the rootfs partition"

sudo parted -s ${DISK} "resizepart ${RFS_PART} -1" quit

sudo hdparm -z ${DISK}

sudo e2fsck -f ${DISK}${RFS_PART}
sudo resize2fs ${DISK}${RFS_PART}

sudo sync
echo "Done..."
