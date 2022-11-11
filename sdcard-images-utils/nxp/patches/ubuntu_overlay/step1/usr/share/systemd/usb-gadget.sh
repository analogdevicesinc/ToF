#!/bin/bash
# SPDX-License-Identifier: MIT

set -e

CONFIGFS="/sys/kernel/config"
GADGET="$CONFIGFS/usb_gadget"
VID="0x0456"
PID="0xa4a2"
MANUF="Analog Devices Inc."
PRODUCT="ADI TOF USB Gadget"
SERIAL="12345678"

USBFILE=/dev/mmcblk1p1
BOARD=$(strings /proc/device-tree/model)
UDC=`ls /sys/class/udc` # will identify the 'first' UDC

echo "Detecting platform:"
echo "  board : $BOARD"
echo "  udc   : $UDC"

create_rndis() {
	# Example usage:
	#	create_rndis <target config> <function name>
       	#	create_rndis configs/c.1 rndis.usb0
	CONFIG=$1
	FUNCTION=$2

	echo "Create $FUNCTION"

	mkdir functions/$FUNCTION

	# OS descriptors
	echo 1 > os_desc/use
	echo 0xcd > os_desc/b_vendor_code
	echo MSFT100 > os_desc/qw_sign
	echo RNDIS > functions/$FUNCTION/os_desc/interface.rndis/compatible_id
	echo 5162001 > functions/$FUNCTION/os_desc/interface.rndis/sub_compatible_id
	
	ln -s functions/$FUNCTION $CONFIG

	ln -s $CONFIG os_desc
}

create_msd() {
	# Example usage:
	#	create_msd <target config> <function name> <image file>
       	#	create_msd configs/c.1 mass_storage.0 /root/backing.img
	CONFIG=$1
	FUNCTION=$2
	BACKING_STORE=$3

	if [ ! -e $BACKING_STORE ]; then
        	echo "\tCreating backing file"
        	dd if=/dev/zero of=$BACKING_STORE bs=1M count=32 > /dev/null 2>&1
	        mkfs.ext4 $USBFILE > /dev/null 2>&1
        	echo "\tOK"
	fi

        echo "Creating MSD gadget functionality"
        mkdir functions/$FUNCTION
        echo 0 > functions/$FUNCTION/stall
        echo $BACKING_STORE > functions/$FUNCTION/lun.0/file
        echo 1 > functions/$FUNCTION/lun.0/removable
        echo 0 > functions/$FUNCTION/lun.0/cdrom

        ln -s functions/$FUNCTION $CONFIG

        echo "OK"
}

create_frame() {
	# Example usage:
	# create_frame <function name> <width> <height> <format> <name> <Bpp> <guid>

	FUNCTION=$1
	WIDTH=$2
	HEIGHT=$3
	FORMAT=$4
	NAME=$5
	BPP=$6
	GUID=$7

	wdir=functions/$FUNCTION/streaming/$FORMAT/$NAME/${WIDTH}x${HEIGHT}p
	mkdir -p $wdir
	echo $WIDTH > $wdir/wWidth
	echo $HEIGHT > $wdir/wHeight
	echo $(( $WIDTH * $HEIGHT * $BPP )) > $wdir/dwMaxVideoFrameBufferSize
	cat <<EOF > $wdir/dwFrameInterval
333333
666666
1000000
2000000
EOF
}

create_uvc() {
	# Example usage:
	#	create_uvc <target config> <function name>
	#	create_uvc config/c.1 uvc.0
	CONFIG=$1
	FUNCTION=$2

	echo "	Creating UVC gadget functionality : $FUNCTION"
	mkdir functions/$FUNCTION

	case $BOARD in
		"NXP i.MX8MPlus ADI TOF board")
			create_frame $FUNCTION 4096 2560 uncompressed u 2
			create_frame $FUNCTION 4096 2304 uncompressed u 2
			create_frame $FUNCTION 4096 640 uncompressed u 2
			create_frame $FUNCTION 4096 256 uncompressed u 2
			create_frame $FUNCTION 4096 192 uncompressed u 2
			create_frame $FUNCTION 3840 216 uncompressed u 2
			;;

		"NXP i.MX8MPlus ADI TOF carrier + ADSD3500")
			create_frame $FUNCTION 512 512 uncompressed u 2
			create_frame $FUNCTION 1024 3072 uncompressed u 2
			create_frame $FUNCTION 1024 4096 uncompressed u 2
			create_frame $FUNCTION 1024 512 uncompressed u1 1
			create_frame $FUNCTION 1280 512 uncompressed u1 1
			create_frame $FUNCTION 1536 512 uncompressed u1 1
			create_frame $FUNCTION 1792 512 uncompressed u1 1
			create_frame $FUNCTION 2048 512 uncompressed u1 1
			create_frame $FUNCTION 2304 512 uncompressed u1 1
			create_frame $FUNCTION 2560 512 uncompressed u1 1
			echo 8 > functions/$FUNCTION/streaming/uncompressed/u1/bBitsPerPixel
			echo -n -e '\x42\x41\x38\x31\x00\x00\x10\x00\x80\x00\x00\xaa\x00\x38\x9b\x71' > $GADGET/g1/functions/$FUNCTION/streaming/uncompressed/u1/guidFormat
			;;

		"NXP i.MX8MPlus ADI TOF carrier + ADSD3030")
			create_frame $FUNCTION 512 640 uncompressed u 2
			create_frame $FUNCTION 1024 960 uncompressed u 2
			create_frame $FUNCTION 1024 2880 uncompressed u 2
			create_frame $FUNCTION 1024 640 uncompressed u1 1
			create_frame $FUNCTION 1280 640 uncompressed u1 1
			create_frame $FUNCTION 1536 640 uncompressed u1 1
			create_frame $FUNCTION 1792 640 uncompressed u1 1
			create_frame $FUNCTION 2048 640 uncompressed u1 1
			create_frame $FUNCTION 2304 640 uncompressed u1 1
			create_frame $FUNCTION 2560 640 uncompressed u1 1
			echo 8 > functions/$FUNCTION/streaming/uncompressed/u1/bBitsPerPixel
			echo -n -e '\x42\x41\x38\x31\x00\x00\x10\x00\x80\x00\x00\xaa\x00\x38\x9b\x71' > $GADGET/g1/functions/$FUNCTION/streaming/uncompressed/u1/guidFormat
			;;

		*)
			echo "Board model not valid"
			exit 1
			;;
	esac

	echo -n -e '\x59\x55\x59\x32\x00\x00\x10\x00\x80\x00\x00\xaa\x00\x38\x9b\x71' > $GADGET/g1/functions/$FUNCTION/streaming/uncompressed/u/guidFormat

	mkdir functions/$FUNCTION/streaming/header/h
	cd functions/$FUNCTION/streaming/header/h
	ln -s ../../uncompressed/u
	if [[ $BOARD != "NXP i.MX8MPlus ADI TOF board" ]]; then
		ln -s ../../uncompressed/u1
	fi
	cd ../../class/fs
	ln -s ../../header/h
	cd ../../class/hs
	ln -s ../../header/h
	cd ../../class/ss
	ln -s ../../header/h
	cd ../../../control
	mkdir header/h
	ln -s header/h class/fs
	ln -s header/h class/ss
	cd ../../../

	# Set the packet size: uvc gadget max size is 3k...
	echo 3072 > functions/$FUNCTION/streaming_maxpacket
	#echo 2048 > functions/$FUNCTION/streaming_maxpacket
	#echo 1024 > functions/$FUNCTION/streaming_maxpacket

	echo 15 > functions/$FUNCTION/streaming_maxburst
	echo 3 > functions/$FUNCTION/streaming_interval

	ln -s functions/$FUNCTION $CONFIG
}

case "$1" in
    start)
	echo "Creating the USB gadget"

	echo "Creating gadget directory g1"
	mkdir -p $GADGET/g1

	cd $GADGET/g1
	if [ $? -ne 0 ]; then
	    echo "Error creating usb gadget in configfs"
	    exit 1;
	else
	    echo "OK"
	fi

	echo "Setting Vendor and Product ID's"
	echo $VID > idVendor
	echo $PID > idProduct
	echo "OK"

	echo "Setting English strings"
	mkdir -p strings/0x409
	echo $SERIAL > strings/0x409/serialnumber
	echo $MANUF > strings/0x409/manufacturer
	echo $PRODUCT > strings/0x409/product

	#Set things for Windows rndis
	echo 0xEF > bDeviceClass
	echo 0x02 > bDeviceSubClass
	echo 0x01 > bDeviceProtocol
	echo 0x100 > bcdDevice

	echo "OK"

	echo "Creating Config"
	mkdir configs/c.1
	mkdir configs/c.1/strings/0x409

	echo 896 > configs/c.1/MaxPower

	echo "Creating functions..."
	case $BOARD in
		"NXP i.MX8MPlus ADI TOF board")
			create_uvc configs/c.1 uvc.0
			;;

		"NXP i.MX8MPlus ADI TOF carrier + ADSD3500")
			#create_uvc configs/c.1 uvc.0
			create_rndis configs/c.1 rndis.0
			create_msd configs/c.1 mass_storage.0 /dev/mmcblk1p1
			;;

		"NXP i.MX8MPlus ADI TOF carrier + ADSD3030")
			#create_uvc configs/c.1 uvc.0
			create_rndis configs/c.1 rndis.0
			create_msd configs/c.1 mass_storage.0 /dev/mmcblk1p1
			;;

		*)
			echo "Board model not valid"
			exit 1
			;;
	esac
	echo "OK"

	echo "Binding USB Device Controller"
	echo $UDC > UDC
	echo "OK"
	;;
    *)
	echo "Usage : $0 {start}"
esac
