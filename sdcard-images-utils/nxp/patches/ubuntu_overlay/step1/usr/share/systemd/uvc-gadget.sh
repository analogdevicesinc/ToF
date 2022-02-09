#!/bin/sh
# SPDX-License-Identifier: MIT

set -e
#set -x

CONFIGFS="/sys/kernel/config"
GADGET="$CONFIGFS/usb_gadget"
VID="0x064b"
PID="0xa4a2"
SERIAL="0123456789"
MANUF="Analog Devices, Inc. Development Tools"
PRODUCT="ADI CMOS TOF UVC Gadget"

USBFILE=/root/usbstorage.img

BOARD=$(strings /proc/device-tree/model)

case $BOARD in
	"Renesas Salvator-X board based on r8a7795 ES1.x")
		UDC_USB2=e6590000.usb
		UDC_USB3=ee020000.usb

		UDC_ROLE2=/sys/devices/platform/soc/ee080200.usb-phy/role
		UDC_ROLE2=/dev/null #Not needed - always peripheral
		UDC_ROLE3=/sys/devices/platform/soc/ee020000.usb/role

		UDC=$UDC_USB2
		UDC_ROLE=$UDC_ROLE2
		;;

	"TI OMAP4 PandaBoard-ES")
		UDC=`ls /sys/class/udc` # Should be musb-hdrc.0.auto
		UDC_ROLE=/dev/null # Not needed - peripheral enabled
		;;

	*)
		UDC=`ls /sys/class/udc` # will identify the 'first' UDC
		UDC_ROLE=/dev/null # Not generic
		;;
esac

echo "Detecting platform:"
echo "  board : $BOARD"
echo "  udc   : $UDC"

create_msd() {
	# Example usage:
	#	create_msd <target config> <function name> <image file>
	#	create_msd configs/c.1 mass_storage.0 /root/backing.img
	CONFIG=$1
	FUNCTION=$2
	BACKING_STORE=$3

	if [ ! -f $BACKING_STORE ]
	then
		echo "\tCreating backing file"
		dd if=/dev/zero of=$BACKING_STORE bs=1M count=32 > /dev/null 2>&1
		mkfs.ext4 $USBFILE > /dev/null 2>&1
		echo "\tOK"
	fi

	echo "\tCreating MSD gadget functionality"
	mkdir functions/$FUNCTION
	echo 1 > functions/$FUNCTION/stall
	echo $BACKING_STORE > functions/$FUNCTION/lun.0/file
	echo 1 > functions/$FUNCTION/lun.0/removable
	echo 0 > functions/$FUNCTION/lun.0/cdrom

	ln -s functions/$FUNCTION configs/c.1

	echo "\tOK"
}

delete_msd() {
	# Example usage:
	#	delete_msd <target config> <function name>
	#	delete_msd config/c.1 uvc.0
	CONFIG=$1
	FUNCTION=$2

	echo "Removing Mass Storage interface : $FUNCTION"
	rm -f $CONFIG/$FUNCTION
	rmdir functions/$FUNCTION
	echo "OK"
}

create_frame() {
	# Example usage:
	# create_frame <function name> <width> <height> <format> <name>

	FUNCTION=$1
	WIDTH=$2
	HEIGHT=$3
	FORMAT=$4
	NAME=$5

	wdir=functions/$FUNCTION/streaming/$FORMAT/$NAME/${HEIGHT}p

	mkdir -p $wdir
	echo $WIDTH > $wdir/wWidth
	echo $HEIGHT > $wdir/wHeight
	echo $(( $WIDTH * $HEIGHT * 2 )) > $wdir/dwMaxVideoFrameBufferSize
	cat <<EOF > $wdir/dwFrameInterval
100000
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

	create_frame $FUNCTION 4096 2560 uncompressed u
	create_frame $FUNCTION 4096 2304 uncompressed u
	create_frame $FUNCTION 4096 640 uncompressed u
	create_frame $FUNCTION 4096 256 uncompressed u
	create_frame $FUNCTION 3840 216 uncompressed u

	mkdir functions/$FUNCTION/streaming/header/h
	cd functions/$FUNCTION/streaming/header/h
	ln -s ../../uncompressed/u
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
	echo 2 > functions/$FUNCTION/streaming_interval

	ln -s functions/$FUNCTION configs/c.1
}

delete_uvc() {
	# Example usage:
	#	delete_uvc <target config> <function name>
	#	delete_uvc config/c.1 uvc.0
	CONFIG=$1
	FUNCTION=$2

	echo "	Deleting UVC gadget functionality : $FUNCTION"
	rm $CONFIG/$FUNCTION

	rm functions/$FUNCTION/control/class/*/h
	rm functions/$FUNCTION/streaming/class/*/h
	rm functions/$FUNCTION/streaming/header/h/u
	rmdir functions/$FUNCTION/streaming/uncompressed/u/*/
	rmdir functions/$FUNCTION/streaming/uncompressed/u
	rmdir functions/$FUNCTION/streaming/header/h
	rmdir functions/$FUNCTION/control/header/h
	rmdir functions/$FUNCTION
}

case "$1" in
    start)
	echo "Creating the USB gadget"
	#echo "Loading composite module"
	#modprobe libcomposite

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
	echo "OK"

	echo "Creating Config"
	mkdir configs/c.1
	mkdir configs/c.1/strings/0x409
	#set requested power to 900mA
	echo 900 > configs/c.1/MaxPower

	echo "Creating functions..."
	#create_msd configs/c.1 mass_storage.0 $USBFILE
	create_uvc configs/c.1 uvc.0
	#create_uvc configs/c.1 uvc.1
	echo "OK"

	echo "Binding USB Device Controller"
	echo $UDC > UDC
	echo peripheral > $UDC_ROLE
	cat $UDC_ROLE
	echo "OK"
	;;

    stop)
	echo "Stopping the USB gadget"

	set +e # Ignore all errors here on a best effort

	cd $GADGET/g1

	if [ $? -ne 0 ]; then
	    echo "Error: no configfs gadget found"
	    exit 1;
	fi

	echo "Unbinding USB Device Controller"
	grep $UDC UDC && echo "" > UDC
	echo "OK"

	#delete_uvc configs/c.1 uvc.1
	delete_uvc configs/c.1 uvc.0
	#delete_msd configs/c.1 mass_storage.0

	echo "Clearing English strings"
	rmdir strings/0x409
	echo "OK"

	echo "Cleaning up configuration"
	rmdir configs/c.1/strings/0x409
	rmdir configs/c.1
	echo "OK"

	echo "Removing gadget directory"
	cd $GADGET
	rmdir g1
	cd /
	echo "OK"

	#echo "Disable composite USB gadgets"
	#modprobe -r libcomposite
	#echo "OK"
	;;
    *)
	echo "Usage : $0 {start|stop}"
esac
