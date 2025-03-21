#!/bin/bash

set -ex

if [[ $EUID > 0 ]]; then
	echo "This script must be run as root user"
	echo "Usage: sudo ./flash.sh"
	exit
fi

ROOTDIR=`pwd`

cd $ROOTDIR/build/Linux_for_Tegra

sudo ./tools/l4t_create_default_user.sh -u analog -p analog -n ubuntu
udo ./flash.sh jetson-orin-nano-devkit mmcblk0p1
