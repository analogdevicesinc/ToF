#!/bin/bash

set -x

if [[ $EUID > 0 ]]; then
	echo "This script must be run as root user"
	echo "Usage: sudo ./apply_patch.sh"
	exit 1
fi

ROOTDIR=`pwd`

function setup_workspace_directory()
{
	# Check if /home/analog/Workspace exists
	if [ -d "/home/analog/Workspace" ]; then
		echo "Workspace directory exists. Removing it..."
		sudo rm -rf /home/analog/Workspace
	fi

	# Delete the existing ToF repo on the device.
	sudo rm -rf /usr/local/Workspace/ToF

	# Copy Workspace directory from /usr/local/ to /home/analog/
	echo "Copying Workspace directory from /usr/local/ to /home/analog/"
	sudo cp -r /usr/local/Workspace /home/analog/

	# Copy the ToF directory to the /usr/local/ and /home/analog/Workspace/ location.
	sudo cp -r $ROOTDIR/ToF /usr/local/Workspace/
	sudo cp -r $ROOTDIR/ToF /home/analog/Workspace/

	# Stop existing services running on the NVIDIA ToF device
	for service in network-gadget; do
		if systemctl is-active --quiet "$service"; then
			sudo systemctl stop "$service"
		fi
	done
}


function build_glog()
{
	pushd .
	cd /home/analog/Workspace/glog
	mkdir -p build_0_6_0 && cd build_0_6_0
	cmake -DWITH_GFLAGS=off -DCMAKE_INSTALL_PREFIX=/opt/glog ..
	sudo cmake --build . --target install
	popd
}

function build_libwebsockets()
{
	pushd .
	cd /home/analog/Workspace/libwebsockets
	mkdir -p build_3_1 && cd build_3_1
	cmake -DLWS_WITH_SSL=OFF -DLWS_STATIC_PIC=ON -DCMAKE_INSTALL_PREFIX=/opt/websockets ..
	sudo cmake --build . --target install
	popd
}

function build_protobuf()
{
	pushd .
	cd /home/analog/Workspace/protobuf
	mkdir -p build_3_9_0 && cd build_3_9_0
	cmake -Dprotobuf_BUILD_TESTS=OFF -DCMAKE_POSITION_INDEPENDENT_CODE=ON -DCMAKE_INSTALL_PREFIX=/opt/protobuf ../cmake
	sudo cmake --build . --target install
	popd
}


function build_sdk()
{
	pushd .
	echo "Build process started"
	build_glog
	build_libwebsockets
	build_protobuf
	cd /home/analog/Workspace/ToF
	rm -rf build
	mkdir build && cd build
	cmake -DNVIDIA=1 -DCMAKE_BUILD_TYPE=Release -DWITH_EXAMPLES=on -DWITH_NETWORK=on -DCMAKE_PREFIX_PATH="/opt/glog;/opt/protobuf;/opt/websockets" ..  -Wno-dev
	make -j4
	echo "Build Completed!"
	popd
}

function update_server()
{
	sudo systemctl stop  network-gadget
	sudo cp /home/analog/Workspace/ToF/build/apps/server/aditof-server /usr/share/systemd/
}

function apply_ubuntu_overlay()
{
        echo "Updating dhcpd.conf"
        cp $ROOTDIR/ubuntu_overlay/opt/nvidia/l4t-usb-device-mode/dhcpd.conf 		/opt/nvidia/l4t-usb-device-mode/

        echo "Update the FAT32 partition to filesystem table"
        cp $ROOTDIR/ubuntu_overlay/etc/fstab 						/etc/fstab

	#Set the MTU size to 15000 by default
        echo "Configuring the MTU size to 15000 by default"
        cp $ROOTDIR/ubuntu_overlay/etc/NetworkManager/conf.d/10-ignore-interface.conf 	/etc/NetworkManager/conf.d/
        cp $ROOTDIR/ubuntu_overlay/etc/systemd/network/10-rndis0.network      		/etc/systemd/network/

        echo "Updating the usb device mode configuration"
        cp $ROOTDIR/ubuntu_overlay/opt/nvidia/l4t-usb-device-mode/nv-l4t-usb-device-mode-*.sh  	/opt/nvidia/l4t-usb-device-mode/

	echo "Copy the network gadget service file"
        cp $ROOTDIR/ubuntu_overlay/etc/systemd/system/network-gadget.service 		/etc/systemd/system


}

function install_packages()
{
	sudo -s <<EOF
        echo "Enter into the root shell"

	echo "Install v4l-utils package"
	apt-get install v4l-utils

	echo "Install cmake package"
	apt install cmake -y

	echo "Install build-essential for C++ compilation tools"
	apt install build-essential -y

	echo "Install ZMQ libraries"
	apt install libzmq3-dev
		
        exit
EOF
    echo "Exited from the root shell"


}

function update_kernel()
{
	mkdir test
	sudo cp sw-versions /boot/
	tar -xvf kernel_supplements.tbz2 -C test
	sudo cp Image.fgv2 /boot/Image
	sudo cp -rf dtb/* /boot/
	cd test/lib/modules/
	sudo cp -rf 5.15.148-tegra /lib/modules/
	sudo rm -rf test/
}

function start_services()
{
	sudo systemctl reload NetworkManager
	sudo systemctl enable systemd-networkd
	sudo systemctl start  systemd-networkd
	sudo cp /home/analog/Workspace/ToF/build/apps/server/aditof-server /usr/share/systemd/
	sudo systemctl enable network-gadget
	sudo systemctl start network-gadget

}

extlinux_conf_file="/boot/extlinux/extlinux.conf"

function get_uuid_count() 
{
	count_part_uuid="$(grep -c "PARTUUID" ${extlinux_conf_file})"
	echo "get_uuid_count = ${count_part_uuid}"
}

function get_uuid_number() 
{
	# Extract the 36 characters after PARTUUID=
	partuuid=$(cat ${extlinux_conf_file} | grep -oP '(?<=PARTUUID=).{36}')
	# Print the extracted PARTUUID
	echo "partuuid = ${partuuid}"
}

function add_boot_label()
{
	echo "Add the backup kernel label"
	echo "LABEL backup" >> ${extlinux_conf_file}
	echo "      MENU LABEL backup kernel" >> ${extlinux_conf_file}
	echo "      LINUX /boot/Image.backup" >> ${extlinux_conf_file}
	echo "      FDT /boot/dtb/kernel_tegra234-p3767-0004-p3768-0000-a0.dtb" >> ${extlinux_conf_file}
	echo "      INITRD /boot/initrd" >> ${extlinux_conf_file}
	echo "      APPEND \${cbootargs} root=PARTUUID=${partuuid} rw rootwait rootfstype=ext4 mminit_loglevel=4 console=ttyTCU0,115200 console=ttyAMA0,115200 firmware_class.path=/etc/firmware fbcon=map:0 net.ifnames=0 nospectre_bhb" >> ${extlinux_conf_file}
	echo " " >> ${extlinux_conf_file}

}

function main()
{
	echo "******* Install Software Packages *******"
	install_packages
	
	echo "******* Setup ToF SDK repo *******"
	setup_workspace_directory
	
	echo "******* Build the SDK *******"
	build_sdk
	
	echo "******* Update the Extlinux Conf file *******"
	get_uuid_count
	if [[ "${count_part_uuid}" == 1 ]]; then
		get_uuid_number
		add_boot_label
	fi

	echo "******* Apply Ubuntu Overlay *******"
	apply_ubuntu_overlay
	
	echo "******* Update Kernel *******"
	update_kernel
	
	echo "******* Start background services *******"
	start_services
	
	echo "******* Reboot the system *******"
	sudo reboot

}

main
