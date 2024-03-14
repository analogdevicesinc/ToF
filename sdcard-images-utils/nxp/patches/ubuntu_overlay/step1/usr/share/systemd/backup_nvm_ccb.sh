#!/bin/bash
nvm_size=741376
sleep 15
if [ -d "/boot" ]; then
	if [ ! -s "/boot/nvm_backup.bin" ]; then
		echo "Backup NVM_Flash image to /boot"
		chmod +x /home/analog/Workspace/Tools/host_boot_tools/NVM_Utils/NVM_READ
		/home/analog/Workspace/Tools/host_boot_tools/NVM_Utils/NVM_READ /boot/nvm_backup.bin
	else
		a=$(wc -c < "/boot/nvm_backup.bin")
		if [ $a != $nvm_size ]; then
			echo "Backup NVM_Flash image to /boot"
			chmod +x /home/analog/Workspace/Tools/host_boot_tools/NVM_Utils/NVM_READ
			/home/analog/Workspace/Tools/host_boot_tools/NVM_Utils/NVM_READ /boot/nvm_backup.bin
		fi

	fi
	if [ ! -s "/boot/ccb_backup.ccb" ]; then
		echo "Backup CCB to /boot"
		chmod +x /home/analog/Workspace/Tools/host_boot_tools/NVM_Utils/CCB_READ
		/home/analog/Workspace/Tools/host_boot_tools/NVM_Utils/CCB_READ /boot/ccb_backup.ccb
	else
		echo "CCB backup already sucessful"
	fi
fi

chown -R analog:analog /home/analog/Workspace
