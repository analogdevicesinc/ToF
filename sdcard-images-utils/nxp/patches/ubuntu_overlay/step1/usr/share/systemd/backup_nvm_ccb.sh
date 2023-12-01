#!/bin/bash
chown -R analog:analog /home/analog/Workspace
nvm_size=741376
sleep 15
if [ -d "/boot" ]; then
	if [ ! -f "/boot/nvm_backup.bin" ]; then
		echo "Backup NVM_Flash image to /boot"
		chmod +x /home/analog/Workspace/Tools/host_boot_tools/NVM_Flash_Utility/NVM_READ
		/home/analog/Workspace/Tools/host_boot_tools/NVM_Flash_Utility/NVM_READ /boot/nvm_backup.bin
	else
		a=$(wc -c < "/boot/nvm_backup.bin")
		if [ $a != $nvm_size ]; then
			echo "Backup NVM_Flash image to /boot"
			chmod +x /home/analog/Workspace/Tools/host_boot_tools/NVM_Flash_Utility/NVM_READ
			/home/analog/Workspace/Tools/host_boot_tools/NVM_Flash_Utility/NVM_READ /boot/nvm_backup.bin
		fi

	fi
	if [ ! -f "/boot/ccb_backup.ccb" ]; then
		echo "Backup CCB to /boot"
		chmod +x /home/analog/Workspace/Tools/host_boot_tools/NVM_Flash_Utility/CCB_READ
		/home/analog/Workspace/Tools/host_boot_tools/NVM_Flash_Utility/CCB_READ /boot/ccb_backup.ccb
	fi
fi
