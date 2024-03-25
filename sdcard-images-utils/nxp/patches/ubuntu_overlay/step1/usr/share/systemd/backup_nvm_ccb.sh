#!/bin/bash
nvm_size=741376
sleep 15
BOARD=$(strings /proc/device-tree/model)
echo ${BOARD}
if [[ $BOARD == "NXP i.MX8MPlus ADI TOF carrier + ADSD3500" ]]; then
        echo "Current platform is ADTF3175D"
        nvm="ADTF3175D.nvm"
        ccb="ADTF3175D.ccb"
        echo "nvm: ${nvm} and ccb: ${ccb}"
else
        echo "Current platform is ADSD3030"
        nvm="ADSD3030.nvm"
        ccb="ADSD3030.ccb"
        echo "nvm: ${nvm} and ccb: ${ccb}"
fi

if [ -d "/boot" ]; then
        if [ ! -s "/boot/${nvm}" ]; then
                echo "Backup NVM_Flash image to /boot"
                chmod +x /home/analog/Workspace/Tools/host_boot_tools/NVM_Utils/NVM_READ
                /home/analog/Workspace/Tools/host_boot_tools/NVM_Utils/NVM_READ /boot/${nvm}
        else
                a=$(wc -c < "/boot/${nvm}")
                if [ $a != $nvm_size ]; then
                        echo "Backup NVM_Flash image to /boot"
                        chmod +x /home/analog/Workspace/Tools/host_boot_tools/NVM_Utils/NVM_READ
                        /home/analog/Workspace/Tools/host_boot_tools/NVM_Utils/NVM_READ /boot/${nvm}
                fi

        fi
        if [ ! -s "/boot/${ccb}" ]; then
                echo "Backup CCB to /boot"
                chmod +x /home/analog/Workspace/Tools/host_boot_tools/NVM_Utils/CCB_READ
                /home/analog/Workspace/Tools/host_boot_tools/NVM_Utils/CCB_READ /boot/${ccb}
        fi
fi

chown -R analog:analog /home/analog/Workspace