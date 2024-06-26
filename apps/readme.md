# 3D Time of Flight : Apps

This directory contains different apps that are shipped with the SDK.

#### List of apps

| Name | Language | Description |
| --------- | ----------- | -------------- |
| server | C++ | Server application that allows clients to connect to camera attached to the target, control it and get frames from it |

# Enabling the apps on the NXP platform

| :exclamation:  N/A |
|-----------------------------------------|

## server

The server app is enabled by default. 
It can be disabled by running:
````
sudo systemctl disable network-gadget
````
It can be enabled by running:
````
sudo systemctl enable network-gadget
````

The network-over-usb connection and the mass storage generated can be enabled by editing the usb-gadget.sh file located in
/usr/share/systemd/usb-gadget.sh. 
In order to enable this functionality you must uncomment the following lines:
````
create_rndis configs/c.1 rndis.0
create_msd configs/c.1 mass_storage.0 /dev/mmcblk1p1
````
If you want to disable this service the lines must be commented.
````
#create_rndis configs/c.1 rndis.0
#create_msd configs/c.1 mass_storage.0 /dev/mmcblk1p1
````

After this operation a reboot is required.
```` 
sudo reboot
````

> **_NOTE:_**  There changes required for ADTF3175D are separate from the ADSD3030 ones.

For ADTF3175D we need to do the changes in:
````
"NXP i.MX8MPlus ADI TOF carrier + ADSD3500")
			#create_uvc configs/c.1 uvc.0
			create_rndis configs/c.1 rndis.0
			create_msd configs/c.1 mass_storage.0 /dev/mmcblk1p1
			;;
````

For ADSD3030 we need to do the changes in:
````
"NXP i.MX8MPlus ADI TOF carrier + ADSD3030")
			#create_uvc configs/c.1 uvc.0
			create_rndis configs/c.1 rndis.0
			create_msd configs/c.1 mass_storage.0 /dev/mmcblk1p1
			;;
````

### Updating the apps in systemctl service 

These apps are started by a systemctl routine at boot. 
After the sdk was updated to the desired version the next steps are required:
Stop the systemctl services by running:
````
sudo systemctl stop network-gadget
````
Copy the apps to /usr/share/systemd:
```` 
sudo cp /home/analog/Workspace/ToF/build/apps/server/aditof-server /usr/share/systemd/
````

After this operation a reboot is required.
```` 
sudo reboot
````