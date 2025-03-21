1. Once the device gets booted, copy the below two scripts to the frame grabber orin nano target board and execute the below commands.


	$ sudo chmod +x *.sh

	$ sudo cp nv-l4t-usb-device-mode-start.sh /opt/nvidia/l4t-usb-device-mode

	$ sudo cp nv-l4t-usb-device-mode-config.sh /opt/nvidia/l4t-usb-device-mode

	$ sudo sed -i -e 's/\r$//' /opt/nvidia/l4t-usb-device-mode/*.sh

2. Enable and disable the UVC gadget function. Open the below file to configure USB device mode.

	$ sudo vim nv-l4t-usb-device-mode-config.sh

	Go to line number 52 and edit the below variable to change the default configurations.

	CONFIG_TYPE="enable_uvc" # valid options: enable_uvc | disable_uvc

	enable_uvc: To enable the USB UVC gadget functions.

	disable_uvc: To enable the USB over Ethernet (RNDIS) and mass storage device.

3. Reboot the system.

	$ sudo reboot 
