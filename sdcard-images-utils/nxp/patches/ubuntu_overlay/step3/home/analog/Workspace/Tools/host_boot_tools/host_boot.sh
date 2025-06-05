#!/bin/bash

# Remove adsd driver
sudo rmmod imx8_media_dev
sudo rmmod adsd3500

BOARD=$(strings /proc/device-tree/model)

case $BOARD in
		"NXP i.MX8MPlus ADI TOF carrier + ADSD3500")
			echo "Running on ADSD3500 + ADSD3100"
			NVM_GPIO='gpio507'
			;;
		"NXP i.MX8MPlus ADI TOF carrier + ADSD3030")
			echo "Running on ADSD3500 + ADSD3030"
			NVM_GPIO='gpio492'
			;;
		*)
			echo "Board model not valid"
			exit 1
			;;
esac

# Make i2c-* devices accessible by normal user
sudo chmod a+rw /dev/i2c-*

# Select interposer Flash memory to force hostboot
echo 1 | sudo tee /sys/class/gpio/$NVM_GPIO/value > /dev/null

# Reset ADSD3500
echo "Reset ADSD3500"
echo 0 | sudo tee /sys/class/gpio/gpio122/value > /dev/null
sleep 0.5
echo 1 | sudo tee /sys/class/gpio/gpio122/value > /dev/null

# Wait for ADSD3500 to boot
echo "Waiting for ADSD3500 to boot"
echo "It can take up to 30 seconds"

while [[ $(i2cdetect -y 1 | grep 38) == "" ]]
do
	sleep 1
done
#Install smbus2 module
pip install smbus2-0.4.2-py2.py3-none-any.whl > /dev/null 2>&1


# Run hostboot script
python3 host_boot.py host_boot.stream

# Select back the imager flash memory
echo 0 | sudo tee /sys/class/gpio/$NVM_GPIO/value > /dev/null

sleep 2

# Read chip ID to check correct hostboot
python3 read_chip_id.py

# Reload ADSD3500 driver
sudo modprobe adsd3500
sudo modprobe imx8_media_dev
