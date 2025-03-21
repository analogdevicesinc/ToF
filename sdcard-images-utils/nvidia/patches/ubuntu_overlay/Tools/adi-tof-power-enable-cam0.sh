#!/bin/bash

if [[ $EUID > 0 ]]; then
	echo "This script must be run as root user"
	echo "Usage: sudo ./adi-tof-power-enable-cam0.sh"
	exit
fi

echo "export CAM0_PWDN and set direction as output"

# VAUX_DAC_EN
if [ ! -d /sys/class/gpio/PH.06 ]
then
	echo 397 > /sys/class/gpio/export
	echo out > /sys/class/gpio/PH.06/direction
fi

#ADSD3500 Reset Pin
sudo echo 0 > /sys/class/gpio/PH.06/value

#Disable the supply voltage
#EN_1P8
sudo gpioset 3 6=0

sleep 1

#EN_0P8
sudo gpioset 3 7=0

# Boot strap MAX7321
#OC0
sudo gpioset 2 0=0

#OC1
sudo gpioset 2 1=0

#OC2
sudo gpioset 2 2=0

#OC3
sudo gpioset 2 3=0

#OC4
sudo gpioset 2 4=0

#OC5
sudo gpioset 2 5=0

#OC6
sudo gpioset 2 6=0

#FLASH_WP
sudo gpioset 2 7=1


# Boot strap MAX7320
#U0
sudo gpioset 3 3=0

#DS2
sudo gpioset 3 5=1

#EN_1P8
sudo gpioset 3 6=1

sleep 1

#EN_0P8
sudo gpioset 3 7=1

# Pull reset high
sudo echo 1 > /sys/class/gpio/PH.06/value

