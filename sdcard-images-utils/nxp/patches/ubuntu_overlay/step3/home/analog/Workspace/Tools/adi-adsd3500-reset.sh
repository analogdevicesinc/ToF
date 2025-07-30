#!/bin/bash

BOARD=$(strings /proc/device-tree/model)

if [[ $BOARD == "NXP i.MX8MPlus ADI TOF carrier ADSD3500-DUAL + ADSD3100" ]]; then
	GPIO_RESET=gpio64
else
	GPIO_RESET=gpio122
fi
#ADSD3500 Reset Pin
if [ ! -d /sys/class/gpio/${GPIO_RESET} ]
then
	echo 122 > /sys/class/gpio/export
	echo out > /sys/class/gpio/${GPIO_RESET}/direction
fi

echo 0 > /sys/class/gpio/${GPIO_RESET}/value

sleep 1

echo 1 > /sys/class/gpio/${GPIO_RESET}/value
