#!/bin/bash

#ADSD3500 Reset Pin
if [ ! -d /sys/class/gpio/gpio122 ]
then
	echo 122 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio122/direction
fi

echo 0 > /sys/class/gpio/gpio122/value

sleep 1

echo 1 > /sys/class/gpio/gpio122/value
