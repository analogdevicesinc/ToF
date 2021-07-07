#!/bin/bash

if [ ! -d /sys/class/gpio/gpio156/ ]
then
	echo 156 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio156/direction
fi

while [ 1 ]
do
	# gpio5, io28: (5-1)*32 + 28 = 156 -> FSYNC
	# echo "Bringing FSYNC pin HIGH"
	echo 1 > /sys/class/gpio/gpio156/value
	sleep 0.037
	# echo "Bringing FSYNC pin LOW"
	echo 0 > /sys/class/gpio/gpio156/value
	sleep 0.037
done
