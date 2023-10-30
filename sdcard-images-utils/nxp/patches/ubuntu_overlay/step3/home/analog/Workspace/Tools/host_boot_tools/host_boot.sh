#!/bin/bash

BOARD=$(strings /proc/device-tree/model)

adsd3100_power_sequence(){

	#ADSD3500 Reset Pin
	echo 0 > /sys/class/gpio/gpio122/value

	#EN_1P8
	echo 0 > /sys/class/gpio/gpio510/value

	sleep 1

	#EN_0P8
	echo 0 > /sys/class/gpio/gpio511/value

	#BS0 Set 0 - self boot and 1 - host boot
	echo 1 > /sys/class/gpio/gpio139/value

	#EN_1P8
	echo 1 > /sys/class/gpio/gpio510/value

	sleep 1

	#EN_0P8
	echo 1 > /sys/class/gpio/gpio511/value

	# Pull reset high
	echo 1 > /sys/class/gpio/gpio122/value

}

adsd3030_power_sequence(){

	#ADSD3500 Reset Pin
	echo 0 > /sys/class/gpio/gpio122/value

	#U5	#EN_3V3
	echo 0 > /sys/class/gpio/gpio504/value

	#U5	#EN_1V2
	echo 0 > /sys/class/gpio/gpio505/value

	#U5	#EN_1V8
	echo 0 > /sys/class/gpio/gpio506/value

	sleep 1

	#U5	#EN_VLDD
	echo 0 > /sys/class/gpio/gpio507/value

	#U5	#EN_0V8
	echo 0 > /sys/class/gpio/gpio509/value

	#BS0 Set 0 - self boot and 1 - host boot
	echo 1 > /sys/class/gpio/gpio139/value

	#U5	#EN_3V3
	echo 1 > /sys/class/gpio/gpio504/value

	#U5	#EN_1V2
	echo 1 > /sys/class/gpio/gpio505/value

	#U5	#EN_1V8
	echo 1 > /sys/class/gpio/gpio506/value

	sleep 1

	#U5	#EN_VLDD
	echo 1 > /sys/class/gpio/gpio507/value

	#U5	#EN_0V8																
	echo 1 > /sys/class/gpio/gpio509/value

	# Pull reset high
	echo 1 > /sys/class/gpio/gpio122/value

}

configure_boot_pin(){

	case $BOARD in
		"NXP i.MX8MPlus ADI TOF carrier + ADSD3500")
			echo "Running on ADSD3500 + ADSD3100"
			adsd3100_power_sequence
			;;
		"NXP i.MX8MPlus ADI TOF carrier + ADSD3030")
			echo "Running on ADSD3500 + ADSD3030"
			adsd3030_power_sequence
			;;
		*)
			echo "Board model not valid"
			exit 1
			;;
	esac
}

main(){

	PIN_STATE=$(sudo cat /sys/class/gpio/gpio139/value)

	echo -e "Boot pin state is $PIN_STATE"

	if [ $PIN_STATE == 1 ]; then
		echo -e "Host boot is enabled\n"
	else
		echo -e "Self boot is enabled\n"
	fi

	# Remove adsd driver
	sudo rmmod imx8_media_dev
	sudo rmmod adsd3500

	echo -e "Reset ADSD3500\n"
	configure_boot_pin

	# Wait for ADSD3500 to boot
	echo "Detect I2C device"
	while [[ $(i2cdetect -y 1 | grep 38) == "" ]]
	do
		sleep 1
	done

	#Reload ADSD3500 driver
	echo "Started ADSD3500 host boot"
	sudo modprobe adsd3500 fw_load=1
	sudo modprobe imx8_media_dev
	sleep 2
	echo "Completed"

}

main

