#!/bin/bash

BOARD=$(strings /proc/device-tree/model)

check_host_boot_pin_state() {

	PIN_STATE=$(sudo cat /sys/class/gpio/gpio139/value)

	echo -e "Boot pin state is $PIN_STATE"

	if [ $PIN_STATE == 1 ]; then
		echo -e "Host boot is enabled\n"
	else
		echo -e "Self boot is enabled\n"
	fi
}

detect_i2c_devices(){

	counter=0

	echo "Detect I2C device"
	while [[ $(i2cdetect -y 1 | grep 38) == "" ]]
	do
		sleep 1
		((counter++))
		if [ $counter -gt 30 ]; then
			echo "Device not found and host boot failed"
			exit 1
		fi
	done
}

remove_adsd3500_i2c_driver() {

	sudo rmmod imx8_media_dev
	sudo rmmod adsd3500
}

probe_adsd3500_i2c_driver() {

	echo "Started ADSD3500 I2C host boot"
	sudo modprobe adsd3500 fw_load=1 dyndbg==pmf
	sudo modprobe imx8_media_dev
	sleep 1
	echo "Completed"
}

remove_adsd3500_spi_driver() {

	sudo rmmod imx8_media_dev
	sudo rmmod adsd3500-spi
}

probe_adsd3500_spi_driver() {

	echo "Started ADSD3500 SPI host boot"
	sudo modprobe adsd3500-spi fw_load=1 dyndbg==pmf
	sudo modprobe imx8_media_dev
	sleep 1
	echo "Completed"
}

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

adsd3500_dual_power_sequence(){

	#Disable supply voltage
	echo 0 > /sys/class/gpio/gpio128/value
	echo 0 > /sys/class/gpio/gpio129/value

	sleep 1

	#EN_PWR
	echo 1 > /sys/class/gpio/gpio128/value

	#EN_PWR2
	echo 1 > /sys/class/gpio/gpio129/value

	sleep 1

	#Enable MAX77857
	sudo i2cset -y 1 0x66 0x14 0x20
	sleep 0.1

	#Enable MAX77542
	sudo i2cset -y 1 0x63 0x10 0xF

	sleep 1

	#ADSD3500 Reset Pin
	echo 0 > /sys/class/gpio/gpio64/value

	#BS0 Set 0 - Self boot and 1 - Host boot
	echo 1 > /sys/class/gpio/gpio139/value

	sleep 1

	# Pull reset high
	echo 1 > /sys/class/gpio/gpio64/value

}

enable_host_boot(){

	case $BOARD in
		"NXP i.MX8MPlus ADI TOF carrier + ADSD3500")
			echo "Running on ADSD3500 + ADSD3100"
			check_host_boot_pin_state
			remove_adsd3500_i2c_driver
			adsd3100_power_sequence
			detect_i2c_devices
			probe_adsd3500_i2c_driver
			;;
		"NXP i.MX8MPlus ADI TOF carrier + ADSD3030")
			echo "Running on ADSD3500 + ADSD3030"
			check_host_boot_pin_state
			remove_adsd3500_i2c_driver
			adsd3030_power_sequence
			detect_i2c_devices
			probe_adsd3500_i2c_driver
			;;
		"NXP i.MX8MPlus ADI TOF carrier ADSD3500-SPI + ADSD3100")
			echo "Running on ADSD3500-SPI + ADSD3100"
			check_host_boot_pin_state
			remove_adsd3500_spi_driver
			adsd3100_power_sequence
			probe_adsd3500_spi_driver
			;;
		"NXP i.MX8MPlus ADI TOF carrier ADSD3500-SPI + ADSD3030")
			echo "Running on ADSD3500-SPI + ADSD3030"
			check_host_boot_pin_state
			remove_adsd3500_spi_driver
			adsd3030_power_sequence
			probe_adsd3500_spi_driver
			;;
		"NXP i.MX8MPlus ADI TOF carrier ADSD3500-DUAL + ADSD3100")
			echo "Running on ADSD3500-DUAL + ADSD3100"
			check_host_boot_pin_state
			remove_adsd3500_i2c_driver
			adsd3500_dual_power_sequence
			detect_i2c_devices
			probe_adsd3500_i2c_driver
			;;
		*)
			echo "Board model not valid"
			exit 1
			;;
	esac
}

main(){

	enable_host_boot
}

if [[ $EUID > 0 ]]; then
                echo "This script must be run as root user"
		echo "Usage: sudo ./host_boot.sh"
		exit
fi

main
