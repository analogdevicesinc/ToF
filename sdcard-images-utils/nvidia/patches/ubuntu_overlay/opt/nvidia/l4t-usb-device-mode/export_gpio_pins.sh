#!/bin/bash

BOARD=$(strings /proc/device-tree/model)

adsd3100_export_gpio(){

	echo "Export the ADSD3100 port expander SYSFS GPIO"

	# VAUX_DAC_EN
	if [ ! -d /sys/class/gpio/gpio308 ]
	then
		echo 308 > /sys/class/gpio/export
		echo 0 > /sys/class/gpio/gpio308/value
	fi

	# VAUX_DISCH_EN
	if [ ! -d /sys/class/gpio/gpio309 ]
	then
		echo 309 > /sys/class/gpio/export
		echo 0 > /sys/class/gpio/gpio309/value
	fi

	# CAM_NVRAM_WP
	if [ ! -d /sys/class/gpio/gpio310 ]
	then
		echo 310 > /sys/class/gpio/export
		echo 0 > /sys/class/gpio/gpio310/value
	fi

	# RUN_VAUX_EN
	if [ ! -d /sys/class/gpio/gpio311 ]
	then
		echo 311 > /sys/class/gpio/export
		echo 0 > /sys/class/gpio/gpio311/value
	fi

	# VAUX_LS_EN
	if [ ! -d /sys/class/gpio/gpio312 ]
	then
		echo 312 > /sys/class/gpio/export
		echo 0 > /sys/class/gpio/gpio312/value
	fi

	# VSYS_LS_EN
	if [ ! -d /sys/class/gpio/gpio313 ]
	then
		echo 313 > /sys/class/gpio/export
		echo 0 > /sys/class/gpio/gpio313/value
	fi

	# FSYNC_DIR
	if [ ! -d /sys/class/gpio/gpio314 ]
	then
		echo 314 > /sys/class/gpio/export
		echo 0 > /sys/class/gpio/gpio314/value
	fi

	#LED_CTRL_EN
	if [ ! -d /sys/class/gpio/gpio315 ]
	then
		echo 315 > /sys/class/gpio/export
		echo 0 > /sys/class/gpio/gpio315/value
	fi

}

adsd3030_export_gpio(){

        echo "Export the ADSD3030 port expander SYSFS GPIO"

        # VAUX_DAC_EN
        if [ ! -d /sys/class/gpio/gpio300 ]
        then
                echo 300 > /sys/class/gpio/export
                echo 0 > /sys/class/gpio/gpio300/value
        fi

        # VAUX_DISCH_EN
        if [ ! -d /sys/class/gpio/gpio301 ]
        then
                echo 301 > /sys/class/gpio/export
                echo 0 > /sys/class/gpio/gpio301/value
        fi

        # CAM_NVRAM_WP
        if [ ! -d /sys/class/gpio/gpio302 ]
        then
                echo 302 > /sys/class/gpio/export
                echo 0 > /sys/class/gpio/gpio302/value
        fi

        # RUN_VAUX_EN
        if [ ! -d /sys/class/gpio/gpio303 ]
        then
                echo 303 > /sys/class/gpio/export
                echo 0 > /sys/class/gpio/gpio303/value
        fi

        # VAUX_LS_EN
        if [ ! -d /sys/class/gpio/gpio304 ]
        then
                echo 304 > /sys/class/gpio/export
                echo 0 > /sys/class/gpio/gpio304/value
        fi

        # VSYS_LS_EN
        if [ ! -d /sys/class/gpio/gpio305 ]
        then
                echo 305 > /sys/class/gpio/export
                echo 0 > /sys/class/gpio/gpio305/value
        fi

        # FSYNC_DIR
        if [ ! -d /sys/class/gpio/gpio306 ]
        then
                echo 306 > /sys/class/gpio/export
                echo 0 > /sys/class/gpio/gpio306/value
        fi

        #LED_CTRL_EN
        if [ ! -d /sys/class/gpio/gpio307 ]
        then
                echo 307 > /sys/class/gpio/export
                echo 0 > /sys/class/gpio/gpio307/value
        fi

	# EN_3V3
        if [ ! -d /sys/class/gpio/gpio308 ]
        then
                echo 308 > /sys/class/gpio/export
                echo 0 > /sys/class/gpio/gpio308/value
        fi

        # EN_1V2
        if [ ! -d /sys/class/gpio/gpio309 ]
        then
                echo 309 > /sys/class/gpio/export
                echo 0 > /sys/class/gpio/gpio309/value
        fi

        # EN_1V8
        if [ ! -d /sys/class/gpio/gpio310 ]
        then
                echo 310 > /sys/class/gpio/export
                echo 0 > /sys/class/gpio/gpio310/value
        fi

        # EN_VLD
        if [ ! -d /sys/class/gpio/gpio311 ]
        then
                echo 311 > /sys/class/gpio/export
                echo 0 > /sys/class/gpio/gpio311/value
        fi


}

configure_port_expander_gpio(){

	case $BOARD in
		"NVIDIA Orin nano ADI FG_V2 carrier + ADSD3100")
			adsd3100_export_gpio
			;;
		"NVIDIA Orin nano ADI FG_V2 carrier + ADSD3030")
			adsd3030_export_gpio
			;;
		*)
			echo "Board model not valid"
			exit 1
			;;
	esac
}


main(){

	configure_port_expander_gpio

	echo "Export SPI_CAM_MOSI GPIO and configure output-low"
	if [ ! -d /sys/class/gpio/PY.02 ]
	then
		echo 472 > /sys/class/gpio/export
		echo out > /sys/class/gpio/PY.02/direction
	fi

	echo "Export Nvidia Standard GPIO pins"

	#CAM_RESET_N
	echo "Exporting PAC.06 (492)"
	if [ ! -d /sys/class/gpio/PAC.06 ]
	then
		echo 492 > /sys/class/gpio/export
		echo out > /sys/class/gpio/PAC.06/direction
		echo 0 > /sys/class/gpio/PAC.06/value
	fi

}

if [[ $EUID > 0 ]]; then
        echo "This script must be run as root user"
        echo "Usage: sudo ./probe_sensor_driver.sh"
        exit
fi

main
