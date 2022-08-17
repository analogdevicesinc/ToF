#!/bin/bash

#V5V0 enable GPIO - GPIO4 / IO30 - Linux GPIO 126
#VMAIN enable GPIO - GPIO5 / IO00 - Linux GPIO 128
#VSYS enable GPIO - GPIO5 / IO01 - Linux GPIO 129
#VAUX enable GPIO - GPIO4 / IO31 - Linux GPIO 127
#VDAC enable GPIO - GPIO5 / IO02 - Linux GPIO 130

BOARD=$(strings /proc/device-tree/model)

if [[ $BOARD == "NXP i.MX8MPlus ADI TOF carrier + ADSD3500" ]]; then
	#ADSD3500 Reset Pin
	echo 122 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio122/direction
	echo 0 > /sys/class/gpio/gpio122/value

	# Boot strap MAX7321
	#OC0
	echo 496 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio496/direction
	echo 0 > /sys/class/gpio/gpio496/value

	#OC1
	echo 497 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio497/direction
	echo 0 > /sys/class/gpio/gpio497/value

	#OC2
	echo 498 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio498/direction
	echo 0 > /sys/class/gpio/gpio498/value

	#OC3
	echo 499 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio499/direction
	echo 0 > /sys/class/gpio/gpio499/value

	#OC4
	echo 500 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio500/direction
	echo 0 > /sys/class/gpio/gpio500/value

	#OC5
	echo 501 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio501/direction
	echo 0 > /sys/class/gpio/gpio501/value

	#OC6
	echo 502 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio502/direction
	echo 0 > /sys/class/gpio/gpio502/value

	#FLASH_WP
	echo 503 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio503/direction
	echo 1 > /sys/class/gpio/gpio503/value

	# Boot strap MAX7320
	#U0
	echo 507 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio507/direction
	echo 0 > /sys/class/gpio/gpio507/value

	#EN_1P8
	echo 510 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio510/direction
	echo 1 > /sys/class/gpio/gpio510/value

	#EN_0P8
	echo 511 > /sys/class/gpio/export
	echo out > /sys/class/gpio/gpio511/direction
	echo 1 > /sys/class/gpio/gpio511/value

	# Pull reset high
	echo 1 > /sys/class/gpio/gpio122/value
fi

# Deassert ADC reset - will pop on /dev/i2c1 address 0x10
echo 132 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio132/direction
echo 0 > /sys/class/gpio/gpio132/value
sleep 0.5
echo 1 > /sys/class/gpio/gpio132/value
i2cset -y 2 0x10 0x05 0x0700 w # Enable DAC bits 2..0
i2cset -y 2 0x10 0x0b 0x0002 w #

# Set DAC values -
i2cset -y 2 0x10 0x10 0x008a w # Vsys = 4.3V
#i2cset -y 2 0x10 0x11 0x6097 w # Vaux = 18V
#i2cset -y 2 0x10 0x11 0x0091 w # Vaux = 27V
i2cset -y 2 0x10 0x11 0x0594 w # Vaux = 22V
i2cset -y 2 0x10 0x12 0x00a3 w # V5v0 = 4.7V

# VDAC (3.3v / 1.8v IOs)
echo 130 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio130/direction
echo 1 > /sys/class/gpio/gpio130/value

i2cset -y 2 0x10 0x04 0xff00 w; i2cset -y 2 0x10 0x02 0xff03 w
for i in {0..8}; do 
VAL=`i2cget -y 2 0x10 0x40 w`
CH=$(($VAL & 0x00f0))
CH=$(($CH >> 4))
VOL1=$(($VAL & 0xf))
VOL1=$(($VOL1 << 8))
VOL2=$(($VAL & 0xff00))
VOL2=$(($VOL2 >> 8))
VOL=$(($VOL1 | $VOL2))
#echo $VAL $CH $VOL $VOL1 $VOL2

if [ $CH -le 2 ]; then
VALUE=`echo 1000*${VOL}*2.5/4096 | bc`
echo "DAC channel is $VALUE mV"
elif [ $CH -le 3 ]; then
VALUE=`echo ${VOL}*2.5/4096*133.2/33.2 | bc -l`
VALUE=`printf "%.3f" ${VALUE}`
echo "V_LD_SNS is $VALUE V"
elif [ $CH -le 4 ]; then
VALUE=`echo ${VOL}*2.5/4096*171.5/71.5 | bc -l`
VALUE=`printf "%.3f" ${VALUE}`
echo "VMAIN is $VALUE V"
elif [ $CH -le 5 ]; then
VALUE=`echo ${VOL}*2.5/4096*171.5/71.5 | bc -l`
VALUE=`printf "%.3f" ${VALUE}`
echo "VSYS is $VALUE V"
elif [ $CH -le 6 ]; then
VALUE=`echo ${VOL}*2.5/4096*108.66/8.66 | bc -l`
VALUE=`printf "%.3f" ${VALUE}`
echo "VAUX is $VALUE V"
elif [ $CH -le 7 ]; then
VALUE=`echo ${VOL}*2.5/4096*171.5/71.5 | bc -l`
VALUE=`printf "%.3f" ${VALUE}`
echo "V5V0 is $VALUE V"
fi
done # Reads back channel 0..7 and temperature (channel #8)

if [[ $BOARD == "NXP i.MX8MPlus ADI TOF board" ]]; then
	modprobe adsd3100 fw_load=0 calib_load=0
	modprobe spi_nor
fi
modprobe imx8_media_dev
