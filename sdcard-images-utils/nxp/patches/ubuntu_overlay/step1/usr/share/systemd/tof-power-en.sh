#!/bin/bash

#mount -t devtmpfs /dev /dev

#i2cset -f -y 2 0x50 0x2e 0x0 # Sink role
#i2cset -f -y 2 0x50 0x2f 0x21 # Enable RX

#V5V0 
#V5V0 enable GPIO - GPIO4 / IO30 - Linux GPIO 126
#VMAIN enable GPIO - GPIO5 / IO00 - Linux GPIO 128
#VSYS enable GPIO - GPIO5 / IO01 - Linux GPIO 129
#VAUX enable GPIO - GPIO4 / IO31 - Linux GPIO 127
#VDAC enable GPIO - GPIO5 / IO02 - Linux GPIO 130

#DAC reset deassert - GPIO5 / IO4 - Linux GPIO 132 (Linux /dev/i2c1)
#ADC internal reference is 2.5v and requires to be enabled

#Vaux = 3.271v
#Vsys = 3.3v
#V5V0 = 

# Deassert ADC reset - will pop on /dev/i2c1 address 0x10
echo 132 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio132/direction
echo 0 > /sys/class/gpio/gpio132/value
sleep 0.5
echo 1 > /sys/class/gpio/gpio132/value
i2cset -y 1 0x10 0x05 0x0700 w # Enable DAC bits 2..0
i2cset -y 1 0x10 0x0b 0x0002 w #

# Set DAC values -
#i2cset -y 1 0x10 0x10 0xcc88 w # Vsys = 4.3v - calculated
i2cset -y 1 0x10 0x10 0x008a w # Vsys = 4.3v - actual
i2cset -y 1 0x10 0x11 0x1397 w # Vaux = 18v - calculated
i2cset -y 1 0x10 0x11 0x6097 w # Vaux = 18v - actual
#i2cset -y 1 0x10 0x11 0x0091 w # Vaux = 27V - actual
i2cset -y 1 0x10 0x12 0x82a3 w # V5v0 = 4.7v - calculated
i2cset -y 1 0x10 0x12 0x00a3 w # V5v0 = 4.7v - actual


#i2cset -y 1 0x10 0x04 0xff00 w; i2cset -y 1 0x10 0x02 0xff03 w; for i in {0..8}; do i2cget -y 1 0x10 0x40 w; done # Reads back channel 0..7 and temperature (channel #8)


# VMAIN
echo 128 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio128/direction

# V5V0
echo 126 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio126/direction

# VSYS
echo 129 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio129/direction

# VDAC (3.3v / 1.8v IOs)

echo 130 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio130/direction

# VAUX
echo 127 > /sys/class/gpio/export
echo out > /sys/class/gpio/gpio127/direction

echo 1 > /sys/class/gpio/gpio128/value
echo 1 > /sys/class/gpio/gpio126/value
echo 1 > /sys/class/gpio/gpio129/value
echo 1 > /sys/class/gpio/gpio130/value
echo 1 > /sys/class/gpio/gpio127/value

i2cset -y 1 0x10 0x04 0xff00 w; i2cset -y 1 0x10 0x02 0xff03 w
for i in {0..8}; do 
VAL=`i2cget -y 1 0x10 0x40 w`
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

modprobe addicmos fw_load=0 calib_load=0
modprobe imx8_media_dev
modprobe spi_nor
