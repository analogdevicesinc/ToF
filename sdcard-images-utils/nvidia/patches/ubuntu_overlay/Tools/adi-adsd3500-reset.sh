#!/bin/bash

sudo echo 0 > /sys/class/gpio/PH.06/value

sleep 1

sudo echo 1 > /sys/class/gpio/PH.06/value

