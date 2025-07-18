#!/bin/bash

if [ -e /sys/kernel/config/usb_gadget/g1/functions/rndis.0 ] || \
   [ -e /sys/kernel/config/usb_gadget/g1/functions/ecm.0 ]; then
    exec /usr/share/systemd/aditof-server
else
    echo "No supported USB gadget function found. Exiting."
    exit 1
fi

