#!/bin/bash

READY_FILE=/run/usb_gadget_ready

if [ -e /sys/kernel/config/usb_gadget/g1/functions/rndis.0 ] || \
   [ -e /sys/kernel/config/usb_gadget/g1/functions/ecm.0 ]; then
    touch "$READY_FILE"
else
    [ -e "$READY_FILE" ] && rm "$READY_FILE"
fi

