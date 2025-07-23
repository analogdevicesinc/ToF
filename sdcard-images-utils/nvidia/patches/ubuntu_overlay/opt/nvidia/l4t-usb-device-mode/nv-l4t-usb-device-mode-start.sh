#!/bin/bash

# SPDX-FileCopyrightText: Copyright (c) 2017-2023 NVIDIA CORPORATION & AFFILIATES. All rights reserved.
# SPDX-License-Identifier: BSD-3-Clause
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
# 1. Redistributions of source code must retain the above copyright notice, this
# list of conditions and the following disclaimer.
#
# 2. Redistributions in binary form must reproduce the above copyright notice,
# this list of conditions and the following disclaimer in the documentation
# and/or other materials provided with the distribution.
#
# 3. Neither the name of the copyright holder nor the names of its
# contributors may be used to endorse or promote products derived from
# this software without specific prior written permission.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE
# DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE LIABLE
# FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL
# DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR
# SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
# CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY,
# OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
# OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

set -e

script_dir="$(cd "$(dirname "$0")" && pwd)"
#. "${script_dir}/export_gpio_pins.sh"
. "${script_dir}/nv-l4t-usb-device-mode-config.sh"

modprobe libcomposite

# Wait for any modules to load and initialize
for attempt in $(seq 60); do
    udc_dev=3550000.xudc
    if [ -e "/sys/class/udc/${udc_dev}" ]; then
        break
    fi
    udc_dev=3550000.usb
    if [ -e "/sys/class/udc/${udc_dev}" ]; then
        break
    fi
    udc_dev=""
    sleep 1
done
if [ "${udc_dev}" == "" ]; then
    echo No known UDC device found
    exit 1
fi

macs_file="${script_dir}/mac-addresses"
if [ -f "${macs_file}" ]; then
    . "${macs_file}"
    if ! [[ "${mac_rndis_h}" =~ ^([a-fA-F0-9]{2}:){5}[a-fA-F0-9]{2}$ &&
         "${mac_rndis_d}" =~ ^([a-fA-F0-9]{2}:){5}[a-fA-F0-9]{2}$ &&
         "${mac_ecm_h}" =~ ^([a-fA-F0-9]{2}:){5}[a-fA-F0-9]{2}$ &&
         "${mac_ecm_d}" =~ ^([a-fA-F0-9]{2}:){5}[a-fA-F0-9]{2}$ ]]; then
        rm "${macs_file}"
    fi
fi

if ! [ -f "${macs_file}" ]; then
    # Generate unique data
    if [ -f /proc/device-tree/serial-number ]; then
        random="$(md5sum /proc/device-tree/serial-number|cut -c1-12)"
    else
        random="$(echo "no-serial"|md5sum|cut -c1-12)"
    fi
    # Extract 6 bytes
    b1="$(echo "${random}"|cut -c1-2)"
    b2="$(echo "${random}"|cut -c3-4)"
    b3="$(echo "${random}"|cut -c5-6)"
    b4="$(echo "${random}"|cut -c7-8)"
    b5="$(echo "${random}"|cut -c9-10)"
    b6="$(echo "${random}"|cut -c11-12)"
    # Clear broadcast/multicast, set locally administered bits
    b1="$(printf "%02x" "$(("0x${b1}" & 0xfe | 0x02))")"
    # Set 4 LSBs to unique value per interface
    b6_rndis_h="$(printf "%02x" "$(("0x${b6}" & 0xfc | 0x00))")"
    b6_rndis_d="$(printf "%02x" "$(("0x${b6}" & 0xfc | 0x01))")"
    b6_ecm_h="$(printf "%02x" "$(("0x${b6}" & 0xfc | 0x02))")"
    b6_ecm_d="$(printf "%02x" "$(("0x${b6}" & 0xfc | 0x03))")"
    # Construct complete MAC per interface
    mac_rndis_h="${b1}:${b2}:${b3}:${b4}:${b5}:${b6_rndis_h}"
    mac_rndis_d="${b1}:${b2}:${b3}:${b4}:${b5}:${b6_rndis_d}"
    mac_ecm_h="${b1}:${b2}:${b3}:${b4}:${b5}:${b6_ecm_h}"
    mac_ecm_d="${b1}:${b2}:${b3}:${b4}:${b5}:${b6_ecm_d}"
    # Save values for next boot
    echo "mac_rndis_h=${mac_rndis_h}" > "${macs_file}"
    echo "mac_rndis_d=${mac_rndis_d}" >> "${macs_file}"
    echo "mac_ecm_h=${mac_ecm_h}" >> "${macs_file}"
    echo "mac_ecm_d=${mac_ecm_d}" >> "${macs_file}"
fi

mkdir -p /sys/kernel/config/usb_gadget/l4t
cd /sys/kernel/config/usb_gadget/l4t

# If this script is modified outside NVIDIA, the idVendor and idProduct values
# MUST be replaced with appropriate vendor-specific values.
echo 0x0955 > idVendor
echo 0x7020 > idProduct
# BCD value. Each nibble should be 0..9. 0x1234 represents version 12.3.4.
echo 0x0002 > bcdDevice

# Informs Windows that this device is a composite device, i.e. it implements
# multiple separate protocols/devices.
echo 0xEF > bDeviceClass
echo 0x02 > bDeviceSubClass
echo 0x01 > bDeviceProtocol

mkdir -p strings/0x409
if [ -f /proc/device-tree/serial-number ]; then
    serialnumber="$(cat /proc/device-tree/serial-number|tr -d '\000')"
else
    serialnumber=no-serial
fi
echo "${serialnumber}" > strings/0x409/serialnumber
# If this script is modified outside NVIDIA, the manufacturer and product values
# MUST be replaced with appropriate vendor-specific values.
echo "NVIDIA" > strings/0x409/manufacturer
echo "Linux for Tegra" > strings/0x409/product

cfg=configs/c.1
mkdir -p "${cfg}"
cfg_str=""
is_remote_wakeup=0

# Note: RNDIS must be the first function in the configuration, or Windows'
# RNDIS support will not operate correctly.
if [ ${enable_rndis} -eq 1 ]; then
    cfg_str="${cfg_str}+RNDIS"
    func=functions/rndis.usb0
    mkdir -p "${func}"
    echo "${mac_rndis_h}" > "${func}/host_addr"
    echo "${mac_rndis_d}" > "${func}/dev_addr"
    ln -sf "${func}" "${cfg}"

    # Informs Windows that this device is compatible with the built-in RNDIS
    # driver. This allows automatic driver installation without any need for
    # a .inf file or manual driver selection.
    echo 1 > os_desc/use
    echo 0xcd > os_desc/b_vendor_code
    echo MSFT100 > os_desc/qw_sign
    echo RNDIS > "${func}/os_desc/interface.rndis/compatible_id"
    echo 5162001 > "${func}/os_desc/interface.rndis/sub_compatible_id"
    ln -sf "${cfg}" os_desc

    is_remote_wakeup=1
fi

# If two USB configs are created, and the second contains RNDIS and ACM, then
# Windows will ignore at the ACM function in that config. Consequently, this
# script creates only a single USB config.
if [ ${enable_acm} -eq 1 ]; then
    cfg_str="${cfg_str}+ACM"
    func=functions/acm.GS0
    mkdir -p "${func}"
    ln -sf "${func}" "${cfg}"
fi

# Copy system version information into the exposed filesystem image,
# so that any system that's attached to the USB port can identify this device.
# Do this even if $enable_ums!=1, since $fs_img is locally mounted too.
mntpoint="/mnt/l4t-devmode-$$"
rm -rf "${mntpoint}"
mkdir -p "${mntpoint}"
modprobe loop
mount -o loop "${fs_img}" "${mntpoint}"
rm -rf "${mntpoint}/version"
mkdir -p "${mntpoint}/version"
if [ -f /etc/nv_tegra_release ]; then
    cp /etc/nv_tegra_release "${mntpoint}/version"
fi
if dpkg -s nvidia-l4t-core > /dev/null 2>&1; then
    dpkg -s nvidia-l4t-core > "${mntpoint}/version/nvidia-l4t-core.dpkg-s.txt"
fi
plug_man_dir="/proc/device-tree/chosen/plugin-manager/"
chosen_dir="/proc/device-tree/chosen/"

if [ -d "${plug_man_dir}" ]; then
    cp -r "${plug_man_dir}" "${mntpoint}/version/plugin-manager"
elif [ -d "${chosen_dir}" ]; then
    cp -r "${chosen_dir}" "${mntpoint}/version/chosen"
fi
umount "${mntpoint}"
rm -rf "${mntpoint}"

if [ ${enable_ums} -eq 1 ]; then
    cfg_str="${cfg_str}+UMS"
    func=functions/mass_storage.0
    mkdir -p "${func}"
    ln -sf "${func}" "${cfg}"
    # Prevent users from corrupting the disk image; make it read-only
    echo 1 > "${func}/lun.0/ro"
    echo "${fs_img}" > "${func}/lun.0/file"
fi

if [ ${enable_ecm} -eq 1 ]; then
    cfg_str="${cfg_str}+${ecm_ncm_name}"
    func=functions/${ecm_ncm}.usb0
    mkdir -p "${func}"
    echo "${mac_ecm_h}" > "${func}/host_addr"
    echo "${mac_ecm_d}" > "${func}/dev_addr"
    ln -sf "${func}" "${cfg}"

    is_remote_wakeup=1
fi

if [ ${is_remote_wakeup} -eq 1 ]; then
    echo "0xe0" > "${cfg}/bmAttributes"
else
    echo "0xc0" > "${cfg}/bmAttributes"
fi

mkdir -p "${cfg}/strings/0x409"
# :1 in the variable expansion strips the first character from the value. This
# removes the unwanted leading + sign. This simplifies the logic to construct
# $cfg_str above; it can always add a leading delimiter rather than only doing
# so unless the string is previously empty.
echo "${cfg_str:1}" > "${cfg}/strings/0x409/configuration"

# Create and configure the network bridge before setting the UDC device. This
# ensures that no matter how quickly udev events (which run -runtime-start.sh)
# are triggered after setting the UDC device below, the bridge device is
# guaranteed to exist, so -runtime-start.sh is guaranteed to be able to
# configure it.
#
# Set the device to "down" initially; if/when -runtime-start.sh runs in response
# to cable presence, the interface will be set to "up".
/sbin/brctl addbr l4tbr0
/sbin/ifconfig l4tbr0 down

echo "${udc_dev}" > UDC

# Ethernet devices require additional configuration. This must happen after the
# UDC device is assigned, since that triggers the creation of the Tegra-side
# Ethernet interfaces.
#
# This script always assigns any-and-all Ethernet devices to an Ethernet
# bridge, and assigns the static IP to that bridge. This allows the script to
# more easily handle the potentially variable set of Ethernet devices.
#
# If your custom use-case requires separate IP addresses per interface, or
# only ever has one interface active, you may modify this script to skip
# bridge creation, and assign IP address(es) directly to the interface(s).

if [ ${enable_rndis} -eq 1 ]; then
    /sbin/brctl addif l4tbr0 "$(cat functions/rndis.usb0/ifname)"
    /sbin/ifconfig "$(cat functions/rndis.usb0/ifname)" up
    sudo ip link set l4tbr0 mtu 15000
fi

if [ ${enable_ecm} -eq 1 ]; then
    /sbin/brctl addif l4tbr0 "$(cat functions/${ecm_ncm}.usb0/ifname)"
    /sbin/ifconfig "$(cat functions/${ecm_ncm}.usb0/ifname)" up
fi

cd - # Out of /sys/kernel/config/usb_gadget

# Create a local disk device that exposes the same filesystem image that's
# exported over USB. This will allow local users to see the files too.
/sbin/losetup -f -r "${fs_img}"

# Trigger udev events on any existing cable detection devices, which will
# cause nv-l4t-usb-device-mode-state-change.sh to run and poll for the
# current cable state. This ensures that if relevant udev events were
# emitted before this script started, then device mode is correctly set up.
udevadm trigger -v --action=change --property-match=SUBSYSTEM=android_usb
udevadm trigger -v --action=change --property-match=SUBSYSTEM=usb_role

exit 0
