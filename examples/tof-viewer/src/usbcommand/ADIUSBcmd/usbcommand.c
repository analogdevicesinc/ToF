// Copyright (c) Microsoft Corporation. All rights reserved.
// Modified by Analog Devices, Inc.
// Licensed under the MIT License.

// This library
#include "usb_cmd_priv.h"

// System dependencies
#include <stdlib.h>
#include <string.h>
#include <stdbool.h>
#include <assert.h>
#include <stdio.h>
#include <vector>

// Ensure we have LIBUSB_API_VERSION defined if not defined by libusb.h
#ifndef LIBUSB_API_VERSION
#define LIBUSB_API_VERSION 0
#endif

/*
Get the number of sensor modules attached
*/
bool usb_cmd_get_device_count(size_t* p_device_count, std::vector<std::pair<uint16_t, uint16_t>>* devicePIDVID)
{
	struct libusb_device_descriptor desc;
	libusb_device** dev_list; // pointer to pointer of device, used to retrieve a list of devices
	ssize_t count;            // holding number of devices in list
	int err;
	if (p_device_count == NULL)
	{
		//LOG_ERROR("Error p_device_count is NULL", 0);
		return false;
	}

	*p_device_count = 0;
	devicePIDVID->clear();
	// initialize library
	if ((err = libusb_init(NULL)) < 0)
	{
		// LOG_ERROR("Error calling libusb_init, result:%s", libusb_error_name(err));
		return false;
	}

#if (LIBUSB_API_VERSION >= 0x01000106)
	libusb_set_option(NULL, LIBUSB_OPTION_LOG_LEVEL, LIBUSB_LOG_LEVEL_WARNING);
#else
	libusb_set_debug(NULL, 3);                       // set verbosity level to 3, as suggested in the documentation
#endif

	count = libusb_get_device_list(NULL, &dev_list); // get the list of devices
	if (count > INT32_MAX)
	{
		//LOG_ERROR("List too large", 0);
		return false;
	}

	if (count == 0)
	{
		//LOG_ERROR("No devices found", 0);
		return false;
	}

	// Loop through and count matching VID / PID
	for (int loop = 0; loop < count; loop++)
	{
		
		if (libusb_get_device_descriptor(dev_list[loop], &desc))
		{
			break;
		}

		//  Just check for one PID assuming the other is in the package
		if (((desc.idVendor == ADI_CCD_VID) && (desc.idProduct == ADI_CCD_PID)) ||
			((desc.idVendor == ADI_FPGA_VID) && (desc.idProduct == ADI_FPGA_PID)) /*||
			((desc.idVendor == FAKE_VID) && (desc.idProduct == FAKE_PID))*/ )
		{
			*p_device_count += 1;
			//.emplace_back(std::make_pair(i, device.get_serialnum()));
			devicePIDVID->emplace_back(std::make_pair(desc.idVendor, desc.idProduct));
		}
	}

	// free the list, unref the devices in it
	libusb_free_device_list(dev_list, (int)count);

	// close the instance
	libusb_exit(NULL);

	return true;
}