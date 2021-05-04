// Copyright (c) Microsoft Corporation. All rights reserved.
// Modified by Analog Devices, Inc.
//Licensed under the MIT License.
 
#ifndef USB_COMMAND_H
#define USB_COMMAND_H

#include<stdint.h>
#include <stdio.h>
#include <vector>

#ifdef __cplusplus
extern "C" {
#endif

// Get the number of connected devices
bool usb_cmd_get_device_count(size_t* p_device_count, std::vector<std::pair<uint16_t, uint16_t>>* devicePIDVID);


#ifdef __cplusplus
} // namespace
#endif

#endif /* USB_COMMAND_H */
