/** \file ADIypes.h
 * Copyright (c) Microsoft Corporation. All rights reserved.
 * Licensed under the MIT License.
 * Modified by Analog Devices, Inc.
 * Imager Type definitions.
 */

#ifndef ADITYPES_H
#define ADITYPES_H

#ifdef __cplusplus
#include <cinttypes>
#include <cstddef>
#include <cstring>
#else
#include <inttypes.h>
#include <stddef.h>
#include <string.h>
#include <stdbool.h>
#endif

#ifdef __cplusplus
extern "C" {
#endif

	typedef enum {
	/** Depth image type DEPTH16.
	 *
	 * \details
	 * Each pixel of DEPTH16 data is two bytes of little endian unsigned depth data. The unit of the data is in
	 * millimeters from the origin of the camera.
	 *
	 * \details
	 * Stride indicates the length of each line in bytes and should be used to determine the start location of each
	 * line of the image in memory.
	 */
		ADI_IMAGE_FORMAT_DEPTH16 = 0,

	/** Image type IR16.
	 *
	 * \details
	 * Each pixel of IR16 data is two bytes of little endian unsigned depth data. The value of the data represents
	 * brightness.
	 *
	 * \details
	 * This format represents infrared light and is captured by the depth camera.
	 *
	 * \details
	 * Stride indicates the length of each line in bytes and should be used to determine the start location of each
	 * line of the image in memory.
	 */
	 ADI_IMAGE_FORMAT_IR16,

	}ADI_Image_Format_t;

#ifdef __cplusplus
}
#endif

#endif //ADITYPES_H
