// Copyright (c) 2020 Analog Devices, Inc.
// This software is proprietary and confidential to Analog Devices, Inc. and its
// licensors
#ifndef TOFI_ERROR_H
#define TOFI_ERROR_H

/** @enum mapper::TofiStatus
 *  @brief enum class representing the status of TOFI image processing
 */
typedef enum TofiStatus
{
    ADI_TOFI_SUCCESS = 0, ///< 0
    ADI_TOFI_WARNING = 1, ///< 1
    ADI_TOFI_ERROR = 2, ///< 2

    ADI_TOFI_MEM = 3, ///< 3
    ADI_TOFI_MEM_ACCESS = 4, ///< 4
    ADI_TOFI_MEM_ALLOC = 5, ///< 5
    ADI_TOFI_MEM_FREE = 6, ///< 6
    ADI_TOFI_MEM_COPY = 7, ///< 7
    ADI_TOFI_NULL_POINTER = 8, ///< 8
    ADI_TOFI_NULL_ARGUMENT = 9, ///< 9

    ADI_TOFI_FILE_IO = 10, ///< 10
    ADI_TOFI_FILE_NOT_FOUND = 11, ///< 11
    ADI_TOFI_PATH_NOT_FOUND = 12, ///< 12
    ADI_TOFI_OPEN_FILE = 13, ///< 13
    ADI_TOFI_FILE_READ = 14, ///< 14
    ADI_TOFI_FILE_WRITE = 15, ///< 15
    ADI_TOFI_FILE_APPEND = 16, ///< 16
    ADI_TOFI_FILE_PARSE = 17, ///< 17
    ADI_TOFI_FILE_CLOSE = 18, ///< 18

    ADI_TOFI_DEVICE_NOT_FOUND = 19, ///< 19 
    ADI_TOFI_DEVICE_OPEN = 20, ///< 20
    ADI_TOFI_DEVICE_CLOSE = 21,///< 21
    ADI_TOFI_DEVICE_INITIALIZE = 22, ///< 22 
    ADI_TOFI_DEVICE_MEMORY = 23, ///< 23
    ADI_TOFI_DEV_MEM_COPY = 24, ///< 24
    ADI_TOFI_DEVICE_ALLOC = 25, ///< 25
    ADI_TOFI_DEVICE_DEALLOC = 26, ///< 26
    ADI_TOFI_DEVICE_QUERY = 27, ///< 27
    ADI_TOFI_DEVICE_READ = 28, ///< 28
    ADI_TOFI_DEVICE_WRITE = 29, ///< 29
    ADI_TOFI_DEVICE_COMPUTE = 30, ///< 30

    ADI_TOFI_FORMAT_INVALID = 31, ///< 31
    ADI_TOFI_IMAGE_FORMAT_INVALID = 32, ///< 32 
    ADI_TOFI_IMAGE_FORMAT_MISMATCH = 33, ///< 33
    ADI_TOFI_IMAGE_PIXEL_VAL = 34, ///< 34
    ADI_TOFI_IMAGE_PIXEL_INDEX = 35, ///< 35
    ADI_TOFI_IMAGE_METADATA_INVALID = 36, ///< 36 

    ADI_TOFI_CONFIG = 37, ///< 37
    ADI_TOFI_CONFIG_INITIALIZE = 38, ///< 38 
    ADI_TOFI_CONFIG_PARSE = 39, ///< 39
    ADI_TOFI_CONFIG_BLOCK_MISSING = 40, ///< 40 
    ADI_TOFI_CONFIG_COMPUTE = 41, ///< 41
    ADI_TOFI_CONFIG_DUPLICATE = 42, ///< 42
    ADI_TOFI_CONFIG_MISMATCH = 43, ///< 43
    ADI_TOFI_CONFIG_VERSION = 44, ///< 44

    ADI_TOFI_CAL_PARSE = 45, ///< 45
    ADI_TOFI_CAL_FORMAT = 46, ///< 46
    ADI_TOFI_CAL_MISMATCH = 47, ///< 47
    ADI_TOFI_CAL_VERSION = 48, ///< 48
    ADI_TOFI_CAL_DUPLICATE = 49, ///< 49
    ADI_TOFI_CAL_BLOCK_MISSING = 50, ///< 50 
    ADI_TOFI_CAL_MODE_MISSING = 51, ///< 51
    ADI_TOFI_CAL_BLOCK_COMPUTE = 52, ///< 52

    ADI_TOFI_COMPUTE = 53, ///< 53
    ADI_TOFI_DEPTH_COMPUTE = 54, ///< 54 
    ADI_TOFI_AB_COMPUTE = 55, ///< 55
    ADI_TOFI_XYZ_COMPUTE = 56, ///< 56
 
    ADI_TOFI_FILTER_COMPUTE = 57, ///< 57 
    ADI_TOFI_FILTER_SIZE = 58, ///< 58
    ADI_TOFI_FILTER_THRESHOLD = 59, ///< 59 
} TofiStatus;


#endif