/*
MIT License

Copyright (c) 2021 Analog Devices, Inc.

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
*/
#ifndef CONFIGURATION_H
#define CONFIGURATION_H

#include <aditof/status_definitions.h>
#include <stdint.h>

namespace aditof {

/**
 * @struct config_header_struct
 * @brief Describes the properties of common configuration file header for all sensors
 */

typedef struct 
{
    uint16_t FileFormatVersion;
    uint16_t ConfigFileHdrSize;
    uint32_t ConfigFileSize;
    uint32_t nBlocks;
    uint16_t ConfigVerMajor;
    uint16_t ConfigVerMinor;
    uint16_t ChipId;
    uint16_t SystemId;
}config_header_struct;

/**
 * @struct config_struct
 * @brief Describes the file header for all sensor CFG files
 */
typedef struct 
{
    uint8_t FileSignature[4];
    config_header_struct header;
} file_header_struct;


} //namespace aditof

#endif //CONFIGURATION_H
