//-----------------------------------------------------------------------------
// Copyright (c) 2000 by Michael Barr.  This software is placed into
// the public domain and may be used for any purpose.  However, this
// notice must not be changed or removed and no warranty is either
// expressed or implied by its publication or distribution.
// https://barrgroup.com/downloads/code-crc-c
//
// Portions Copyright (c) 2020 Analog Devices, Inc.

#ifndef _CRC_H_
#define _CRC_H_
#ifdef __cplusplus
extern "C" {
#endif /* __cplusplus */

#include <stdint.h>

//Calculates a CRC of a block of memory using same algorithm as in the fuse table.
uint32_t crcFast(uint8_t const message[], int nBytes);

#ifdef __cplusplus
}
#endif /* __cplusplus */
#endif //_CRC_H_