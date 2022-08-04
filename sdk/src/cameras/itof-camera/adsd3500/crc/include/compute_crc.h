/*!
*****************************************************************************
@file    compute_crc.h

CRC Header file
-----------------------------------------------------------------------------

Copyright (c) 2020 Analog Devices, Inc.

All rights reserved.

Redistribution and use in source and binary forms, with or without modification,
are permitted provided that the following conditions are met:
- Redistributions of source code must retain the above copyright notice,
this list of conditions and the following disclaimer.
- Redistributions in binary form must reproduce the above copyright notice,
this list of conditions and the following disclaimer in the documentation
and/or other materials provided with the distribution.
- Modified versions of the software must be conspicuously marked as such.
- This software is licensed solely and exclusively for use with processors
manufactured by or for Analog Devices, Inc.
- This software may not be combined or merged with other code in any manner
that would cause the software to become subject to terms and conditions
which differ from those listed here.
- Neither the name of Analog Devices, Inc. nor the names of its
contributors may be used to endorse or promote products derived
from this software without specific prior written permission.
- The use of this software may or may not infringe the patent rights of one
or more patent holders.  This license does not release you from the
requirement that you obtain separate licenses from these patent holders
to use this software.

THIS SOFTWARE IS PROVIDED BY ANALOG DEVICES, INC. AND CONTRIBUTORS "AS IS" AND ANY
EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, NON-INFRINGEMENT,
TITLE, MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN
NO EVENT SHALL ANALOG DEVICES, INC. OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT,
INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, PUNITIVE OR CONSEQUENTIAL DAMAGES
(INCLUDING, BUT NOT LIMITED TO, DAMAGES ARISING OUT OF CLAIMS OF INTELLECTUAL
PROPERTY RIGHTS INFRINGEMENT; PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS
OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING
NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE,
EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

*****************************************************************************/

#ifndef __COMPUTE_CRC_H__
#define __COMPUTE_CRC_H__

#if __cplusplus
extern "C" {
#endif

#define IS_CRC_MIRROR (1 << 0)

typedef enum {
    CRC_8bit = 8,
    CRC_16bit = 16,
    CRC_32bit = 32,
} CRC_TYPE;

typedef union {
    uint8_t crc_8bit;
    uint16_t crc_16bit;
    uint32_t crc_32bit;
} crc_output_t;

/* Structure to control the parameters of the algorithm */
typedef struct {
    CRC_TYPE type;
    union {
        uint8_t polynomial_crc8_bit;
        uint16_t polynomial_crc16_bit;
        uint32_t polynomial_crc32_bit;
    } polynomial;

    crc_output_t initial_crc;
    uint8_t crc_compute_flags;

} crc_parameters_t;

extern uint32_t const crc32_table[256];

crc_output_t compute_crc(crc_parameters_t *crc_parameters, uint8_t *data,
                         uint32_t data_len);

#if __cplusplus
}
#endif

#endif
