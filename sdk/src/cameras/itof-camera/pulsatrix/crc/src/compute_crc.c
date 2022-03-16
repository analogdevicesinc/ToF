/*!
*****************************************************************************
@file    compute_crc.c

CRC source file
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

#include <stdint.h>
#include <string.h>
#include <assert.h>

#include "../include/compute_crc.h"


//extern uint32_t const crc32_table;

/**
 * Returns the pointer to CRC table
 */
void*
get_pointer_to_crctable(crc_parameters_t *crc_parameters)
{
    /* Only CRC32 supported for now */
    assert(crc_parameters->type == CRC_32bit);

    if(crc_parameters->type == CRC_32bit) 
        return (void*)crc32_table;

    return NULL;
}

unsigned char generate_mirror(unsigned char value)
{
    unsigned char loopCount = (sizeof(unsigned char) * 8) - 1;
    unsigned char mirrorValue = value;

    value >>= 1;
    while(value)
    {
       mirrorValue <<= 1;
       mirrorValue |= value & 1;
       value >>= 1;
       loopCount--;
    }
    mirrorValue <<= loopCount;

    return mirrorValue;
}

crc_output_t
compute_crc(crc_parameters_t *crc_parameters, uint8_t *data, uint32_t data_len)
{
    crc_output_t temp_value; 
    uint32_t loop_count;
    void *tmp_ptr;

    memcpy(&temp_value, &crc_parameters->initial_crc, sizeof(temp_value));
    
    if(crc_parameters->type == CRC_32bit)
    {
        tmp_ptr = ((uint32_t*)get_pointer_to_crctable(crc_parameters));
        for(loop_count=0; loop_count<data_len; loop_count++)
        {
            if(crc_parameters->crc_compute_flags & IS_CRC_MIRROR)
                temp_value.crc_32bit = 
                    ((uint32_t*)tmp_ptr)[generate_mirror(data[loop_count]) ^ 
                        ((temp_value.crc_32bit >> 0x18) & 0xFF)] ^ 
                            (temp_value.crc_32bit << 8);
        }
    }

    return temp_value;
}
