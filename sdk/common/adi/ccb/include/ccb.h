/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#pragma once
#ifndef CCB_H
#define CCB_H

#ifdef __cplusplus
extern "C" { // only need to export C interface if
             // used by C++ source code
#endif
#ifdef _WIN32
//disable zero-length array warning in MSVC
#pragma warning( push )
#pragma warning( disable : 4200 )
#endif

#include <stdlib.h>
#include "TOF_Calibration_Types.h"

typedef struct {
    const unsigned char *const p;
    const size_t size;
} ccb_data_t;

typedef const struct block__t *const block_t;

const struct CAL_HEADER_BLOCK_V3 *ccb_read_header_block(const ccb_data_t *const ccb_data);
const struct CAL_GEOMETRIC_BLOCK_V3 *ccb_get_cal_block_geometric(const ccb_data_t *const ccb_data, const size_t index);

/*Linked list to contain
all the block of specific types*/
typedef struct {
    void *prev;
    void *block_node;
} mode_block_list;

/*Structure to contain all the
block of specific types
for the specific mode*/
typedef struct {
    uint16_t no_of_blocks;
    mode_block_list *p_block_list_head;
} ccb_mode_data;

ccb_mode_data ccb_get_mode_block_p0(const ccb_data_t *const ccb_data, const uint16_t mode);

#ifdef _WIN32
#pragma warning( pop )
#endif
#ifdef __cplusplus
}
#endif

#endif // CCB_H