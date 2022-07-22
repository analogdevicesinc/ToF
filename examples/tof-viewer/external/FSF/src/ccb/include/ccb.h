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

extern block_t BLOCK_ALL;
extern block_t BLOCK_ADDRVAL_REG_BLOCK_V1;
extern block_t BLOCK_FOI_MASK_BLOCK_V0_V1INFO;
extern block_t BLOCK_GEOMETRIC_BLOCK_V3;
extern block_t BLOCK_ILLUM_PROFILE_BLOCK_V2;
extern block_t BLOCK_LSDAC_BLOCK_V1;
extern block_t BLOCK_P0BLOCK_V4;
extern block_t BLOCK_SPATIAL_TEMP_CORR_BLOCK;
extern block_t BLOCK_TEMP_CORR_BLOCK_V0_V1INFO;
extern block_t BLOCK_RELATIVE_ILLUM_BLOCK;
extern block_t BLOCK_GAIN_CORRECTION_BLOCK;

const struct CAL_FILE_HEADER_V1 *ccb_read_header(const ccb_data_t *const ccb_data);
const struct CAL_HEADER_BLOCK_V3 *ccb_read_header_block(const ccb_data_t *const ccb_data);
const struct CAL_BLOCK_INFO_V1 *ccb_get_cal_block_info(const ccb_data_t *const ccb_data, block_t block_id, const size_t index);
const struct CAL_ADDRVAL_REG_BLOCK_V1 *ccb_get_cal_block_addrval_reglist(const ccb_data_t *const ccb_data, const size_t index);
const struct CAL_FOI_MASK_BLOCK_V0_V1INFO *ccb_get_cal_block_foi_mask(const ccb_data_t *const ccb_data, const size_t index);
const struct CAL_GEOMETRIC_BLOCK_V3 *ccb_get_cal_block_geometric(const ccb_data_t *const ccb_data, const size_t index);
const struct CAL_ILLUM_PROFILE_BLOCK_V2 *ccb_get_cal_block_illum_profile(const ccb_data_t *const ccb_data, const size_t index);
const struct CAL_LSDAC_BLOCK_V1 *ccb_get_cal_block_lsdacs(const ccb_data_t *const ccb_data, const size_t index);
const struct CAL_P0BLOCK_V4 *ccb_get_cal_block_p0(const ccb_data_t *const ccb_data, const size_t index);
uint8_t ccb_get_modes_from_blocks_p0(const ccb_data_t *const ccb_data, uint8_t *modes, uint8_t *NumberofModes);
const struct CAL_SPATIAL_TEMP_CORR_BLOCK *ccb_get_cal_block_spatial_tempcor(const ccb_data_t *const ccb_data, const size_t index);
const struct CAL_TEMP_CORR_BLOCK_V0_V1INFO *ccb_get_cal_block_tempcorrection(const ccb_data_t *const ccb_data, const size_t index);
const struct CAL_RELATIVE_ILLUM_BLOCK *ccb_get_cal_block_relative_illum(const ccb_data_t *const ccb_data, const size_t index);
const struct CAL_GAIN_CORRECTION_BLOCK *ccb_get_cal_block_gaincorrection(const ccb_data_t *const ccb_data, const size_t index);

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

typedef struct {
    int num_blocks;
    const struct CAL_ADDRVAL_REG_BLOCK_V1 **addrval_reglists;
    size_t num_addrval_reglists;
    const struct CAL_FOI_MASK_BLOCK_V0_V1INFO **foi_masks;
    size_t num_foi_masks;
    const struct CAL_GEOMETRIC_BLOCK_V3 **geometrics;
    size_t num_geometrics;
    const struct CAL_ILLUM_PROFILE_BLOCK_V2 **illum_profiles;
    size_t num_illum_profiles;
    const struct CAL_LSDAC_BLOCK_V1 **lsdacss;
    size_t num_lsdacss;
    const struct CAL_P0BLOCK_V4 **p0s;
    size_t num_p0s;
    const struct CAL_SPATIAL_TEMP_CORR_BLOCK **spatial_tempcors;
    size_t num_spatial_tempcors;
    const struct CAL_TEMP_CORR_BLOCK_V0_V1INFO **tempcorrections;
    size_t num_tempcorrections;
    const struct CAL_RELATIVE_ILLUM_BLOCK **relative_illums;
    size_t num_relative_illums;
    const struct CAL_GAIN_CORRECTION_BLOCK **gaincorrections;
    size_t num_gaincorrections;
    const struct CAL_BLOCK_INFO_V1 *_storage[];
} ccb_index_t;

const ccb_index_t *ccb_build_cal_block_index(const ccb_data_t *const ccb_data, unsigned char *buffer, const size_t buffer_size);

#ifdef _WIN32
#pragma warning( pop )
#endif
#ifdef __cplusplus
}
#endif

#endif // CCB_H