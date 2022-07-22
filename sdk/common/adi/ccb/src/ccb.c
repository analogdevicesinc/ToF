/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#include <ccb.h>

enum index_kind_t { ALL, BLOCK_ID };

struct block__t {
    enum index_kind_t kind;
    uint8_t block_id;
};

const struct block__t BLOCK_TYPE_ALL = {.kind = ALL};
block_t BLOCK_ALL = &BLOCK_TYPE_ALL;

#define DEFINE_BLOCK_TYPE(name, block_type)                                                                                                                    \
    const struct block__t BLOCK_S_##name = {.kind = BLOCK_ID, .block_id = CAL_BLOCK_ID_##block_type};                                                          \
    block_t BLOCK_##name = &BLOCK_S_##name;

DEFINE_BLOCK_TYPE(ADDRVAL_REG_BLOCK_V1, ADDRVAL_REGLIST)
DEFINE_BLOCK_TYPE(FOI_MASK_BLOCK_V0_V1INFO, FOI_MASK)
DEFINE_BLOCK_TYPE(GEOMETRIC_BLOCK_V3, GEOMETRIC)
DEFINE_BLOCK_TYPE(ILLUM_PROFILE_BLOCK_V2, ILLUM_PROFILE)
DEFINE_BLOCK_TYPE(LSDAC_BLOCK_V1, LSDACS)
DEFINE_BLOCK_TYPE(P0BLOCK_V4, P0)
DEFINE_BLOCK_TYPE(SPATIAL_TEMP_CORR_BLOCK, SPATIAL_TEMPCOR)
DEFINE_BLOCK_TYPE(TEMP_CORR_BLOCK_V0_V1INFO, TEMPCORRECTION)
DEFINE_BLOCK_TYPE(RELATIVE_ILLUM_BLOCK, RELATIVE_ILLUM)
DEFINE_BLOCK_TYPE(GAIN_CORRECTION_BLOCK, GAINCORRECTION)

int block_id_matches(uint8_t ccb_block_id, block_t block_id) { return block_id->kind == ALL || ccb_block_id == block_id->block_id; }

const struct CAL_FILE_HEADER_V1 *ccb_read_header(const ccb_data_t *const ccb_data) {
    const unsigned char *p = ccb_data->p + 0;
    size_t required_size = sizeof(struct CAL_FILE_HEADER_V1);
    if ((size_t)(p + required_size - ccb_data->p) >= ccb_data->size) {
        return 0;
    } else {
        return (const struct CAL_FILE_HEADER_V1 *)p;
    }
}

const struct CAL_HEADER_BLOCK_V3 *ccb_read_header_block(const ccb_data_t *const ccb_data) {
    const unsigned char *p = ccb_data->p + sizeof(struct CAL_FILE_HEADER_V1);
    size_t required_size = sizeof(struct CAL_HEADER_BLOCK_V3);
    if ((size_t)(p + required_size - ccb_data->p) >= ccb_data->size) {
        return 0;
    } else {
        return (const struct CAL_HEADER_BLOCK_V3 *)p;
    }
}

const struct CAL_BLOCK_INFO_V1 *ccb_get_cal_block_info(const ccb_data_t *const ccb_data, block_t block_id, const size_t index) {
    const struct CAL_FILE_HEADER_V1 *header = ccb_read_header(ccb_data);
    if (!header) {
        return NULL;
    }
    const struct CAL_HEADER_BLOCK_V3 *header_block = ccb_read_header_block(ccb_data);
    if (!header_block) {
        return NULL;
    }
    const unsigned char *p = ccb_data->p + sizeof(struct CAL_FILE_HEADER_V1) + sizeof(struct CAL_HEADER_BLOCK_V3);
    size_t filter_count = 0;
    for (uint16_t block_index = 0; block_index < header_block->nBlocks; block_index++) {
        size_t required_size = sizeof(struct CAL_BLOCK_INFO_V1);
        if ((size_t)(p + required_size - ccb_data->p) >= ccb_data->size) {
            return NULL;
        }
        const struct CAL_BLOCK_INFO_V1 *cal_block_info = (const struct CAL_BLOCK_INFO_V1 *)p;
        if (block_id_matches(cal_block_info->BlockID, block_id)) {
            if (filter_count == index) {
                return (const struct CAL_BLOCK_INFO_V1 *)p;
            }
            filter_count += 1;
        }
        p += cal_block_info->BlockSize;
    }
    return NULL;
}

#define DEFINE_GET_CAL_BLOCK(block_type, block)                                                                                                                \
    const struct CAL_##block_type *ccb_get_cal_block_##block(const ccb_data_t *const ccb_data, const size_t index) {                                           \
        return (const struct CAL_##block_type *)ccb_get_cal_block_info(ccb_data, BLOCK_##block_type, index);                                                   \
    }

DEFINE_GET_CAL_BLOCK(ADDRVAL_REG_BLOCK_V1, addrval_reglist)
DEFINE_GET_CAL_BLOCK(FOI_MASK_BLOCK_V0_V1INFO, foi_mask)
DEFINE_GET_CAL_BLOCK(GEOMETRIC_BLOCK_V3, geometric)
DEFINE_GET_CAL_BLOCK(ILLUM_PROFILE_BLOCK_V2, illum_profile)
DEFINE_GET_CAL_BLOCK(LSDAC_BLOCK_V1, lsdacs)
DEFINE_GET_CAL_BLOCK(P0BLOCK_V4, p0)
DEFINE_GET_CAL_BLOCK(SPATIAL_TEMP_CORR_BLOCK, spatial_tempcor)
DEFINE_GET_CAL_BLOCK(TEMP_CORR_BLOCK_V0_V1INFO, tempcorrection)
DEFINE_GET_CAL_BLOCK(RELATIVE_ILLUM_BLOCK, relative_illum)
DEFINE_GET_CAL_BLOCK(GAIN_CORRECTION_BLOCK, gaincorrection)

uint8_t ccb_get_available_modes(const ccb_data_t *const ccb_data, block_t block_id, uint8_t *modes, uint8_t *NumberofModes) {
    const struct CAL_FILE_HEADER_V1 *header = ccb_read_header(ccb_data);
    if (!header) {
        return 0;
    }
    const struct CAL_HEADER_BLOCK_V3 *header_block = ccb_read_header_block(ccb_data);
    if (!header_block) {
        return 0;
    }
    const unsigned char *p = ccb_data->p + sizeof(struct CAL_FILE_HEADER_V1) + sizeof(struct CAL_HEADER_BLOCK_V3);

    for (uint16_t block_index = 0; block_index < header_block->nBlocks; block_index++) {
        size_t required_size = sizeof(struct CAL_BLOCK_INFO_V1);
        if ((size_t)(p + required_size - ccb_data->p) >= ccb_data->size) {
            return 0;
        }
        const struct CAL_BLOCK_INFO_V1 *cal_block_info = (const struct CAL_BLOCK_INFO_V1 *)p;
        if (block_id_matches(cal_block_info->BlockID, block_id)) {

            // Get the mode from the p0 blocks, each mode will have different number of frequencies, depending 
            // on the # of number frequencies there will be corresponding # of p0 block entries
            const struct CAL_P0BLOCK_V4 *cal_mode_info = (const struct CAL_P0BLOCK_V4 *)p;
            if ((modes != NULL) && (modes[*NumberofModes] != cal_mode_info->Mode)) {
                *modes = cal_mode_info->Mode;
                modes++;
                (*NumberofModes)++;
            }
        }
        p += cal_block_info->BlockSize;
    }
    return 1;
}

#define DEFINE_GET_MODES(block_type, block)                                                                                                   \
    uint8_t ccb_get_modes_from_blocks_##block(const ccb_data_t *const ccb_data, uint8_t *modes, uint8_t *NumberofModes) {                                             \
        return ccb_get_available_modes(ccb_data, BLOCK_##block_type, modes, NumberofModes);                                                                  \
    }

DEFINE_GET_MODES(P0BLOCK_V4, p0)

ccb_mode_data ccb_get_mode_block_info(const ccb_data_t *const ccb_data, block_t block_id, const uint16_t mode) {
    ccb_mode_data p_mode_data = {0, NULL};
    const struct CAL_FILE_HEADER_V1 *header = ccb_read_header(ccb_data);
    if (!header) {
        return (ccb_mode_data){0, NULL};
    }
    const struct CAL_HEADER_BLOCK_V3 *header_block = ccb_read_header_block(ccb_data);
    if (!header_block) {
        return (ccb_mode_data){0, NULL};
    }
    const unsigned char *p = ccb_data->p + sizeof(struct CAL_FILE_HEADER_V1) + sizeof(struct CAL_HEADER_BLOCK_V3);

    for (uint16_t block_index = 0; block_index < header_block->nBlocks; block_index++) {
        size_t required_size = sizeof(struct CAL_BLOCK_INFO_V1);
        if ((size_t)(p + required_size - ccb_data->p) >= ccb_data->size) {
            return (ccb_mode_data){0, NULL};
        }
        const struct CAL_BLOCK_INFO_V1 *cal_block_info = (const struct CAL_BLOCK_INFO_V1 *)p;
        if (block_id_matches(cal_block_info->BlockID, block_id)) {
            const struct CAL_P0BLOCK_V4 *cal_mode_info = (const struct CAL_P0BLOCK_V4 *)p;
            if (cal_mode_info->Mode == mode) {
                mode_block_list *p_new_block_node = (mode_block_list *)calloc(1, sizeof(mode_block_list));
                if (p_new_block_node == NULL) {
                    return (ccb_mode_data){0, NULL};
                }
                p_new_block_node->prev = p_mode_data.p_block_list_head;
                p_new_block_node->block_node = (void *)p;
                p_mode_data.no_of_blocks++;
                p_mode_data.p_block_list_head = p_new_block_node;
            }
        }
        p += cal_block_info->BlockSize;
    }
    return p_mode_data;
}

#define DEFINE_GET_MODE_BLOCK(block_type, block)                                                                                                               \
    ccb_mode_data ccb_get_mode_block_##block(const ccb_data_t *const ccb_data, const uint16_t mode) {                                                          \
        return ccb_get_mode_block_info(ccb_data, BLOCK_##block_type, mode);                                                                                    \
    }
DEFINE_GET_MODE_BLOCK(P0BLOCK_V4, p0)

const struct CAL_BLOCK_INFO_V1 **index_cal_blocks(const ccb_data_t *ccb_data, block_t block_id, int *ccb_index_num_blocks,
                                                  int *const complete, size_t *const available, size_t *const count,
                                                  const struct CAL_BLOCK_INFO_V1 ***const ppp_block) {
    const struct CAL_BLOCK_INFO_V1 **result = *ppp_block;
    *count = 0;
    const struct CAL_BLOCK_INFO_V1 *p_block;
    for (int i = 0; (p_block = ccb_get_cal_block_info(ccb_data, block_id, i)); i++) {
        *ccb_index_num_blocks += 1;
        if (*available > 0) {
            const struct CAL_BLOCK_INFO_V1 **pp_block = *ppp_block;
            *pp_block = p_block;
            *ppp_block += 1;
            *count += 1;
            *available -= 1;
        } else {
            *complete = 0;
        }
    }
    return result;
}

#define INDEX_BLOCKS(block_type, block)                                                                                                                        \
    ccb_index->block = (const struct CAL_##block_type **)index_cal_blocks(ccb_data, BLOCK_##block_type, &ccb_index->num_blocks, &complete, &available,         \
                                                                          &ccb_index->num_##block, &pp_block)

const ccb_index_t *ccb_build_cal_block_index(const ccb_data_t *const ccb_data, unsigned char *buffer, const size_t buffer_size) {
    if (buffer_size < sizeof(ccb_index_t)) {
        return 0;
    }
    ccb_index_t *ccb_index = (ccb_index_t *)buffer;
    *ccb_index = (ccb_index_t){0};
    int complete = 1;
    const struct CAL_BLOCK_INFO_V1 **pp_block = (const struct CAL_BLOCK_INFO_V1 **)ccb_index->_storage;
    size_t available = (buffer_size - sizeof(ccb_index_t)) / sizeof(const struct CAL_BLOCK_INFO_V1 *);
    INDEX_BLOCKS(ADDRVAL_REG_BLOCK_V1, addrval_reglists);
    INDEX_BLOCKS(FOI_MASK_BLOCK_V0_V1INFO, foi_masks);
    INDEX_BLOCKS(GEOMETRIC_BLOCK_V3, geometrics);
    INDEX_BLOCKS(ILLUM_PROFILE_BLOCK_V2, illum_profiles);
    INDEX_BLOCKS(LSDAC_BLOCK_V1, lsdacss);
    INDEX_BLOCKS(P0BLOCK_V4, p0s);
    INDEX_BLOCKS(SPATIAL_TEMP_CORR_BLOCK, spatial_tempcors);
    INDEX_BLOCKS(TEMP_CORR_BLOCK_V0_V1INFO, tempcorrections);
    INDEX_BLOCKS(RELATIVE_ILLUM_BLOCK, relative_illums);
    INDEX_BLOCKS(GAIN_CORRECTION_BLOCK, gaincorrections);
    if (!complete) {
        ccb_index->num_blocks = -ccb_index->num_blocks;
    }
    return ccb_index;
}