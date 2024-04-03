// Copyright (c) 2022 Analog Devices, Inc-> All Rights Reserved.
// This software is proprietary to Analog Devices, Inc. and its licensors.

#include <cstdint>
#include <iostream>
#include <math.h>
#include <string.h>

#include "algorithms.h"
#include "tofi_compute.h"
#include "tofi_error.h"

#define GEN_XYZ_ITERATIONS 20

typedef struct {
    int n_depth;
    int n_ab;
    int n_conf;
    XYZTable xyz_table;
} PrivateData;

TofiComputeContext *InitTofiCompute(const void *p_tofi_cal_config,
                                    uint32_t *p_status) {
    TofiComputeContext *Obj = new TofiComputeContext;
    PrivateData *privDataObj = new PrivateData;

    // Extract the number of bits for: Depth, AB, Confidence
    TofiXYZDealiasData *ccb_data = (TofiXYZDealiasData *)p_tofi_cal_config;
    uint16_t bits = ccb_data->Freq[0];
    privDataObj->n_depth = bits & 0x001F;
    privDataObj->n_ab = (bits & 0x03E0) >> 5;
    privDataObj->n_conf = (bits & 0x3C00) >> 10;

    // Generate the X, Y, Z tables
    memset(&privDataObj->xyz_table, 0, sizeof(privDataObj->xyz_table));

    int n_cols = ccb_data->n_cols;
    int n_rows = ccb_data->n_rows;

    int status = Algorithms::GenerateXYZTables(
        &privDataObj->xyz_table.p_x_table, &privDataObj->xyz_table.p_y_table,
        &privDataObj->xyz_table.p_z_table, &(ccb_data->camera_intrinsics),
        ccb_data->n_sensor_rows, ccb_data->n_sensor_cols, n_rows, n_cols,
        ccb_data->n_offset_rows, ccb_data->n_offset_cols,
        ccb_data->row_bin_factor, ccb_data->col_bin_factor, GEN_XYZ_ITERATIONS);
    if (status != 0 || !privDataObj->xyz_table.p_x_table ||
        !privDataObj->xyz_table.p_y_table || !privDataObj->xyz_table.p_z_table)
        return nullptr;

    // Set context
    Obj->n_cols = 0;
    Obj->n_rows = 0;
    Obj->p_ab_frame = 0;
    Obj->p_cal_config = (void *)p_tofi_cal_config;
    Obj->p_conf_frame = 0;
    Obj->p_depth16_frame = 0;
    Obj->p_depth_frame = 0;
    Obj->p_tofi_processor_config = (void *)privDataObj;
    Obj->p_xyz_frame = 0;
    return Obj;
};

#define NUM_BITS(Input, n_pos, n_bits) (((1 << n_bits) - 1) & (Input >> n_pos))

static uint32_t
DeInterleaveDepth(uint8_t *p_frame_data, uint32_t n_bits_in_depth,
                  uint32_t n_bits_in_conf, uint32_t n_bits_in_ab,
                  uint32_t n_bytes, uint32_t width, uint32_t height,
                  uint16_t *p_depth, uint16_t *p_conf, uint16_t *p_ab) {
    uint8_t *input_buffer = p_frame_data;

    uint16_t *out_depth = p_depth;
    uint16_t *out_conf = p_conf;
    uint16_t *out_ab = p_ab;

    uint32_t n_pos_conf = (16 - n_bits_in_depth) ? 16 - n_bits_in_depth : 8;
    uint32_t n_depth_conf = n_bits_in_depth + n_bits_in_conf;
    uint32_t div = n_depth_conf % 8;
    uint32_t n_count_conf = n_bits_in_ab ? n_depth_conf / 8 : 0;
    uint32_t n_pos_ab = div ? 4 : 0;
    uint32_t is_conf = n_depth_conf == 16 ? 0 : 2;
    uint32_t n_ab_count = n_bits_in_ab == 8 ? 0 : n_count_conf + 1;

    uint32_t n_pixels = width * height;

    for (uint32_t pix_id = 0; pix_id < n_pixels; pix_id++) {
        input_buffer = p_frame_data + pix_id * n_bytes;

        uint16_t temp = input_buffer[0] | (uint16_t)(input_buffer[1] << 8);
        out_depth[pix_id] = NUM_BITS(temp, 0, n_bits_in_depth);

        if (out_conf) {
            temp = input_buffer[1] | (uint16_t)(input_buffer[is_conf] << 8);
            out_conf[pix_id] = NUM_BITS(temp, n_pos_conf, n_bits_in_conf);
        }

        if (out_ab) {
            temp = input_buffer[n_count_conf] |
                   (uint16_t)(input_buffer[n_ab_count] << 8);
            out_ab[pix_id] = NUM_BITS(temp, n_pos_ab, n_bits_in_ab);
        }
    }
    return 0;
}

int TofiCompute(const uint16_t *const input_frame,
                TofiComputeContext *const p_tofi_compute_context,
                TemperatureInfo *p_temperature) {

    TofiXYZDealiasData *ccb_data =
        (TofiXYZDealiasData *)p_tofi_compute_context->p_cal_config;
    int n_cols = ccb_data->n_cols;
    int n_rows = ccb_data->n_rows;

    PrivateData *p =
        (PrivateData *)p_tofi_compute_context->p_tofi_processor_config;
    int n_depth = p->n_depth;
    int n_ab = p->n_ab;
    int n_conf = p->n_conf;
    int n_sum_bits = n_depth + n_conf + n_ab;
    int n_bytes = n_sum_bits / 8;

    int status = DeInterleaveDepth(
        (uint8_t *)input_frame, n_depth, n_conf, n_ab, n_bytes, n_cols, n_rows,
        p_tofi_compute_context->p_depth_frame,
        (uint16_t *)p_tofi_compute_context->p_conf_frame,
        p_tofi_compute_context->p_ab_frame);

    // Compute Point cloud
    Algorithms::ComputeXYZ(p_tofi_compute_context->p_depth_frame, &p->xyz_table,
                           p_tofi_compute_context->p_xyz_frame, n_rows, n_cols);

    if (status != 0) {
        std::cout << "Unable to compute XYZ !" << std::endl;
    }

    return 0;
};

void FreeTofiCompute(TofiComputeContext *p_tofi_compute_context) {
    PrivateData *p =
        (PrivateData *)p_tofi_compute_context->p_tofi_processor_config;
    // Free tabels x, y, z
    if (p->xyz_table.p_x_table) {
        free((void *)p->xyz_table.p_x_table);
    }
    if (p->xyz_table.p_y_table) {
        free((void *)p->xyz_table.p_y_table);
    }
    if (p->xyz_table.p_z_table) {
        free((void *)p->xyz_table.p_z_table);
    }
    delete p;
    delete p_tofi_compute_context;
};
