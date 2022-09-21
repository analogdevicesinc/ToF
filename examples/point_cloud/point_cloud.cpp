/********************************************************************************/

/*                                                                              */

/* Copyright (c) 2022 Analog Devices, Inc. All Rights Reserved.                 */

/* This software is proprietary to Analog Devices, Inc. and its licensors.      */

/*                                                                              */

/********************************************************************************/


#include "point_cloud.h"
#include <ctype.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include <math.h>



int MapCcbGeometricsCameraInstrinsics(
    const struct CAL_GEOMETRIC_BLOCK_V3* geometrics,
    CameraIntrinsics* camera_intrinsics) {
    if (geometrics != NULL && camera_intrinsics != NULL) {
        camera_intrinsics->fx = geometrics->Fc1;
        camera_intrinsics->fy = geometrics->Fc2;
        camera_intrinsics->cx = geometrics->cc1;
        camera_intrinsics->cy = geometrics->cc2;
        camera_intrinsics->k1 = geometrics->Kc1;
        camera_intrinsics->k2 = geometrics->Kc2;
        camera_intrinsics->k3 = geometrics->Kc3;
        camera_intrinsics->k4 = geometrics->Kc4;
        camera_intrinsics->k5 = geometrics->Kc5;
        camera_intrinsics->k6 = geometrics->Kc6;
        camera_intrinsics->p2 = geometrics->Tx;
        camera_intrinsics->p1 = geometrics->Ty;
        camera_intrinsics->codx = geometrics->Cx;
        camera_intrinsics->cody = geometrics->Cy;
        return 0;
    }
    else
        return -1;
}

uint32_t GetCameraIntrinsics(FileData* ccb_data, TofiCCBData* p_ccb_data, uint16_t mode) {
    if (p_ccb_data == NULL || ccb_data == NULL)
        return -1;
    uint32_t status = 0;

    ccb_mode_data p0_block = ccb_get_mode_block_p0((ccb_data_t*)ccb_data, mode);
    if (p0_block.no_of_blocks == 0 || p0_block.p_block_list_head->block_node
        == NULL) {
        return -1;
    }
    if ((p0_block.no_of_blocks != 0) &&
        (p0_block.p_block_list_head->block_node != NULL)) {
        mode_block_list* p_block_node = p0_block.p_block_list_head;
        const struct CAL_P0BLOCK_V4* p_p0_block =
            (const struct CAL_P0BLOCK_V4*)p0_block.p_block_list_head->block_node;

        p_ccb_data->n_freqs = (uint8_t)p0_block.no_of_blocks;

        p_ccb_data->n_rows = p_p0_block->nRows;

        p_ccb_data->n_cols = p_p0_block->nCols;

        p_ccb_data->row_bin_factor =
            p_p0_block->DigitalBinRows * p_p0_block->AnalogBinRows;

        p_ccb_data->col_bin_factor =
            p_p0_block->DigitalBinCols * p_p0_block->AnalogBinCols;

        p_ccb_data->n_offset_rows = p_p0_block->OffsetRows;

        p_ccb_data->n_offset_cols = p_p0_block->OffsetCols;

        mode_block_list* temp_node = p_block_node;
        p_block_node = (mode_block_list*)p_block_node->prev;
        if (p_block_node != NULL) p_p0_block = (const struct CAL_P0BLOCK_V4*)p_block_node->block_node;
        free(temp_node);
        free(p_block_node);

        const struct CAL_HEADER_BLOCK_V3* header_block =
            ccb_read_header_block((const ccb_data_t*)ccb_data);
        p_ccb_data->n_sensor_rows = header_block ? header_block->nRows : 1024;
        p_ccb_data->n_sensor_cols = header_block ? header_block->nCols : 1024;

        // Read geometric intrinsics
        const struct CAL_GEOMETRIC_BLOCK_V3* geometric;
        geometric = ccb_get_cal_block_geometric((ccb_data_t*)ccb_data, 0);
        if (geometric == NULL) return -1;

        status = MapCcbGeometricsCameraInstrinsics(
            geometric, &(p_ccb_data->camera_intrinsics));
        if (status != 0) return status;
    }

    return status;
}


 uint32_t GenerateXYZTables(float** pp_x_table, float** pp_y_table,
        float** pp_z_table, CameraIntrinsics* p_intr_data,
        uint32_t n_sensor_rows, uint32_t n_sensor_cols,
        uint32_t n_out_rows, uint32_t n_out_cols,
        uint32_t n_offset_rows, uint32_t n_offset_cols,
        uint8_t row_bin_factor, uint8_t col_bin_factor,
        uint8_t iter)
{
    uint32_t n_cols = n_sensor_cols / col_bin_factor;
    uint32_t n_rows = n_sensor_rows / row_bin_factor;

    float* p_xp = (float*)malloc(n_rows * n_cols * sizeof(float));
    float* p_yp = (float*)malloc(n_rows * n_cols * sizeof(float));
    float* p_z = (float*)malloc(n_rows * n_cols * sizeof(float));


    if ((p_xp == NULL) || (p_yp == NULL) || ((p_z == NULL))) {
        if (p_xp) free(p_xp);
        if (p_yp) free(p_yp);
        if (p_z) free(p_z);
        return -1;
    }


    // Adjust values based on optical center and focal length
    float cx = p_intr_data->cx / row_bin_factor;
    float cy = p_intr_data->cy / col_bin_factor;
    float fx = p_intr_data->fx / row_bin_factor;
    float fy = p_intr_data->fy / col_bin_factor;
    // float codx = p_intr_data->codx;
     //float cody = p_intr_data->cody;

   
    float r_min = sqrt((float)(n_rows * n_rows + n_cols * n_cols));

    // Generate the initial x,y tables using the positional
  // index and crop the unused pixels from the maximum in
  // each dimension
    for (uint32_t i = 0; i < n_cols; i++) {
        // Each value in a row increments by one
        p_xp[i] = (float)i;
    }
    // Replicate the rows
    for (uint32_t j = 0; j < n_rows; j++) {
        memcpy(&p_xp[j * n_cols], p_xp, n_cols * sizeof(float));
    }

    for (uint32_t j = 0; j < n_rows; j++) {
        // Each row is one more than the last
        float value = (float)j;
        for (uint32_t i = 0; i < n_cols; i++) {
            // Every value in a row is the same
            p_yp[j * n_cols + i] = value;
        }
    }

    UndistortPoints(p_xp, p_yp, p_xp, p_yp, p_intr_data, iter, n_rows, n_cols,
        row_bin_factor, col_bin_factor);

    for (uint32_t j = 0; j < n_rows; j++) {
        for (uint32_t i = 0; i < n_cols; i++) {
            int idx = j * n_cols + i;
            float xp = p_xp[idx];
            float yp = p_yp[idx];
            p_z[idx] = sqrtf(xp * xp + yp * yp + 1);
            //Check for invalid values
            if (isnan(xp) || isnan(yp) || isnan(p_z[idx]) || p_z[idx] == 0) {
                // Calculate the coordinates relative to the center pixel
                float ix = (float)i - cx;
                float iy = (float)j - cy;
                float r = sqrt(ix * ix + iy * iy);
                // Find the minimum radius with an invalid number
                if (r < r_min) {
                    r_min = r;
                }
            }
        }
    }
    //Add a 2 pixel buffer
    r_min -= 2;

    //Filter for invalid pixels
    for (uint32_t j = 0; j < n_rows; j++) {
        for (uint32_t i = 0; i < n_cols; i++) {
            int idx = j * n_cols + i;
            float ix = (float)i - cx;
            float iy = (float)j - cy;
            float r = sqrt(ix * ix + iy * iy);
            if (r >= r_min) {
                // zero if the pixel is outside the valid radius
                p_xp[idx] = 0;
                p_yp[idx] = 0;
                p_z[idx] = 0;
            }
        }
    }

    // Add a 2 pixel buffer
    r_min -= 2;


    float* p_xfull = p_xp;
    float* p_yfull = p_yp;
    float* p_zfull = p_z;



    p_xp = (float*)malloc(n_out_rows * n_out_cols * sizeof(float));
    p_yp = (float*)malloc(n_out_rows * n_out_cols * sizeof(float));
    p_z = (float*)malloc(n_out_rows * n_out_cols * sizeof(float));


    for (uint32_t j = 0; j < n_out_rows; j++) {
        for (uint32_t i = 0; i < n_out_cols; i++) {
            int idx = (j + n_offset_rows) * n_cols + i + n_offset_cols;
            int crop_idx = j * n_out_cols + i;
            float x = p_xfull[idx];
            float y = p_yfull[idx];
            float z = p_zfull[idx];
            if (z != 0) {
                p_xp[crop_idx] = x / z;
                p_yp[crop_idx] = y / z;
                p_z[crop_idx] = 1 / z;
            }
        }
    }  

    free(p_xfull);
    free(p_yfull);
    free(p_zfull);

    // Set the config pointers to the new buffers
    *pp_x_table = p_xp;
    *pp_y_table = p_yp;
    *pp_z_table = p_z;

    
    return 0;
}


uint32_t ComputeXYZ(uint16_t* p_depth, XYZData* p_xyz_data,
    int16_t* p_xyz_image) {

  
    const uint32_t n_rows = p_xyz_data->n_rows;
    const uint32_t n_cols = p_xyz_data->n_cols;
    
        for (uint32_t pixel_id = 0; pixel_id < n_rows * n_cols; pixel_id++) {
            p_xyz_image[3 * pixel_id + 0] = (int16_t)(floorf(
                p_xyz_data->p_x_table[pixel_id] * (float)p_depth[pixel_id] +
                0.5f));

            p_xyz_image[3 * pixel_id + 1] = (int16_t)(floorf(
                p_xyz_data->p_y_table[pixel_id] * (float)p_depth[pixel_id] +
                0.5f));

            p_xyz_image[3 * pixel_id + 2] =
                (int16_t)((p_xyz_data->p_z_table[pixel_id] *
                    (float)p_depth[pixel_id] +
                    0.5f));

        }
    


    return 0;
}


void FreeXYZTables(float* p_x_table, float* p_y_table, float* p_z_table) {
    if (p_x_table) {

        free(p_x_table);
        p_x_table = NULL;
    }
    if (p_y_table) {

        free(p_y_table);
        p_y_table = NULL;
    }
    if (p_z_table) {

        free(p_z_table);
        p_z_table = NULL;
    }
}