#pragma once
#ifndef TOFI_POINTCLOUD_H
#define TOFI_POINTCLOUD_H

#include "ccb.h"
#include <stdint.h>

#define GEN_XYZ_ITERATIONS 20

typedef struct {
    float fx;
    float fy;
    float cx;
    float cy;
    float codx;
    float cody;
    float k1;
    float k2;
    float k3;
    float k4;
    float k5;
    float k6;
    float p2;
    float p1;
} CameraIntrinsics;

typedef struct {
    int n_rows;
    int n_cols;
    uint8_t n_freqs;
    uint8_t row_bin_factor;
    uint8_t col_bin_factor;
    uint16_t n_offset_rows;
    uint16_t n_offset_cols;
    uint16_t n_sensor_rows;
    uint16_t n_sensor_cols;
    CameraIntrinsics camera_intrinsics;
} TofiCCBData;

typedef struct {
    unsigned char *p_data;
    size_t size;
} FileData;

typedef struct {
    int n_rows;
    int n_cols;
    float *p_z_table; // Size = nRows*nCols
    float *p_x_table;
    float *p_y_table;
} XYZData;

uint32_t GenerateXYZTables(float **pp_x_table, float **pp_y_table,
                           float **pp_z_table, CameraIntrinsics *p_intr_data,
                           uint32_t n_sensor_rows, uint32_t n_sensor_cols,
                           uint32_t n_out_rows, uint32_t n_out_cols,
                           uint32_t n_offset_rows, uint32_t n_offset_cols,
                           uint8_t row_bin_factor, uint8_t col_bin_factor,
                           uint8_t iter);

uint32_t GetCameraIntrinsics(FileData *ccb_data, TofiCCBData *p_ccb_data,
                             uint16_t mode);

void UndistortPoints(float *_srcx, float *_srcy, float *_dstx, float *_dsty,
                     CameraIntrinsics *_cameraMatrix, int maxcount, int rows,
                     int cols, uint8_t row_bin_factor, uint8_t col_bin_factor);

uint32_t ComputeXYZ(uint16_t *p_depth, XYZData *p_xyz_data,
                    int16_t *p_xyz_image);

void FreeXYZTables(float *p_x_table, float *p_y_table, float *p_z_table);

#endif