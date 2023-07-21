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
#ifndef ALGORITHMS_H
#define ALGORITHMS_H

#include <cstdint>

#include "aditof/sdk_exports.h"
#include "tofi_camera_intrinsics.h"
#include "tofi_config.h"

class Algorithms {
  public:
    static uint32_t GenerateXYZTables(
        const float **pp_x_table, const float **pp_y_table,
        const float **pp_z_table, CameraIntrinsics *p_intr_data,
        uint32_t n_sensor_rows, uint32_t n_sensor_cols, uint32_t n_out_rows,
        uint32_t n_out_cols, uint32_t n_offset_rows, uint32_t n_offset_cols,
        uint8_t row_bin_factor, uint8_t col_bin_factor, uint8_t iter);

    static uint32_t ComputeXYZ(const uint16_t *p_depth, XYZTable *p_xyz_data,
                               int16_t *p_xyz_image, uint32_t n_rows,
                               uint32_t n_cols);
};

#endif // ALGORITHMS_H
