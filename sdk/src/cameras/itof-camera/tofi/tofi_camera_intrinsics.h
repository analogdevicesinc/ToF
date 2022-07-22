/// Copyright (c) 2021 Analog Devices, Inc. All Rights Reserved.
/// This software is proprietary to Analog Devices, Inc. and its licensors.

#ifndef TOFI_CAMERA_INTRINSICS_H
#define TOFI_CAMERA_INTRINSICS_H

#ifdef __cplusplus
extern "C" {  // only need to export C interface if
              // used by C++ source code
#endif

#define MAX_N_FREQS 3
#define MAX_N_MODES 10
// Structure for the camera intrinsic data
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


// Structure for the CCB data
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
  uint8_t FreqIndex[MAX_N_FREQS];
  uint16_t Freq[MAX_N_FREQS];
  CameraIntrinsics camera_intrinsics;
} TofiXYZDealiasData;

#ifdef __cplusplus
}
#endif

#endif  // TOFI_CAMERA_INTRINSICS_H
