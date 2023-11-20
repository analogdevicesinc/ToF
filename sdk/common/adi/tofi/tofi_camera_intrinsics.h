/// Copyright (c) 2021 Analog Devices, Inc. All Rights Reserved.
/// This software is proprietary to Analog Devices, Inc. and its licensors.

#ifndef TOFI_CAMERA_INTRINSICS_H
#define TOFI_CAMERA_INTRINSICS_H

#ifdef __cplusplus
extern "C" {  // only need to export C interface if
              // used by C++ source code
#endif
#define MAX_PATH_SIZE 512
#define MAX_CHAR_SIZE 24
#define MAX_N_FREQS 3
#define MAX_N_MODES 17
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
typedef struct {
  int depthComputeIspEnable;
  int partialDepthEnable;
  int interleavingEnable;
  int bitsInPhaseOrDepth;
  int bitsInAB;
  int bitsInConf;
  int phaseInvalid;
  int withABFrame;
  int littleEndian;
  char inputFormat[MAX_CHAR_SIZE];
} DepthComputeISPParams;
typedef struct {
  char inputFormat[MAX_CHAR_SIZE];
  int deltaCompEnable;
  int headerSize;
  int tempCompEnabled;
} InputRawDataParams;
typedef struct {
  int jblf_apply_flag;
  int jblf_window_size;
  float jblf_gaussian_sigma;
  float jblf_exponential_term;
  float jblf_max_edge;
  float jblf_ab_threshold;
  int ab_filter_enable;
} JBLFConfigParams;
typedef struct {
  float ab_thresh_min;
  float ab_sum_thresh;
} ABThresholdsParams;
typedef struct {
  float conf_thresh;
  float radial_thresh_min;
  float radial_thresh_max;
} DepthRangeParams;
typedef struct {
  int xyz_enable;
  int depth16_enable;
  int ab_only_enable;
  char cl_processor_path[MAX_PATH_SIZE];
} MiscellaneousParams;
typedef enum {
  DepthComputeISP,
  InputRawData,
  JBLFConfig,
  ABThresholds,
  DepthRangeThresholds,
  Miscellaneous
} INIParamsGroup;
#ifdef __cplusplus
}
#endif

#endif  // TOFI_CAMERA_INTRINSICS_H
