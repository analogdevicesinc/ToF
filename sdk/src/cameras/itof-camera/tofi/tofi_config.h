/// Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
/// This software is proprietary to Analog Devices, Inc. and its licensors.

#ifndef TOFI_CONFIG_H
#define TOFI_CONFIG_H

#ifdef __cplusplus
extern "C" {  // only need to export C interface if
              // used by C++ source code
#endif

#ifdef _WIN32
#ifdef TOFI_CONFIG_EXPORTS
#define TOFI_CONFIG_API __declspec(dllexport)
#else
#define TOFI_CONFIG_API __declspec(dllimport)
#endif
#else
#define TOFI_CONFIG_API
#endif

#include <TOF_Calibration_Types.h>
#include <stdint.h>
#include <stdlib.h>

#include "tofi_camera_intrinsics.h"
#include "tofi_error.h"

typedef struct ConfigFileData {
  unsigned char *p_data;  ///< Pointer to the data
  size_t size;            ///< Size of the data
} ConfigFileData;

typedef struct XYZTable {
  const float *p_x_table;  ///< Pointer to the radial correction X Table
  const float *p_y_table;  ///< Pointer to the radial correction Y Table
  const float *p_z_table;  ///< Pointer to the radial correction Z Table
} XYZTable;

typedef struct TofiConfig {
  uint32_t n_rows;  ///< Number of rows
  uint32_t n_cols;  ///< Number of Columns
  CameraIntrinsics
      *p_camera_intrinsics;  ///< Pointer to the camera intrinsic parameters
  XYZTable xyz_table;  ///< Structure holding pointer to the X,Y,and Z table
  const struct CAL_LSDAC_BLOCK_V1
      *p_lsdac_block;  ///< Pointer to the LSDAC Block
  const struct CAL_GAIN_CORRECTION_BLOCK
      *p_cal_gain_block;  ///< Pointer to the Gain Block
  const struct CAL_ADDRVAL_REG_BLOCK_V1
      *p_cal_reg_block;           ///< Pointer to the register writes block
  const void *p_tofi_cal_config;  ///< Pointer to the calibration config block
  const char *p_tofi_config_str;  ///< Pointer to a string of ini config data
} TofiConfig;

///
/// @brief Function to Initialize the configuration for TOFI cal config
/// (p_tofi_cal_config) using calibration/config/INI file
/// @param[in] ConfigFileData *p_cal_file_data: Pointer to cal data
/// @param[in] ConfigFileData *p_config_file_data: pointer to json config data,
/// it is initialized to default if NULL
/// @param[in] ConfigFileData *p_ini_file_data: pointer to ini data,
/// it is initialized to default if NULL
/// @param[in] mode - uint16_t - mode of camera operation
/// @param[out] uint32_t p_status: pointer to status, assigned as
/// ADI_TOFI_SUCCESS on success, assigned as error code incase of failure
///
/// @return[out] TofiConfig *: returns p_tofi_config pointer on success,
/// returns NULL on failure
TOFI_CONFIG_API TofiConfig *InitTofiConfig(ConfigFileData *p_cal_file_data,
                                           ConfigFileData *p_config_file_data,
                                           ConfigFileData *p_ini_file_data,
                                           uint16_t mode, uint32_t *p_status);

/// Function to release memory for configuration structure and
/// Depth/AB/Confidence memory buffers
/// @param[in, out] TofiConfig *p_tofi_cal_config: pointer to the TOFI
/// calibration configuration parameter structure to be freed
TOFI_CONFIG_API void FreeTofiConfig(TofiConfig *p_tofi_cal_config);

#ifdef __cplusplus
}
#endif

#endif  // TOFI_CONFIG_H
