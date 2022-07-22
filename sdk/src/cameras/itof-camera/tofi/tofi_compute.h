// Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.
// This software is proprietary to Analog Devices, Inc. and its licensors.

#ifndef TOFI_COMPUTE_H
#define TOFI_COMPUTE_H

#ifdef __cplusplus
extern "C" {  // only need to export C interface if
              // used by C++ source code
#endif

#ifdef _WIN32
#ifdef TOFI_COMPUTE_EXPORTS
#define TOFI_COMPUTE_API __declspec(dllexport)
#else
#define TOFI_COMPUTE_API __declspec(dllimport)
#endif
#else
#define TOFI_COMPUTE_API
#endif

#include <stdint.h>

#include "tofi_error.h"

// TODO: Confirm if its fine to hardcode
// number of phases as 3, else make it a pointer
#define NO_OF_PHASES 3

typedef struct {
  float sensor_temp[NO_OF_PHASES];
  float laser_temp[NO_OF_PHASES];
} TemperatureInfo;

typedef struct {
  uint32_t n_rows;                ///< Number of rows
  uint32_t n_cols;                ///< Number of columns
  uint16_t *p_depth_frame;        ///< Pointer to the Depth Frame
  uint16_t *p_ab_frame;           ///< Pointer to the AB Frame
  float *p_conf_frame;            ///< Pointer to the Confidence Frame
  int16_t *p_xyz_frame;           ///< Pointer to the XYZ Frame
  void *p_tofi_processor_config;  ///< Pointer to the Processor Config
  void *p_cal_config;             ///< Pointer to the Calibration config data
  uint16_t *p_depth16_frame;      ///< Pointer to the Depth16 Frame
} TofiComputeContext;

/// Function to Initialize the configuration for TOFI compute context
/// including memory allocation for parameters like Depth/AB/Confidence
/// memory buffers
/// @param[in] const void *p_tofi_cal_config: pointer to calibration
/// configuration
/// @param[out] uint32_t *p_status: pointer to status, assigned as
///    ADI_TOFI_SUCCESS on success, assigned as error code incase of failure
/// @return TofiComputeContext *: returns pointer to TofiComputeContext on
/// success, returns NULL on failure
TOFI_COMPUTE_API TofiComputeContext *InitTofiCompute(
    const void *p_tofi_cal_config, uint32_t *p_status);

/// Function to process input frame data and output Depth/AB/Confidence frame
/// data.
/// @param[in] const uint16_t *const input_frame: Input frame data(unsigned
/// short format) buffer
/// @param[in] TofiComputeContext *const p_tofi_compute_context: pointer to the
/// TOFI compute context structure
/// @param[in] TemperatureInfo *p_temperature: pointer to the temperature data,
/// temperature correction applied based on the value, no temperature correction
/// is applied if this pointer is NULL
/// @return int: returns ADI_TOFI_SUCCESS(0) on success, returns error code (>1)
/// on failure
TOFI_COMPUTE_API int TofiCompute(
    const uint16_t *const input_frame,
    TofiComputeContext *const p_tofi_compute_context,
    TemperatureInfo *p_temperature);

/// Function to release memory for TOFI compute
/// context structure including its parameters
/// like Depth/AB/Confidence memory buffers
/// @param[in, out] TofiComputeContext *p_tofi_compute_context: pointer to the
/// TOFI compute context structure to be freed
TOFI_COMPUTE_API void FreeTofiCompute(
    TofiComputeContext *p_tofi_compute_context);

/// Function to enable/disable AB only output
/// @param[in] TofiConfig *p_tofi_cal_config: pointer to the TOFI
/// @param[in] int option: '1' for enable & '0' for disable
TOFI_COMPUTE_API void TofiSetABOnly(
    TofiComputeContext *const p_tofi_compute_context, int option);

#ifdef __cplusplus
}
#endif

#endif  // TOFI_COMPUTE_H