// Copyright (C) Microsoft Corporation. All rights reserved.
// Portions Copyright (c) 2020 Analog Devices, Inc.
// This software is proprietary and confidential to Analog Devices, Inc. and its
// licensors.

#ifndef TOFI_UTIL_H
#define TOFI_UTIL_H

#if __cplusplus
extern "C" {
#endif
// Copyright (C) Microsoft Corporation. All rights reserved.

#include <ctype.h>
#include <errno.h>
#include <stdint.h>
#include <stdio.h>
#include <stdlib.h>
#include <string.h>

#include "tofi_error.h"

#define MAX_PHASES 3
#define MAX_FREQUENCIES 3
#define MAX_IMAGE_WIDTH 1024
#define MAX_IMAGE_HEIGHT 1024
#define MAX_KERNEL_SIZE 7
#define CLK_FREQ ((long double)1200000.0)

#ifndef ION_HEAP_ID_SYSTEM
#define ION_HEAP_ID_SYSTEM 25
#endif

#ifdef HEXAGON
#include "hvx.cfg.h"
#include "rpcmem.h"
typedef uint16_t ConfBuf;
#else
typedef float ConfBuf;
#endif

typedef struct {
  float a;
  float b;
} Point2F;

typedef struct {
  int32_t a;
  int32_t b;
} Point2I;

typedef struct {
  int16_t a;
  int16_t b;
  int16_t c;
} Point3I;

typedef struct {
  float a;
  float b;
  float c;
} Point3F;

typedef struct {
  unsigned char *p_data;
  size_t size;
} FileData;

#ifdef BUILD_FIXED
typedef Point2I Point2;
typedef Point3I Point3;
typedef int16_t DataBuffer;
#else
typedef Point2F Point2;
typedef Point3F Point3;
typedef float DataBuffer;
#endif

// Function to Transform XYZ to Z
// Params:
//    uint32_t n_rows (in): Number of rows
//    uint32_t n_cols (in): Number of columns
//    void * p_xyz_image_data(in): pointer to XYZ image data
//    void *p_zdepth_image_data (out): pointer to Z Depth Image
// Returns:
//    uint32_t (out): ADI_TOFI_SUCCESS(0) on success, ADI_TOFI_ERROR(1) on
//    failure
uint32_t TransformationXyzToZ(uint32_t n_rows, uint32_t n_cols,
                              const Point3I *p_xyz_image_data,
                              uint16_t *p_zdepth_image_data);

// Function to return the size of the file
// returns size if success, returns 0 if fails
uint32_t GetDataFileSize(char *file_name);

// Return FileData structure with contents
// to file and size.
FileData LoadFileContents(char *file_name);

// Writes Data to file, returns ADI_TOFI_SUCCESS on success
// returns ADI_TOFI_ERROR on failure
uint32_t WriteDataToFile(char *file_name, uint8_t *buffer,
                         uint32_t buffer_size);
uint32_t GetProcessPath(char *process_path, uint32_t path_size);

uint32_t Gcd(uint32_t a, uint32_t b);

#if __cplusplus
}
#endif

#endif