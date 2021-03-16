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

typedef struct {
  float a;
  float b;

} Point2F;

typedef struct {
  int16_t a;
  int16_t b;

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

typedef struct FileData {
  const uint8_t *const p_data;
  const size_t size;
} FileData;

#ifdef BUILD_FIXED
typedef Point2I Point2;
typedef Point3I Point3;
typedef short DataBuffer;
#else
typedef Point2F Point2;
typedef Point3F Point3;
typedef float DataBuffer;
#endif

uint32_t GetDataFileSize(const char *file_name);
uint32_t LoadFileContents(const char *file_name, uint8_t *buffer,
                          uint32_t *buffer_size);
uint32_t WriteDataToFile(const char *file_name, uint8_t *buffer,
                         uint32_t buffer_size);
uint32_t GetProcessPath(char *process_path, uint32_t path_size);

#if __cplusplus
}
#endif

#endif