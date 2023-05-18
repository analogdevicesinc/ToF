// Copyright (C) Microsoft Corporation. All rights reserved.
// Portions Copyright (c) 2020 Analog Devices, Inc.
// This software is proprietary and confidential to Analog Devices, Inc. and its
// licensors.

#include "tofi_util.h"

#include <math.h>

#include "string.h"

#define MAX_PATH_SIZE 512
#ifdef _WIN32
#include <Windows.h>
#elif __unix
#include <unistd.h>
#endif

#ifdef HEXAGON
#ifndef ION_HEAP_ID_SYSTEM
#define ION_HEAP_ID_SYSTEM 25
#endif
#define rpcmem_alloc(a, b, c) memalign(VLEN * 2, (c))
#endif

uint32_t TransformationXyzToZ(uint32_t n_rows, uint32_t n_cols,
                              const Point3I *p_xyz_image_data,
                              uint16_t *p_zdepth_image_data) {
  if (p_xyz_image_data == NULL || p_zdepth_image_data == NULL || n_rows == 0 ||
      n_cols == 0)
    return ADI_TOFI_NULL_ARGUMENT;

  for (int i = 0; i < n_cols * n_rows; i++) {
    p_zdepth_image_data[i] = p_xyz_image_data[i].c;
  }
  return ADI_TOFI_SUCCESS;
}

uint32_t GetDataFileSize(char *file_name) {
  uint32_t size = 0;
  FILE *fp = fopen(file_name, "rb");
  if (fp != NULL) {
    fseek(fp, 0, SEEK_END);  // seek to end of file
    size = ftell(fp);        // get current file pointer
    rewind(fp);              // seek back to beginning of file
    fclose(fp);
  }
  return size;
}

FileData LoadFileContents(char *filename) {
  const unsigned char *p = NULL;
  size_t size = 0;
  FILE *f = fopen(filename, "rb");

  if (f) {
    if (0 == fseek(f, 0L, SEEK_END)) {
      size = ftell(f);
      rewind(f);
#ifdef HEXAGON
      p = (unsigned char *)rpcmem_alloc(ION_HEAP_ID_SYSTEM,
                                        RPCMEM_DEFAULT_FLAGS, size);
#else
      p = (unsigned char *)malloc(size);
#endif
      if (p) {
        if (1 != fread((void *)p, size, 1, f) || size == 0) {
          fprintf(stderr, "Failed to read data file %s.\n", filename);
          free((void *)p);
          p = 0;
          size = 0;
        }
      } else {
        fprintf(stderr, "Failed to allocate memory for reading data file %s.\n",
                filename);
      }
    } else {
      fprintf(stderr, "Failed to seek in data file %s.\n", filename);
    }
    fclose(f);
  } else {
    fprintf(stderr, "Failed to open data file %s.\n", filename);
  }
  FileData ret_val = {(unsigned char *)p, size};
  return (ret_val);
}

uint32_t WriteDataToFile(char *file_name, uint8_t *buffer,
                         uint32_t buffer_size) {
  uint32_t status = ADI_TOFI_ERROR;

  FILE *fp = fopen(file_name, "wb");

  if (fp != NULL) {
    size_t n_bytes_written = fwrite(buffer, sizeof(uint8_t), buffer_size, fp);
    fclose(fp);

    if (n_bytes_written == buffer_size) {
      status = ADI_TOFI_SUCCESS;
    } else {
      status = ADI_TOFI_FILE_WRITE;
    }
  } else
    status = ADI_TOFI_OPEN_FILE;

  return status;
}

uint32_t GetProcessPath(char *process_path, uint32_t path_size) {
#ifdef _WIN32
  uint32_t status = 0;
  char path[MAX_PATH_SIZE];
  status = GetModuleFileName(NULL, path, path_size);
  if (status == 0 || status == path_size) return ADI_TOFI_ERROR;
  char *last_slash = strrchr(path, '\\');
  strncpy(process_path, path, last_slash - path + 1);
#elif __unix
  uint32_t status = 0;
  char path[MAX_PATH_SIZE];
  status = readlink("/proc/self/exe", path, path_size);
  if (status == 0 || status == path_size) return ADI_TOFI_ERROR;
  char *last_slash = strrchr(path, '/');
  strncpy(process_path, path, last_slash - path + 1);
#endif
  return ADI_TOFI_SUCCESS;
}

uint32_t Gcd(uint32_t a, uint32_t b) {
  if (a == 0) return b;
  return Gcd(b % a, a);
}