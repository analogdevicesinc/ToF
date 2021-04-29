// Copyright (C) Microsoft Corporation. All rights reserved.
//
//
#ifdef __cplusplus
extern "C"
{
#endif
#include "tofi/tofi_utils.h"
#include "string.h"
#include <tofi/aditof_common.h>

#define MAX_PATH_SIZE 512
#ifdef _WIN32
#include <Windows.h>
#else
#include <unistd.h>
#endif

uint32_t GetDataFileSize(const char *file_name) {
    uint32_t size = 0;
    FILE *fp;
    fopen_s(&fp, file_name, "rb");
    if (fp != NULL) {
        fseek(fp, 0, SEEK_END);  // seek to end of file
        size = ftell(fp);        // get current file pointer
         rewind(fp);             // seek back to beginning of file
         fclose(fp);
    }
    return size;
}

uint32_t LoadFileContents(const char *file_name, uint8_t *buffer,
                          uint32_t *buffer_size) {
    uint32_t status = 0;

    if (buffer != NULL) {
        FILE *fp;
        fopen_s(&fp, file_name, "rb");
        if (fp != NULL) {
            size_t n_bytes_read = fread(buffer, 1, *buffer_size, fp);
            fclose(fp);

            if (n_bytes_read == *buffer_size) {
                status = 1;
            }
        }
    }
    return status;
}

uint32_t WriteDataToFile(const char *file_name, uint8_t *buffer,
                         uint32_t buffer_size) {
    uint32_t status = 0;
    FILE *fp;
    fopen_s(&fp, file_name, "wb");

    if (fp != NULL) {
        size_t n_bytes_written = fwrite(buffer, sizeof(uint8_t), buffer_size, fp);
        fclose(fp);

        if (n_bytes_written == buffer_size) {
            status = 1;
        }
    }

    return status;
}

uint32_t GetProcessPath(char *process_path, uint32_t path_size) {
    uint32_t status = 0;
    char path[MAX_PATH_SIZE];
#ifdef _WIN32
    status = GetModuleFileName(NULL, path, path_size);
    if (status == 0 || status == path_size) {
        return status;
    }
    char *last_slash = strrchr(path, '\\');
#elif defined(unix) || defined(__unix__) || defined(__unix) || \
    (defined(__APPLE__) && defined(__MACH__))
    status = readlink("/proc/self/exe", path, path_size);
    if (status == 0 || status == path_size) {
        return status;
    }
    char *last_slash = strrchr(path, '/');
#endif
    strncpy_s(process_path, (last_slash - path + 1), path, (last_slash - path + 1));
    return 1;
}

#ifdef __cplusplus
}
#endif
