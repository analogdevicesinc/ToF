#ifndef __TOF_SDK_INTERFACE_H
#define __TOF_SDK_INTERFACE_H

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

#include <stdio.h>

EXTERNC int init_tof_sdk(char *cap_dev_path);
EXTERNC void handleClientRequest(const char *in_buf, const size_t in_len,
                                 char **out_buf, size_t *out_len);

#endif
