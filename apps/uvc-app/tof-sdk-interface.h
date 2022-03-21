#ifndef __TOF_SDK_INTERFACE_H
#define __TOF_SDK_INTERFACE_H

#ifdef __cplusplus
#define EXTERNC extern "C"
#else
#define EXTERNC
#endif

EXTERNC int init_tof_sdk(char* cap_dev_path);
EXTERNC const char* handleClientRequest(const char *buf);

#endif