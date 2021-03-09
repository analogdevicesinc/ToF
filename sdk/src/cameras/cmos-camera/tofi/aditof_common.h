/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#ifndef ADITOF_COMMON_H
#define ADITOF_COMMON_H

#ifndef _WIN32
#define fopen_s(pFile,filename,mode) ((*(pFile))=fopen((filename),(mode)))==NULL
#define strncpy_s(dest,destsize,src,count) (strncpy((dest),(src),(count)))==NULL
#endif

#endif //ADITOF_COMMON_H



