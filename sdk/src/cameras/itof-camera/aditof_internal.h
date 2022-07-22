/********************************************************************************/
/*                                                                              */
/* Copyright (c) 2020 Analog Devices, Inc. All Rights Reserved.                 */
/* This software is proprietary to Analog Devices, Inc. and its licensors.      */
/*                                                                              */
/********************************************************************************/
#ifndef ADITOF_INTERNAL_H
#define ADITOF_INTERNAL_H

#include <stdint.h>

/**
 * @enum Integer defines used internally to configure the FPGA registers, power monitor registers, UVC driver, camera registers, AFE uSeq indirect access  
 * @brief Maintains the definition of FPGA registers, power monitor registers, UVC driver, camera registers, AFE uSeq indirect access  
 */
enum : uint16_t {
    EMBED_HDR_LENGTH                = 128,
    CHIPID                          = 0x5931,
    REG_CAPTURE_ID_LOC              = 49,
    REG_MODE_ID_CURR_LOC            = 48,
    FRAME_WIDTH_LOC                 = 12,
    FRAME_HEIGHT_LOC                = 14,
    FRAME_NUM_LOC                   =  8,
    CHIPID_LOC                      =  0,
    SET_MODE_REG_ADDR               = 0x0200,
};


#endif //ADITOF_INTERNAL_H
