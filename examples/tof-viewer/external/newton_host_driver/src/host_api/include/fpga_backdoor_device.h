
/* ================================================================================
     Created by  :   wpeet
     Created on  :   2020 Apr 09, 09:54 EDT

     Project     :   fpga_backdoor
     File        :   fpga_backdoor_device.h
     Description :   C typedef structures for bit-fields and enums for enumerations.

     !! ADI Confidential !!
       INTERNAL USE ONLY. 
       NOT FOR EXTERNAL DISTRIBUTION.

     Copyright (c) 2020 Analog Devices, Inc.  All Rights Reserved.
     This software is proprietary and confidential to Analog Devices, Inc. and
     its licensors.

     This file was auto-generated. Do not make local changes to this file.

     Auto generation script information:
       Script        : /usr/cadtools/bin/yoda.dir/generators/inc/GenHeaders_main.py
       Last modified : 5-Apr-2020
   ================================================================================ */

#ifndef FPGA_BACKDOOR_DEVICE_H
#define FPGA_BACKDOOR_DEVICE_H

/* pickup integer types */
#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

/* pickup register bitfield and bit masks */
#include "fpga_backdoor_typedefs.h"

#if defined ( __CC_ARM   )
#pragma push
#pragma anon_unions
#endif

#ifndef ADI_FPGA_BACKDOOR_DEVICE_H_
#define ADI_FPGA_BACKDOOR_DEVICE_H_

/*! ========================================================================
 *  \struct ADI_FPGA_BACKDOOR_TypeDef
 *  \brief Your module description, here.
 *  ======================================================================== */
typedef struct _ADI_FPGA_BACKDOOR_TypeDef
{
    volatile const uint16_t RESERVED0[128];                 
    volatile       uint16_t REGMAP1_BACKDOOR_RESET;          /**< 80 Control HSP Reset During Backdoor */
    volatile const uint16_t RESERVED1;                      
    volatile       uint16_t REGMAP1_BACKDOOR_ENABLE;         /**< 82 Selects Memory for Backdoor Load */
    volatile const uint16_t RESERVED2;                      
    volatile       uint16_t REGMAP1_FPGA_DEBUG1;             /**< 84 FPGA Debug Register for HSP AEB */
    volatile const uint16_t RESERVED3;                      
    volatile       uint16_t REGMAP1_FPGA_SCRATCH0;           /**< 86 FPGA Scratch Register */
    volatile const uint16_t RESERVED4;                      
    volatile       uint16_t REGMAP1_FPGA_SCRATCH1;           /**< 88 FPGA Scratch Register */
    volatile const uint16_t RESERVED5;                      
    volatile       uint16_t REGMAP1_FPGA_SCRATCH2;           /**< 8A FPGA Scratch Register */
    volatile const uint16_t RESERVED6;                      
    volatile       uint16_t REGMAP1_FPGA_SCRATCH3;           /**< 8C FPGA Scratch Register */
    volatile const uint16_t RESERVED7;                      
    volatile       uint16_t REGMAP1_HSP_BYPASS;              /**< 8E FPGA HSP Bypass Enable */
} ADI_FPGA_BACKDOOR_TypeDef;


#define ADI_FPGA_BACKDOOR_BASE              (0X0F00U)    /* Base address of fpga_backdoor: Your module description, here.*/
#define pADI_FPGA_BACKDOOR                  ((ADI_FPGA_BACKDOOR_TypeDef *) ADI_FPGA_BACKDOOR_BASE )    /* Pointer to Your module description, here. (fpga_backdoor)*/

#endif  /* end ifndef ADI_FPGA_BACKDOOR_DEVICE_H_ */

#if defined (__CC_ARM)
#pragma pop
#endif

#endif /* FPGA_BACKDOOR_DEVICE_H */

