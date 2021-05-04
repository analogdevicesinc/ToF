
/* ================================================================================
     Created by  :   wpeet
     Created on  :   2020 Apr 03, 15:33 EDT

     Project     :   hsp_regs
     File        :   hsp_regs_device.h
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
       Last modified : 25-Mar-2020
   ================================================================================ */

#ifndef HSP_REGS_DEVICE_H
#define HSP_REGS_DEVICE_H

/* pickup integer types */
#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

/* pickup register bitfield and bit masks */
#include "hsp_regs_typedefs.h"

#if defined ( __CC_ARM   )
#pragma push
#pragma anon_unions
#endif

#ifndef ADI_HSP_REGS_DEVICE_H_
#define ADI_HSP_REGS_DEVICE_H_

/*! ========================================================================
 *  \struct ADI_HSP_REGS_TypeDef
 *  \brief Your module description, here.
 *  ======================================================================== */
typedef struct _ADI_HSP_REGS_TypeDef
{
    volatile       uint16_t ADI_S2H_MBX_CTRL;                /**< 0 System to HSP Mailbox Interrupt Control */
    volatile const uint16_t RESERVED0;                      
    volatile       uint16_t ADI_S2H_MBX_STS;                 /**< 2 System to HSP Mailbox Interrupt Status */
    volatile const uint16_t RESERVED1;                      
    volatile       uint16_t ADI_S2H_MBX_FIFO_PUSH;           /**< 4 Write to this register pushes data into the S2H FIFO */
    volatile const uint16_t RESERVED2;                      
    volatile       uint16_t ADI_S2H_MBX_FIFO_POP;            /**< 6 System Write Data into the FIFO. */
    volatile const uint16_t RESERVED3;                      
    volatile       uint16_t ADI_H2S_MBX_CTRL;                /**< 8 HSP to System Mailbox Interrupt Control */
    volatile const uint16_t RESERVED4;                      
    volatile       uint16_t ADI_H2S_MBX_INSTS;               /**< A HSP to System Mailbox Interrupt Statu */
    volatile const uint16_t RESERVED5;                      
    volatile       uint16_t ADI_H2S_MBX_FIFO_PUSH;           /**< C HSP to System Mailbox Write Data into the FIFO. */
    volatile const uint16_t RESERVED6;                      
    volatile       uint16_t ADI_H2S_MBX_FIFO_POP;            /**< E HSP to System Mailbox Read Data from the FIFO */
} ADI_HSP_REGS_TypeDef;


#define ADI_HSP_REGS_BASE                   (0X00U)    /* Base address of hsp_regs: Your module description, here.*/
#define pADI_HSP_REGS                       ((ADI_HSP_REGS_TypeDef *) ADI_HSP_REGS_BASE )    /* Pointer to Your module description, here. (hsp_regs)*/

#endif  /* end ifndef ADI_HSP_REGS_DEVICE_H_ */

#if defined (__CC_ARM)
#pragma pop
#endif

#endif /* HSP_REGS_DEVICE_H */

