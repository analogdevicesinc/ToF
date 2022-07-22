
/* ================================================================================
     Created by  :   wpeet
     Created on  :   2020 Apr 03, 15:33 EDT

     Project     :   hsp_regs
     File        :   hsp_regs_addr_def.h
     Description :   C header file contains macros for Registers' address absolute to instances.

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

#ifndef HSP_REGS_ADDR_DEF_H
#define HSP_REGS_ADDR_DEF_H


#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

/* ====================================================================================================
        HSP_REGS Module Instances Register Address Defintions
   ==================================================================================================== */
#define ADDR_HSP_REGS_ADI_S2H_MBX_CTRL                           (0X00U)    /* System to HSP Mailbox Interrupt Control */
#define ADDR_HSP_REGS_ADI_S2H_MBX_STS                            (0X02U)    /* System to HSP Mailbox Interrupt Status */
#define ADDR_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH                      (0X04U)    /* Write to this register pushes data into the S2H FIFO */
#define ADDR_HSP_REGS_ADI_S2H_MBX_FIFO_POP                       (0X06U)    /* System Write Data into the FIFO. */
#define ADDR_HSP_REGS_ADI_H2S_MBX_CTRL                           (0X08U)    /* HSP to System Mailbox Interrupt Control */
#define ADDR_HSP_REGS_ADI_H2S_MBX_INSTS                          (0X0AU)    /* HSP to System Mailbox Interrupt Statu */
#define ADDR_HSP_REGS_ADI_H2S_MBX_FIFO_PUSH                      (0X0CU)    /* HSP to System Mailbox Write Data into the FIFO. */
#define ADDR_HSP_REGS_ADI_H2S_MBX_FIFO_POP                       (0X0EU)    /* HSP to System Mailbox Read Data from the FIFO */

#endif /* HSP_REGS_ADDR_DEF_H */

