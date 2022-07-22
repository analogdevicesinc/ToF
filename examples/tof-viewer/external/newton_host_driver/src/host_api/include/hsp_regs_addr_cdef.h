
/* ================================================================================
     Created by  :   wpeet
     Created on  :   2020 Apr 03, 15:33 EDT

     Project     :   hsp_regs
     File        :   hsp_regs_addr_cdef.h
     Description :   C header file contains pointer definitions for registers.

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

#ifndef HSP_REGS_ADDR_CDEF_H
#define HSP_REGS_ADDR_CDEF_H


#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

/* Dependent Header files to include */
#include "hsp_regs_addr_rdef.h"
#include "hsp_regs_addr_def.h"


#ifndef HSP_REGS_ADDR_CDEF_H_
#define HSP_REGS_ADDR_CDEF_H_    /* HSP_REGS: Your module description, here. */

/* ====================================================================================================
        HSP_REGS Module Instances Register Pointer Definitions
   ==================================================================================================== */
#define pREG_HSP_REGS_ADI_S2H_MBX_CTRL                           ((volatile       uint16_t *)      ADDR_HSP_REGS_ADI_S2H_MBX_CTRL)
#define REG_HSP_REGS_ADI_S2H_MBX_CTRL                           (*((volatile       uint16_t *)    ADDR_HSP_REGS_ADI_S2H_MBX_CTRL))
#define pREG_HSP_REGS_ADI_S2H_MBX_STS                            ((volatile       uint16_t *)      ADDR_HSP_REGS_ADI_S2H_MBX_STS)
#define REG_HSP_REGS_ADI_S2H_MBX_STS                            (*((volatile       uint16_t *)    ADDR_HSP_REGS_ADI_S2H_MBX_STS))
#define pREG_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH                      ((volatile       uint16_t *)      ADDR_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH)
#define REG_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH                      (*((volatile       uint16_t *)    ADDR_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH))
#define pREG_HSP_REGS_ADI_S2H_MBX_FIFO_POP                       ((volatile       uint16_t *)      ADDR_HSP_REGS_ADI_S2H_MBX_FIFO_POP)
#define REG_HSP_REGS_ADI_S2H_MBX_FIFO_POP                       (*((volatile       uint16_t *)    ADDR_HSP_REGS_ADI_S2H_MBX_FIFO_POP))
#define pREG_HSP_REGS_ADI_H2S_MBX_CTRL                           ((volatile       uint16_t *)      ADDR_HSP_REGS_ADI_H2S_MBX_CTRL)
#define REG_HSP_REGS_ADI_H2S_MBX_CTRL                           (*((volatile       uint16_t *)    ADDR_HSP_REGS_ADI_H2S_MBX_CTRL))
#define pREG_HSP_REGS_ADI_H2S_MBX_INSTS                          ((volatile       uint16_t *)      ADDR_HSP_REGS_ADI_H2S_MBX_INSTS)
#define REG_HSP_REGS_ADI_H2S_MBX_INSTS                          (*((volatile       uint16_t *)    ADDR_HSP_REGS_ADI_H2S_MBX_INSTS))
#define pREG_HSP_REGS_ADI_H2S_MBX_FIFO_PUSH                      ((volatile       uint16_t *)      ADDR_HSP_REGS_ADI_H2S_MBX_FIFO_PUSH)
#define REG_HSP_REGS_ADI_H2S_MBX_FIFO_PUSH                      (*((volatile       uint16_t *)    ADDR_HSP_REGS_ADI_H2S_MBX_FIFO_PUSH))
#define pREG_HSP_REGS_ADI_H2S_MBX_FIFO_POP                       ((volatile       uint16_t *)      ADDR_HSP_REGS_ADI_H2S_MBX_FIFO_POP)
#define REG_HSP_REGS_ADI_H2S_MBX_FIFO_POP                       (*((volatile       uint16_t *)    ADDR_HSP_REGS_ADI_H2S_MBX_FIFO_POP))

#endif  /* end ifndef HSP_REGS_ADDR_CDEF_H_ */

#endif /* HSP_REGS_ADDR_CDEF_H */

