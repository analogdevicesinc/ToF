
/* ================================================================================
     Created by  :   wpeet
     Created on  :   2020 Apr 09, 09:54 EDT

     Project     :   fpga_backdoor
     File        :   fpga_backdoor_addr_cdef.h
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
       Last modified : 5-Apr-2020
   ================================================================================ */

#ifndef FPGA_BACKDOOR_ADDR_CDEF_H
#define FPGA_BACKDOOR_ADDR_CDEF_H


#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

/* Dependent Header files to include */
#include "fpga_backdoor_addr_rdef.h"
#include "fpga_backdoor_addr_def.h"


#ifndef FPGA_BACKDOOR_ADDR_CDEF_H_
#define FPGA_BACKDOOR_ADDR_CDEF_H_    /* FPGA_BACKDOOR: Your module description, here. */

/* ====================================================================================================
        FPGA_BACKDOOR Module Instances Register Pointer Definitions
   ==================================================================================================== */
#define pREG_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET                ((volatile       uint16_t *)      ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET)
#define REG_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET                (*((volatile       uint16_t *)    ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET))
#define pREG_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE               ((volatile       uint16_t *)      ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE)
#define REG_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE               (*((volatile       uint16_t *)    ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE))
#define pREG_FPGA_BACKDOOR_REGMAP1_FPGA_DEBUG1                   ((volatile       uint16_t *)      ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_DEBUG1)
#define REG_FPGA_BACKDOOR_REGMAP1_FPGA_DEBUG1                   (*((volatile       uint16_t *)    ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_DEBUG1))
#define pREG_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH0                 ((volatile       uint16_t *)      ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH0)
#define REG_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH0                 (*((volatile       uint16_t *)    ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH0))
#define pREG_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH1                 ((volatile       uint16_t *)      ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH1)
#define REG_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH1                 (*((volatile       uint16_t *)    ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH1))
#define pREG_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH2                 ((volatile       uint16_t *)      ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH2)
#define REG_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH2                 (*((volatile       uint16_t *)    ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH2))
#define pREG_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH3                 ((volatile       uint16_t *)      ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH3)
#define REG_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH3                 (*((volatile       uint16_t *)    ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH3))
#define pREG_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS                    ((volatile       uint16_t *)      ADDR_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS)
#define REG_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS                    (*((volatile       uint16_t *)    ADDR_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS))

#endif  /* end ifndef FPGA_BACKDOOR_ADDR_CDEF_H_ */

#endif /* FPGA_BACKDOOR_ADDR_CDEF_H */

