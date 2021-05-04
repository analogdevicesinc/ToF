
/* ================================================================================
     Created by  :   wpeet
     Created on  :   2020 Apr 09, 09:54 EDT

     Project     :   fpga_backdoor
     File        :   fpga_backdoor_addr_def.h
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
       Last modified : 5-Apr-2020
   ================================================================================ */

#ifndef FPGA_BACKDOOR_ADDR_DEF_H
#define FPGA_BACKDOOR_ADDR_DEF_H


#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

/* ====================================================================================================
        FPGA_BACKDOOR Module Instances Register Address Defintions
   ==================================================================================================== */
#define ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET                (0X0F80U)    /* Control HSP Reset During Backdoor */
#define ADDR_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE               (0X0F82U)    /* Selects Memory for Backdoor Load */
#define ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_DEBUG1                   (0X0F84U)    /* FPGA Debug Register for HSP AEB */
#define ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH0                 (0X0F86U)    /* FPGA Scratch Register */
#define ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH1                 (0X0F88U)    /* FPGA Scratch Register */
#define ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH2                 (0X0F8AU)    /* FPGA Scratch Register */
#define ADDR_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH3                 (0X0F8CU)    /* FPGA Scratch Register */
#define ADDR_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS                    (0X0F8EU)    /* FPGA HSP Bypass Enable */

#endif /* FPGA_BACKDOOR_ADDR_DEF_H */

