
/* ================================================================================
     Created by  :   wpeet
     Created on  :   2020 Apr 09, 09:54 EDT

     Project     :   fpga_backdoor
     File        :   fpga_backdoor_addr_rdef.h
     Description :   C header file contains macros for Registers' address relative to instances and plain bit-fields.

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

#ifndef FPGA_BACKDOOR_ADDR_RDEF_H
#define FPGA_BACKDOOR_ADDR_RDEF_H


#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

/* ====================================================================================================
        FPGA_BACKDOOR Module Instances Address and Mask Definitions
   ==================================================================================================== */
#define INST_FPGA_BACKDOOR                                       (0X0F00U)    /* fpga_backdoor: Your module description, here.*/


#ifndef FPGA_BACKDOOR_ADDR_RDEF_H_
#define FPGA_BACKDOOR_ADDR_RDEF_H_    /* FPGA_BACKDOOR: Your module description, here. */

#define MASK_FPGA_BACKDOOR                                       (0XFFFFU)    /* FPGA_BACKDOOR: Your module description, here. */

/* ====================================================================================================
        FPGA_BACKDOOR Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET                (0X0) 
#define RSTVAL_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE               (0X0) 
#define RSTVAL_FPGA_BACKDOOR_REGMAP1_FPGA_DEBUG1                   (0X0) 
#define RSTVAL_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH0                 (0X0) 
#define RSTVAL_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH1                 (0X0) 
#define RSTVAL_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH2                 (0X0) 
#define RSTVAL_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH3                 (0X0) 
#define RSTVAL_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS                    (0X0) 

/* ====================================================================================================
        FPGA_BACKDOOR Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          REGMAP1_BACKDOOR_RESET                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET_HSP_SOFTRESETB_STICKY (0U)           /* HSP Soft Reset Active Low (FPGA) */
#define BITL_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET_HSP_SOFTRESETB_STICKY (1U)           /* HSP Soft Reset Active Low (FPGA) */
#define BITM_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET_HSP_SOFTRESETB_STICKY (0X0001U)      /* HSP Soft Reset Active Low (FPGA) */

#define ENUM_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET_HSP_SOFTRESETB_STICKY_DISABLE (0X0000U)      /* No description provided */
#define ENUM_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET_HSP_SOFTRESETB_STICKY_ENABLE (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          REGMAP1_BACKDOOR_ENABLE                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_HSPROM_BDEN (0U)           /* Enables Backdoor Load of HSP ROM */
#define BITL_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_HSPROM_BDEN (1U)           /* Enables Backdoor Load of HSP ROM */
#define BITM_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_HSPROM_BDEN (0X0001U)      /* Enables Backdoor Load of HSP ROM */
#define BITP_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_FMCEFUSE_BDEN (1U)           /* Enables Backdoor Load of Secure Fuse */
#define BITL_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_FMCEFUSE_BDEN (1U)           /* Enables Backdoor Load of Secure Fuse */
#define BITM_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_FMCEFUSE_BDEN (0X0002U)      /* Enables Backdoor Load of Secure Fuse */
#define BITP_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_HSPRAM_BDEN (2U)           /* Enables Backdoor Load of HSP RAM */
#define BITL_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_HSPRAM_BDEN (1U)           /* Enables Backdoor Load of HSP RAM */
#define BITM_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_HSPRAM_BDEN (0X0004U)      /* Enables Backdoor Load of HSP RAM */

/* ----------------------------------------------------------------------------------------------------
          REGMAP1_FPGA_DEBUG1                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FPGA_BACKDOOR_REGMAP1_FPGA_DEBUG1_HSP_AEB        (0U)           /* HSP Access Enable Bits */
#define BITL_FPGA_BACKDOOR_REGMAP1_FPGA_DEBUG1_HSP_AEB        (16U)          /* HSP Access Enable Bits */
#define BITM_FPGA_BACKDOOR_REGMAP1_FPGA_DEBUG1_HSP_AEB        (0XFFFFU)      /* HSP Access Enable Bits */

/* ----------------------------------------------------------------------------------------------------
          REGMAP1_FPGA_SCRATCH0                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH0_SCRATCH0     (0U)           /* FPGA Scratch Register 0 */
#define BITL_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH0_SCRATCH0     (16U)          /* FPGA Scratch Register 0 */
#define BITM_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH0_SCRATCH0     (0XFFFFU)      /* FPGA Scratch Register 0 */

/* ----------------------------------------------------------------------------------------------------
          REGMAP1_FPGA_SCRATCH1                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH1_SCRATCH1     (0U)           /* FPGA Scratch Register 1 */
#define BITL_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH1_SCRATCH1     (16U)          /* FPGA Scratch Register 1 */
#define BITM_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH1_SCRATCH1     (0XFFFFU)      /* FPGA Scratch Register 1 */

/* ----------------------------------------------------------------------------------------------------
          REGMAP1_FPGA_SCRATCH2                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH2_SCRATCH2     (0U)           /* FPGA Scratch Register 2 */
#define BITL_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH2_SCRATCH2     (16U)          /* FPGA Scratch Register 2 */
#define BITM_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH2_SCRATCH2     (0XFFFFU)      /* FPGA Scratch Register 2 */

/* ----------------------------------------------------------------------------------------------------
          REGMAP1_FPGA_SCRATCH3                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH3_SCRATCH3     (0U)           /* FPGA Scratch Register 3 */
#define BITL_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH3_SCRATCH3     (16U)          /* FPGA Scratch Register 3 */
#define BITM_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH3_SCRATCH3     (0XFFFFU)      /* FPGA Scratch Register 3 */

/* ----------------------------------------------------------------------------------------------------
          REGMAP1_HSP_BYPASS                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS_HSP_BYPASS_EN   (0U)           /* FPGA HSP Bypass Enable */
#define BITL_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS_HSP_BYPASS_EN   (1U)           /* FPGA HSP Bypass Enable */
#define BITM_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS_HSP_BYPASS_EN   (0X0001U)      /* FPGA HSP Bypass Enable */

#endif  /* end ifndef FPGA_BACKDOOR_ADDR_RDEF_H_ */

#endif /* FPGA_BACKDOOR_ADDR_RDEF_H */

