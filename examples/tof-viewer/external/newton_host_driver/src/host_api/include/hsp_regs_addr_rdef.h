
/* ================================================================================
     Created by  :   wpeet
     Created on  :   2020 Apr 03, 15:33 EDT

     Project     :   hsp_regs
     File        :   hsp_regs_addr_rdef.h
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
       Last modified : 25-Mar-2020
   ================================================================================ */

#ifndef HSP_REGS_ADDR_RDEF_H
#define HSP_REGS_ADDR_RDEF_H


#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

/* ====================================================================================================
        HSP_REGS Module Instances Address and Mask Definitions
   ==================================================================================================== */
#define INST_HSP_REGS                                            (0X00U)    /* hsp_regs: Your module description, here.*/


#ifndef HSP_REGS_ADDR_RDEF_H_
#define HSP_REGS_ADDR_RDEF_H_    /* HSP_REGS: Your module description, here. */

#define MASK_HSP_REGS                                            (0XFFU)    /* HSP_REGS: Your module description, here. */

/* ====================================================================================================
        HSP_REGS Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_HSP_REGS_ADI_S2H_MBX_CTRL                           (0X2) 
#define RSTVAL_HSP_REGS_ADI_S2H_MBX_STS                            (0X0) 
#define RSTVAL_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH                      (0X0) 
#define RSTVAL_HSP_REGS_ADI_S2H_MBX_FIFO_POP                       (0X0) 
#define RSTVAL_HSP_REGS_ADI_H2S_MBX_CTRL                           (0X4) 
#define RSTVAL_HSP_REGS_ADI_H2S_MBX_INSTS                          (0X0) 
#define RSTVAL_HSP_REGS_ADI_H2S_MBX_FIFO_PUSH                      (0X0) 
#define RSTVAL_HSP_REGS_ADI_H2S_MBX_FIFO_POP                       (0X0) 

/* ====================================================================================================
        HSP_REGS Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          ADI_S2H_MBX_CTRL                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_FULL_INT_EN   (0U)           /* FIFO full Interrupt Enable */
#define BITL_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_FULL_INT_EN   (1U)           /* FIFO full Interrupt Enable */
#define BITM_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_FULL_INT_EN   (0X0001U)      /* FIFO full Interrupt Enable */
#define BITP_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_NEMPTY_INT_EN (1U)           /* FIFO Not-empty Interrupt Enable */
#define BITL_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_NEMPTY_INT_EN (1U)           /* FIFO Not-empty Interrupt Enable */
#define BITM_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_NEMPTY_INT_EN (0X0002U)      /* FIFO Not-empty Interrupt Enable */
#define BITP_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_VALID_INT_EN  (2U)           /* S2H FIFO valid interrupt enable */
#define BITL_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_VALID_INT_EN  (1U)           /* S2H FIFO valid interrupt enable */
#define BITM_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_VALID_INT_EN  (0X0004U)      /* S2H FIFO valid interrupt enable */
#define BITP_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_ERR_INT_EN         (3U)           /* S2H error interrupt enable */
#define BITL_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_ERR_INT_EN         (1U)           /* S2H error interrupt enable */
#define BITM_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_ERR_INT_EN         (0X0008U)      /* S2H error interrupt enable */

#define ENUM_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_NEMPTY_INT_EN_INTERRUPTDISABLE (0X0000U)      /* No description provided */
#define ENUM_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_NEMPTY_INT_EN_INTERRUPTENABLE (0X0001U)      /* No description provided */
#define ENUM_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_FULL_INT_EN_INTERRUPTDISABLE (0X0000U)      /* No description provided */
#define ENUM_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_FULL_INT_EN_INTERRUPTENABLE (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ADI_S2H_MBX_STS                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_HSP_REGS_ADI_S2H_MBX_STS_S2H_FIFO_FULL_INSTS     (0U)           /* FIFO Full Status */
#define BITL_HSP_REGS_ADI_S2H_MBX_STS_S2H_FIFO_FULL_INSTS     (1U)           /* FIFO Full Status */
#define BITM_HSP_REGS_ADI_S2H_MBX_STS_S2H_FIFO_FULL_INSTS     (0X0001U)      /* FIFO Full Status */
#define BITP_HSP_REGS_ADI_S2H_MBX_STS_S2H_FIFO_NEMPTY_INSTS   (1U)           /* FIFO Not empty Status */
#define BITL_HSP_REGS_ADI_S2H_MBX_STS_S2H_FIFO_NEMPTY_INSTS   (1U)           /* FIFO Not empty Status */
#define BITM_HSP_REGS_ADI_S2H_MBX_STS_S2H_FIFO_NEMPTY_INSTS   (0X0002U)      /* FIFO Not empty Status */
#define BITP_HSP_REGS_ADI_S2H_MBX_STS_S2H_FIFO_VALID          (2U)           /* S2H FIFO Valid */
#define BITL_HSP_REGS_ADI_S2H_MBX_STS_S2H_FIFO_VALID          (1U)           /* S2H FIFO Valid */
#define BITM_HSP_REGS_ADI_S2H_MBX_STS_S2H_FIFO_VALID          (0X0004U)      /* S2H FIFO Valid */
#define BITP_HSP_REGS_ADI_S2H_MBX_STS_S2H_ERR_BIT             (3U)           /* S2H Error Status Bit */
#define BITL_HSP_REGS_ADI_S2H_MBX_STS_S2H_ERR_BIT             (1U)           /* S2H Error Status Bit */
#define BITM_HSP_REGS_ADI_S2H_MBX_STS_S2H_ERR_BIT             (0X0008U)      /* S2H Error Status Bit */
#define BITP_HSP_REGS_ADI_S2H_MBX_STS_S2H_FIFO_CNT            (8U)           /* S2H FIFO Count */
#define BITL_HSP_REGS_ADI_S2H_MBX_STS_S2H_FIFO_CNT            (8U)           /* S2H FIFO Count */
#define BITM_HSP_REGS_ADI_S2H_MBX_STS_S2H_FIFO_CNT            (0XFF00U)      /* S2H FIFO Count */

/* ----------------------------------------------------------------------------------------------------
          ADI_S2H_MBX_FIFO_PUSH                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH_S2H_FIFO_DATA_PUSH (0U)           /* Write to this register pushes data into the S2H FIFO */
#define BITL_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH_S2H_FIFO_DATA_PUSH (16U)          /* Write to this register pushes data into the S2H FIFO */
#define BITM_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH_S2H_FIFO_DATA_PUSH (0XFFFFU)      /* Write to this register pushes data into the S2H FIFO */

/* ----------------------------------------------------------------------------------------------------
          ADI_S2H_MBX_FIFO_POP                                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_HSP_REGS_ADI_S2H_MBX_FIFO_POP_S2H_FIFO_DATA_POP  (0U)           /* The data available at the S2H FIFO output */
#define BITL_HSP_REGS_ADI_S2H_MBX_FIFO_POP_S2H_FIFO_DATA_POP  (16U)          /* The data available at the S2H FIFO output */
#define BITM_HSP_REGS_ADI_S2H_MBX_FIFO_POP_S2H_FIFO_DATA_POP  (0XFFFFU)      /* The data available at the S2H FIFO output */

/* ----------------------------------------------------------------------------------------------------
          ADI_H2S_MBX_CTRL                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_FULL_INT_EN   (0U)           /* FIFO full Interrupt Enable */
#define BITL_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_FULL_INT_EN   (1U)           /* FIFO full Interrupt Enable */
#define BITM_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_FULL_INT_EN   (0X0001U)      /* FIFO full Interrupt Enable */
#define BITP_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_NEMPTY_INT_EN (1U)           /* FIFO Not-empty Interrupt Enable */
#define BITL_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_NEMPTY_INT_EN (1U)           /* FIFO Not-empty Interrupt Enable */
#define BITM_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_NEMPTY_INT_EN (0X0002U)      /* FIFO Not-empty Interrupt Enable */
#define BITP_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_VALID_INT_EN  (2U)           /* H2S FIFO valid interrupt enable */
#define BITL_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_VALID_INT_EN  (1U)           /* H2S FIFO valid interrupt enable */
#define BITM_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_VALID_INT_EN  (0X0004U)      /* H2S FIFO valid interrupt enable */
#define BITP_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_ERR_INT_EN         (3U)           /* H2S error interrupt enable */
#define BITL_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_ERR_INT_EN         (1U)           /* H2S error interrupt enable */
#define BITM_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_ERR_INT_EN         (0X0008U)      /* H2S error interrupt enable */

#define ENUM_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_NEMPTY_INT_EN_INTERRUPTDISABLE (0X0000U)      /* No description provided */
#define ENUM_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_NEMPTY_INT_EN_INTERRUPTENABLE (0X0001U)      /* No description provided */
#define ENUM_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_FULL_INT_EN_INTERRUPTDISABLE (0X0000U)      /* No description provided */
#define ENUM_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_FULL_INT_EN_INTERRUPTENABLE (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ADI_H2S_MBX_INSTS                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_FIFO_FULL_INSTS   (0U)           /* FIFO Full Status */
#define BITL_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_FIFO_FULL_INSTS   (1U)           /* FIFO Full Status */
#define BITM_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_FIFO_FULL_INSTS   (0X0001U)      /* FIFO Full Status */
#define BITP_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_FIFO_NEMPTY_INSTS (1U)           /* FIFO Not empty Status */
#define BITL_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_FIFO_NEMPTY_INSTS (1U)           /* FIFO Not empty Status */
#define BITM_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_FIFO_NEMPTY_INSTS (0X0002U)      /* FIFO Not empty Status */
#define BITP_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_FIFO_VALID        (2U)           /* H2S FIFO Valid */
#define BITL_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_FIFO_VALID        (1U)           /* H2S FIFO Valid */
#define BITM_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_FIFO_VALID        (0X0004U)      /* H2S FIFO Valid */
#define BITP_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_ERR_BIT           (3U)           /* H2S error status bit */
#define BITL_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_ERR_BIT           (1U)           /* H2S error status bit */
#define BITM_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_ERR_BIT           (0X0008U)      /* H2S error status bit */
#define BITP_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_FIFO_CNT          (8U)           /* H2S FIFI Count */
#define BITL_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_FIFO_CNT          (8U)           /* H2S FIFI Count */
#define BITM_HSP_REGS_ADI_H2S_MBX_INSTS_H2S_FIFO_CNT          (0XFF00U)      /* H2S FIFI Count */

/* ----------------------------------------------------------------------------------------------------
          ADI_H2S_MBX_FIFO_PUSH                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_HSP_REGS_ADI_H2S_MBX_FIFO_PUSH_H2S_FIFO_DATA_PUSH (0U)           /* Write to this register pushes data into the S2H FIFO */
#define BITL_HSP_REGS_ADI_H2S_MBX_FIFO_PUSH_H2S_FIFO_DATA_PUSH (16U)          /* Write to this register pushes data into the S2H FIFO */
#define BITM_HSP_REGS_ADI_H2S_MBX_FIFO_PUSH_H2S_FIFO_DATA_PUSH (0XFFFFU)      /* Write to this register pushes data into the S2H FIFO */

/* ----------------------------------------------------------------------------------------------------
          ADI_H2S_MBX_FIFO_POP                                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_HSP_REGS_ADI_H2S_MBX_FIFO_POP_H2S_FIFO_DATA_POP  (0U)           /* The data available at the S2H FIFO output */
#define BITL_HSP_REGS_ADI_H2S_MBX_FIFO_POP_H2S_FIFO_DATA_POP  (16U)          /* The data available at the S2H FIFO output */
#define BITM_HSP_REGS_ADI_H2S_MBX_FIFO_POP_H2S_FIFO_DATA_POP  (0XFFFFU)      /* The data available at the S2H FIFO output */

#endif  /* end ifndef HSP_REGS_ADDR_RDEF_H_ */

#endif /* HSP_REGS_ADDR_RDEF_H */

