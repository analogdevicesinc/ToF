
/* ================================================================================
     Created by  :   nguthrie
     Created on  :   2020 Apr 03, 08:29 EDT

     Project     :   newton
     File        :   newton_addr_rdef.h
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

#ifndef NEWTON_ADDR_RDEF_H
#define NEWTON_ADDR_RDEF_H


#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

/* ====================================================================================================
        USEQ_REGS_MAP1 Module Instances Address and Mask Definitions
   ==================================================================================================== */
#define INST_USEQ_REGS                                           (0X00000000U)    /* useq_regs: */


#ifndef USEQ_REGS_MAP1_ADDR_RDEF_H_
#define USEQ_REGS_MAP1_ADDR_RDEF_H_    /* USEQ_REGS_MAP1: Your module description, here. */

#define MASK_USEQ_REGS_MAP1                                      (0XFFFFFFFFU)    /* USEQ_REGS_MAP1: Your module description, here. */

/* ====================================================================================================
        USEQ_REGS_MAP1 Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_USEQ_REGS_MAP1_SEQUENCESTARTADDR                    (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_SEQUENCEENDADDR                      (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_USEQCONTROLREGISTER                  (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_FRAMESYNCCTRL                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_BREAKPOINTCTRL                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_UPDATESTAMP                          (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_DIGPWRDOWN                           (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL                 (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_PIXGAINTAG1READOUTCTRL               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL                 (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_PIXSATURATEREADOUTCTRL               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_ROWCNTINCRCONTROL                    (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL                 (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_PIXGAINTAG0READOUTCTRL               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_I2CCTRL                              (0X2) 
#define RSTVAL_USEQ_REGS_MAP1_SEQUENCESTATUS                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_SYSTEMCLOCKCONTROL                   (0X1) 
#define RSTVAL_USEQ_REGS_MAP1_CALLRPTCOUNT                         (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GTSWAP                               (0X2) 
#define RSTVAL_USEQ_REGS_MAP1_INTERRUPTENABLE                      (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_ERRORSET                             (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_ERRORSTATUS                          (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPIOCTRL                             (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPIOINPUT                            (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPIOOUTPUTSET                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPIOOUTPUTCLR                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_PIXELINTERFACECTRL                   (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_ROWCNTINCRCONTROL2                   (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPIOFSYNCSNAPSHOT                    (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE                    (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_CTIMECTRL                            (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_CTIME_0                              (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_CTIME_1                              (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_CTIME_2                              (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_CTIME_3                              (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_CTIME_4                              (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_CTIME_5                              (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_SOFT_RESET                           (0X7B19) 
#define RSTVAL_USEQ_REGS_MAP1_WAITFORSYNCPOLARITY                  (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_USEQ_DFT                             (0X22) 
#define RSTVAL_USEQ_REGS_MAP1_USEQ_PARITY                          (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_HSP_DFT                              (0X2) 
#define RSTVAL_USEQ_REGS_MAP1_PCCOND                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR0                                (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR1                                (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR2                                (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR3                                (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR4                                (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR5                                (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR6                                (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR7                                (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR8                                (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR9                                (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR10                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR11                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR12                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR13                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR14                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR15                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_AMPCLKCTRL                           (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_AMPCLK2CTRL                          (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_AMPCLK3CTRL1                         (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_AMPCLK3CTRL2                         (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_NOISERESETCTRL1                      (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_NOISERESETCTRL2                      (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_PIXRESETCTRL1                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_PIXRESETCTRL2                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPIOPINFUNC1                         (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPIOPINFUNC2                         (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_USEQ_DBGMUX                          (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX                     (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_MM_CTRL                              (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_ERRJMPADDR                           (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_STOPERRENA                           (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_ADCCNVTCTRL1                         (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_ADCCNVTCTRL2                         (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_ADCCNVTCTRL3                         (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_ADCCNVTCTRL4                         (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GAINTAG1CLKCTRL1                     (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GAINTAG1CLKCTRL2                     (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL1                   (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL2                   (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GAINTAGTHRESHSEL                     (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GAINTAG0CLKCTRL1                     (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GAINTAG0CLKCTRL2                     (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_FORCESFCTRL1                         (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_FORCESFCTRL2                         (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_FORCEIPDACTRL1                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_FORCEIPDACTRL2                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_USEQRAMLOADADDR                      (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_USEQRAMRDSTADDR                      (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_USEQRAMLOADDATA                      (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_USEQRAMLOADDATAALIAS                 (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_USEQRAMRDDATA                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_USEQRAMRDDATAALIAS                   (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_PWM_CTRL_0                           (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_PWM_CTRL_1                           (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_FSYNCCTRL                            (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_FSYNCSTATUS                          (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_FSYNCLSMODCNTR_0                     (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_FSYNCLSMODCNTR_1                     (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_FSYNCINTCNTR_0                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_FSYNCINTCNTR_1                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_FSYNCSYSCNTR_0                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_FSYNCSYSCNTR_1                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR16                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR17                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR18                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR19                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR20                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR21                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR22                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR23                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR24                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR25                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR26                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR27                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR28                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR29                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR30                               (0X0) 
#define RSTVAL_USEQ_REGS_MAP1_GPRR31                               (0X0) 

/* ====================================================================================================
        USEQ_REGS_MAP1 Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          SEQUENCESTARTADDR                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_SEQUENCESTARTADDR_SEQUENCE_START_ADDR (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_SEQUENCESTARTADDR_SEQUENCE_START_ADDR (12U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_SEQUENCESTARTADDR_SEQUENCE_START_ADDR (0X0FFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SEQUENCEENDADDR                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_SEQUENCEENDADDR_SEQUENCE_END_ADDR (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_SEQUENCEENDADDR_SEQUENCE_END_ADDR (12U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_SEQUENCEENDADDR_SEQUENCE_END_ADDR (0X0FFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USEQCONTROLREGISTER                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_USEQCONTROLREGISTER_START_EXEC    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQCONTROLREGISTER_START_EXEC    (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQCONTROLREGISTER_START_EXEC    (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQCONTROLREGISTER_STOP_EXEC     (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQCONTROLREGISTER_STOP_EXEC     (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQCONTROLREGISTER_STOP_EXEC     (0X0002U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQCONTROLREGISTER_LOAD_PAD      (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQCONTROLREGISTER_LOAD_PAD      (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQCONTROLREGISTER_LOAD_PAD      (0X0004U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQCONTROLREGISTER_UPDATE_SEQRAM (3U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQCONTROLREGISTER_UPDATE_SEQRAM (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQCONTROLREGISTER_UPDATE_SEQRAM (0X0008U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQCONTROLREGISTER_UPDATE_WAVERAM (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQCONTROLREGISTER_UPDATE_WAVERAM (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQCONTROLREGISTER_UPDATE_WAVERAM (0X0010U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQCONTROLREGISTER_UPDATE_MAPRAM (5U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQCONTROLREGISTER_UPDATE_MAPRAM (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQCONTROLREGISTER_UPDATE_MAPRAM (0X0020U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQCONTROLREGISTER_LOAD_LC       (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQCONTROLREGISTER_LOAD_LC       (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQCONTROLREGISTER_LOAD_LC       (0X0040U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQCONTROLREGISTER_LOAD_POKE     (7U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQCONTROLREGISTER_LOAD_POKE     (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQCONTROLREGISTER_LOAD_POKE     (0X0080U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FRAMESYNCCTRL                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_FRAMESYNCCTRL_EN_EXT_SYNC         (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FRAMESYNCCTRL_EN_EXT_SYNC         (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FRAMESYNCCTRL_EN_EXT_SYNC         (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FRAMESYNCCTRL_EXT_SYNC_MODE       (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FRAMESYNCCTRL_EXT_SYNC_MODE       (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FRAMESYNCCTRL_EXT_SYNC_MODE       (0X0002U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FRAMESYNCCTRL_EXT_SYNC_CNT        (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FRAMESYNCCTRL_EXT_SYNC_CNT        (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FRAMESYNCCTRL_EXT_SYNC_CNT        (0X003CU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          BREAKPOINTCTRL                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_BREAKPOINTCTRL_BRK                (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_BREAKPOINTCTRL_BRK                (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_BREAKPOINTCTRL_BRK                (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_BREAKPOINTCTRL_BRK_RES            (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_BREAKPOINTCTRL_BRK_RES            (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_BREAKPOINTCTRL_BRK_RES            (0X0002U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_BREAKPOINTCTRL_SEQ_ENABLE         (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_BREAKPOINTCTRL_SEQ_ENABLE         (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_BREAKPOINTCTRL_SEQ_ENABLE         (0X0004U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_BREAKPOINTCTRL_BRK_SEQ_RAM_ADDR   (3U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_BREAKPOINTCTRL_BRK_SEQ_RAM_ADDR   (12U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_BREAKPOINTCTRL_BRK_SEQ_RAM_ADDR   (0X7FF8U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          UPDATESTAMP                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_UPDATESTAMP_UPDATE_STAMP          (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_UPDATESTAMP_UPDATE_STAMP          (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_UPDATESTAMP_UPDATE_STAMP          (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DIGPWRDOWN                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_DIGPWRDOWN_PD_DE                  (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_DIGPWRDOWN_PD_DE                  (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_DIGPWRDOWN_PD_DE                  (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_DIGPWRDOWN_PD_CSI                 (3U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_DIGPWRDOWN_PD_CSI                 (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_DIGPWRDOWN_PD_CSI                 (0X0008U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_DIGPWRDOWN_PD_DATAPATH            (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_DIGPWRDOWN_PD_DATAPATH            (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_DIGPWRDOWN_PD_DATAPATH            (0X0010U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_DIGPWRDOWN_PD_SPI_MASTER          (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_DIGPWRDOWN_PD_SPI_MASTER          (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_DIGPWRDOWN_PD_SPI_MASTER          (0X0100U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_DIGPWRDOWN_PD_PCM                 (11U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_DIGPWRDOWN_PD_PCM                 (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_DIGPWRDOWN_PD_PCM                 (0X0800U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_DIGPWRDOWN_PD_LPS2                (12U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_DIGPWRDOWN_PD_LPS2                (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_DIGPWRDOWN_PD_LPS2                (0X1000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_DIGPWRDOWN_PD_SS                  (13U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_DIGPWRDOWN_PD_SS                  (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_DIGPWRDOWN_PD_SS                  (0X2000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_DIGPWRDOWN_PD_LPS1                (14U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_DIGPWRDOWN_PD_LPS1                (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_DIGPWRDOWN_PD_LPS1                (0X4000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PIXGAINTAG1LATCHCTRL                                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_EN (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_EN (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_RISE_CNVT_CNT (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_RISE_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_RISE_CNVT_CNT (0X0006U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_RISE_CLK_CNT (3U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_RISE_CLK_CNT (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_RISE_CLK_CNT (0X0078U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_FALL_CNVT_CNT (7U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_FALL_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_FALL_CNVT_CNT (0X0180U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_FALL_CLK_CNT (9U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_FALL_CLK_CNT (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_PIXGAINTAG1_LATCH_FALL_CLK_CNT (0X1E00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PIXGAINTAG1READOUTCTRL                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG1READOUTCTRL_PIXGAINTAG1_READOUT_CNVT_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG1READOUTCTRL_PIXGAINTAG1_READOUT_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG1READOUTCTRL_PIXGAINTAG1_READOUT_CNVT_CNT (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG1READOUTCTRL_PIXGAINTAG1_READOUT_CLK_CNT (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG1READOUTCTRL_PIXGAINTAG1_READOUT_CLK_CNT (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG1READOUTCTRL_PIXGAINTAG1_READOUT_CLK_CNT (0X003CU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PIXSATURATELATCHCTRL                                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_EN (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_EN (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_RISE_CNVT_CNT (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_RISE_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_RISE_CNVT_CNT (0X0006U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_RISE_CLK_CNT (3U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_RISE_CLK_CNT (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_RISE_CLK_CNT (0X0078U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_FALL_CNVT_CNT (7U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_FALL_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_FALL_CNVT_CNT (0X0180U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_FALL_CLK_CNT (9U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_FALL_CLK_CNT (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_PIXSATURATE_LATCH_FALL_CLK_CNT (0X1E00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PIXSATURATEREADOUTCTRL                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_PIXSATURATEREADOUTCTRL_PIXSATURATE_READOUT_CNVT_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXSATURATEREADOUTCTRL_PIXSATURATE_READOUT_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXSATURATEREADOUTCTRL_PIXSATURATE_READOUT_CNVT_CNT (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXSATURATEREADOUTCTRL_PIXSATURATE_READOUT_CLK_CNT (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXSATURATEREADOUTCTRL_PIXSATURATE_READOUT_CLK_CNT (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXSATURATEREADOUTCTRL_PIXSATURATE_READOUT_CLK_CNT (0X003CU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ROWCNTINCRCONTROL                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_AUTO_ROW_CNT_INCR_EN (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_AUTO_ROW_CNT_INCR_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_AUTO_ROW_CNT_INCR_EN (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_NO_OF_ADC_CONVERT (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_NO_OF_ADC_CONVERT (3U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_NO_OF_ADC_CONVERT (0X000EU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_NO_OF_POSEDGE_CLKCYCLE (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_NO_OF_POSEDGE_CLKCYCLE (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_NO_OF_POSEDGE_CLKCYCLE (0X00F0U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_AUTO_ROW_INCR_STRIDE (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_AUTO_ROW_INCR_STRIDE (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_AUTO_ROW_INCR_STRIDE (0X0F00U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_AUTO_ROW_INCR_DIR (12U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_AUTO_ROW_INCR_DIR (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_AUTO_ROW_INCR_DIR (0X3000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PIXGAINTAG0LATCHCTRL                                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_EN (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_EN (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_RISE_CNVT_CNT (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_RISE_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_RISE_CNVT_CNT (0X0006U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_RISE_CLK_CNT (3U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_RISE_CLK_CNT (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_RISE_CLK_CNT (0X0078U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_FALL_CNVT_CNT (7U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_FALL_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_FALL_CNVT_CNT (0X0180U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_FALL_CLK_CNT (9U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_FALL_CLK_CNT (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_PIXGAINTAG0_LATCH_FALL_CLK_CNT (0X1E00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PIXGAINTAG0READOUTCTRL                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG0READOUTCTRL_PIXGAINTAG0_READOUT_CNVT_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG0READOUTCTRL_PIXGAINTAG0_READOUT_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG0READOUTCTRL_PIXGAINTAG0_READOUT_CNVT_CNT (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXGAINTAG0READOUTCTRL_PIXGAINTAG0_READOUT_CLK_CNT (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXGAINTAG0READOUTCTRL_PIXGAINTAG0_READOUT_CLK_CNT (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXGAINTAG0READOUTCTRL_PIXGAINTAG0_READOUT_CLK_CNT (0X003CU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          I2CCTRL                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_I2CCTRL_I2CRDPREFETCHENABLE       (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_I2CCTRL_I2CRDPREFETCHENABLE       (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_I2CCTRL_I2CRDPREFETCHENABLE       (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_I2CCTRL_I2CBURSTMODE              (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_I2CCTRL_I2CBURSTMODE              (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_I2CCTRL_I2CBURSTMODE              (0X0006U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SEQUENCESTATUS                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_SEQUENCESTATUS_CALL_STACK_NO_LSB  (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_SEQUENCESTATUS_CALL_STACK_NO_LSB  (3U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SEQUENCESTATUS_CALL_STACK_NO_LSB  (0X0007U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_SEQUENCESTATUS_CALL_RPT_CNT_LSB   (3U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_SEQUENCESTATUS_CALL_RPT_CNT_LSB   (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SEQUENCESTATUS_CALL_RPT_CNT_LSB   (0X07F8U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_SEQUENCESTATUS_SEQ_STATE_MACHINE_STS (11U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_SEQUENCESTATUS_SEQ_STATE_MACHINE_STS (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SEQUENCESTATUS_SEQ_STATE_MACHINE_STS (0X0800U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_SEQUENCESTATUS_CALL_STACK_NO_MSB  (15U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_SEQUENCESTATUS_CALL_STACK_NO_MSB  (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SEQUENCESTATUS_CALL_STACK_NO_MSB  (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SYSTEMCLOCKCONTROL                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_SYSTEMCLOCKCONTROL_CLK_SYS_SEL    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_SYSTEMCLOCKCONTROL_CLK_SYS_SEL    (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SYSTEMCLOCKCONTROL_CLK_SYS_SEL    (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_SYSTEMCLOCKCONTROL_CLK_PROC_SEL   (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_SYSTEMCLOCKCONTROL_CLK_PROC_SEL   (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SYSTEMCLOCKCONTROL_CLK_PROC_SEL   (0X0004U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_SYSTEMCLOCKCONTROL_CLK_DE_SEL     (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_SYSTEMCLOCKCONTROL_CLK_DE_SEL     (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SYSTEMCLOCKCONTROL_CLK_DE_SEL     (0X0010U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CALLRPTCOUNT                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_CALLRPTCOUNT_CALL_RPT_CNT         (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_CALLRPTCOUNT_CALL_RPT_CNT         (12U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_CALLRPTCOUNT_CALL_RPT_CNT         (0X0FFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GTSWAP                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GTSWAP_GT01_SWAP                  (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GTSWAP_GT01_SWAP                  (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GTSWAP_GT01_SWAP                  (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GTSWAP_AMP_CLK_FINAL_INVERT       (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GTSWAP_AMP_CLK_FINAL_INVERT       (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GTSWAP_AMP_CLK_FINAL_INVERT       (0X0002U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          INTERRUPTENABLE                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_PSEUDO_ERR_EN (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_PSEUDO_ERR_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_PSEUDO_ERR_EN (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_CAL_STACK_OVRFLOW_ERR_EN (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_CAL_STACK_OVRFLOW_ERR_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_CAL_STACK_OVRFLOW_ERR_EN (0X0002U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_CAL_STACK_UNDR_RUN_ERR_EN (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_CAL_STACK_UNDR_RUN_ERR_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_CAL_STACK_UNDR_RUN_ERR_EN (0X0004U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_INVALID_OPCOD_ERR_EN (3U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_INVALID_OPCOD_ERR_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_INVALID_OPCOD_ERR_EN (0X0008U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_CSI_TX_PKT_CMD_ERR_EN (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_CSI_TX_PKT_CMD_ERR_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_CSI_TX_PKT_CMD_ERR_EN (0X0010U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_FIRMWARE_PARITY_ERR_EN (5U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_FIRMWARE_PARITY_ERR_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_FIRMWARE_PARITY_ERR_EN (0X0020U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_APB_TIMEOUT_ERR_EN (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_APB_TIMEOUT_ERR_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_APB_TIMEOUT_ERR_EN (0X0040U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_COLCORRECT_PARITY_ERR_EN (7U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_COLCORRECT_PARITY_ERR_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_COLCORRECT_PARITY_ERR_EN (0X0080U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_MIPI_CSI2_UNDERFLOW_ERR_EN (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_MIPI_CSI2_UNDERFLOW_ERR_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_MIPI_CSI2_UNDERFLOW_ERR_EN (0X0100U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_COMPRESSION_PARITY_ERR_EN (9U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_COMPRESSION_PARITY_ERR_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_COMPRESSION_PARITY_ERR_EN (0X0200U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_REG_WR_ERR_EN (10U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_REG_WR_ERR_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_REG_WR_ERR_EN (0X0400U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_INVALID_OPERAND_ERR_EN (11U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_INVALID_OPERAND_ERR_EN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_INVALID_OPERAND_ERR_EN (0X0800U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_USER_DEFINED_ERR_EN (12U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_USER_DEFINED_ERR_EN (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_INTERRUPTENABLE_INTERRUPT_USER_DEFINED_ERR_EN (0XF000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ERRORSET                                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_ERRORSET_SET_PSEUDO_ERROR         (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSET_SET_PSEUDO_ERROR         (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSET_SET_PSEUDO_ERROR         (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ERRORSET_SET_USER_DEFINED_ERROR   (12U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSET_SET_USER_DEFINED_ERROR   (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSET_SET_USER_DEFINED_ERROR   (0XF000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ERRORSTATUS                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_ERRORSTATUS_PSEUDO_ERROR          (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSTATUS_PSEUDO_ERROR          (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSTATUS_PSEUDO_ERROR          (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ERRORSTATUS_CALL_STACK_OVERFLOW_ERROR (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSTATUS_CALL_STACK_OVERFLOW_ERROR (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSTATUS_CALL_STACK_OVERFLOW_ERROR (0X0002U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ERRORSTATUS_CALL_STACK_UNDERRUN_ERROR (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSTATUS_CALL_STACK_UNDERRUN_ERROR (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSTATUS_CALL_STACK_UNDERRUN_ERROR (0X0004U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ERRORSTATUS_INVALID_OPCODE_ERROR  (3U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSTATUS_INVALID_OPCODE_ERROR  (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSTATUS_INVALID_OPCODE_ERROR  (0X0008U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ERRORSTATUS_BUF_CSI_TX_PKT_CMD_ERROR (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSTATUS_BUF_CSI_TX_PKT_CMD_ERROR (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSTATUS_BUF_CSI_TX_PKT_CMD_ERROR (0X0010U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ERRORSTATUS_FIRMWARE_PARITY_ERROR (5U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSTATUS_FIRMWARE_PARITY_ERROR (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSTATUS_FIRMWARE_PARITY_ERROR (0X0020U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ERRORSTATUS_APB_TIMEOUT_ERROR     (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSTATUS_APB_TIMEOUT_ERROR     (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSTATUS_APB_TIMEOUT_ERROR     (0X0040U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ERRORSTATUS_COLCORRECT_PARITY_ERROR (7U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSTATUS_COLCORRECT_PARITY_ERROR (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSTATUS_COLCORRECT_PARITY_ERROR (0X0080U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ERRORSTATUS_MIPI_CSI_2_UNDERFLOW_ERROR (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSTATUS_MIPI_CSI_2_UNDERFLOW_ERROR (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSTATUS_MIPI_CSI_2_UNDERFLOW_ERROR (0X0100U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ERRORSTATUS_COMPRESSION_PARITY_ERROR (9U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSTATUS_COMPRESSION_PARITY_ERROR (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSTATUS_COMPRESSION_PARITY_ERROR (0X0200U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ERRORSTATUS_REG_WR_ERROR          (10U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSTATUS_REG_WR_ERROR          (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSTATUS_REG_WR_ERROR          (0X0400U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ERRORSTATUS_INVALID_OPERAND_ERROR (11U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSTATUS_INVALID_OPERAND_ERROR (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSTATUS_INVALID_OPERAND_ERROR (0X0800U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ERRORSTATUS_USER_DEFINED_ERROR    (12U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRORSTATUS_USER_DEFINED_ERROR    (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRORSTATUS_USER_DEFINED_ERROR    (0XF000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPIOCTRL                                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPIOCTRL_GPIO_DIRECTION           (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOCTRL_GPIO_DIRECTION           (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOCTRL_GPIO_DIRECTION           (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPIOINPUT                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPIOINPUT_GPIO_IN                 (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOINPUT_GPIO_IN                 (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOINPUT_GPIO_IN                 (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPIOOUTPUTSET                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPIOOUTPUTSET_GPIO_OUT            (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOOUTPUTSET_GPIO_OUT            (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOOUTPUTSET_GPIO_OUT            (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPIOOUTPUTCLR                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPIOOUTPUTCLR_GPIO_OUT            (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOOUTPUTCLR_GPIO_OUT            (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOOUTPUTCLR_GPIO_OUT            (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PIXELINTERFACECTRL                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_PIXELINTERFACECTRL_READ           (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXELINTERFACECTRL_READ           (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXELINTERFACECTRL_READ           (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXELINTERFACECTRL_GLOBAL_RESET   (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXELINTERFACECTRL_GLOBAL_RESET   (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXELINTERFACECTRL_GLOBAL_RESET   (0X0002U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ROWCNTINCRCONTROL2                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_ROWCNTINCRCONTROL2_AUTO_ROW_INCR_STRIDE_MSB (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ROWCNTINCRCONTROL2_AUTO_ROW_INCR_STRIDE_MSB (5U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ROWCNTINCRCONTROL2_AUTO_ROW_INCR_STRIDE_MSB (0X01F0U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPIOFSYNCSNAPSHOT                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPIOFSYNCSNAPSHOT_GPIO_FSYNC_SNAPSHOT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOFSYNCSNAPSHOT_GPIO_FSYNC_SNAPSHOT (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOFSYNCSNAPSHOT_GPIO_FSYNC_SNAPSHOT (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          WAITFORSYNCSOURCE                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_DATAPATH_DONE_SOURCE (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_DATAPATH_DONE_SOURCE (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_DATAPATH_DONE_SOURCE (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_SHIFT_CHAIN_DONE_SOURCE (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_SHIFT_CHAIN_DONE_SOURCE (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_SHIFT_CHAIN_DONE_SOURCE (0X0002U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_MIPI_ULPS_END_SOURCE (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_MIPI_ULPS_END_SOURCE (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_MIPI_ULPS_END_SOURCE (0X0004U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_DE_DONE_SOURCE  (3U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_DE_DONE_SOURCE  (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_DE_DONE_SOURCE  (0X0008U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_SPIM_WRITE_DONE_SOURCE (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_SPIM_WRITE_DONE_SOURCE (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_SPIM_WRITE_DONE_SOURCE (0X0010U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_SPIM_READ_DONE  (5U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_SPIM_READ_DONE  (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_SPIM_READ_DONE  (0X0020U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_ADCPLL_LOCK_SOURCE (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_ADCPLL_LOCK_SOURCE (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_ADCPLL_LOCK_SOURCE (0X0040U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_TEMP_SENSOR_DONE_SOURCE (7U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_TEMP_SENSOR_DONE_SOURCE (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_TEMP_SENSOR_DONE_SOURCE (0X0080U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO2_SOURCE    (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO2_SOURCE    (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO2_SOURCE    (0X0100U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO3_SOURCE    (9U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO3_SOURCE    (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO3_SOURCE    (0X0200U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO4_SOURCE    (10U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO4_SOURCE    (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO4_SOURCE    (0X0400U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO5_SOURCE    (11U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO5_SOURCE    (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO5_SOURCE    (0X0800U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO6_SOURCE    (12U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO6_SOURCE    (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO6_SOURCE    (0X1000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO7_SOURCE    (13U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO7_SOURCE    (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO7_SOURCE    (0X2000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO8_SOURCE    (14U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO8_SOURCE    (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO8_SOURCE    (0X4000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO9_SOURCE    (15U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO9_SOURCE    (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_GPIO9_SOURCE    (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CTIMECTRL                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_CTIMECTRL_CTIME_ENABLE            (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_CTIMECTRL_CTIME_ENABLE            (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_CTIMECTRL_CTIME_ENABLE            (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CTIME_0                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_CTIME_0_CTIME_LO                  (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_CTIME_0_CTIME_LO                  (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_CTIME_0_CTIME_LO                  (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CTIME_1                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_CTIME_1_CTIME_HI                  (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_CTIME_1_CTIME_HI                  (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_CTIME_1_CTIME_HI                  (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CTIME_2                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_CTIME_2_CAPTURE_START_TIME_LO     (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_CTIME_2_CAPTURE_START_TIME_LO     (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_CTIME_2_CAPTURE_START_TIME_LO     (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CTIME_3                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_CTIME_3_CAPTURE_START_TIME_HI     (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_CTIME_3_CAPTURE_START_TIME_HI     (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_CTIME_3_CAPTURE_START_TIME_HI     (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CTIME_4                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_CTIME_4_CAPTURE_END_TIME_LO       (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_CTIME_4_CAPTURE_END_TIME_LO       (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_CTIME_4_CAPTURE_END_TIME_LO       (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CTIME_5                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_CTIME_5_CAPTURE_END_TIME_HI       (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_CTIME_5_CAPTURE_END_TIME_HI       (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_CTIME_5_CAPTURE_END_TIME_HI       (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SOFT_RESET                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_DE          (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_DE          (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_DE          (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_CSI         (3U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_CSI         (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_CSI         (0X0008U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_DATAPATH    (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_DATAPATH    (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_DATAPATH    (0X0010U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_SPI_MASTER  (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_SPI_MASTER  (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_SPI_MASTER  (0X0100U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_BOOT_MEM    (9U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_BOOT_MEM    (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_BOOT_MEM    (0X0200U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_PCM         (11U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_PCM         (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_PCM         (0X0800U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_LPS2        (12U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_LPS2        (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_LPS2        (0X1000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_SS          (13U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_SS          (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_SS          (0X2000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_LPS1        (14U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_LPS1        (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_SOFT_RESET_SOFT_RESET_LPS1        (0X4000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          WAITFORSYNCPOLARITY                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_WAITFORSYNCPOLARITY_WAIT_FOR_SYNC_POL (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_WAITFORSYNCPOLARITY_WAIT_FOR_SYNC_POL (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_WAITFORSYNCPOLARITY_WAIT_FOR_SYNC_POL (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USEQ_DFT                                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_USEQ_DFT_SEQRAM_MARGIN            (0U)           /* SeqRAM Margin */
#define BITL_USEQ_REGS_MAP1_USEQ_DFT_SEQRAM_MARGIN            (2U)           /* SeqRAM Margin */
#define BITM_USEQ_REGS_MAP1_USEQ_DFT_SEQRAM_MARGIN            (0X0003U)      /* SeqRAM Margin */
#define BITP_USEQ_REGS_MAP1_USEQ_DFT_SEQRAM_DST               (2U)           /* SeqRAM DST (Disable Self Timing) Register */
#define BITL_USEQ_REGS_MAP1_USEQ_DFT_SEQRAM_DST               (1U)           /* SeqRAM DST (Disable Self Timing) Register */
#define BITM_USEQ_REGS_MAP1_USEQ_DFT_SEQRAM_DST               (0X0004U)      /* SeqRAM DST (Disable Self Timing) Register */
#define BITP_USEQ_REGS_MAP1_USEQ_DFT_WAVERAM_MARGIN           (4U)           /* WaveRAM Margin */
#define BITL_USEQ_REGS_MAP1_USEQ_DFT_WAVERAM_MARGIN           (2U)           /* WaveRAM Margin */
#define BITM_USEQ_REGS_MAP1_USEQ_DFT_WAVERAM_MARGIN           (0X0030U)      /* WaveRAM Margin */
#define BITP_USEQ_REGS_MAP1_USEQ_DFT_WAVERAM_DST              (6U)           /* WaveRAM DST (Disable Self Timing) Register */
#define BITL_USEQ_REGS_MAP1_USEQ_DFT_WAVERAM_DST              (1U)           /* WaveRAM DST (Disable Self Timing) Register */
#define BITM_USEQ_REGS_MAP1_USEQ_DFT_WAVERAM_DST              (0X0040U)      /* WaveRAM DST (Disable Self Timing) Register */
#define BITP_USEQ_REGS_MAP1_USEQ_DFT_MAPRAM_MARGIN            (8U)           /* WaveRAM Margin */
#define BITL_USEQ_REGS_MAP1_USEQ_DFT_MAPRAM_MARGIN            (1U)           /* WaveRAM Margin */
#define BITM_USEQ_REGS_MAP1_USEQ_DFT_MAPRAM_MARGIN            (0X0100U)      /* WaveRAM Margin */

/* ----------------------------------------------------------------------------------------------------
          USEQ_PARITY                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_USEQ_PARITY_SEQRAM_PARITY_ERR     (0U)           /* Parity Error in Sequence RAM */
#define BITL_USEQ_REGS_MAP1_USEQ_PARITY_SEQRAM_PARITY_ERR     (1U)           /* Parity Error in Sequence RAM */
#define BITM_USEQ_REGS_MAP1_USEQ_PARITY_SEQRAM_PARITY_ERR     (0X0001U)      /* Parity Error in Sequence RAM */
#define BITP_USEQ_REGS_MAP1_USEQ_PARITY_MAPRAM_PARITY_ERR     (1U)           /* Parity error in map RAM */
#define BITL_USEQ_REGS_MAP1_USEQ_PARITY_MAPRAM_PARITY_ERR     (1U)           /* Parity error in map RAM */
#define BITM_USEQ_REGS_MAP1_USEQ_PARITY_MAPRAM_PARITY_ERR     (0X0002U)      /* Parity error in map RAM */
#define BITP_USEQ_REGS_MAP1_USEQ_PARITY_WAVERAM_PARITY_ERR    (2U)           /* Parity error in wave RAM */
#define BITL_USEQ_REGS_MAP1_USEQ_PARITY_WAVERAM_PARITY_ERR    (1U)           /* Parity error in wave RAM */
#define BITM_USEQ_REGS_MAP1_USEQ_PARITY_WAVERAM_PARITY_ERR    (0X0004U)      /* Parity error in wave RAM */

/* ----------------------------------------------------------------------------------------------------
          HSP_DFT                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_HSP_DFT_HSP_SPRAM_MARGIN          (0U)           /* HSP SPRAM Margin */
#define BITL_USEQ_REGS_MAP1_HSP_DFT_HSP_SPRAM_MARGIN          (2U)           /* HSP SPRAM Margin */
#define BITM_USEQ_REGS_MAP1_HSP_DFT_HSP_SPRAM_MARGIN          (0X0003U)      /* HSP SPRAM Margin */
#define BITP_USEQ_REGS_MAP1_HSP_DFT_HSP_SPRAM_DST             (2U)           /* HSP SPRAM DST (Disable Self Timing) Register */
#define BITL_USEQ_REGS_MAP1_HSP_DFT_HSP_SPRAM_DST             (1U)           /* HSP SPRAM DST (Disable Self Timing) Register */
#define BITM_USEQ_REGS_MAP1_HSP_DFT_HSP_SPRAM_DST             (0X0004U)      /* HSP SPRAM DST (Disable Self Timing) Register */
#define BITP_USEQ_REGS_MAP1_HSP_DFT_HSP_SPROM_MARGIN          (4U)           /* HSP SPROM Margin */
#define BITL_USEQ_REGS_MAP1_HSP_DFT_HSP_SPROM_MARGIN          (1U)           /* HSP SPROM Margin */
#define BITM_USEQ_REGS_MAP1_HSP_DFT_HSP_SPROM_MARGIN          (0X0010U)      /* HSP SPROM Margin */
#define BITP_USEQ_REGS_MAP1_HSP_DFT_HSP_1024X38_MARGIN        (5U)           /* HSP 1024x38 Margin */
#define BITL_USEQ_REGS_MAP1_HSP_DFT_HSP_1024X38_MARGIN        (1U)           /* HSP 1024x38 Margin */
#define BITM_USEQ_REGS_MAP1_HSP_DFT_HSP_1024X38_MARGIN        (0X0020U)      /* HSP 1024x38 Margin */
#define BITP_USEQ_REGS_MAP1_HSP_DFT_HSP_256X76_MARGIN         (6U)           /* HSP 256x76 Margin */
#define BITL_USEQ_REGS_MAP1_HSP_DFT_HSP_256X76_MARGIN         (1U)           /* HSP 256x76 Margin */
#define BITM_USEQ_REGS_MAP1_HSP_DFT_HSP_256X76_MARGIN         (0X0040U)      /* HSP 256x76 Margin */
#define BITP_USEQ_REGS_MAP1_HSP_DFT_HSP_128X76_MARGIN         (7U)           /* HSP 128x76 Margin */
#define BITL_USEQ_REGS_MAP1_HSP_DFT_HSP_128X76_MARGIN         (1U)           /* HSP 128x76 Margin */
#define BITM_USEQ_REGS_MAP1_HSP_DFT_HSP_128X76_MARGIN         (0X0080U)      /* HSP 128x76 Margin */

/* ----------------------------------------------------------------------------------------------------
          PCCOND                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_PCCOND_PC                         (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PCCOND_PC                         (12U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_PCCOND_PC                         (0X0FFFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PCCOND_COND                       (15U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_PCCOND_COND                       (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PCCOND_COND                       (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR0                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR0_GPR_R0                      (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR0_GPR_R0                      (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR0_GPR_R0                      (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR1                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR1_GPR_R1                      (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR1_GPR_R1                      (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR1_GPR_R1                      (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR2                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR2_GPR_R2                      (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR2_GPR_R2                      (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR2_GPR_R2                      (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR3                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR3_GPR_R3                      (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR3_GPR_R3                      (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR3_GPR_R3                      (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR4                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR4_GPR_R4                      (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR4_GPR_R4                      (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR4_GPR_R4                      (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR5                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR5_GPR_R5                      (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR5_GPR_R5                      (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR5_GPR_R5                      (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR6                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR6_GPR_R6                      (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR6_GPR_R6                      (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR6_GPR_R6                      (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR7                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR7_GPR_R7                      (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR7_GPR_R7                      (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR7_GPR_R7                      (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR8                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR8_GPR_R8                      (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR8_GPR_R8                      (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR8_GPR_R8                      (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR9                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR9_GPR_R9                      (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR9_GPR_R9                      (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR9_GPR_R9                      (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR10                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR10_GPR_R10                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR10_GPR_R10                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR10_GPR_R10                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR11                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR11_GPR_R11                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR11_GPR_R11                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR11_GPR_R11                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR12                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR12_GPR_R12                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR12_GPR_R12                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR12_GPR_R12                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR13                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR13_GPR_R13                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR13_GPR_R13                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR13_GPR_R13                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR14                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR14_GPR_R14                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR14_GPR_R14                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR14_GPR_R14                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR15                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR15_GPR_R15                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR15_GPR_R15                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR15_GPR_R15                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMPCLKCTRL                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_AMPCLKCTRL_AMP_CLK_RISE_CNVT_CNT  (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_AMPCLKCTRL_AMP_CLK_RISE_CNVT_CNT  (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_AMPCLKCTRL_AMP_CLK_RISE_CNVT_CNT  (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_AMPCLKCTRL_AMP_CLK_RISE_CLK_CNT   (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_AMPCLKCTRL_AMP_CLK_RISE_CLK_CNT   (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_AMPCLKCTRL_AMP_CLK_RISE_CLK_CNT   (0X003CU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_AMPCLKCTRL_AMP_CLK_FALL_CNVT_CNT  (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_AMPCLKCTRL_AMP_CLK_FALL_CNVT_CNT  (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_AMPCLKCTRL_AMP_CLK_FALL_CNVT_CNT  (0X00C0U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_AMPCLKCTRL_AMP_CLK_FALL_CLK_CNT   (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_AMPCLKCTRL_AMP_CLK_FALL_CLK_CNT   (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_AMPCLKCTRL_AMP_CLK_FALL_CLK_CNT   (0X0F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMPCLK2CTRL                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_AMPCLK2CTRL_AMP_CLK2_RISE_CNVT_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_AMPCLK2CTRL_AMP_CLK2_RISE_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_AMPCLK2CTRL_AMP_CLK2_RISE_CNVT_CNT (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_AMPCLK2CTRL_AMP_CLK2_RISE_CLK_CNT (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_AMPCLK2CTRL_AMP_CLK2_RISE_CLK_CNT (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_AMPCLK2CTRL_AMP_CLK2_RISE_CLK_CNT (0X003CU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_AMPCLK2CTRL_AMP_CLK2_FALL_CNVT_CNT (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_AMPCLK2CTRL_AMP_CLK2_FALL_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_AMPCLK2CTRL_AMP_CLK2_FALL_CNVT_CNT (0X00C0U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_AMPCLK2CTRL_AMP_CLK2_FALL_CLK_CNT (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_AMPCLK2CTRL_AMP_CLK2_FALL_CLK_CNT (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_AMPCLK2CTRL_AMP_CLK2_FALL_CLK_CNT (0X0F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMPCLK3CTRL1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_AMPCLK3CTRL1_AMP_CLK3_RISE_CLK_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_AMPCLK3CTRL1_AMP_CLK3_RISE_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_AMPCLK3CTRL1_AMP_CLK3_RISE_CLK_CNT (0X00FFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_AMPCLK3CTRL1_AMP_CLK3_FALL_CLK_CNT (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_AMPCLK3CTRL1_AMP_CLK3_FALL_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_AMPCLK3CTRL1_AMP_CLK3_FALL_CLK_CNT (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMPCLK3CTRL2                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_AMPCLK3CTRL2_AMP_CLK3_RISE_CNVT_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_AMPCLK3CTRL2_AMP_CLK3_RISE_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_AMPCLK3CTRL2_AMP_CLK3_RISE_CNVT_CNT (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_AMPCLK3CTRL2_AMP_CLK3_FALL_CNVT_CNT (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_AMPCLK3CTRL2_AMP_CLK3_FALL_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_AMPCLK3CTRL2_AMP_CLK3_FALL_CNVT_CNT (0X00C0U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          NOISERESETCTRL1                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_NOISERESETCTRL1_NOISE_RESET_RISE_CLK_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_NOISERESETCTRL1_NOISE_RESET_RISE_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_NOISERESETCTRL1_NOISE_RESET_RISE_CLK_CNT (0X00FFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_NOISERESETCTRL1_NOISE_RESET_FALL_CLK_CNT (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_NOISERESETCTRL1_NOISE_RESET_FALL_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_NOISERESETCTRL1_NOISE_RESET_FALL_CLK_CNT (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          NOISERESETCTRL2                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_NOISERESETCTRL2_NOISE_RESET_RISE_CNVT_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_NOISERESETCTRL2_NOISE_RESET_RISE_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_NOISERESETCTRL2_NOISE_RESET_RISE_CNVT_CNT (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_NOISERESETCTRL2_NOISE_RESET_FALL_CNVT_CNT (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_NOISERESETCTRL2_NOISE_RESET_FALL_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_NOISERESETCTRL2_NOISE_RESET_FALL_CNVT_CNT (0X00C0U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PIXRESETCTRL1                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_PIXRESETCTRL1_PIX_RESET_RISE_CLK_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXRESETCTRL1_PIX_RESET_RISE_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXRESETCTRL1_PIX_RESET_RISE_CLK_CNT (0X00FFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXRESETCTRL1_PIX_RESET_FALL_CLK_CNT (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXRESETCTRL1_PIX_RESET_FALL_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXRESETCTRL1_PIX_RESET_FALL_CLK_CNT (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PIXRESETCTRL2                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_PIXRESETCTRL2_PIX_RESET_RISE_CNVT_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXRESETCTRL2_PIX_RESET_RISE_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXRESETCTRL2_PIX_RESET_RISE_CNVT_CNT (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PIXRESETCTRL2_PIX_RESET_FALL_CNVT_CNT (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PIXRESETCTRL2_PIX_RESET_FALL_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PIXRESETCTRL2_PIX_RESET_FALL_CNVT_CNT (0X00C0U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPIOPINFUNC1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO0_PIN_FUNC       (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO0_PIN_FUNC       (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO0_PIN_FUNC       (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO1_PIN_FUNC       (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO1_PIN_FUNC       (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO1_PIN_FUNC       (0X000CU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO2_PIN_FUNC       (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO2_PIN_FUNC       (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO2_PIN_FUNC       (0X0030U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO3_PIN_FUNC       (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO3_PIN_FUNC       (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO3_PIN_FUNC       (0X00C0U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO4_PIN_FUNC       (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO4_PIN_FUNC       (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO4_PIN_FUNC       (0X0300U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO5_PIN_FUNC       (10U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO5_PIN_FUNC       (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO5_PIN_FUNC       (0X0C00U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO6_PIN_FUNC       (12U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO6_PIN_FUNC       (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO6_PIN_FUNC       (0X3000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO7_PIN_FUNC       (14U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO7_PIN_FUNC       (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC1_GPIO7_PIN_FUNC       (0XC000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPIOPINFUNC2                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO8_PIN_FUNC       (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO8_PIN_FUNC       (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO8_PIN_FUNC       (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO9_PIN_FUNC       (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO9_PIN_FUNC       (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO9_PIN_FUNC       (0X000CU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO10_PIN_FUNC      (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO10_PIN_FUNC      (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO10_PIN_FUNC      (0X0030U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO11_PIN_FUNC      (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO11_PIN_FUNC      (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO11_PIN_FUNC      (0X00C0U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO12_PIN_FUNC      (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO12_PIN_FUNC      (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO12_PIN_FUNC      (0X0300U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO13_PIN_FUNC      (10U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO13_PIN_FUNC      (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO13_PIN_FUNC      (0X0C00U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO14_PIN_FUNC      (12U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO14_PIN_FUNC      (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO14_PIN_FUNC      (0X3000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO15_PIN_FUNC      (14U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO15_PIN_FUNC      (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPIOPINFUNC2_GPIO15_PIN_FUNC      (0XC000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USEQ_DBGMUX                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_USEQ_DBGMUX_MUXGRP_SEL            (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQ_DBGMUX_MUXGRP_SEL            (3U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQ_DBGMUX_MUXGRP_SEL            (0X0007U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQ_DBGMUX_CMN_SEL0              (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQ_DBGMUX_CMN_SEL0              (3U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQ_DBGMUX_CMN_SEL0              (0X0700U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQ_DBGMUX_CMN_SEL1              (11U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQ_DBGMUX_CMN_SEL1              (3U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQ_DBGMUX_CMN_SEL1              (0X3800U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQ_DBGMUX_DBG_ENABLE            (15U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQ_DBGMUX_DBG_ENABLE            (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQ_DBGMUX_DBG_ENABLE            (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USEQ_CHIP_DBGMUX                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_IPMUX_SEL        (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_IPMUX_SEL        (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_IPMUX_SEL        (0X000FU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_DBG_ENABLE       (7U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_DBG_ENABLE       (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_DBG_ENABLE       (0X0080U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_MM_IPMUX_SEL     (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_MM_IPMUX_SEL     (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_MM_IPMUX_SEL     (0X0300U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_MM_ENABLE        (15U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_MM_ENABLE        (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_MM_ENABLE        (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          MM_CTRL                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_MM_CTRL_MM_OUT_SEL                (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_MM_CTRL_MM_OUT_SEL                (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_MM_CTRL_MM_OUT_SEL                (0X000FU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ERRJMPADDR                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_ERRJMPADDR_USEQERRJUMPADDR        (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ERRJMPADDR_USEQERRJUMPADDR        (12U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_ERRJMPADDR_USEQERRJUMPADDR        (0X0FFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          STOPERRENA                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_STOPERRENA_PSEUDO_ERREN           (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_STOPERRENA_PSEUDO_ERREN           (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_STOPERRENA_PSEUDO_ERREN           (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_STOPERRENA_CALL_STACK_OVERFLOW_ERREN (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_STOPERRENA_CALL_STACK_OVERFLOW_ERREN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_STOPERRENA_CALL_STACK_OVERFLOW_ERREN (0X0002U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_STOPERRENA_CALL_STACK_UNDERRUN_ERREN (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_STOPERRENA_CALL_STACK_UNDERRUN_ERREN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_STOPERRENA_CALL_STACK_UNDERRUN_ERREN (0X0004U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_STOPERRENA_INVALID_OPCODE_ERREN   (3U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_STOPERRENA_INVALID_OPCODE_ERREN   (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_STOPERRENA_INVALID_OPCODE_ERREN   (0X0008U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_STOPERRENA_BUF_MEMORYT_OVERFLOW_ERREN (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_STOPERRENA_BUF_MEMORYT_OVERFLOW_ERREN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_STOPERRENA_BUF_MEMORYT_OVERFLOW_ERREN (0X0010U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_STOPERRENA_FIRMWARE_PARITY_ERREN  (5U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_STOPERRENA_FIRMWARE_PARITY_ERREN  (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_STOPERRENA_FIRMWARE_PARITY_ERREN  (0X0020U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_STOPERRENA_FRAME_HEADERT_OVERFLOW_ERREN (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_STOPERRENA_FRAME_HEADERT_OVERFLOW_ERREN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_STOPERRENA_FRAME_HEADERT_OVERFLOW_ERREN (0X0040U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_STOPERRENA_COLCORRECT_PARITY_ERREN (7U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_STOPERRENA_COLCORRECT_PARITY_ERREN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_STOPERRENA_COLCORRECT_PARITY_ERREN (0X0080U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_STOPERRENA_MIPI_CSI_2T_UNDERFLOW_ERREN (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_STOPERRENA_MIPI_CSI_2T_UNDERFLOW_ERREN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_STOPERRENA_MIPI_CSI_2T_UNDERFLOW_ERREN (0X0100U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_STOPERRENA_COMPRESSON_PARITY_ERREN (9U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_STOPERRENA_COMPRESSON_PARITY_ERREN (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_STOPERRENA_COMPRESSON_PARITY_ERREN (0X0200U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_STOPERRENA_REG_WR_ERREN           (10U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_STOPERRENA_REG_WR_ERREN           (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_STOPERRENA_REG_WR_ERREN           (0X0400U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_STOPERRENA_INVALID_OPERAND_ERREN  (11U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_STOPERRENA_INVALID_OPERAND_ERREN  (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_STOPERRENA_INVALID_OPERAND_ERREN  (0X0800U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_STOPERRENA_USER_DEFINED_ERREN     (12U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_STOPERRENA_USER_DEFINED_ERREN     (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_STOPERRENA_USER_DEFINED_ERREN     (0XF000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ADCCNVTCTRL1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_ADCCNVTCTRL1_ASTART               (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ADCCNVTCTRL1_ASTART               (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ADCCNVTCTRL1_ASTART               (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ADCCNVTCTRL1_REPEAT_COUNT         (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ADCCNVTCTRL1_REPEAT_COUNT         (12U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_ADCCNVTCTRL1_REPEAT_COUNT         (0X1FFEU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ADCCNVTCTRL1_POL                  (14U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_ADCCNVTCTRL1_POL                  (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ADCCNVTCTRL1_POL                  (0X4000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ADCCNVTCTRL1_ABUSY                (15U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_ADCCNVTCTRL1_ABUSY                (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ADCCNVTCTRL1_ABUSY                (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ADCCNVTCTRL2                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_ADCCNVTCTRL2_EXEC_LENGTH          (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ADCCNVTCTRL2_EXEC_LENGTH          (14U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_ADCCNVTCTRL2_EXEC_LENGTH          (0X3FFFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ADCCNVTCTRL2_CDIV                 (14U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_ADCCNVTCTRL2_CDIV                 (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ADCCNVTCTRL2_CDIV                 (0XC000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ADCCNVTCTRL3                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_ADCCNVTCTRL3_FALL_TIME            (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ADCCNVTCTRL3_FALL_TIME            (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ADCCNVTCTRL3_FALL_TIME            (0X00FFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_ADCCNVTCTRL3_RISE_TIME            (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ADCCNVTCTRL3_RISE_TIME            (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ADCCNVTCTRL3_RISE_TIME            (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ADCCNVTCTRL4                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_ADCCNVTCTRL4_ADC_CNVT_DELAY       (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_ADCCNVTCTRL4_ADC_CNVT_DELAY       (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_ADCCNVTCTRL4_ADC_CNVT_DELAY       (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GAINTAG1CLKCTRL1                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GAINTAG1CLKCTRL1_GAINTAG1_CLK_RISE_CLK_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAG1CLKCTRL1_GAINTAG1_CLK_RISE_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAG1CLKCTRL1_GAINTAG1_CLK_RISE_CLK_CNT (0X00FFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GAINTAG1CLKCTRL1_GAINTAG1_CLK_FALL_CLK_CNT (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAG1CLKCTRL1_GAINTAG1_CLK_FALL_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAG1CLKCTRL1_GAINTAG1_CLK_FALL_CLK_CNT (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GAINTAG1CLKCTRL2                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GAINTAG1CLKCTRL2_GAINTAG1_CLK_RISE_CNVT_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAG1CLKCTRL2_GAINTAG1_CLK_RISE_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAG1CLKCTRL2_GAINTAG1_CLK_RISE_CNVT_CNT (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GAINTAG1CLKCTRL2_GAINTAG1_CLK_FALL_CNVT_CNT (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAG1CLKCTRL2_GAINTAG1_CLK_FALL_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAG1CLKCTRL2_GAINTAG1_CLK_FALL_CNVT_CNT (0X00C0U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GAINTAGTHRESHCTRL1                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL1_GAINTAG_THRESH_RISE_CLK_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL1_GAINTAG_THRESH_RISE_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL1_GAINTAG_THRESH_RISE_CLK_CNT (0X00FFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL1_GAINTAG_THRESH_FALL_CLK_CNT (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL1_GAINTAG_THRESH_FALL_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL1_GAINTAG_THRESH_FALL_CLK_CNT (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GAINTAGTHRESHCTRL2                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL2_GAINTAG_THRESH_RISE_CNVT_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL2_GAINTAG_THRESH_RISE_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL2_GAINTAG_THRESH_RISE_CNVT_CNT (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL2_GAINTAG_THRESH_FALL_CNVT_CNT (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL2_GAINTAG_THRESH_FALL_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL2_GAINTAG_THRESH_FALL_CNVT_CNT (0X00C0U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GAINTAGTHRESHSEL                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GAINTAGTHRESHSEL_GAINTAG_THRESH_SEL (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAGTHRESHSEL_GAINTAG_THRESH_SEL (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAGTHRESHSEL_GAINTAG_THRESH_SEL (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GAINTAGTHRESHSEL_GAINTAG_THRESH_STATIC (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAGTHRESHSEL_GAINTAG_THRESH_STATIC (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAGTHRESHSEL_GAINTAG_THRESH_STATIC (0X0002U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GAINTAG0CLKCTRL1                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GAINTAG0CLKCTRL1_GAINTAG0_CLK_RISE_CLK_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAG0CLKCTRL1_GAINTAG0_CLK_RISE_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAG0CLKCTRL1_GAINTAG0_CLK_RISE_CLK_CNT (0X00FFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GAINTAG0CLKCTRL1_GAINTAG0_CLK_FALL_CLK_CNT (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAG0CLKCTRL1_GAINTAG0_CLK_FALL_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAG0CLKCTRL1_GAINTAG0_CLK_FALL_CLK_CNT (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GAINTAG0CLKCTRL2                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GAINTAG0CLKCTRL2_GAINTAG0_CLK_RISE_CNVT_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAG0CLKCTRL2_GAINTAG0_CLK_RISE_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAG0CLKCTRL2_GAINTAG0_CLK_RISE_CNVT_CNT (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_GAINTAG0CLKCTRL2_GAINTAG0_CLK_FALL_CNVT_CNT (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GAINTAG0CLKCTRL2_GAINTAG0_CLK_FALL_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_GAINTAG0CLKCTRL2_GAINTAG0_CLK_FALL_CNVT_CNT (0X00C0U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FORCESFCTRL1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_FORCESFCTRL1_FORCE_SF_RISE_CLK_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FORCESFCTRL1_FORCE_SF_RISE_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FORCESFCTRL1_FORCE_SF_RISE_CLK_CNT (0X00FFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FORCESFCTRL1_FORCE_SF_FALL_CLK_CNT (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FORCESFCTRL1_FORCE_SF_FALL_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FORCESFCTRL1_FORCE_SF_FALL_CLK_CNT (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FORCESFCTRL2                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_FORCESFCTRL2_FORCE_SF_RISE_CNVT_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FORCESFCTRL2_FORCE_SF_RISE_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FORCESFCTRL2_FORCE_SF_RISE_CNVT_CNT (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FORCESFCTRL2_FORCE_SF_FALL_CNVT_CNT (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FORCESFCTRL2_FORCE_SF_FALL_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FORCESFCTRL2_FORCE_SF_FALL_CNVT_CNT (0X00C0U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FORCEIPDACTRL1                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_FORCEIPDACTRL1_FORCE_IPDA_RISE_CLK_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FORCEIPDACTRL1_FORCE_IPDA_RISE_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FORCEIPDACTRL1_FORCE_IPDA_RISE_CLK_CNT (0X00FFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FORCEIPDACTRL1_FORCE_IPDA_FALL_CLK_CNT (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FORCEIPDACTRL1_FORCE_IPDA_FALL_CLK_CNT (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FORCEIPDACTRL1_FORCE_IPDA_FALL_CLK_CNT (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FORCEIPDACTRL2                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_FORCEIPDACTRL2_FORCE_IPDA_RISE_CNVT_CNT (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FORCEIPDACTRL2_FORCE_IPDA_RISE_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FORCEIPDACTRL2_FORCE_IPDA_RISE_CNVT_CNT (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FORCEIPDACTRL2_FORCE_IPDA_FALL_CNVT_CNT (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FORCEIPDACTRL2_FORCE_IPDA_FALL_CNVT_CNT (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FORCEIPDACTRL2_FORCE_IPDA_FALL_CNVT_CNT (0X00C0U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USEQRAMLOADADDR                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_USEQRAMLOADADDR_LD_RAM_SEL        (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQRAMLOADADDR_LD_RAM_SEL        (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQRAMLOADADDR_LD_RAM_SEL        (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQRAMLOADADDR_LD_ADDR           (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQRAMLOADADDR_LD_ADDR           (12U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQRAMLOADADDR_LD_ADDR           (0X3FFCU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USEQRAMRDSTADDR                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_USEQRAMRDSTADDR_RD_RAM_SEL        (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQRAMRDSTADDR_RD_RAM_SEL        (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQRAMRDSTADDR_RD_RAM_SEL        (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_USEQRAMRDSTADDR_RD_ADDR           (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQRAMRDSTADDR_RD_ADDR           (12U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQRAMRDSTADDR_RD_ADDR           (0X3FFCU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USEQRAMLOADDATA                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_USEQRAMLOADDATA_LD_DATA           (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQRAMLOADDATA_LD_DATA           (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQRAMLOADDATA_LD_DATA           (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USEQRAMLOADDATAALIAS                                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_USEQRAMLOADDATAALIAS_LD_DATA_ALIAS (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQRAMLOADDATAALIAS_LD_DATA_ALIAS (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQRAMLOADDATAALIAS_LD_DATA_ALIAS (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USEQRAMRDDATA                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_USEQRAMRDDATA_USEQ_RAM_RD_DATA    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQRAMRDDATA_USEQ_RAM_RD_DATA    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQRAMRDDATA_USEQ_RAM_RD_DATA    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USEQRAMRDDATAALIAS                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_USEQRAMRDDATAALIAS_USEQ_RAM_RD_DATA_ALIAS (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_USEQRAMRDDATAALIAS_USEQ_RAM_RD_DATA_ALIAS (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_USEQRAMRDDATAALIAS_USEQ_RAM_RD_DATA_ALIAS (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PWM_CTRL_0                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_PWM_CTRL_0_PWM_PERIOD             (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PWM_CTRL_0_PWM_PERIOD             (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PWM_CTRL_0_PWM_PERIOD             (0X00FFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PWM_CTRL_0_PWM_STEP_SIZE          (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PWM_CTRL_0_PWM_STEP_SIZE          (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PWM_CTRL_0_PWM_STEP_SIZE          (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PWM_CTRL_1                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_PWM_CTRL_1_PWM_REPEAT_PER_STEP    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_PWM_CTRL_1_PWM_REPEAT_PER_STEP    (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PWM_CTRL_1_PWM_REPEAT_PER_STEP    (0X00FFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PWM_CTRL_1_CURRENT_CG             (14U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_PWM_CTRL_1_CURRENT_CG             (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PWM_CTRL_1_CURRENT_CG             (0X4000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_PWM_CTRL_1_PWM_BUSY               (15U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_PWM_CTRL_1_PWM_BUSY               (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_PWM_CTRL_1_PWM_BUSY               (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FSYNCCTRL                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_FSYNC_OUT_EN            (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_FSYNC_OUT_EN            (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_FSYNC_OUT_EN            (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_FSYNC_OUT_MODE          (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_FSYNC_OUT_MODE          (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_FSYNC_OUT_MODE          (0X0002U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_EXT_FSYNC_ESH           (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_EXT_FSYNC_ESH           (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_EXT_FSYNC_ESH           (0X0004U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_LSMODCTR_ESH            (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_LSMODCTR_ESH            (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_LSMODCTR_ESH            (0X0010U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_LSMODCTR_AUTORESTART    (5U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_LSMODCTR_AUTORESTART    (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_LSMODCTR_AUTORESTART    (0X0020U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_LSMODCTR_FREEZE         (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_LSMODCTR_FREEZE         (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_LSMODCTR_FREEZE         (0X0040U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_LSMODCTR_START          (7U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_LSMODCTR_START          (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_LSMODCTR_START          (0X0080U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_INTCTR_ESH              (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_INTCTR_ESH              (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_INTCTR_ESH              (0X0100U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_INTCTR_AUTORESTART      (9U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_INTCTR_AUTORESTART      (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_INTCTR_AUTORESTART      (0X0200U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_INTCTR_FREEZE           (10U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_INTCTR_FREEZE           (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_INTCTR_FREEZE           (0X0400U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_INTCTR_START            (11U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_INTCTR_START            (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_INTCTR_START            (0X0800U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_SYSCTR_ESH              (12U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_SYSCTR_ESH              (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_SYSCTR_ESH              (0X1000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_SYSCTR_AUTORESTART      (13U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_SYSCTR_AUTORESTART      (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_SYSCTR_AUTORESTART      (0X2000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_SYSCTR_FREEZE           (14U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_SYSCTR_FREEZE           (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_SYSCTR_FREEZE           (0X4000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCCTRL_SYSCTR_START            (15U)          /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCCTRL_SYSCTR_START            (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCCTRL_SYSCTR_START            (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FSYNCSTATUS                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_FSYNCSTATUS_FSYNC_FLAG            (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCSTATUS_FSYNC_FLAG            (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCSTATUS_FSYNC_FLAG            (0X0001U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCSTATUS_LSMODCTR_FLAG         (1U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCSTATUS_LSMODCTR_FLAG         (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCSTATUS_LSMODCTR_FLAG         (0X0002U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCSTATUS_INTCTR_FLAG           (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCSTATUS_INTCTR_FLAG           (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCSTATUS_INTCTR_FLAG           (0X0004U)      /* No description provided */
#define BITP_USEQ_REGS_MAP1_FSYNCSTATUS_SYSCTR_FLAG           (3U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCSTATUS_SYSCTR_FLAG           (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCSTATUS_SYSCTR_FLAG           (0X0008U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FSYNCLSMODCNTR_0                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_FSYNCLSMODCNTR_0_FSYNC_LSMOD_COUNTER_0 (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCLSMODCNTR_0_FSYNC_LSMOD_COUNTER_0 (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCLSMODCNTR_0_FSYNC_LSMOD_COUNTER_0 (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FSYNCLSMODCNTR_1                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_FSYNCLSMODCNTR_1_FSYNC_LSMOD_COUNTER_1 (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCLSMODCNTR_1_FSYNC_LSMOD_COUNTER_1 (4U)           /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCLSMODCNTR_1_FSYNC_LSMOD_COUNTER_1 (0X000FU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FSYNCINTCNTR_0                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_FSYNCINTCNTR_0_FSYNC_INT_COUNTER_0 (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCINTCNTR_0_FSYNC_INT_COUNTER_0 (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCINTCNTR_0_FSYNC_INT_COUNTER_0 (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FSYNCINTCNTR_1                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_FSYNCINTCNTR_1_FSYNC_INT_COUNTER_1 (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCINTCNTR_1_FSYNC_INT_COUNTER_1 (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCINTCNTR_1_FSYNC_INT_COUNTER_1 (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FSYNCSYSCNTR_0                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_FSYNCSYSCNTR_0_FSYNC_SYS_COUNTER_0 (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCSYSCNTR_0_FSYNC_SYS_COUNTER_0 (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCSYSCNTR_0_FSYNC_SYS_COUNTER_0 (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          FSYNCSYSCNTR_1                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_FSYNCSYSCNTR_1_FSYNC_SYS_COUNTER_1 (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_FSYNCSYSCNTR_1_FSYNC_SYS_COUNTER_1 (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_FSYNCSYSCNTR_1_FSYNC_SYS_COUNTER_1 (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR16                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR16_GPR_R16                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR16_GPR_R16                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR16_GPR_R16                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR17                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR17_GPR_R17                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR17_GPR_R17                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR17_GPR_R17                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR18                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR18_GPR_R18                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR18_GPR_R18                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR18_GPR_R18                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR19                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR19_GPR_R19                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR19_GPR_R19                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR19_GPR_R19                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR20                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR20_GPR_R20                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR20_GPR_R20                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR20_GPR_R20                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR21                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR21_GPR_R21                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR21_GPR_R21                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR21_GPR_R21                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR22                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR22_GPR_R22                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR22_GPR_R22                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR22_GPR_R22                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR23                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR23_GPR_R23                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR23_GPR_R23                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR23_GPR_R23                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR24                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR24_GPR_R24                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR24_GPR_R24                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR24_GPR_R24                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR25                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR25_GPR_R25                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR25_GPR_R25                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR25_GPR_R25                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR26                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR26_GPR_R26                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR26_GPR_R26                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR26_GPR_R26                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR27                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR27_GPR_R27                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR27_GPR_R27                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR27_GPR_R27                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR28                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR28_GPR_R28                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR28_GPR_R28                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR28_GPR_R28                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR29                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR29_GPR_R29                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR29_GPR_R29                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR29_GPR_R29                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR30                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR30_GPR_R30                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR30_GPR_R30                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR30_GPR_R30                    (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          GPRR31                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP1_GPRR31_GPR_R31                    (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP1_GPRR31_GPR_R31                    (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP1_GPRR31_GPR_R31                    (0XFFFFU)      /* No description provided */

#endif  /* end ifndef USEQ_REGS_MAP1_ADDR_RDEF_H_ */

/* ====================================================================================================
        AI_REGS_YODA Module Instances Address and Mask Definitions
   ==================================================================================================== */
#define INST_AI_REGS                                             (0X00000100U)    /* ai_regs: */


#ifndef AI_REGS_YODA_ADDR_RDEF_H_
#define AI_REGS_YODA_ADDR_RDEF_H_    /* AI_REGS_YODA: Your module description, here. */

#define MASK_AI_REGS_YODA                                        (0X000000FFU)    /* AI_REGS_YODA: Your module description, here. */

/* ====================================================================================================
        AI_REGS_YODA Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_AI_REGS_YODA_ADC_CTRL0_S1                           (0X4) 
#define RSTVAL_AI_REGS_YODA_ADC_CTRL1_S1                           (0X3333) 
#define RSTVAL_AI_REGS_YODA_ADC_CTRL2_S1                           (0X0) 
#define RSTVAL_AI_REGS_YODA_ADCPLL_CTRL0_S1                        (0X1C10) 
#define RSTVAL_AI_REGS_YODA_ADCPLL_CTRL1_S1                        (0X810) 
#define RSTVAL_AI_REGS_YODA_ADCPLL_CTRL2_S1                        (0X114) 
#define RSTVAL_AI_REGS_YODA_AMP_CTRL0_S1                           (0X0) 
#define RSTVAL_AI_REGS_YODA_AMP_CTRL1_S1                           (0X0) 
#define RSTVAL_AI_REGS_YODA_AMP_CTRL2_S1                           (0X0) 
#define RSTVAL_AI_REGS_YODA_CHIP_ID                                (0X5931) 
#define RSTVAL_AI_REGS_YODA_CKGEN_CTRL                             (0X0) 
#define RSTVAL_AI_REGS_YODA_CKGEN_S1                               (0X0) 
#define RSTVAL_AI_REGS_YODA_CLK_CTRL                               (0X0) 
#define RSTVAL_AI_REGS_YODA_CLK_DE_CTRL_S1                         (0X102) 
#define RSTVAL_AI_REGS_YODA_CLK_LVDSTX_S1                          (0X2) 
#define RSTVAL_AI_REGS_YODA_CLKTREE0                               (0XF) 
#define RSTVAL_AI_REGS_YODA_CLKTREE_S1                             (0X3E0) 
#define RSTVAL_AI_REGS_YODA_DAC_CTRL1                              (0X0) 
#define RSTVAL_AI_REGS_YODA_DAC_CTRL2                              (0X83FF) 
#define RSTVAL_AI_REGS_YODA_DAC_CTRL0_S1                           (0XAA) 
#define RSTVAL_AI_REGS_YODA_DAC_CTRL1_S1                           (0X4) 
#define RSTVAL_AI_REGS_YODA_DAC_CTRL2_S1                           (0X1F1F) 
#define RSTVAL_AI_REGS_YODA_DAC_CTRL3_S1                           (0X1F1F) 
#define RSTVAL_AI_REGS_YODA_DAC_DATA                               (0X0) 
#define RSTVAL_AI_REGS_YODA_IPDA_CTRL_S1                           (0X20) 
#define RSTVAL_AI_REGS_YODA_LS_LVDSTX_S1                           (0X2) 
#define RSTVAL_AI_REGS_YODA_LSCTRL0_S1                             (0X88) 
#define RSTVAL_AI_REGS_YODA_LSMOD_EN                               (0X3A) 
#define RSTVAL_AI_REGS_YODA_ROW_CTRL                               (0X200) 
#define RSTVAL_AI_REGS_YODA_PLL_CTRL                               (0X1033) 
#define RSTVAL_AI_REGS_YODA_PLL_STATUS                             (0X0) 
#define RSTVAL_AI_REGS_YODA_POWER_DOWN_0                           (0X5) 
#define RSTVAL_AI_REGS_YODA_POWER_DOWN_ADC_OTHERS                  (0X7F) 
#define RSTVAL_AI_REGS_YODA_POWER_DOWN_READOUT                     (0X0) 
#define RSTVAL_AI_REGS_YODA_PUMP_S1                                (0X2000) 
#define RSTVAL_AI_REGS_YODA_READOUT_S1                             (0X8) 
#define RSTVAL_AI_REGS_YODA_REGIF_CTRL                             (0X0) 
#define RSTVAL_AI_REGS_YODA_REGIF_RDATA                            (0X0) 
#define RSTVAL_AI_REGS_YODA_SSPLL_CTRL0_S1                         (0XF10) 
#define RSTVAL_AI_REGS_YODA_SSPLL_CTRL1_S1                         (0X810) 
#define RSTVAL_AI_REGS_YODA_SSPLL_CTRL2_S1                         (0XE) 
#define RSTVAL_AI_REGS_YODA_SYSPLL_CTRL0_S1                        (0X1F10) 
#define RSTVAL_AI_REGS_YODA_SYSPLL_CTRL1_S1                        (0X810) 
#define RSTVAL_AI_REGS_YODA_SYSPLL_CTRL2_S1                        (0X8315) 
#define RSTVAL_AI_REGS_YODA_ANA_TEST_MUX_S1                        (0X0) 
#define RSTVAL_AI_REGS_YODA_TS_CTRL_S1                             (0X0) 
#define RSTVAL_AI_REGS_YODA_TS_CTRL                                (0X4) 
#define RSTVAL_AI_REGS_YODA_TS_DATA                                (0X0) 
#define RSTVAL_AI_REGS_YODA_VLOWENABLE                             (0X8005) 
#define RSTVAL_AI_REGS_YODA_VLOWREGCTRL0_S2                        (0XAAC0) 
#define RSTVAL_AI_REGS_YODA_VLOWREGCTRL1_S2                        (0X1000) 
#define RSTVAL_AI_REGS_YODA_VLOWREGCTRL2_S2                        (0X404) 
#define RSTVAL_AI_REGS_YODA_VLOWREGCTRL3_S2                        (0X40) 
#define RSTVAL_AI_REGS_YODA_VLOWREGCTRL4_S2                        (0XD0D) 
#define RSTVAL_AI_REGS_YODA_VLOWSHOCTRL1                           (0X0) 
#define RSTVAL_AI_REGS_YODA_VLOWSHOCTRL2                           (0X0) 
#define RSTVAL_AI_REGS_YODA_VLOWSHOCTRL3                           (0X0) 
#define RSTVAL_AI_REGS_YODA_VLOWSHODETECT                          (0X0) 
#define RSTVAL_AI_REGS_YODA_XOSC_CTRL                              (0X0) 
#define RSTVAL_AI_REGS_YODA_CHAIN1_LEN                             (0X162) 
#define RSTVAL_AI_REGS_YODA_CHAIN2_LEN                             (0X59) 
#define RSTVAL_AI_REGS_YODA_MIPITX_CTRL                            (0X1) 
#define RSTVAL_AI_REGS_YODA_SSPLL_CTRL3_S1                         (0X0) 
#define RSTVAL_AI_REGS_YODA_PIXEL_BIAS                             (0X3) 
#define RSTVAL_AI_REGS_YODA_DLL_CONTROL                            (0X8) 
#define RSTVAL_AI_REGS_YODA_ANA_SPARE_0                            (0XA) 
#define RSTVAL_AI_REGS_YODA_ANA_SPARE_1                            (0X2AAA) 
#define RSTVAL_AI_REGS_YODA_ANA_SERIAL_SPARE_0                     (0X8AAA) 
#define RSTVAL_AI_REGS_YODA_ANA_SERIAL_SPARE_1                     (0X6A) 
#define RSTVAL_AI_REGS_YODA_ANA_SERIAL_SPARE_2                     (0XAAAA) 
#define RSTVAL_AI_REGS_YODA_DEBUG_MUX_CONTROL_REG                  (0X0) 

/* ====================================================================================================
        AI_REGS_YODA Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          ADC_CTRL0_S1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_CLK_DE_MODE                         (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_CLK_DE_MODE                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_CLK_DE_MODE                         (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADC_EXT_RESET_SEL                   (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADC_EXT_RESET_SEL                   (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADC_EXT_RESET_SEL                   (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADC_EN_10B                          (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADC_EN_10B                          (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADC_EN_10B                          (0X0004U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADC_MUX                             (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADC_MUX                             (3U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADC_MUX                             (0X0070U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ADC_CTRL1_S1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_ADC_DN_DELAY                        (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADC_DN_DELAY                        (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADC_DN_DELAY                        (0X00FFU)      /* No description provided */
#define BITP_AI_REGS_YODA_ADC_UP_DELAY                        (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADC_UP_DELAY                        (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADC_UP_DELAY                        (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ADC_CTRL2_S1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_ADC_IRAMP                           (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADC_IRAMP                           (9U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADC_IRAMP                           (0X01FFU)      /* No description provided */
#define BITP_AI_REGS_YODA_ADC_REGRESET                        (9U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADC_REGRESET                        (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADC_REGRESET                        (0X0200U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ADCPLL_CTRL0_S1                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_ADCPLL_PHASE_LOCK_DELAY             (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_PHASE_LOCK_DELAY             (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_PHASE_LOCK_DELAY             (0X00FFU)      /* No description provided */
#define BITP_AI_REGS_YODA_ADCPLL_QP                           (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_QP                           (5U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_QP                           (0X1F00U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADCPLL_RZ                           (13U)          /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_RZ                           (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_RZ                           (0X6000U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADCPLL_DLPF                         (15U)          /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_DLPF                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_DLPF                         (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ADCPLL_CTRL1_S1                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_ADCPLL_LOCK_ACC                     (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_LOCK_ACC                     (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_LOCK_ACC                     (0X00FFU)      /* No description provided */
#define BITP_AI_REGS_YODA_ADCPLL_UNLOCK_ACC                   (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_UNLOCK_ACC                   (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_UNLOCK_ACC                   (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ADCPLL_CTRL2_S1                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_ADCPLL_C                            (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_C                            (5U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_C                            (0X001FU)      /* No description provided */
#define BITP_AI_REGS_YODA_ADCPLL_END5                         (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_END5                         (4U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_END5                         (0X0F00U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADCPLL_TESTMUX                      (12U)          /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_TESTMUX                      (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_TESTMUX                      (0X3000U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADCPLL_TEST_SEL                     (14U)          /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_TEST_SEL                     (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_TEST_SEL                     (0XC000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMP_CTRL0_S1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_AMP_CFBA                            (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_AMP_CFBA                            (4U)           /* No description provided */
#define BITM_AI_REGS_YODA_AMP_CFBA                            (0X000FU)      /* No description provided */
#define BITP_AI_REGS_YODA_AMP_CFBB                            (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_AMP_CFBB                            (4U)           /* No description provided */
#define BITM_AI_REGS_YODA_AMP_CFBB                            (0X00F0U)      /* No description provided */
#define BITP_AI_REGS_YODA_AMP_CFBC                            (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_AMP_CFBC                            (4U)           /* No description provided */
#define BITM_AI_REGS_YODA_AMP_CFBC                            (0X0F00U)      /* No description provided */
#define BITP_AI_REGS_YODA_AMP_CFBD                            (12U)          /* No description provided */
#define BITL_AI_REGS_YODA_AMP_CFBD                            (4U)           /* No description provided */
#define BITM_AI_REGS_YODA_AMP_CFBD                            (0XF000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMP_CTRL1_S1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_AMP_CINA                            (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_AMP_CINA                            (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_AMP_CINA                            (0X0003U)      /* No description provided */
#define BITP_AI_REGS_YODA_AMP_CINB                            (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_AMP_CINB                            (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_AMP_CINB                            (0X0030U)      /* No description provided */
#define BITP_AI_REGS_YODA_AMP_CINC                            (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_AMP_CINC                            (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_AMP_CINC                            (0X0300U)      /* No description provided */
#define BITP_AI_REGS_YODA_AMP_CIND                            (12U)          /* No description provided */
#define BITL_AI_REGS_YODA_AMP_CIND                            (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_AMP_CIND                            (0X3000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMP_CTRL2_S1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_AMP_S                               (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_AMP_S                               (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_AMP_S                               (0X0003U)      /* No description provided */
#define BITP_AI_REGS_YODA_AMP_TESTSELP                        (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_AMP_TESTSELP                        (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_AMP_TESTSELP                        (0X000CU)      /* No description provided */
#define BITP_AI_REGS_YODA_AMP_TESTSELN                        (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_AMP_TESTSELN                        (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_AMP_TESTSELN                        (0X0030U)      /* No description provided */
#define BITP_AI_REGS_YODA_GAINTAG_LATCH_INHIBIT               (6U)           /* No description provided */
#define BITL_AI_REGS_YODA_GAINTAG_LATCH_INHIBIT               (3U)           /* No description provided */
#define BITM_AI_REGS_YODA_GAINTAG_LATCH_INHIBIT               (0X01C0U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CHIP_ID                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_CHIP_ID                             (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_CHIP_ID                             (16U)          /* No description provided */
#define BITM_AI_REGS_YODA_CHIP_ID                             (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CKGEN_CTRL                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_CG_STATE                            (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_CG_STATE                            (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_CG_STATE                            (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_CG_QBUF_STATE                       (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_CG_QBUF_STATE                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_CG_QBUF_STATE                       (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_CG_LIGHT_STATE                      (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_CG_LIGHT_STATE                      (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_CG_LIGHT_STATE                      (0X0004U)      /* No description provided */
#define BITP_AI_REGS_YODA_CG_RSTN                             (3U)           /* No description provided */
#define BITL_AI_REGS_YODA_CG_RSTN                             (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_CG_RSTN                             (0X0008U)      /* No description provided */
#define BITP_AI_REGS_YODA_CG_XTAL_PRESCALE                    (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_CG_XTAL_PRESCALE                    (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_CG_XTAL_PRESCALE                    (0X0030U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CKGEN_S1                                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_CG_INVERT_CLK                       (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_CG_INVERT_CLK                       (4U)           /* No description provided */
#define BITM_AI_REGS_YODA_CG_INVERT_CLK                       (0X000FU)      /* No description provided */
#define BITP_AI_REGS_YODA_CLK_TEST_SEL                        (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_CLK_TEST_SEL                        (6U)           /* No description provided */
#define BITM_AI_REGS_YODA_CLK_TEST_SEL                        (0X3F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLK_CTRL                                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_CLKDE_RESET                         (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_CLKDE_RESET                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_CLKDE_RESET                         (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLK_DE_CTRL_S1                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_CLK_DE_C                            (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_CLK_DE_C                            (5U)           /* No description provided */
#define BITM_AI_REGS_YODA_CLK_DE_C                            (0X001FU)      /* No description provided */
#define BITP_AI_REGS_YODA_CLK_DE_END5                         (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_CLK_DE_END5                         (4U)           /* No description provided */
#define BITM_AI_REGS_YODA_CLK_DE_END5                         (0X0F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLK_LVDSTX_S1                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_CLK_LVDSTX_I2X                      (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_CLK_LVDSTX_I2X                      (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_CLK_LVDSTX_I2X                      (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_CLK_LVDSTX_TE                       (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_CLK_LVDSTX_TE                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_CLK_LVDSTX_TE                       (0X0004U)      /* No description provided */
#define BITP_AI_REGS_YODA_CLK_LVDSTX_TEST                     (3U)           /* No description provided */
#define BITL_AI_REGS_YODA_CLK_LVDSTX_TEST                     (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_CLK_LVDSTX_TEST                     (0X0008U)      /* No description provided */
#define BITP_AI_REGS_YODA_CLK_LVDSTX_TESTD                    (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_CLK_LVDSTX_TESTD                    (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_CLK_LVDSTX_TESTD                    (0X0010U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLKTREE0                                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_DLL_CLKA_PARK                       (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_DLL_CLKA_PARK                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_CLKA_PARK                       (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_DLL_CLKB_PARK                       (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_DLL_CLKB_PARK                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_CLKB_PARK                       (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_DLL_CLKIN_EN                        (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_DLL_CLKIN_EN                        (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_CLKIN_EN                        (0X0004U)      /* No description provided */
#define BITP_AI_REGS_YODA_DLL_LPF_EN                          (3U)           /* No description provided */
#define BITL_AI_REGS_YODA_DLL_LPF_EN                          (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_LPF_EN                          (0X0008U)      /* No description provided */
#define BITP_AI_REGS_YODA_DLL_REST_EN                         (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_DLL_REST_EN                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_REST_EN                         (0X0010U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLKTREE_S1                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_DLL_LEAK                            (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_DLL_LEAK                            (5U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_LEAK                            (0X001FU)      /* No description provided */
#define BITP_AI_REGS_YODA_DLL_CPLF                            (5U)           /* No description provided */
#define BITL_AI_REGS_YODA_DLL_CPLF                            (7U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_CPLF                            (0X0FE0U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DAC_CTRL1                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_DAC_LATCH                           (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_DAC_LATCH                           (9U)           /* No description provided */
#define BITM_AI_REGS_YODA_DAC_LATCH                           (0X01FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DAC_CTRL2                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_DAC_PD                              (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_DAC_PD                              (10U)          /* No description provided */
#define BITM_AI_REGS_YODA_DAC_PD                              (0X03FFU)      /* No description provided */
#define BITP_AI_REGS_YODA_DAC_LOWLOAD                         (10U)          /* No description provided */
#define BITL_AI_REGS_YODA_DAC_LOWLOAD                         (4U)           /* No description provided */
#define BITM_AI_REGS_YODA_DAC_LOWLOAD                         (0X3C00U)      /* No description provided */
#define BITP_AI_REGS_YODA_DAC_GLOBAL_PD                       (15U)          /* No description provided */
#define BITL_AI_REGS_YODA_DAC_GLOBAL_PD                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DAC_GLOBAL_PD                       (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DAC_CTRL0_S1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_DAC_SPARE1                          (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_DAC_SPARE1                          (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_DAC_SPARE1                          (0X00FFU)      /* No description provided */
#define BITP_AI_REGS_YODA_DAC_SHARE                           (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_DAC_SHARE                           (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DAC_SHARE                           (0X0100U)      /* No description provided */
#define BITP_AI_REGS_YODA_COMP_REF_DISABLE                    (9U)           /* No description provided */
#define BITL_AI_REGS_YODA_COMP_REF_DISABLE                    (6U)           /* No description provided */
#define BITM_AI_REGS_YODA_COMP_REF_DISABLE                    (0X7E00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DAC_CTRL1_S1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_DAC_SPARE2                          (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_DAC_SPARE2                          (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DAC_SPARE2                          (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_DAC_SPARE0                          (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_DAC_SPARE0                          (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_DAC_SPARE0                          (0X0006U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DAC_CTRL2_S1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_COMP_REF_DAC1                       (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_COMP_REF_DAC1                       (6U)           /* No description provided */
#define BITM_AI_REGS_YODA_COMP_REF_DAC1                       (0X003FU)      /* No description provided */
#define BITP_AI_REGS_YODA_COMP_REF_DAC2                       (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_COMP_REF_DAC2                       (6U)           /* No description provided */
#define BITM_AI_REGS_YODA_COMP_REF_DAC2                       (0X3F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DAC_CTRL3_S1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_COMP_REF_DAC3                       (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_COMP_REF_DAC3                       (6U)           /* No description provided */
#define BITM_AI_REGS_YODA_COMP_REF_DAC3                       (0X003FU)      /* No description provided */
#define BITP_AI_REGS_YODA_COMP_REF_DAC4                       (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_COMP_REF_DAC4                       (6U)           /* No description provided */
#define BITM_AI_REGS_YODA_COMP_REF_DAC4                       (0X3F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DAC_DATA                                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_DAC_IN                              (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_DAC_IN                              (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_DAC_IN                              (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          IPDA_CTRL_S1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_IBOUT_ADJ                           (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_IBOUT_ADJ                           (4U)           /* No description provided */
#define BITM_AI_REGS_YODA_IBOUT_ADJ                           (0X000FU)      /* No description provided */
#define BITP_AI_REGS_YODA_IPDA_GAIN                           (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_IPDA_GAIN                           (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_IPDA_GAIN                           (0X0010U)      /* No description provided */
#define BITP_AI_REGS_YODA_IPDA_SPARE0                         (5U)           /* No description provided */
#define BITL_AI_REGS_YODA_IPDA_SPARE0                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_IPDA_SPARE0                         (0X0020U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          LS_LVDSTX_S1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_LS_LVDSTX_I2X                       (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_LS_LVDSTX_I2X                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_LS_LVDSTX_I2X                       (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_LS_LVDSTX_TE                        (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_LS_LVDSTX_TE                        (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_LS_LVDSTX_TE                        (0X0004U)      /* No description provided */
#define BITP_AI_REGS_YODA_LS_LVDSTX_TEST                      (3U)           /* No description provided */
#define BITL_AI_REGS_YODA_LS_LVDSTX_TEST                      (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_LS_LVDSTX_TEST                      (0X0008U)      /* No description provided */
#define BITP_AI_REGS_YODA_LS_LVDSTX_TESTD                     (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_LS_LVDSTX_TESTD                     (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_LS_LVDSTX_TESTD                     (0X0010U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          LSCTRL0_S1                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_LIGHTMUX_SEL                        (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_LIGHTMUX_SEL                        (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_LIGHTMUX_SEL                        (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          LSMOD_EN                                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_CLK_LVDS_OE                         (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_CLK_LVDS_OE                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_CLK_LVDS_OE                         (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_CLK_LVDSTX_PD                       (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_CLK_LVDSTX_PD                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_CLK_LVDSTX_PD                       (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_LS_LVDS_OE                          (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_LS_LVDS_OE                          (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_LS_LVDS_OE                          (0X0004U)      /* No description provided */
#define BITP_AI_REGS_YODA_LS_LVDSTX_PD                        (3U)           /* No description provided */
#define BITL_AI_REGS_YODA_LS_LVDSTX_PD                        (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_LS_LVDSTX_PD                        (0X0008U)      /* No description provided */
#define BITP_AI_REGS_YODA_CLK_LVDSTX_HZ                       (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_CLK_LVDSTX_HZ                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_CLK_LVDSTX_HZ                       (0X0010U)      /* No description provided */
#define BITP_AI_REGS_YODA_LS_LVDSTX_HZ                        (5U)           /* No description provided */
#define BITL_AI_REGS_YODA_LS_LVDSTX_HZ                        (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_LS_LVDSTX_HZ                        (0X0020U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ROW_CTRL                                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_GLOBAL_RESET                        (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_GLOBAL_RESET                        (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_GLOBAL_RESET                        (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_ROW_WRAPEN                          (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_ROW_WRAPEN                          (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ROW_WRAPEN                          (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_ROW_READ_DEF                        (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_ROW_READ_DEF                        (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ROW_READ_DEF                        (0X0100U)      /* No description provided */
#define BITP_AI_REGS_YODA_ROW_RESET_DEF                       (9U)           /* No description provided */
#define BITL_AI_REGS_YODA_ROW_RESET_DEF                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ROW_RESET_DEF                       (0X0200U)      /* No description provided */
#define BITP_AI_REGS_YODA_ROW_CAPODD_DEF                      (10U)          /* No description provided */
#define BITL_AI_REGS_YODA_ROW_CAPODD_DEF                      (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ROW_CAPODD_DEF                      (0X0400U)      /* No description provided */
#define BITP_AI_REGS_YODA_ROW_CAPEVEN_DEF                     (11U)          /* No description provided */
#define BITL_AI_REGS_YODA_ROW_CAPEVEN_DEF                     (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ROW_CAPEVEN_DEF                     (0X0800U)      /* No description provided */
#define BITP_AI_REGS_YODA_ROW_TX_PD_DEF                       (12U)          /* No description provided */
#define BITL_AI_REGS_YODA_ROW_TX_PD_DEF                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ROW_TX_PD_DEF                       (0X1000U)      /* No description provided */
#define BITP_AI_REGS_YODA_ROW_TX_DEF                          (13U)          /* No description provided */
#define BITL_AI_REGS_YODA_ROW_TX_DEF                          (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ROW_TX_DEF                          (0X2000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PLL_CTRL                                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_SSPLL_PD                            (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_PD                            (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_PD                            (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_SSPLL_RST                           (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_RST                           (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_RST                           (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_SSPLL_RESET_LOCK_LOST               (3U)           /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_RESET_LOCK_LOST               (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_RESET_LOCK_LOST               (0X0008U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADCPLL_PD                           (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_PD                           (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_PD                           (0X0010U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADCPLL_RST                          (5U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_RST                          (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_RST                          (0X0020U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADCPLL_RESET_LOCK_LOST              (7U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_RESET_LOCK_LOST              (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_RESET_LOCK_LOST              (0X0080U)      /* No description provided */
#define BITP_AI_REGS_YODA_SYSPLL_PD                           (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_PD                           (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_PD                           (0X0100U)      /* No description provided */
#define BITP_AI_REGS_YODA_SYSPLL_RESET_LOCK_LOST              (11U)          /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_RESET_LOCK_LOST              (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_RESET_LOCK_LOST              (0X0800U)      /* No description provided */
#define BITP_AI_REGS_YODA_MIPIPLL_PD                          (12U)          /* No description provided */
#define BITL_AI_REGS_YODA_MIPIPLL_PD                          (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_MIPIPLL_PD                          (0X1000U)      /* No description provided */
#define BITP_AI_REGS_YODA_MIPIPLL_RESET_LOCK_LOST             (15U)          /* No description provided */
#define BITL_AI_REGS_YODA_MIPIPLL_RESET_LOCK_LOST             (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_MIPIPLL_RESET_LOCK_LOST             (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PLL_STATUS                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_SSPLL_PHASE_LOCK_REG                (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_PHASE_LOCK_REG                (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_PHASE_LOCK_REG                (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_SSPLL_LOCK_LOST                     (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_LOCK_LOST                     (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_LOCK_LOST                     (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADCPLL_PHASE_LOCK_REG               (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_PHASE_LOCK_REG               (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_PHASE_LOCK_REG               (0X0010U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADCPLL_LOCK_LOST                    (5U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_LOCK_LOST                    (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_LOCK_LOST                    (0X0020U)      /* No description provided */
#define BITP_AI_REGS_YODA_SYSPLL_PHASE_LOCK_REG               (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_PHASE_LOCK_REG               (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_PHASE_LOCK_REG               (0X0100U)      /* No description provided */
#define BITP_AI_REGS_YODA_SYSPLL_LOCK_LOST                    (9U)           /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_LOCK_LOST                    (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_LOCK_LOST                    (0X0200U)      /* No description provided */
#define BITP_AI_REGS_YODA_SYSPLL_ACTIVE                       (10U)          /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_ACTIVE                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_ACTIVE                       (0X0400U)      /* No description provided */
#define BITP_AI_REGS_YODA_MIPIPLL_PHASE_LOCK_REG              (12U)          /* No description provided */
#define BITL_AI_REGS_YODA_MIPIPLL_PHASE_LOCK_REG              (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_MIPIPLL_PHASE_LOCK_REG              (0X1000U)      /* No description provided */
#define BITP_AI_REGS_YODA_MIPIPLL_LOCK_LOST                   (13U)          /* No description provided */
#define BITL_AI_REGS_YODA_MIPIPLL_LOCK_LOST                   (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_MIPIPLL_LOCK_LOST                   (0X2000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          POWER_DOWN_0                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_ROW_VECTOR_LD                       (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_ROW_VECTOR_LD                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ROW_VECTOR_LD                       (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_PUMP_BYPASS                         (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_PUMP_BYPASS                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_PUMP_BYPASS                         (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_DLL_PD                              (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_DLL_PD                              (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_PD                              (0X0004U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          POWER_DOWN_ADC_OTHERS                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_ADC_RAMP_PD                         (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADC_RAMP_PD                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADC_RAMP_PD                         (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_COL_BIAS_PD                         (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_COL_BIAS_PD                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_COL_BIAS_PD                         (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_ROW_UP_DNB                          (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_ROW_UP_DNB                          (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ROW_UP_DNB                          (0X0004U)      /* No description provided */
#define BITP_AI_REGS_YODA_SAT_DETECT_PD                       (3U)           /* No description provided */
#define BITL_AI_REGS_YODA_SAT_DETECT_PD                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SAT_DETECT_PD                       (0X0008U)      /* No description provided */
#define BITP_AI_REGS_YODA_COL_COMPARATOR_PD                   (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_COL_COMPARATOR_PD                   (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_COL_COMPARATOR_PD                   (0X0010U)      /* No description provided */
#define BITP_AI_REGS_YODA_AMP_PD                              (5U)           /* No description provided */
#define BITL_AI_REGS_YODA_AMP_PD                              (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_AMP_PD                              (0X0020U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADC_PD                              (6U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADC_PD                              (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADC_PD                              (0X0040U)      /* No description provided */
#define BITP_AI_REGS_YODA_REFGEN_BGR_PD                       (7U)           /* No description provided */
#define BITL_AI_REGS_YODA_REFGEN_BGR_PD                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_REFGEN_BGR_PD                       (0X0080U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          POWER_DOWN_READOUT                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_READOUT_PD                          (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_READOUT_PD                          (16U)          /* No description provided */
#define BITM_AI_REGS_YODA_READOUT_PD                          (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PUMP_S1                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_PUMP_ADJ                            (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_PUMP_ADJ                            (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_PUMP_ADJ                            (0X0003U)      /* No description provided */
#define BITP_AI_REGS_YODA_QPD                                 (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_QPD                                 (6U)           /* No description provided */
#define BITM_AI_REGS_YODA_QPD                                 (0X3F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          READOUT_S1                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_BITLINE_TURBO                       (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_BITLINE_TURBO                       (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_BITLINE_TURBO                       (0X0003U)      /* No description provided */
#define BITP_AI_REGS_YODA_NCDS_MODE                           (3U)           /* No description provided */
#define BITL_AI_REGS_YODA_NCDS_MODE                           (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_NCDS_MODE                           (0X0008U)      /* No description provided */
#define BITP_AI_REGS_YODA_SATDETECT_S                         (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_SATDETECT_S                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SATDETECT_S                         (0X0010U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          REGIF_CTRL                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_REGIF_RESETB1                       (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_REGIF_RESETB1                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_REGIF_RESETB1                       (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_REGIF_RD_WRB                        (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_REGIF_RD_WRB                        (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_REGIF_RD_WRB                        (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_REGIF_START1                        (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_REGIF_START1                        (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_REGIF_START1                        (0X0004U)      /* No description provided */
#define BITP_AI_REGS_YODA_REGIF_BUSY1                         (3U)           /* No description provided */
#define BITL_AI_REGS_YODA_REGIF_BUSY1                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_REGIF_BUSY1                         (0X0008U)      /* No description provided */
#define BITP_AI_REGS_YODA_REGIF_RESETB2                       (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_REGIF_RESETB2                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_REGIF_RESETB2                       (0X0100U)      /* No description provided */
#define BITP_AI_REGS_YODA_REGIF_START2                        (10U)          /* No description provided */
#define BITL_AI_REGS_YODA_REGIF_START2                        (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_REGIF_START2                        (0X0400U)      /* No description provided */
#define BITP_AI_REGS_YODA_REGIF_BUSY2                         (11U)          /* No description provided */
#define BITL_AI_REGS_YODA_REGIF_BUSY2                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_REGIF_BUSY2                         (0X0800U)      /* No description provided */
#define BITP_AI_REGS_YODA_REGIF_READ_SHIFT                    (15U)          /* No description provided */
#define BITL_AI_REGS_YODA_REGIF_READ_SHIFT                    (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_REGIF_READ_SHIFT                    (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          REGIF_RDATA                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_REGIF_READ_DATA                     (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_REGIF_READ_DATA                     (16U)          /* No description provided */
#define BITM_AI_REGS_YODA_REGIF_READ_DATA                     (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSPLL_CTRL0_S1                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_SSPLL_PHASE_LOCK_DELAY              (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_PHASE_LOCK_DELAY              (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_PHASE_LOCK_DELAY              (0X00FFU)      /* No description provided */
#define BITP_AI_REGS_YODA_SSPLL_QP                            (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_QP                            (5U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_QP                            (0X1F00U)      /* No description provided */
#define BITP_AI_REGS_YODA_SSPLL_RZ                            (13U)          /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_RZ                            (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_RZ                            (0X6000U)      /* No description provided */
#define BITP_AI_REGS_YODA_SSPLL_DLPF                          (15U)          /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_DLPF                          (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_DLPF                          (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSPLL_CTRL1_S1                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_SSPLL_LOCK_ACC                      (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_LOCK_ACC                      (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_LOCK_ACC                      (0X00FFU)      /* No description provided */
#define BITP_AI_REGS_YODA_SSPLL_UNLOCK_ACC                    (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_UNLOCK_ACC                    (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_UNLOCK_ACC                    (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSPLL_CTRL2_S1                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_SSPLL_C                             (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_C                             (5U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_C                             (0X001FU)      /* No description provided */
#define BITP_AI_REGS_YODA_SSPLL_END5                          (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_END5                          (4U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_END5                          (0X0F00U)      /* No description provided */
#define BITP_AI_REGS_YODA_SSPLL_TESTMUX                       (12U)          /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_TESTMUX                       (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_TESTMUX                       (0X3000U)      /* No description provided */
#define BITP_AI_REGS_YODA_SSPLL_TEST_SEL                      (14U)          /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_TEST_SEL                      (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_TEST_SEL                      (0XC000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SYSPLL_CTRL0_S1                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_SYSPLL_PHASE_LOCK_DELAY             (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_PHASE_LOCK_DELAY             (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_PHASE_LOCK_DELAY             (0X00FFU)      /* No description provided */
#define BITP_AI_REGS_YODA_SYSPLL_QP                           (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_QP                           (5U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_QP                           (0X1F00U)      /* No description provided */
#define BITP_AI_REGS_YODA_SYSPLL_RZ                           (13U)          /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_RZ                           (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_RZ                           (0X6000U)      /* No description provided */
#define BITP_AI_REGS_YODA_SYSPLL_DLPF                         (15U)          /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_DLPF                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_DLPF                         (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SYSPLL_CTRL1_S1                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_SYSPLL_LOCK_ACC                     (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_LOCK_ACC                     (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_LOCK_ACC                     (0X00FFU)      /* No description provided */
#define BITP_AI_REGS_YODA_SYSPLL_UNLOCK_ACC                   (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_UNLOCK_ACC                   (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_UNLOCK_ACC                   (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SYSPLL_CTRL2_S1                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_SYSPLL_M                            (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_M                            (6U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_M                            (0X003FU)      /* No description provided */
#define BITP_AI_REGS_YODA_SYSPLL_N                            (7U)           /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_N                            (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_N                            (0X0080U)      /* No description provided */
#define BITP_AI_REGS_YODA_SYSPLL_P                            (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_P                            (6U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_P                            (0X3F00U)      /* No description provided */
#define BITP_AI_REGS_YODA_SYSPLL_LX                           (15U)          /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_LX                           (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_LX                           (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ANA_TEST_MUX_S1                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_ANA_TEST_MUX                        (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_ANA_TEST_MUX                        (5U)           /* No description provided */
#define BITM_AI_REGS_YODA_ANA_TEST_MUX                        (0X001FU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          TS_CTRL_S1                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_TS_CALIB_ENABLE                     (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_TS_CALIB_ENABLE                     (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_TS_CALIB_ENABLE                     (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_TS_CALIB_INPUTS                     (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_TS_CALIB_INPUTS                     (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_TS_CALIB_INPUTS                     (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_TS_FREQ_SELECT                      (3U)           /* No description provided */
#define BITL_AI_REGS_YODA_TS_FREQ_SELECT                      (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_TS_FREQ_SELECT                      (0X0008U)      /* No description provided */
#define BITP_AI_REGS_YODA_TS_FSADJ                            (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_TS_FSADJ                            (7U)           /* No description provided */
#define BITM_AI_REGS_YODA_TS_FSADJ                            (0X7F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          TS_CTRL                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_TS_INIT_LOGIC                       (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_TS_INIT_LOGIC                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_TS_INIT_LOGIC                       (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_TS_RST                              (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_TS_RST                              (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_TS_RST                              (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_TS_PD                               (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_TS_PD                               (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_TS_PD                               (0X0004U)      /* No description provided */
#define BITP_AI_REGS_YODA_TS_EOC_REG                          (15U)          /* No description provided */
#define BITL_AI_REGS_YODA_TS_EOC_REG                          (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_TS_EOC_REG                          (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          TS_DATA                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_TS_Q_REG                            (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_TS_Q_REG                            (12U)          /* No description provided */
#define BITM_AI_REGS_YODA_TS_Q_REG                            (0X0FFFU)      /* No description provided */
#define BITP_AI_REGS_YODA_TS_OVRFL_REG                        (12U)          /* No description provided */
#define BITL_AI_REGS_YODA_TS_OVRFL_REG                        (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_TS_OVRFL_REG                        (0X1000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          VLOWENABLE                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_VLOWENABLECTRL                      (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOWENABLECTRL                      (3U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOWENABLECTRL                      (0X0007U)      /* No description provided */
#define BITP_AI_REGS_YODA_VLOW_LIN_EN                         (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_LIN_EN                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_LIN_EN                         (0X0100U)      /* No description provided */
#define BITP_AI_REGS_YODA_VLOW_LIN_PD                         (15U)          /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_LIN_PD                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_LIN_PD                         (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          VLOWREGCTRL0_S2                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_VLOWSETPOINTCTRL                    (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOWSETPOINTCTRL                    (7U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOWSETPOINTCTRL                    (0X007FU)      /* No description provided */
#define BITP_AI_REGS_YODA_VLOW_GMPS2X                         (7U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_GMPS2X                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_GMPS2X                         (0X0080U)      /* No description provided */
#define BITP_AI_REGS_YODA_VLOW_SPARE                          (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_SPARE                          (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_SPARE                          (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          VLOWREGCTRL1_S2                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_VLOWMISCCTRL                        (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOWMISCCTRL                        (16U)          /* No description provided */
#define BITM_AI_REGS_YODA_VLOWMISCCTRL                        (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          VLOWREGCTRL2_S2                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_VLOWCLKLOCNT                        (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOWCLKLOCNT                        (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOWCLKLOCNT                        (0X00FFU)      /* No description provided */
#define BITP_AI_REGS_YODA_VLOWCLKHICNT                        (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOWCLKHICNT                        (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOWCLKHICNT                        (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          VLOWREGCTRL3_S2                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_VREGMISCCTRL                        (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_VREGMISCCTRL                        (16U)          /* No description provided */
#define BITM_AI_REGS_YODA_VREGMISCCTRL                        (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          VLOWREGCTRL4_S2                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_VLOW_SW_VADJ                        (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_SW_VADJ                        (6U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_SW_VADJ                        (0X003FU)      /* No description provided */
#define BITP_AI_REGS_YODA_VLOW_LIN_VADJ                       (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_LIN_VADJ                       (6U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_LIN_VADJ                       (0X3F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          VLOWSHOCTRL1                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_VLOW_IC_START                       (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_IC_START                       (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_IC_START                       (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_VLOW_IGNORE_CNT                     (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_IGNORE_CNT                     (15U)          /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_IGNORE_CNT                     (0XFFFEU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          VLOWSHOCTRL2                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_VLOW_HITIME_CTR                     (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_HITIME_CTR                     (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_HITIME_CTR                     (0X00FFU)      /* No description provided */
#define BITP_AI_REGS_YODA_VLOW_EVENT_CTR                      (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_EVENT_CTR                      (8U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_EVENT_CTR                      (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          VLOWSHOCTRL3                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_VLOW_CTR_RD                         (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_CTR_RD                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_CTR_RD                         (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_VLOW_CTR_RD_RDY                     (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_CTR_RD_RDY                     (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_CTR_RD_RDY                     (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_VLOW_CTR_RESET                      (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_CTR_RESET                      (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_CTR_RESET                      (0X0004U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          VLOWSHODETECT                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_SHO_VLOW                            (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_SHO_VLOW                            (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SHO_VLOW                            (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_VLOW_SHO_DETECT                     (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_VLOW_SHO_DETECT                     (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_VLOW_SHO_DETECT                     (0X0002U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          XOSC_CTRL                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_XIDLE_HOST_SET                      (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_XIDLE_HOST_SET                      (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_XIDLE_HOST_SET                      (0X0010U)      /* No description provided */
#define BITP_AI_REGS_YODA_XIDLE_HOST_CLEAR                    (5U)           /* No description provided */
#define BITL_AI_REGS_YODA_XIDLE_HOST_CLEAR                    (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_XIDLE_HOST_CLEAR                    (0X0020U)      /* No description provided */
#define BITP_AI_REGS_YODA_XIDLE_HOST                          (6U)           /* No description provided */
#define BITL_AI_REGS_YODA_XIDLE_HOST                          (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_XIDLE_HOST                          (0X0040U)      /* No description provided */
#define BITP_AI_REGS_YODA_XIDLE_YEATS_SET                     (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_XIDLE_YEATS_SET                     (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_XIDLE_YEATS_SET                     (0X0100U)      /* No description provided */
#define BITP_AI_REGS_YODA_XIDLE_YEATS_CLEAR                   (9U)           /* No description provided */
#define BITL_AI_REGS_YODA_XIDLE_YEATS_CLEAR                   (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_XIDLE_YEATS_CLEAR                   (0X0200U)      /* No description provided */
#define BITP_AI_REGS_YODA_XIDLE_YEATS                         (10U)          /* No description provided */
#define BITL_AI_REGS_YODA_XIDLE_YEATS                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_XIDLE_YEATS                         (0X0400U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CHAIN1_LEN                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_CHAIN1_LEN                          (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_CHAIN1_LEN                          (9U)           /* No description provided */
#define BITM_AI_REGS_YODA_CHAIN1_LEN                          (0X01FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CHAIN2_LEN                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_CHAIN2_LEN                          (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_CHAIN2_LEN                          (9U)           /* No description provided */
#define BITM_AI_REGS_YODA_CHAIN2_LEN                          (0X01FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          MIPITX_CTRL                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_MIPITX_PD                           (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_MIPITX_PD                           (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_MIPITX_PD                           (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSPLL_CTRL3_S1                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_SSPLL_ACOFF                         (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_ACOFF                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_ACOFF                         (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PIXEL_BIAS                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_PIXEL_BIAS_CDN                      (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_PIXEL_BIAS_CDN                      (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_PIXEL_BIAS_CDN                      (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_PIXEL_BIAS_SDN                      (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_PIXEL_BIAS_SDN                      (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_PIXEL_BIAS_SDN                      (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_PIXEL_BIAS_DIR                      (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_PIXEL_BIAS_DIR                      (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_PIXEL_BIAS_DIR                      (0X0004U)      /* No description provided */
#define BITP_AI_REGS_YODA_PIXEL_BIAS_PD_DEF                   (3U)           /* No description provided */
#define BITL_AI_REGS_YODA_PIXEL_BIAS_PD_DEF                   (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_PIXEL_BIAS_PD_DEF                   (0X0008U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DLL_CONTROL                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_DLL_COL_SREG_DIR                    (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_DLL_COL_SREG_DIR                    (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_COL_SREG_DIR                    (0X0004U)      /* No description provided */
#define BITP_AI_REGS_YODA_DLL_COL_EN_SDN                      (3U)           /* No description provided */
#define BITL_AI_REGS_YODA_DLL_COL_EN_SDN                      (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_COL_EN_SDN                      (0X0008U)      /* No description provided */
#define BITP_AI_REGS_YODA_DLL_COL_EN_CDN                      (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_DLL_COL_EN_CDN                      (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_COL_EN_CDN                      (0X0010U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ANA_SPARE_0                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_AMP_CLK2_DUMMY                      (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_AMP_CLK2_DUMMY                      (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_AMP_CLK2_DUMMY                      (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_SEL_D2ORD3N_LOAD1                   (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_SEL_D2ORD3N_LOAD1                   (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SEL_D2ORD3N_LOAD1                   (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_XA                                  (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_XA                                  (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_XA                                  (0X000CU)      /* No description provided */
#define BITP_AI_REGS_YODA_XAGCOFF                             (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_XAGCOFF                             (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_XAGCOFF                             (0X0010U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ANA_SPARE_1                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_TS_SOC_CAL                          (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_TS_SOC_CAL                          (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_TS_SOC_CAL                          (0X0001U)      /* No description provided */
#define BITP_AI_REGS_YODA_DLL_OPTION_1                        (1U)           /* No description provided */
#define BITL_AI_REGS_YODA_DLL_OPTION_1                        (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_OPTION_1                        (0X0002U)      /* No description provided */
#define BITP_AI_REGS_YODA_SEL_D2ORD3N_LOAD2                   (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_SEL_D2ORD3N_LOAD2                   (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_SEL_D2ORD3N_LOAD2                   (0X0004U)      /* No description provided */
#define BITP_AI_REGS_YODA_DLL_OPTION_2                        (3U)           /* No description provided */
#define BITL_AI_REGS_YODA_DLL_OPTION_2                        (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_OPTION_2                        (0X0008U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADC_RAMP_CM_SEL                     (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADC_RAMP_CM_SEL                     (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADC_RAMP_CM_SEL                     (0X0030U)      /* No description provided */
#define BITP_AI_REGS_YODA_ANA_SPARE_I1                        (6U)           /* No description provided */
#define BITL_AI_REGS_YODA_ANA_SPARE_I1                        (9U)           /* No description provided */
#define BITM_AI_REGS_YODA_ANA_SPARE_I1                        (0X7FC0U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ANA_SERIAL_SPARE_0                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_ROW_SPARE                           (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_ROW_SPARE                           (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_ROW_SPARE                           (0X0003U)      /* No description provided */
#define BITP_AI_REGS_YODA_PUMP_SPARE                          (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_PUMP_SPARE                          (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_PUMP_SPARE                          (0X000CU)      /* No description provided */
#define BITP_AI_REGS_YODA_TS_SPARE                            (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_TS_SPARE                            (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_TS_SPARE                            (0X0030U)      /* No description provided */
#define BITP_AI_REGS_YODA_SYSPLL_SPARE                        (6U)           /* No description provided */
#define BITL_AI_REGS_YODA_SYSPLL_SPARE                        (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_SYSPLL_SPARE                        (0X00C0U)      /* No description provided */
#define BITP_AI_REGS_YODA_ADCPLL_SPARE                        (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_ADCPLL_SPARE                        (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_ADCPLL_SPARE                        (0X0300U)      /* No description provided */
#define BITP_AI_REGS_YODA_SSPLL_SPARE                         (10U)          /* No description provided */
#define BITL_AI_REGS_YODA_SSPLL_SPARE                         (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_SSPLL_SPARE                         (0X0C00U)      /* No description provided */
#define BITP_AI_REGS_YODA_DAC_SPARE                           (12U)          /* No description provided */
#define BITL_AI_REGS_YODA_DAC_SPARE                           (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_DAC_SPARE                           (0X3000U)      /* No description provided */
#define BITP_AI_REGS_YODA_DLL_SPARE                           (14U)          /* No description provided */
#define BITL_AI_REGS_YODA_DLL_SPARE                           (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_DLL_SPARE                           (0XC000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ANA_SERIAL_SPARE_1                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_LSMOD_SPARE                         (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_LSMOD_SPARE                         (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_LSMOD_SPARE                         (0X0003U)      /* No description provided */
#define BITP_AI_REGS_YODA_CG_SPARE                            (2U)           /* No description provided */
#define BITL_AI_REGS_YODA_CG_SPARE                            (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_CG_SPARE                            (0X000CU)      /* No description provided */
#define BITP_AI_REGS_YODA_LS_SPARE                            (4U)           /* No description provided */
#define BITL_AI_REGS_YODA_LS_SPARE                            (2U)           /* No description provided */
#define BITM_AI_REGS_YODA_LS_SPARE                            (0X0030U)      /* No description provided */
#define BITP_AI_REGS_YODA_LS_SPARE2                           (6U)           /* No description provided */
#define BITL_AI_REGS_YODA_LS_SPARE2                           (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_LS_SPARE2                           (0X0040U)      /* No description provided */
#define BITP_AI_REGS_YODA_LS_SPARE3                           (7U)           /* No description provided */
#define BITL_AI_REGS_YODA_LS_SPARE3                           (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_LS_SPARE3                           (0X0080U)      /* No description provided */
#define BITP_AI_REGS_YODA_IPDA_SPARE1                         (8U)           /* No description provided */
#define BITL_AI_REGS_YODA_IPDA_SPARE1                         (1U)           /* No description provided */
#define BITM_AI_REGS_YODA_IPDA_SPARE1                         (0X0100U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ANA_SERIAL_SPARE_2                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_ARRAY_SPARE                         (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_ARRAY_SPARE                         (16U)          /* No description provided */
#define BITM_AI_REGS_YODA_ARRAY_SPARE                         (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DEBUG_MUX_CONTROL_REG                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_AI_REGS_YODA_DEBUG_MUX_CONTROL                   (0U)           /* No description provided */
#define BITL_AI_REGS_YODA_DEBUG_MUX_CONTROL                   (6U)           /* No description provided */
#define BITM_AI_REGS_YODA_DEBUG_MUX_CONTROL                   (0X003FU)      /* No description provided */

#endif  /* end ifndef AI_REGS_YODA_ADDR_RDEF_H_ */

/* ====================================================================================================
        USEQ_REGS_MAP2 Module Instances Address and Mask Definitions
   ==================================================================================================== */
#define INST_USEQ_REGS2                                          (0X00000200U)    /* useq_regs2: */


#ifndef USEQ_REGS_MAP2_ADDR_RDEF_H_
#define USEQ_REGS_MAP2_ADDR_RDEF_H_    /* USEQ_REGS_MAP2: Your module description, here. */

#define MASK_USEQ_REGS_MAP2                                      (0XFFFFFFFFU)    /* USEQ_REGS_MAP2: Your module description, here. */

/* ====================================================================================================
        USEQ_REGS_MAP2 Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_0_                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_1_                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_2_                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_3_                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_4_                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_5_                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_6_                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_7_                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_8_                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_9_                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_10_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_11_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_12_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_13_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_14_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_15_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_16_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_17_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_18_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_19_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_20_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_21_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_22_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_23_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_24_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_25_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_26_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_27_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_28_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_29_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_30_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_31_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_32_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_33_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_34_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_35_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_36_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_37_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_38_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_39_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_40_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_41_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_42_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_43_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_44_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_45_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_46_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_47_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_48_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_49_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_50_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_51_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_52_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_53_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_54_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_55_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_56_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_57_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_58_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_59_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_60_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_61_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_62_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_SCRATCHPAD_63_                       (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_CLKGEN_CK1                           (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_CLKGEN_CK2                           (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_CLKGEN_CK1REF                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_CLKGEN_CK2REF                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_CLKGEN_L1                            (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_CLKGEN_L2                            (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_CLKGEN_CKX                           (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_CLKGEN_CYCLE                         (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_CLKGEN_OFFSET                        (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_CLKGEN_LTOFFSET                      (0X0) 
#define RSTVAL_USEQ_REGS_MAP2_CLKGEN_BURST_PERIOD                  (0X0) 

/* ====================================================================================================
        USEQ_REGS_MAP2 Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          SCRATCHPAD_0_                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP2_SCRATCHPAD_N__SCRATCHPAD          (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_SCRATCHPAD_N__SCRATCHPAD          (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP2_SCRATCHPAD_N__SCRATCHPAD          (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLKGEN_CK1                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CK1_CK1RISE                (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CK1_CK1RISE                (7U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CK1_CK1RISE                (0X007FU)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CK1_CK1FALL                (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CK1_CK1FALL                (7U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CK1_CK1FALL                (0X7F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLKGEN_CK2                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CK2_CK2RISE                (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CK2_CK2RISE                (7U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CK2_CK2RISE                (0X007FU)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CK2_CK2FALL                (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CK2_CK2FALL                (7U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CK2_CK2FALL                (0X7F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLKGEN_CK1REF                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CK1REF_CK1REFRISE          (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CK1REF_CK1REFRISE          (7U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CK1REF_CK1REFRISE          (0X007FU)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CK1REF_CK1REFFALL          (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CK1REF_CK1REFFALL          (7U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CK1REF_CK1REFFALL          (0X7F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLKGEN_CK2REF                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CK2REF_CK2REFRISE          (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CK2REF_CK2REFRISE          (7U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CK2REF_CK2REFRISE          (0X007FU)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CK2REF_CK2REFFALL          (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CK2REF_CK2REFFALL          (7U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CK2REF_CK2REFFALL          (0X7F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLKGEN_L1                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP2_CLKGEN_L1_L1RISE                  (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_L1_L1RISE                  (7U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_L1_L1RISE                  (0X007FU)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_L1_L1FALL                  (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_L1_L1FALL                  (7U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_L1_L1FALL                  (0X7F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLKGEN_L2                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP2_CLKGEN_L2_L2RISE                  (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_L2_L2RISE                  (7U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_L2_L2RISE                  (0X007FU)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_L2_L2FALL                  (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_L2_L2FALL                  (7U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_L2_L2FALL                  (0X7F00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLKGEN_CKX                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CKX_CK1RISE_MSB            (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CKX_CK1RISE_MSB            (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CKX_CK1RISE_MSB            (0X0003U)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CKX_CK1FALL_MSB            (2U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CKX_CK1FALL_MSB            (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CKX_CK1FALL_MSB            (0X000CU)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CKX_CK2RISE_MSB            (4U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CKX_CK2RISE_MSB            (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CKX_CK2RISE_MSB            (0X0030U)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CKX_CK2FALL_MSB            (6U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CKX_CK2FALL_MSB            (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CKX_CK2FALL_MSB            (0X00C0U)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CKX_CK1REFRISE_MSB         (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CKX_CK1REFRISE_MSB         (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CKX_CK1REFRISE_MSB         (0X0300U)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CKX_CK1REFFALL_MSB         (10U)          /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CKX_CK1REFFALL_MSB         (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CKX_CK1REFFALL_MSB         (0X0C00U)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CKX_CK2REFRISE_MSB         (12U)          /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CKX_CK2REFRISE_MSB         (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CKX_CK2REFRISE_MSB         (0X3000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CKX_CK2REFFALL_MSB         (14U)          /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CKX_CK2REFFALL_MSB         (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CKX_CK2REFFALL_MSB         (0XC000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLKGEN_CYCLE                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CYCLE_CYCLE_MST            (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CYCLE_CYCLE_MST            (7U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CYCLE_CYCLE_MST            (0X007FU)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CYCLE_CGBM                 (7U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CYCLE_CGBM                 (1U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CYCLE_CGBM                 (0X0080U)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CYCLE_L1RISE_MSB           (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CYCLE_L1RISE_MSB           (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CYCLE_L1RISE_MSB           (0X0300U)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CYCLE_L1FALL_MSB           (10U)          /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CYCLE_L1FALL_MSB           (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CYCLE_L1FALL_MSB           (0X0C00U)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CYCLE_L2RISE_MSB           (12U)          /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CYCLE_L2RISE_MSB           (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CYCLE_L2RISE_MSB           (0X3000U)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_CYCLE_L2FALL_MSB           (14U)          /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_CYCLE_L2FALL_MSB           (2U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_CYCLE_L2FALL_MSB           (0XC000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLKGEN_OFFSET                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP2_CLKGEN_OFFSET_CLKOFFSET           (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_OFFSET_CLKOFFSET           (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_OFFSET_CLKOFFSET           (0X00FFU)      /* No description provided */
#define BITP_USEQ_REGS_MAP2_CLKGEN_OFFSET_REFCLKOFFSET        (8U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_OFFSET_REFCLKOFFSET        (8U)           /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_OFFSET_REFCLKOFFSET        (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLKGEN_LTOFFSET                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP2_CLKGEN_LTOFFSET_LTOFFSET          (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_LTOFFSET_LTOFFSET          (16U)          /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_LTOFFSET_LTOFFSET          (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CLKGEN_BURST_PERIOD                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_USEQ_REGS_MAP2_CLKGEN_BURST_PERIOD_BURSTPERIOD   (0U)           /* No description provided */
#define BITL_USEQ_REGS_MAP2_CLKGEN_BURST_PERIOD_BURSTPERIOD   (11U)          /* No description provided */
#define BITM_USEQ_REGS_MAP2_CLKGEN_BURST_PERIOD_BURSTPERIOD   (0X07FFU)      /* No description provided */

#endif  /* end ifndef USEQ_REGS_MAP2_ADDR_RDEF_H_ */

/* ====================================================================================================
        SPIM_REGS Module Instances Address and Mask Definitions
   ==================================================================================================== */
#define INST_SPIM_REGS                                           (0X00000300U)    /* spim_regs: */


#ifndef SPIM_REGS_ADDR_RDEF_H_
#define SPIM_REGS_ADDR_RDEF_H_    /* SPIM_REGS: Your module description, here. */

#define MASK_SPIM_REGS                                           (0X0000FFFFU)    /* SPIM_REGS: Your module description, here. */

/* ====================================================================================================
        SPIM_REGS Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_SPIM_REGS_CTRLR0                                    (0X7) 
#define RSTVAL_SPIM_REGS_CTRLR1                                    (0X0) 
#define RSTVAL_SPIM_REGS_SSIENR                                    (0X0) 
#define RSTVAL_SPIM_REGS_MWCR                                      (0X0) 
#define RSTVAL_SPIM_REGS_SER                                       (0X0) 
#define RSTVAL_SPIM_REGS_BAUDR                                     (0X0) 
#define RSTVAL_SPIM_REGS_TXFTLR                                    (0X0) 
#define RSTVAL_SPIM_REGS_RXFTLR                                    (0X0) 
#define RSTVAL_SPIM_REGS_TXFLR                                     (0X0) 
#define RSTVAL_SPIM_REGS_RXFLR                                     (0X0) 
#define RSTVAL_SPIM_REGS_SR                                        (0X6) 
#define RSTVAL_SPIM_REGS_IMR                                       (0X3F) 
#define RSTVAL_SPIM_REGS_ISR                                       (0X0) 
#define RSTVAL_SPIM_REGS_RISR                                      (0X0) 
#define RSTVAL_SPIM_REGS_TXOICR                                    (0X0) 
#define RSTVAL_SPIM_REGS_RXOICR                                    (0X0) 
#define RSTVAL_SPIM_REGS_RXUICR                                    (0X0) 
#define RSTVAL_SPIM_REGS_MSTICR                                    (0X0) 
#define RSTVAL_SPIM_REGS_ICR                                       (0X0) 
#define RSTVAL_SPIM_REGS_IDR                                       (0XFFFF) 
#define RSTVAL_SPIM_REGS_SSI_VERSION_ID                            (0X322A) 
#define RSTVAL_SPIM_REGS_DR0                                       (0X0) 

/* ====================================================================================================
        SPIM_REGS Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          CTRLR0                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_CTRLR0_DFS                             (0U)           /* No description provided */
#define BITL_SPIM_REGS_CTRLR0_DFS                             (4U)           /* No description provided */
#define BITM_SPIM_REGS_CTRLR0_DFS                             (0X000FU)      /* No description provided */
#define BITP_SPIM_REGS_CTRLR0_FRF                             (4U)           /* No description provided */
#define BITL_SPIM_REGS_CTRLR0_FRF                             (2U)           /* No description provided */
#define BITM_SPIM_REGS_CTRLR0_FRF                             (0X0030U)      /* No description provided */
#define BITP_SPIM_REGS_CTRLR0_SCPH                            (6U)           /* No description provided */
#define BITL_SPIM_REGS_CTRLR0_SCPH                            (1U)           /* No description provided */
#define BITM_SPIM_REGS_CTRLR0_SCPH                            (0X0040U)      /* No description provided */
#define BITP_SPIM_REGS_CTRLR0_SCPOL                           (7U)           /* No description provided */
#define BITL_SPIM_REGS_CTRLR0_SCPOL                           (1U)           /* No description provided */
#define BITM_SPIM_REGS_CTRLR0_SCPOL                           (0X0080U)      /* No description provided */
#define BITP_SPIM_REGS_CTRLR0_TMOD                            (8U)           /* No description provided */
#define BITL_SPIM_REGS_CTRLR0_TMOD                            (2U)           /* No description provided */
#define BITM_SPIM_REGS_CTRLR0_TMOD                            (0X0300U)      /* No description provided */
#define BITP_SPIM_REGS_CTRLR0_NOT_USED                        (10U)          /* No description provided */
#define BITL_SPIM_REGS_CTRLR0_NOT_USED                        (1U)           /* No description provided */
#define BITM_SPIM_REGS_CTRLR0_NOT_USED                        (0X0400U)      /* No description provided */
#define BITP_SPIM_REGS_CTRLR0_SRL                             (11U)          /* No description provided */
#define BITL_SPIM_REGS_CTRLR0_SRL                             (1U)           /* No description provided */
#define BITM_SPIM_REGS_CTRLR0_SRL                             (0X0800U)      /* No description provided */
#define BITP_SPIM_REGS_CTRLR0_CFS                             (12U)          /* No description provided */
#define BITL_SPIM_REGS_CTRLR0_CFS                             (4U)           /* No description provided */
#define BITM_SPIM_REGS_CTRLR0_CFS                             (0XF000U)      /* No description provided */

#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_01_BIT                 (0X0000U)      /* 01-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_02_BIT                 (0X0001U)      /* 02-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_03_BIT                 (0X0002U)      /* 03-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_04_BIT                 (0X0003U)      /* 04-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_05_BIT                 (0X0004U)      /* 05-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_06_BIT                 (0X0005U)      /* 06-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_07_BIT                 (0X0006U)      /* 07-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_08_BIT                 (0X0007U)      /* 08-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_09_BIT                 (0X0008U)      /* 09-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_10_BIT                 (0X0009U)      /* 10-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_11_BIT                 (0X000AU)      /* 11-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_12_BIT                 (0X000BU)      /* 12-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_13_BIT                 (0X000CU)      /* 13-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_14_BIT                 (0X000DU)      /* 14-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_15_BIT                 (0X000EU)      /* 15-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_CFS_SIZE_16_BIT                 (0X000FU)      /* 16-bit Control Word */
#define ENUM_SPIM_REGS_CTRLR0_SRL_NORMAL_MODE                 (0X0000U)      /* Normal mode operation */
#define ENUM_SPIM_REGS_CTRLR0_SRL_TESTING_MODE                (0X0001U)      /* Test mode: Tx & Rx shift reg connected */
#define ENUM_SPIM_REGS_CTRLR0_TMOD_EEPROM_READ                (0X0003U)      /* EEPROM  Read  mode */
#define ENUM_SPIM_REGS_CTRLR0_TMOD_RX_ONLY                    (0X0002U)      /* Receive  only mode */
#define ENUM_SPIM_REGS_CTRLR0_TMOD_TX_AND_RX                  (0X0000U)      /* Transmit & receive */
#define ENUM_SPIM_REGS_CTRLR0_TMOD_TX_ONLY                    (0X0001U)      /* Transmit only mode */
#define ENUM_SPIM_REGS_CTRLR0_SCPOL_INACTIVE_HIGH             (0X0000U)      /* Clock is inactive when high */
#define ENUM_SPIM_REGS_CTRLR0_SCPOL_INACTIVE_LOW              (0X0001U)      /* Clock is inactive when low */
#define ENUM_SPIM_REGS_CTRLR0_SCPH_MIDDLE_BIT                 (0X0000U)      /* Clock toggles in middle of first bit */
#define ENUM_SPIM_REGS_CTRLR0_SCPH_START_BIT                  (0X0001U)      /* Clock toggles at start  of first bit */
#define ENUM_SPIM_REGS_CTRLR0_FRF_MOTOROLLA_SPI               (0X0000U)      /* Motorolla SPI Frame Format */
#define ENUM_SPIM_REGS_CTRLR0_FRF_NS_MICROWIRE                (0X0002U)      /* National Microwire Frame Format */
#define ENUM_SPIM_REGS_CTRLR0_FRF_TEXAS_SSP                   (0X0001U)      /* Texas Instruments SSP Frame Format */
#define ENUM_SPIM_REGS_CTRLR0_DFS_FRAME_04BITS                (0X0003U)      /* 04-bit serial data transfer */
#define ENUM_SPIM_REGS_CTRLR0_DFS_FRAME_05BITS                (0X0004U)      /* 05-bit serial data transfer */
#define ENUM_SPIM_REGS_CTRLR0_DFS_FRAME_06BITS                (0X0005U)      /* 06-bit serial data transfer */
#define ENUM_SPIM_REGS_CTRLR0_DFS_FRAME_07BITS                (0X0006U)      /* 07-bit serial data transfer */
#define ENUM_SPIM_REGS_CTRLR0_DFS_FRAME_08BITS                (0X0007U)      /* 08-bit serial data transfer */
#define ENUM_SPIM_REGS_CTRLR0_DFS_FRAME_09BITS                (0X0008U)      /* 09-bit serial data transfer */
#define ENUM_SPIM_REGS_CTRLR0_DFS_FRAME_10BITS                (0X0009U)      /* 10-bit serial data transfer */
#define ENUM_SPIM_REGS_CTRLR0_DFS_FRAME_11BITS                (0X000AU)      /* 11-bit serial data transfer */
#define ENUM_SPIM_REGS_CTRLR0_DFS_FRAME_12BITS                (0X000BU)      /* 12-bit serial data transfer */
#define ENUM_SPIM_REGS_CTRLR0_DFS_FRAME_13BITS                (0X000CU)      /* 13-bit serial data transfer */
#define ENUM_SPIM_REGS_CTRLR0_DFS_FRAME_14BITS                (0X000DU)      /* 14-bit serial data transfer */
#define ENUM_SPIM_REGS_CTRLR0_DFS_FRAME_15BITS                (0X000EU)      /* 15-bit serial data transfer */
#define ENUM_SPIM_REGS_CTRLR0_DFS_FRAME_16BITS                (0X000FU)      /* 16-bit serial data transfer */

/* ----------------------------------------------------------------------------------------------------
          CTRLR1                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_CTRLR1_NDF                             (0U)           /* No description provided */
#define BITL_SPIM_REGS_CTRLR1_NDF                             (16U)          /* No description provided */
#define BITM_SPIM_REGS_CTRLR1_NDF                             (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSIENR                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_SSIENR_SSI_EN                          (0U)           /* No description provided */
#define BITL_SPIM_REGS_SSIENR_SSI_EN                          (1U)           /* No description provided */
#define BITM_SPIM_REGS_SSIENR_SSI_EN                          (0X0001U)      /* No description provided */

#define ENUM_SPIM_REGS_SSIENR_SSI_EN_DISABLE                  (0X0000U)      /* Disables Serial Transfer */
#define ENUM_SPIM_REGS_SSIENR_SSI_EN_ENABLED                  (0X0001U)      /* Enables  Serial Transfer */

/* ----------------------------------------------------------------------------------------------------
          MWCR                                                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_MWCR_MWMOD                             (0U)           /* No description provided */
#define BITL_SPIM_REGS_MWCR_MWMOD                             (1U)           /* No description provided */
#define BITM_SPIM_REGS_MWCR_MWMOD                             (0X0001U)      /* No description provided */
#define BITP_SPIM_REGS_MWCR_MDD                               (1U)           /* No description provided */
#define BITL_SPIM_REGS_MWCR_MDD                               (1U)           /* No description provided */
#define BITM_SPIM_REGS_MWCR_MDD                               (0X0002U)      /* No description provided */
#define BITP_SPIM_REGS_MWCR_MHS                               (2U)           /* No description provided */
#define BITL_SPIM_REGS_MWCR_MHS                               (1U)           /* No description provided */
#define BITM_SPIM_REGS_MWCR_MHS                               (0X0004U)      /* No description provided */

#define ENUM_SPIM_REGS_MWCR_MHS_DISABLE                       (0X0000U)      /* Disables Handshaking IF */
#define ENUM_SPIM_REGS_MWCR_MHS_ENABLED                       (0X0001U)      /* Enables  Handshaking IF */
#define ENUM_SPIM_REGS_MWCR_MDD_RECEIVE                       (0X0000U)      /* SSI  receives data */
#define ENUM_SPIM_REGS_MWCR_MDD_TRANSMIT                      (0X0001U)      /* SSI transmits data */
#define ENUM_SPIM_REGS_MWCR_MWMOD_NON_SEQUENTIAL              (0X0000U)      /* Non-Sequential Microwire Transfer */
#define ENUM_SPIM_REGS_MWCR_MWMOD_SEQUENTIAL                  (0X0001U)      /* Sequential Microwire Transfer */

/* ----------------------------------------------------------------------------------------------------
          SER                                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_SER_SER                                (0U)           /* No description provided */
#define BITL_SPIM_REGS_SER_SER                                (1U)           /* No description provided */
#define BITM_SPIM_REGS_SER_SER                                (0X0001U)      /* No description provided */

#define ENUM_SPIM_REGS_SER_SER_NOT_SELECTED                   (0X0000U)      /* No slave selected */
#define ENUM_SPIM_REGS_SER_SER_SELECTED                       (0X0001U)      /* Slave is selected */

/* ----------------------------------------------------------------------------------------------------
          BAUDR                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_BAUDR_SCKDV                            (0U)           /* No description provided */
#define BITL_SPIM_REGS_BAUDR_SCKDV                            (16U)          /* No description provided */
#define BITM_SPIM_REGS_BAUDR_SCKDV                            (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          TXFTLR                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_TXFTLR_TFT                             (0U)           /* No description provided */
#define BITL_SPIM_REGS_TXFTLR_TFT                             (2U)           /* No description provided */
#define BITM_SPIM_REGS_TXFTLR_TFT                             (0X0003U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          RXFTLR                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_RXFTLR_RFT                             (0U)           /* No description provided */
#define BITL_SPIM_REGS_RXFTLR_RFT                             (1U)           /* No description provided */
#define BITM_SPIM_REGS_RXFTLR_RFT                             (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          TXFLR                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_TXFLR_TXTFL                            (0U)           /* No description provided */
#define BITL_SPIM_REGS_TXFLR_TXTFL                            (3U)           /* No description provided */
#define BITM_SPIM_REGS_TXFLR_TXTFL                            (0X0007U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          RXFLR                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_RXFLR_RXTFL                            (0U)           /* No description provided */
#define BITL_SPIM_REGS_RXFLR_RXTFL                            (2U)           /* No description provided */
#define BITM_SPIM_REGS_RXFLR_RXTFL                            (0X0003U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SR                                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_SR_BUSY                                (0U)           /* No description provided */
#define BITL_SPIM_REGS_SR_BUSY                                (1U)           /* No description provided */
#define BITM_SPIM_REGS_SR_BUSY                                (0X0001U)      /* No description provided */
#define BITP_SPIM_REGS_SR_TFNF                                (1U)           /* No description provided */
#define BITL_SPIM_REGS_SR_TFNF                                (1U)           /* No description provided */
#define BITM_SPIM_REGS_SR_TFNF                                (0X0002U)      /* No description provided */
#define BITP_SPIM_REGS_SR_TFE                                 (2U)           /* No description provided */
#define BITL_SPIM_REGS_SR_TFE                                 (1U)           /* No description provided */
#define BITM_SPIM_REGS_SR_TFE                                 (0X0004U)      /* No description provided */
#define BITP_SPIM_REGS_SR_RFNE                                (3U)           /* No description provided */
#define BITL_SPIM_REGS_SR_RFNE                                (1U)           /* No description provided */
#define BITM_SPIM_REGS_SR_RFNE                                (0X0008U)      /* No description provided */
#define BITP_SPIM_REGS_SR_RFF                                 (4U)           /* No description provided */
#define BITL_SPIM_REGS_SR_RFF                                 (1U)           /* No description provided */
#define BITM_SPIM_REGS_SR_RFF                                 (0X0010U)      /* No description provided */
#define BITP_SPIM_REGS_SR_DCOL                                (6U)           /* No description provided */
#define BITL_SPIM_REGS_SR_DCOL                                (1U)           /* No description provided */
#define BITM_SPIM_REGS_SR_DCOL                                (0X0040U)      /* No description provided */

#define ENUM_SPIM_REGS_SR_DCOL_NO_ERROR_CONDITION             (0X0000U)      /* No Data Error Condition */
#define ENUM_SPIM_REGS_SR_DCOL_TX_COLLISION_ERROR             (0X0001U)      /* TX Data Collision Error */
#define ENUM_SPIM_REGS_SR_RFF_FULL                            (0X0001U)      /* Rx FIFO is full */
#define ENUM_SPIM_REGS_SR_RFF_NOT_FULL                        (0X0000U)      /* RX FIFO is not full */
#define ENUM_SPIM_REGS_SR_RFNE_EMPTY                          (0X0000U)      /* Rx FIFO  is   empty */
#define ENUM_SPIM_REGS_SR_RFNE_NOT_EMPTY                      (0X0001U)      /* Rx FIFO is nonempty */
#define ENUM_SPIM_REGS_SR_TFE_EMPTY                           (0X0001U)      /* Tx FIFO  is   empty */
#define ENUM_SPIM_REGS_SR_TFE_NOT_EMPTY                       (0X0000U)      /* Tx FIFO is nonempty */
#define ENUM_SPIM_REGS_SR_TFNF_FULL                           (0X0000U)      /* Tx FIFO is full */
#define ENUM_SPIM_REGS_SR_TFNF_NOT_FULL                       (0X0001U)      /* Tx FIFO is not Full */
#define ENUM_SPIM_REGS_SR_BUSY_ACTIVE                         (0X0001U)      /* SSI is Active */
#define ENUM_SPIM_REGS_SR_BUSY_INACTIVE                       (0X0000U)      /* SSI is Idle/Disabled */

/* ----------------------------------------------------------------------------------------------------
          IMR                                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_IMR_TXEIM                              (0U)           /* No description provided */
#define BITL_SPIM_REGS_IMR_TXEIM                              (1U)           /* No description provided */
#define BITM_SPIM_REGS_IMR_TXEIM                              (0X0001U)      /* No description provided */
#define BITP_SPIM_REGS_IMR_TXOIM                              (1U)           /* No description provided */
#define BITL_SPIM_REGS_IMR_TXOIM                              (1U)           /* No description provided */
#define BITM_SPIM_REGS_IMR_TXOIM                              (0X0002U)      /* No description provided */
#define BITP_SPIM_REGS_IMR_RXUIM                              (2U)           /* No description provided */
#define BITL_SPIM_REGS_IMR_RXUIM                              (1U)           /* No description provided */
#define BITM_SPIM_REGS_IMR_RXUIM                              (0X0004U)      /* No description provided */
#define BITP_SPIM_REGS_IMR_RXOIM                              (3U)           /* No description provided */
#define BITL_SPIM_REGS_IMR_RXOIM                              (1U)           /* No description provided */
#define BITM_SPIM_REGS_IMR_RXOIM                              (0X0008U)      /* No description provided */
#define BITP_SPIM_REGS_IMR_RXFIM                              (4U)           /* No description provided */
#define BITL_SPIM_REGS_IMR_RXFIM                              (1U)           /* No description provided */
#define BITM_SPIM_REGS_IMR_RXFIM                              (0X0010U)      /* No description provided */
#define BITP_SPIM_REGS_IMR_MSTIM                              (5U)           /* No description provided */
#define BITL_SPIM_REGS_IMR_MSTIM                              (1U)           /* No description provided */
#define BITM_SPIM_REGS_IMR_MSTIM                              (0X0020U)      /* No description provided */

#define ENUM_SPIM_REGS_IMR_MSTIM_MASKED                       (0X0000U)      /* Multi-Master Contention Interrupt is masked */
#define ENUM_SPIM_REGS_IMR_MSTIM_UNMASKED                     (0X0001U)      /* Multi-Master Contention Interrupt un-masked */
#define ENUM_SPIM_REGS_IMR_RXFIM_MASKED                       (0X0000U)      /* RX FIFO Full Interrupt is masked */
#define ENUM_SPIM_REGS_IMR_RXFIM_UNMASKED                     (0X0001U)      /* RX FIFO Full Interrupt un-masked */
#define ENUM_SPIM_REGS_IMR_RXOIM_MASKED                       (0X0000U)      /* RX FIFO Overflow Interrupt is masked */
#define ENUM_SPIM_REGS_IMR_RXOIM_UNMASKED                     (0X0001U)      /* RX FIFO Overflow Interrupt un-masked */
#define ENUM_SPIM_REGS_IMR_RXUIM_MASKED                       (0X0000U)      /* RX FIFO Underflow Interrupt is masked */
#define ENUM_SPIM_REGS_IMR_RXUIM_UNMASKED                     (0X0001U)      /* RX FIFO Underflow Interrupt un-masked */
#define ENUM_SPIM_REGS_IMR_TXOIM_MASKED                       (0X0000U)      /* TX FIFO Overflow Interrupt is masked */
#define ENUM_SPIM_REGS_IMR_TXOIM_UNMASKED                     (0X0001U)      /* TX FIFO Overflow Interrupt un-masked */
#define ENUM_SPIM_REGS_IMR_TXEIM_MASKED                       (0X0000U)      /* TX FIFO Empty Interrupt is masked */
#define ENUM_SPIM_REGS_IMR_TXEIM_UNMASKED                     (0X0001U)      /* TX FIFO Empty Interrupt un-masked */

/* ----------------------------------------------------------------------------------------------------
          ISR                                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_ISR_TXEIS                              (0U)           /* No description provided */
#define BITL_SPIM_REGS_ISR_TXEIS                              (1U)           /* No description provided */
#define BITM_SPIM_REGS_ISR_TXEIS                              (0X0001U)      /* No description provided */
#define BITP_SPIM_REGS_ISR_TXOIS                              (1U)           /* No description provided */
#define BITL_SPIM_REGS_ISR_TXOIS                              (1U)           /* No description provided */
#define BITM_SPIM_REGS_ISR_TXOIS                              (0X0002U)      /* No description provided */
#define BITP_SPIM_REGS_ISR_RXUIS                              (2U)           /* No description provided */
#define BITL_SPIM_REGS_ISR_RXUIS                              (1U)           /* No description provided */
#define BITM_SPIM_REGS_ISR_RXUIS                              (0X0004U)      /* No description provided */
#define BITP_SPIM_REGS_ISR_RXOIS                              (3U)           /* No description provided */
#define BITL_SPIM_REGS_ISR_RXOIS                              (1U)           /* No description provided */
#define BITM_SPIM_REGS_ISR_RXOIS                              (0X0008U)      /* No description provided */
#define BITP_SPIM_REGS_ISR_RXFIS                              (4U)           /* No description provided */
#define BITL_SPIM_REGS_ISR_RXFIS                              (1U)           /* No description provided */
#define BITM_SPIM_REGS_ISR_RXFIS                              (0X0010U)      /* No description provided */
#define BITP_SPIM_REGS_ISR_MSTIS                              (5U)           /* No description provided */
#define BITL_SPIM_REGS_ISR_MSTIS                              (1U)           /* No description provided */
#define BITM_SPIM_REGS_ISR_MSTIS                              (0X0020U)      /* No description provided */

#define ENUM_SPIM_REGS_ISR_MSTIS_ACTIVE                       (0X0001U)      /* Multi-master Contention Interrupt is active */
#define ENUM_SPIM_REGS_ISR_MSTIS_INACTIVE                     (0X0000U)      /* Multi-master Contention Interrupt nonactive */
#define ENUM_SPIM_REGS_ISR_RXFIS_ACTIVE                       (0X0001U)      /* RX FIFO Full Interrupt is active */
#define ENUM_SPIM_REGS_ISR_RXFIS_INACTIVE                     (0X0000U)      /* RX FIFO Full Interrupt nonactive */
#define ENUM_SPIM_REGS_ISR_RXOIS_ACTIVE                       (0X0001U)      /* RX FIFO Overflow Interrupt is active */
#define ENUM_SPIM_REGS_ISR_RXOIS_INACTIVE                     (0X0000U)      /* RX FIFO Overflow Interrupt nonactive */
#define ENUM_SPIM_REGS_ISR_RXUIS_ACTIVE                       (0X0001U)      /* RX FIFO underflow Interrupt is active */
#define ENUM_SPIM_REGS_ISR_RXUIS_INACTIVE                     (0X0000U)      /* RX FIFO Underflow Interrupt nonactive */
#define ENUM_SPIM_REGS_ISR_TXOIS_ACTIVE                       (0X0001U)      /* TX FIFO Overflow Interrupt is active */
#define ENUM_SPIM_REGS_ISR_TXOIS_INACTIVE                     (0X0000U)      /* TX FIFO Overflow Interrupt nonactive */
#define ENUM_SPIM_REGS_ISR_TXEIS_ACTIVE                       (0X0001U)      /* TX FIFO Empty Interrupt is active */
#define ENUM_SPIM_REGS_ISR_TXEIS_INACTIVE                     (0X0000U)      /* TX FIFO Empty Interrupt nonactive */

/* ----------------------------------------------------------------------------------------------------
          RISR                                                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_RISR_TXEIR                             (0U)           /* No description provided */
#define BITL_SPIM_REGS_RISR_TXEIR                             (1U)           /* No description provided */
#define BITM_SPIM_REGS_RISR_TXEIR                             (0X0001U)      /* No description provided */
#define BITP_SPIM_REGS_RISR_TXOIR                             (1U)           /* No description provided */
#define BITL_SPIM_REGS_RISR_TXOIR                             (1U)           /* No description provided */
#define BITM_SPIM_REGS_RISR_TXOIR                             (0X0002U)      /* No description provided */
#define BITP_SPIM_REGS_RISR_RXUIR                             (2U)           /* No description provided */
#define BITL_SPIM_REGS_RISR_RXUIR                             (1U)           /* No description provided */
#define BITM_SPIM_REGS_RISR_RXUIR                             (0X0004U)      /* No description provided */
#define BITP_SPIM_REGS_RISR_RXOIR                             (3U)           /* No description provided */
#define BITL_SPIM_REGS_RISR_RXOIR                             (1U)           /* No description provided */
#define BITM_SPIM_REGS_RISR_RXOIR                             (0X0008U)      /* No description provided */
#define BITP_SPIM_REGS_RISR_RXFIR                             (4U)           /* No description provided */
#define BITL_SPIM_REGS_RISR_RXFIR                             (1U)           /* No description provided */
#define BITM_SPIM_REGS_RISR_RXFIR                             (0X0010U)      /* No description provided */
#define BITP_SPIM_REGS_RISR_MSTIR                             (5U)           /* No description provided */
#define BITL_SPIM_REGS_RISR_MSTIR                             (1U)           /* No description provided */
#define BITM_SPIM_REGS_RISR_MSTIR                             (0X0020U)      /* No description provided */

#define ENUM_SPIM_REGS_RISR_MSTIR_ACTIVE                      (0X0001U)      /* Raw Multi-master Contention Interrupt is active */
#define ENUM_SPIM_REGS_RISR_MSTIR_INACTIVE                    (0X0000U)      /* Raw Multi-master Contention Interrupt nonactive */
#define ENUM_SPIM_REGS_RISR_RXFIR_ACTIVE                      (0X0001U)      /* Raw RX FIFO Full Interrupt is active */
#define ENUM_SPIM_REGS_RISR_RXFIR_INACTIVE                    (0X0000U)      /* Raw RX FIFO Full Interrupt nonactive */
#define ENUM_SPIM_REGS_RISR_RXOIR_ACTIVE                      (0X0001U)      /* Raw RX FIFO Overflow Interrupt is active */
#define ENUM_SPIM_REGS_RISR_RXOIR_INACTIVE                    (0X0000U)      /* Raw RX FIFO Overflow Interrupt nonactive */
#define ENUM_SPIM_REGS_RISR_RXUIR_ACTIVE                      (0X0001U)      /* Raw RX FIFO underflow Interrupt is active */
#define ENUM_SPIM_REGS_RISR_RXUIR_INACTIVE                    (0X0000U)      /* Raw RX FIFO Underflow Interrupt nonactive */
#define ENUM_SPIM_REGS_RISR_TXOIR_ACTIVE                      (0X0001U)      /* Raw TX FIFO Overflow Interrupt is active */
#define ENUM_SPIM_REGS_RISR_TXOIR_INACTIVE                    (0X0000U)      /* Raw TX FIFO Overflow Interrupt nonactive */
#define ENUM_SPIM_REGS_RISR_TXEIR_ACTIVE                      (0X0001U)      /* Raw TX FIFO Empty Interrupt is active */
#define ENUM_SPIM_REGS_RISR_TXEIR_INACTIVE                    (0X0000U)      /* Raw TX FIFO Empty Interrupt nonactive */

/* ----------------------------------------------------------------------------------------------------
          TXOICR                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_TXOICR_TXOICR                          (0U)           /* No description provided */
#define BITL_SPIM_REGS_TXOICR_TXOICR                          (1U)           /* No description provided */
#define BITM_SPIM_REGS_TXOICR_TXOICR                          (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          RXOICR                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_RXOICR_RXOICR                          (0U)           /* No description provided */
#define BITL_SPIM_REGS_RXOICR_RXOICR                          (1U)           /* No description provided */
#define BITM_SPIM_REGS_RXOICR_RXOICR                          (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          RXUICR                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_RXUICR_RXUICR                          (0U)           /* No description provided */
#define BITL_SPIM_REGS_RXUICR_RXUICR                          (1U)           /* No description provided */
#define BITM_SPIM_REGS_RXUICR_RXUICR                          (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          MSTICR                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_MSTICR_MSTICR                          (0U)           /* No description provided */
#define BITL_SPIM_REGS_MSTICR_MSTICR                          (1U)           /* No description provided */
#define BITM_SPIM_REGS_MSTICR_MSTICR                          (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ICR                                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_ICR_ICR                                (0U)           /* No description provided */
#define BITL_SPIM_REGS_ICR_ICR                                (1U)           /* No description provided */
#define BITM_SPIM_REGS_ICR_ICR                                (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          IDR                                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_IDR_IDCODE                             (0U)           /* No description provided */
#define BITL_SPIM_REGS_IDR_IDCODE                             (16U)          /* No description provided */
#define BITM_SPIM_REGS_IDR_IDCODE                             (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSI_VERSION_ID                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_SSI_VERSION_ID_SSI_COMP_VERSION        (0U)           /* No description provided */
#define BITL_SPIM_REGS_SSI_VERSION_ID_SSI_COMP_VERSION        (16U)          /* No description provided */
#define BITM_SPIM_REGS_SSI_VERSION_ID_SSI_COMP_VERSION        (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DR0                                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SPIM_REGS_DR0_DR0                                (0U)           /* No description provided */
#define BITL_SPIM_REGS_DR0_DR0                                (16U)          /* No description provided */
#define BITM_SPIM_REGS_DR0_DR0                                (0XFFFFU)      /* No description provided */

#endif  /* end ifndef SPIM_REGS_ADDR_RDEF_H_ */

/* ====================================================================================================
        CSI2_REGSPEC_TOP_CPU0 Module Instances Address and Mask Definitions
   ==================================================================================================== */
#define INST_MIPI_REGS                                           (0X00000400U)    /* mipi_regs: */


#ifndef CSI2_REGSPEC_TOP_CPU0_ADDR_RDEF_H_
#define CSI2_REGSPEC_TOP_CPU0_ADDR_RDEF_H_    /* CSI2_REGSPEC_TOP_CPU0: Your module description, here. */

#define MASK_CSI2_REGSPEC_TOP_CPU0                               (0XFFFFFFFFU)    /* CSI2_REGSPEC_TOP_CPU0: Your module description, here. */

/* ====================================================================================================
        CSI2_REGSPEC_TOP_CPU0 Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_NUM_LANES    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_T_PRE        (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_T_POST       (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_TX_GAP       (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_T_CLK_GAP    (0XF) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_CONTINUOUS_HS_CLK    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_TWAKEUP      (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_CLK_ENABLE    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_ENABLE      (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_CLK_ACTIVE    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_ACTIVE      (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_IRQ_STATUS       (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_IRQ_ENABLE       (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CSI2TX_IRQ_CLR    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_CLK_LANE_EN    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_DATA_LANE_EN    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_CPHY_EN      (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_PPI_16_EN    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_PACKET_INTERFACE_EN    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_VCX_EN       (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_I    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_P    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PLL    (0X1) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PLL_CTRL    (0X1) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PHY    (0X1) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PHY_CTRL    (0X1) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TST_PLL    (0X9) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CN    (0X1F) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CM    (0XA8) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CO    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_BYP    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_BYPASS_PLL    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_LATCH    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TX_RCAL    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_AUTO_PD_EN    (0X1) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_ENBL    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_ZERO    (0X1F) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_ZERO    (0X54) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_TRAIL    (0X18) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_TRAIL    (0X18) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_PREPARE    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_PREPARE    (0X1) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN0    (0X0) 
#define RSTVAL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN1    (0X0) 

/* ====================================================================================================
        CSI2_REGSPEC_TOP_CPU0 Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CFG_NUM_LANES                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_NUM_LANES              (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_NUM_LANES              (4U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_NUM_LANES              (0X0000000FU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CFG_T_PRE                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_T_PRE                  (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_T_PRE                  (8U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_T_PRE                  (0X000000FFU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CFG_T_POST                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_T_POST                 (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_T_POST                 (8U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_T_POST                 (0X000000FFU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CFG_TX_GAP                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_TX_GAP                 (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_TX_GAP                 (8U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_TX_GAP                 (0X000000FFU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CFG_T_CLK_GAP                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_T_CLK_GAP              (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_T_CLK_GAP              (8U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_T_CLK_GAP              (0X000000FFU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CFG_CONTINUOUS_HS_CLK                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_CONTINUOUS_HS_CLK      (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_CONTINUOUS_HS_CLK      (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_CONTINUOUS_HS_CLK      (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CFG_TWAKEUP                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_TWAKEUP                (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_TWAKEUP                (19U)          /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_TWAKEUP                (0X0007FFFFU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_ULPS_CLK_ENABLE                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_ULPS_CLK_ENABLE            (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_ULPS_CLK_ENABLE            (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_ULPS_CLK_ENABLE            (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_ULPS_ENABLE                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_ULPS_ENABLE                (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_ULPS_ENABLE                (2U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_ULPS_ENABLE                (0X00000003U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_ULPS_CLK_ACTIVE                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_ULPS_CLK_ACTIVE            (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_ULPS_CLK_ACTIVE            (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_ULPS_CLK_ACTIVE            (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_ULPS_ACTIVE                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_ULPS_ACTIVE                (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_ULPS_ACTIVE                (2U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_ULPS_ACTIVE                (0X00000003U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_IRQ_STATUS                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_IRQ_STATUS                 (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_IRQ_STATUS                 (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_IRQ_STATUS                 (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_IRQ_ENABLE                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_IRQ_ENABLE                 (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_IRQ_ENABLE                 (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_IRQ_ENABLE                 (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CSI2TX_IRQ_CLR                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_IRQ_CLR                    (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_IRQ_CLR                    (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_IRQ_CLR                    (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CFG_CLK_LANE_EN                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_CLK_LANE_EN            (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_CLK_LANE_EN            (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_CLK_LANE_EN            (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CFG_DATA_LANE_EN                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_DATA_LANE_EN           (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_DATA_LANE_EN           (2U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_DATA_LANE_EN           (0X00000003U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CFG_CPHY_EN                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_CPHY_EN                (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_CPHY_EN                (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_CPHY_EN                (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CFG_PPI_16_EN                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_PPI_16_EN              (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_PPI_16_EN              (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_PPI_16_EN              (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CFG_PACKET_INTERFACE_EN                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_PACKET_INTERFACE_EN    (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_PACKET_INTERFACE_EN    (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_PACKET_INTERFACE_EN    (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_BASE_CFG_VCX_EN                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_VCX_EN                 (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_VCX_EN                 (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_VCX_EN                 (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_I                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_SKEWCAL_TIME_I         (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_SKEWCAL_TIME_I         (16U)          /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_SKEWCAL_TIME_I         (0X0000FFFFU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_P                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_SKEWCAL_TIME_P         (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_SKEWCAL_TIME_P         (16U)          /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_SKEWCAL_TIME_P         (0X0000FFFFU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PLL                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_PD_PLL           (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_PD_PLL           (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_PD_PLL           (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PLL_CTRL            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_ULPS_PLL_CTRL    (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_ULPS_PLL_CTRL    (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_ULPS_PLL_CTRL    (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PHY                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_PD_PHY           (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_PD_PHY           (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_PD_PHY           (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PHY_CTRL            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_ULPS_PHY_CTRL    (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_ULPS_PHY_CTRL    (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_ULPS_PHY_CTRL    (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TST_PLL                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_TST_PLL          (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_TST_PLL          (4U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_TST_PLL          (0X0000000FU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CN                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_CN               (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_CN               (5U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_CN               (0X0000001FU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CM                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_CM               (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_CM               (8U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_CM               (0X000000FFU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CO                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_CO               (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_CO               (3U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_CO               (0X00000007U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_BYP                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_LOCK_BYP         (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_LOCK_BYP         (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_LOCK_BYP         (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_BYPASS_PLL               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_BYPASS_PLL       (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_BYPASS_PLL       (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_BYPASS_PLL       (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_LATCH               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_LOCK_LATCH       (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_LOCK_LATCH       (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_LOCK_LATCH       (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TX_RCAL                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_TX_RCAL          (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_TX_RCAL          (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_TX_RCAL          (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_AUTO_PD_EN               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_AUTO_PD_EN       (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_AUTO_PD_EN       (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_AUTO_PD_EN       (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_ENBL                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_TEST_ENBL        (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_TEST_ENBL        (6U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_TEST_ENBL        (0X0000003FU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_LOCK             (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_LOCK             (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_LOCK             (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_ZERO            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_M_PRG_HS_ZERO    (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_M_PRG_HS_ZERO    (6U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_M_PRG_HS_ZERO    (0X0000003FU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_ZERO           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_MC_PRG_HS_ZERO   (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_MC_PRG_HS_ZERO   (7U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_MC_PRG_HS_ZERO   (0X0000007FU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_TRAIL           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_M_PRG_HS_TRAIL   (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_M_PRG_HS_TRAIL   (5U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_M_PRG_HS_TRAIL   (0X0000001FU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_TRAIL          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_MC_PRG_HS_TRAIL  (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_MC_PRG_HS_TRAIL  (5U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_MC_PRG_HS_TRAIL  (0X0000001FU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_PREPARE         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_M_PRG_HS_PREPARE (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_M_PRG_HS_PREPARE (2U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_M_PRG_HS_PREPARE (0X00000003U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_PREPARE        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_MC_PRG_HS_PREPARE (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_MC_PRG_HS_PREPARE (1U)           /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CFG_MIXEL_MC_PRG_HS_PREPARE (0X00000001U)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN0            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN0_CFG_MIXEL_TEST_PATTERN (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN0_CFG_MIXEL_TEST_PATTERN (16U)          /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN0_CFG_MIXEL_TEST_PATTERN (0X0000FFFFU)  /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN1            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN1_CFG_MIXEL_TEST_PATTERN (0U)           /* No description provided */
#define BITL_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN1_CFG_MIXEL_TEST_PATTERN (16U)          /* No description provided */
#define BITM_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN1_CFG_MIXEL_TEST_PATTERN (0X0000FFFFU)  /* No description provided */

#endif  /* end ifndef CSI2_REGSPEC_TOP_CPU0_ADDR_RDEF_H_ */

/* ====================================================================================================
        EFUSE Module Instances Address and Mask Definitions
   ==================================================================================================== */
#define INST_EFUSE_REGS                                          (0X00000600U)    /* efuse_regs: */


#ifndef EFUSE_ADDR_RDEF_H_
#define EFUSE_ADDR_RDEF_H_    /* EFUSE: eFuse */

#define MASK_EFUSE                                               (0X000000FFU)    /* EFUSE: eFuse */

/* ====================================================================================================
        EFUSE Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_EFUSE_PWR_CTRL                                      (0X0) 
#define RSTVAL_EFUSE_STATUS                                        (0X0) 
#define RSTVAL_EFUSE_ERR_LOCATION                                  (0X0) 
#define RSTVAL_EFUSE_TIMING                                        (0XA53C) 
#define RSTVAL_EFUSE_CHARACTERIZATION                              (0X0) 

/* ====================================================================================================
        EFUSE Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          PWR_CTRL                                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_EFUSE_POWER_CONTROL                              (0U)           /* Power Control */
#define BITL_EFUSE_POWER_CONTROL                              (1U)           /* Power Control */
#define BITM_EFUSE_POWER_CONTROL                              (0X0001U)      /* Power Control */

#define ENUM_EFUSE_POWER_CONTROL_POWERDOWN                    (0X0000U)      /* POWERDOWN: Lowest power state. All accesses incur extra 800ns wait to enter standby state */
#define ENUM_EFUSE_POWER_CONTROL_STANDBY                      (0X0001U)      /* STANDBY: Ready to program or read */

/* ----------------------------------------------------------------------------------------------------
          STATUS                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_EFUSE_FUSE_NOT_BURNED                            (0U)           /* Post-programming error: fuse not burned */
#define BITL_EFUSE_FUSE_NOT_BURNED                            (1U)           /* Post-programming error: fuse not burned */
#define BITM_EFUSE_FUSE_NOT_BURNED                            (0X0001U)      /* Post-programming error: fuse not burned */
#define BITP_EFUSE_FUSE_ACCIDENTAL_BURN                       (1U)           /* Post-programming error: fuse accidentally burned */
#define BITL_EFUSE_FUSE_ACCIDENTAL_BURN                       (1U)           /* Post-programming error: fuse accidentally burned */
#define BITM_EFUSE_FUSE_ACCIDENTAL_BURN                       (0X0002U)      /* Post-programming error: fuse accidentally burned */

/* ----------------------------------------------------------------------------------------------------
          ERR_LOCATION                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_EFUSE_PGM_ERR_BIT_ADDR                           (0U)           /* Error location */
#define BITL_EFUSE_PGM_ERR_BIT_ADDR                           (12U)          /* Error location */
#define BITM_EFUSE_PGM_ERR_BIT_ADDR                           (0X0FFFU)      /* Error location */

/* ----------------------------------------------------------------------------------------------------
          TIMING                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_EFUSE_TPGM_PULSE                                 (0U)           /* Program STROBE pulse width */
#define BITL_EFUSE_TPGM_PULSE                                 (8U)           /* Program STROBE pulse width */
#define BITM_EFUSE_TPGM_PULSE                                 (0X00FFU)      /* Program STROBE pulse width */
#define BITP_EFUSE_TREAD_PULSE                                (8U)           /* Read STROBE pulse width */
#define BITL_EFUSE_TREAD_PULSE                                (2U)           /* Read STROBE pulse width */
#define BITM_EFUSE_TREAD_PULSE                                (0X0300U)      /* Read STROBE pulse width */
#define BITP_EFUSE_TUNIT                                      (10U)          /* Efuse timing diagram step size */
#define BITL_EFUSE_TUNIT                                      (2U)           /* Efuse timing diagram step size */
#define BITM_EFUSE_TUNIT                                      (0X0C00U)      /* Efuse timing diagram step size */
#define BITP_EFUSE_TPDPS                                      (12U)          /* PD to PS setup time */
#define BITL_EFUSE_TPDPS                                      (4U)           /* PD to PS setup time */
#define BITM_EFUSE_TPDPS                                      (0XF000U)      /* PD to PS setup time */

/* ----------------------------------------------------------------------------------------------------
          CHARACTERIZATION                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_EFUSE_RM_ENABLE                                  (0U)           /* Enable resistance measurement mode */
#define BITL_EFUSE_RM_ENABLE                                  (1U)           /* Enable resistance measurement mode */
#define BITM_EFUSE_RM_ENABLE                                  (0X0001U)      /* Enable resistance measurement mode */
#define BITP_EFUSE_PGM_MODE                                   (4U)           /* Program mode */
#define BITL_EFUSE_PGM_MODE                                   (1U)           /* Program mode */
#define BITM_EFUSE_PGM_MODE                                   (0X0010U)      /* Program mode */
#define BITP_EFUSE_MARGIN                                     (8U)           /* Set the margin level in the sense amplifiers */
#define BITL_EFUSE_MARGIN                                     (2U)           /* Set the margin level in the sense amplifiers */
#define BITM_EFUSE_MARGIN                                     (0X0300U)      /* Set the margin level in the sense amplifiers */

#define ENUM_EFUSE_MARGIN_MARGIN_AUTO                         (0X0000U)      /* AUTO: Margin set automatically by state machine */
#define ENUM_EFUSE_MARGIN_MARGIN_LOW                          (0X0001U)      /* LOW: Applies lowest margin. Used at time 0 to test for unusual high resistance fuse */
#define ENUM_EFUSE_MARGIN_MARGIN_NORMAL                       (0X0002U)      /* NOMINAL: Applies nominal margin level */
#define ENUM_EFUSE_MARGIN_MARGIN_HIGH                         (0X0003U)      /* HIGH: Applies highest margin. Used in post program readback to ensure no weakly blown fuses. */
#define ENUM_EFUSE_PGM_MODE_FUNCTIONAL_PGM                    (0X0000U)      /* Functional mode */
#define ENUM_EFUSE_PGM_MODE_DEBUG_PGM                         (0X0001U)      /* Debug Mode */

/* ====================================================================================================
        EFUSE Module MemoryRegions Address Offset, Size Definitions 
   ==================================================================================================== */
#define MEMR_IDX_EFUSE_EF_MEM                               (0X00U)    /* Efuse Memory */
#define MEMR_SIZE_EFUSE_EF_MEM                              (0X80U)    /* Size in Bytes. */

#endif  /* end ifndef EFUSE_ADDR_RDEF_H_ */

/* ====================================================================================================
        LPS_REGS_YODA Module Instances Address and Mask Definitions
   ==================================================================================================== */
#define INST_LPS1_REGS                                           (0X00000900U)    /* lps1_regs: */
#define INST_LPS2_REGS                                           (0X00000A00U)    /* lps2_regs: */


#ifndef LPS_REGS_YODA_ADDR_RDEF_H_
#define LPS_REGS_YODA_ADDR_RDEF_H_    /* LPS_REGS_YODA: Your module description, here. */

#define MASK_LPS_REGS_YODA                                       (0X000000FFU)    /* LPS_REGS_YODA: Your module description, here. */

/* ====================================================================================================
        LPS_REGS_YODA Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_LPS_REGS_YODA_LPSCTRL                               (0X40) 
#define RSTVAL_LPS_REGS_YODA_LPSWAVEFREQ                           (0X1) 
#define RSTVAL_LPS_REGS_YODA_LPSWAVEGENACC                         (0X0) 
#define RSTVAL_LPS_REGS_YODA_LPSRAMADDR                            (0X0) 
#define RSTVAL_LPS_REGS_YODA_LPSRAMRDCMD                           (0X0) 
#define RSTVAL_LPS_REGS_YODA_LPSWAVEGENADDR                        (0X0) 
#define RSTVAL_LPS_REGS_YODA_LPSMARGIN                             (0X0) 
#define RSTVAL_LPS_REGS_YODA_LPSDBG                                (0X0) 
#define RSTVAL_LPS_REGS_YODA_LPSRAMDATA                            (0X0) 
#define RSTVAL_LPS_REGS_YODA_LPSRAMDATA_ALIAS                      (0X0) 

/* ====================================================================================================
        LPS_REGS_YODA Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          LPSCTRL                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LPS_REGS_YODA_LPSCTRL_LPS_ENA                    (0U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSCTRL_LPS_ENA                    (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSCTRL_LPS_ENA                    (0X0001U)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSCTRL_RESERVED1                  (1U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSCTRL_RESERVED1                  (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSCTRL_RESERVED1                  (0X0002U)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSCTRL_LPS_BUSY                   (2U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSCTRL_LPS_BUSY                   (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSCTRL_LPS_BUSY                   (0X0004U)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSCTRL_RESERVED                   (3U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSCTRL_RESERVED                   (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSCTRL_RESERVED                   (0X0008U)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSCTRL_LPS_START_EN               (4U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSCTRL_LPS_START_EN               (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSCTRL_LPS_START_EN               (0X0010U)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSCTRL_LPS_RPT_EN                 (5U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSCTRL_LPS_RPT_EN                 (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSCTRL_LPS_RPT_EN                 (0X0020U)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSCTRL_LPS_ON                     (6U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSCTRL_LPS_ON                     (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSCTRL_LPS_ON                     (0X0040U)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSCTRL_LPS_OFF                    (7U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSCTRL_LPS_OFF                    (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSCTRL_LPS_OFF                    (0X0080U)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSCTRL_LPS_ON_IGNORE              (8U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSCTRL_LPS_ON_IGNORE              (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSCTRL_LPS_ON_IGNORE              (0X0100U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          LPSWAVEFREQ                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LPS_REGS_YODA_LPSWAVEFREQ_LPS_WAVE_FREQ          (0U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSWAVEFREQ_LPS_WAVE_FREQ          (8U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSWAVEFREQ_LPS_WAVE_FREQ          (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          LPSWAVEGENACC                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LPS_REGS_YODA_LPSWAVEGENACC_LPS_WG_ACC           (0U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSWAVEGENACC_LPS_WG_ACC           (16U)          /* No description provided */
#define BITM_LPS_REGS_YODA_LPSWAVEGENACC_LPS_WG_ACC           (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          LPSRAMADDR                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LPS_REGS_YODA_LPSRAMADDR_LPS_RAM_ADDR            (0U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSRAMADDR_LPS_RAM_ADDR            (9U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSRAMADDR_LPS_RAM_ADDR            (0X01FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          LPSRAMRDCMD                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LPS_REGS_YODA_LPSRAMRDCMD_LPS_RAM_READ_EN        (0U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSRAMRDCMD_LPS_RAM_READ_EN        (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSRAMRDCMD_LPS_RAM_READ_EN        (0X0001U)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSRAMRDCMD_LPS_RAM_READ_RDY       (1U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSRAMRDCMD_LPS_RAM_READ_RDY       (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSRAMRDCMD_LPS_RAM_READ_RDY       (0X0002U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          LPSWAVEGENADDR                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LPS_REGS_YODA_LPSWAVEGENADDR_LPS_RAM_START       (0U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSWAVEGENADDR_LPS_RAM_START       (8U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSWAVEGENADDR_LPS_RAM_START       (0X00FFU)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSWAVEGENADDR_LPS_RAM_END         (8U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSWAVEGENADDR_LPS_RAM_END         (8U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSWAVEGENADDR_LPS_RAM_END         (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          LPSMARGIN                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LPS_REGS_YODA_LPSMARGIN_LPS_PORT0_MARGIN         (0U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSMARGIN_LPS_PORT0_MARGIN         (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSMARGIN_LPS_PORT0_MARGIN         (0X0001U)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSMARGIN_LPS_PORT1_MARGIN         (1U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSMARGIN_LPS_PORT1_MARGIN         (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSMARGIN_LPS_PORT1_MARGIN         (0X0002U)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSMARGIN_LPS_PARITY_ERR           (15U)          /* No description provided */
#define BITL_LPS_REGS_YODA_LPSMARGIN_LPS_PARITY_ERR           (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSMARGIN_LPS_PARITY_ERR           (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          LPSDBG                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LPS_REGS_YODA_LPSDBG_LPSDBGSEL                   (0U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSDBG_LPSDBGSEL                   (3U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSDBG_LPSDBGSEL                   (0X0007U)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSDBG_LPSDBGCOM                   (8U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSDBG_LPSDBGCOM                   (2U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSDBG_LPSDBGCOM                   (0X0300U)      /* No description provided */
#define BITP_LPS_REGS_YODA_LPSDBG_LPSDBGEN                    (15U)          /* No description provided */
#define BITL_LPS_REGS_YODA_LPSDBG_LPSDBGEN                    (1U)           /* No description provided */
#define BITM_LPS_REGS_YODA_LPSDBG_LPSDBGEN                    (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          LPSRAMDATA                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LPS_REGS_YODA_LPSRAMDATA_LPS_RAM_DATA            (0U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSRAMDATA_LPS_RAM_DATA            (16U)          /* No description provided */
#define BITM_LPS_REGS_YODA_LPSRAMDATA_LPS_RAM_DATA            (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          LPSRAMDATA_ALIAS                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_LPS_REGS_YODA_LPSRAMDATA_ALIAS_LPS_RAM_DATA_ALIAS (0U)           /* No description provided */
#define BITL_LPS_REGS_YODA_LPSRAMDATA_ALIAS_LPS_RAM_DATA_ALIAS (16U)          /* No description provided */
#define BITM_LPS_REGS_YODA_LPSRAMDATA_ALIAS_LPS_RAM_DATA_ALIAS (0XFFFFU)      /* No description provided */

#endif  /* end ifndef LPS_REGS_YODA_ADDR_RDEF_H_ */

/* ====================================================================================================
        SS_REGS Module Instances Address and Mask Definitions
   ==================================================================================================== */
#define INST_SS_REGS                                             (0X00000B00U)    /* ss_regs: */


#ifndef SS_REGS_ADDR_RDEF_H_
#define SS_REGS_ADDR_RDEF_H_    /* SS_REGS: Your module description, here. */

#define MASK_SS_REGS                                             (0XFFFFFFFFU)    /* SS_REGS: Your module description, here. */

/* ====================================================================================================
        SS_REGS Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_SSWAVEFREQ                                          (0X0) 
#define RSTVAL_SSWAVEPERIOD                                        (0XA0) 
#define RSTVAL_SSWAVEAMP                                           (0XFFFF) 
#define RSTVAL_SSPINLO                                             (0X7454) 
#define RSTVAL_SSPINHI                                             (0X9B) 
#define RSTVAL_SSINTERVLO                                          (0X2B8) 
#define RSTVAL_SSINTERVHI                                          (0X2) 
#define RSTVAL_SSWAVEOFFSET                                        (0X0) 
#define RSTVAL_SSCTRL                                              (0X0) 
#define RSTVAL_SSMODFREQ                                           (0X0) 
#define RSTVAL_SSMODAMP                                            (0X0) 
#define RSTVAL_SSINTERV_10                                         (0X5600) 
#define RSTVAL_SSINTERV_11                                         (0X355) 
#define RSTVAL_SSINTERV_20                                         (0X0) 
#define RSTVAL_SSINTERV_21                                         (0X0) 
#define RSTVAL_SSINTERV_30                                         (0XAA00) 
#define RSTVAL_SSINTERV_31                                         (0XAA) 
#define RSTVAL_SSVALUE_00                                          (0X100) 
#define RSTVAL_SSVALUE_01                                          (0X300) 
#define RSTVAL_SSVALUE_10                                          (0XAB00) 
#define RSTVAL_SSVALUE_11                                          (0X3AA) 
#define RSTVAL_SSVALUE_20                                          (0X5500) 
#define RSTVAL_SSVALUE_21                                          (0X55) 
#define RSTVAL_SSVALUE_30                                          (0XFF00) 
#define RSTVAL_SSVALUE_31                                          (0XFF) 
#define RSTVAL_SSDBG                                               (0X0) 

/* ====================================================================================================
        SS_REGS Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          SSWAVEFREQ                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSWAVEFREQ_WAVE_FREQ                             (0U)           /* No description provided */
#define BITL_SSWAVEFREQ_WAVE_FREQ                             (8U)           /* No description provided */
#define BITM_SSWAVEFREQ_WAVE_FREQ                             (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSWAVEPERIOD                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSWAVEPERIOD_WAVE_PERIOD                         (0U)           /* No description provided */
#define BITL_SSWAVEPERIOD_WAVE_PERIOD                         (10U)          /* No description provided */
#define BITM_SSWAVEPERIOD_WAVE_PERIOD                         (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSWAVEAMP                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSWAVEAMP_WAVE_AMPLITUDE                         (0U)           /* No description provided */
#define BITL_SSWAVEAMP_WAVE_AMPLITUDE                         (16U)          /* No description provided */
#define BITM_SSWAVEAMP_WAVE_AMPLITUDE                         (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSPINLO                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSPINLO_PI_N_LSBS                                (0U)           /* No description provided */
#define BITL_SSPINLO_PI_N_LSBS                                (16U)          /* No description provided */
#define BITM_SSPINLO_PI_N_LSBS                                (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSPINHI                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSPINHI_PI_N_MSBS                                (0U)           /* No description provided */
#define BITL_SSPINHI_PI_N_MSBS                                (8U)           /* No description provided */
#define BITM_SSPINHI_PI_N_MSBS                                (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSINTERVLO                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSINTERVLO_INTERVAL_LSBS                         (0U)           /* No description provided */
#define BITL_SSINTERVLO_INTERVAL_LSBS                         (16U)          /* No description provided */
#define BITM_SSINTERVLO_INTERVAL_LSBS                         (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSINTERVHI                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSINTERVHI_INTERVAL_MSBS                         (0U)           /* No description provided */
#define BITL_SSINTERVHI_INTERVAL_MSBS                         (8U)           /* No description provided */
#define BITM_SSINTERVHI_INTERVAL_MSBS                         (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSWAVEOFFSET                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSWAVEOFFSET_WAVE_OFFSET                         (0U)           /* No description provided */
#define BITL_SSWAVEOFFSET_WAVE_OFFSET                         (16U)          /* No description provided */
#define BITM_SSWAVEOFFSET_WAVE_OFFSET                         (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSCTRL                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSCTRL_SS_ENA                                    (0U)           /* No description provided */
#define BITL_SSCTRL_SS_ENA                                    (1U)           /* No description provided */
#define BITM_SSCTRL_SS_ENA                                    (0X0001U)      /* No description provided */
#define BITP_SSCTRL_WTYPE                                     (1U)           /* No description provided */
#define BITL_SSCTRL_WTYPE                                     (2U)           /* No description provided */
#define BITM_SSCTRL_WTYPE                                     (0X0006U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSMODFREQ                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSMODFREQ_MOD_FREQ                               (0U)           /* No description provided */
#define BITL_SSMODFREQ_MOD_FREQ                               (10U)          /* No description provided */
#define BITM_SSMODFREQ_MOD_FREQ                               (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSMODAMP                                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSMODAMP_MOD_AMP                                 (0U)           /* No description provided */
#define BITL_SSMODAMP_MOD_AMP                                 (8U)           /* No description provided */
#define BITM_SSMODAMP_MOD_AMP                                 (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSINTERV_10                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSINTERV_10_INTERV_1_0                           (0U)           /* No description provided */
#define BITL_SSINTERV_10_INTERV_1_0                           (16U)          /* No description provided */
#define BITM_SSINTERV_10_INTERV_1_0                           (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSINTERV_11                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSINTERV_11_INTERV_1_1                           (0U)           /* No description provided */
#define BITL_SSINTERV_11_INTERV_1_1                           (10U)          /* No description provided */
#define BITM_SSINTERV_11_INTERV_1_1                           (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSINTERV_20                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSINTERV_20_INTERV_2_0                           (0U)           /* No description provided */
#define BITL_SSINTERV_20_INTERV_2_0                           (16U)          /* No description provided */
#define BITM_SSINTERV_20_INTERV_2_0                           (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSINTERV_21                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSINTERV_21_INTERV_2_1                           (0U)           /* No description provided */
#define BITL_SSINTERV_21_INTERV_2_1                           (10U)          /* No description provided */
#define BITM_SSINTERV_21_INTERV_2_1                           (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSINTERV_30                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSINTERV_30_INTERV_3_0                           (0U)           /* No description provided */
#define BITL_SSINTERV_30_INTERV_3_0                           (16U)          /* No description provided */
#define BITM_SSINTERV_30_INTERV_3_0                           (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSINTERV_31                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSINTERV_31_INTERV_3_1                           (0U)           /* No description provided */
#define BITL_SSINTERV_31_INTERV_3_1                           (10U)          /* No description provided */
#define BITM_SSINTERV_31_INTERV_3_1                           (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSVALUE_00                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSVALUE_00_VALUE_0_0                             (0U)           /* No description provided */
#define BITL_SSVALUE_00_VALUE_0_0                             (16U)          /* No description provided */
#define BITM_SSVALUE_00_VALUE_0_0                             (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSVALUE_01                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSVALUE_01_VALUE_0_1                             (0U)           /* No description provided */
#define BITL_SSVALUE_01_VALUE_0_1                             (10U)          /* No description provided */
#define BITM_SSVALUE_01_VALUE_0_1                             (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSVALUE_10                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSVALUE_10_VALUE_1_0                             (0U)           /* No description provided */
#define BITL_SSVALUE_10_VALUE_1_0                             (16U)          /* No description provided */
#define BITM_SSVALUE_10_VALUE_1_0                             (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSVALUE_11                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSVALUE_11_VALUE_1_1                             (0U)           /* No description provided */
#define BITL_SSVALUE_11_VALUE_1_1                             (10U)          /* No description provided */
#define BITM_SSVALUE_11_VALUE_1_1                             (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSVALUE_20                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSVALUE_20_VALUE_2_0                             (0U)           /* No description provided */
#define BITL_SSVALUE_20_VALUE_2_0                             (16U)          /* No description provided */
#define BITM_SSVALUE_20_VALUE_2_0                             (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSVALUE_21                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSVALUE_21_VALUE_2_1                             (0U)           /* No description provided */
#define BITL_SSVALUE_21_VALUE_2_1                             (10U)          /* No description provided */
#define BITM_SSVALUE_21_VALUE_2_1                             (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSVALUE_30                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSVALUE_30_VALUE_3_0                             (0U)           /* No description provided */
#define BITL_SSVALUE_30_VALUE_3_0                             (16U)          /* No description provided */
#define BITM_SSVALUE_30_VALUE_3_0                             (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSVALUE_31                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSVALUE_31_VALUE_3_1                             (0U)           /* No description provided */
#define BITL_SSVALUE_31_VALUE_3_1                             (10U)          /* No description provided */
#define BITM_SSVALUE_31_VALUE_3_1                             (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SSDBG                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_SSDBG_SSDBGSEL                                   (0U)           /* No description provided */
#define BITL_SSDBG_SSDBGSEL                                   (2U)           /* No description provided */
#define BITM_SSDBG_SSDBGSEL                                   (0X0003U)      /* No description provided */
#define BITP_SSDBG_SSDBGCOM                                   (8U)           /* No description provided */
#define BITL_SSDBG_SSDBGCOM                                   (2U)           /* No description provided */
#define BITM_SSDBG_SSDBGCOM                                   (0X0300U)      /* No description provided */
#define BITP_SSDBG_SSDBGEN                                    (15U)          /* No description provided */
#define BITL_SSDBG_SSDBGEN                                    (1U)           /* No description provided */
#define BITM_SSDBG_SSDBGEN                                    (0X8000U)      /* No description provided */

#endif  /* end ifndef SS_REGS_ADDR_RDEF_H_ */

/* ====================================================================================================
        PCM_REGS_YODA Module Instances Address and Mask Definitions
   ==================================================================================================== */
#define INST_PCM_REGS                                            (0X00000C00U)    /* pcm_regs: */


#ifndef PCM_REGS_YODA_ADDR_RDEF_H_
#define PCM_REGS_YODA_ADDR_RDEF_H_    /* PCM_REGS_YODA: Your module description, here. */

#define MASK_PCM_REGS_YODA                                       (0X000000FFU)    /* PCM_REGS_YODA: Your module description, here. */

/* ====================================================================================================
        PCM_REGS_YODA Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_PCM_REGS_YODA_PCMCTRL_0                             (0X0) 
#define RSTVAL_PCM_REGS_YODA_PCMCTRL_1                             (0X0) 
#define RSTVAL_PCM_REGS_YODA_PCMOUT                                (0X0) 
#define RSTVAL_PCM_REGS_YODA_OSC_PERIOD_CTRL                       (0X0) 
#define RSTVAL_PCM_REGS_YODA_OSC_PERIOD_RD                         (0X0) 

/* ====================================================================================================
        PCM_REGS_YODA Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          PCMCTRL_0                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PCM_REGS_YODA_PCMCTRL_0_PCM_START                (0U)           /* No description provided */
#define BITL_PCM_REGS_YODA_PCMCTRL_0_PCM_START                (1U)           /* No description provided */
#define BITM_PCM_REGS_YODA_PCMCTRL_0_PCM_START                (0X0001U)      /* No description provided */
#define BITP_PCM_REGS_YODA_PCMCTRL_0_PCM_SELECT               (1U)           /* No description provided */
#define BITL_PCM_REGS_YODA_PCMCTRL_0_PCM_SELECT               (5U)           /* No description provided */
#define BITM_PCM_REGS_YODA_PCMCTRL_0_PCM_SELECT               (0X003EU)      /* No description provided */
#define BITP_PCM_REGS_YODA_PCMCTRL_0_PCM_ENABLE_ALL           (6U)           /* No description provided */
#define BITL_PCM_REGS_YODA_PCMCTRL_0_PCM_ENABLE_ALL           (1U)           /* No description provided */
#define BITM_PCM_REGS_YODA_PCMCTRL_0_PCM_ENABLE_ALL           (0X0040U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PCMCTRL_1                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PCM_REGS_YODA_PCMCTRL_1_PCM_CLK_COUNT            (0U)           /* No description provided */
#define BITL_PCM_REGS_YODA_PCMCTRL_1_PCM_CLK_COUNT            (16U)          /* No description provided */
#define BITM_PCM_REGS_YODA_PCMCTRL_1_PCM_CLK_COUNT            (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PCMOUT                                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PCM_REGS_YODA_PCMOUT_PCM_OUT                     (0U)           /* No description provided */
#define BITL_PCM_REGS_YODA_PCMOUT_PCM_OUT                     (16U)          /* No description provided */
#define BITM_PCM_REGS_YODA_PCMOUT_PCM_OUT                     (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          OSC_PERIOD_CTRL                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PCM_REGS_YODA_OSC_PERIOD_CTRL_OSC_PERIOD_ENABLE  (0U)           /* No description provided */
#define BITL_PCM_REGS_YODA_OSC_PERIOD_CTRL_OSC_PERIOD_ENABLE  (1U)           /* No description provided */
#define BITM_PCM_REGS_YODA_OSC_PERIOD_CTRL_OSC_PERIOD_ENABLE  (0X0001U)      /* No description provided */
#define BITP_PCM_REGS_YODA_OSC_PERIOD_CTRL_RESERVED1          (1U)           /* No description provided */
#define BITL_PCM_REGS_YODA_OSC_PERIOD_CTRL_RESERVED1          (3U)           /* No description provided */
#define BITM_PCM_REGS_YODA_OSC_PERIOD_CTRL_RESERVED1          (0X000EU)      /* No description provided */
#define BITP_PCM_REGS_YODA_OSC_PERIOD_CTRL_OSC_PERIOD_DIVCLK_DIVIDE (4U)           /* No description provided */
#define BITL_PCM_REGS_YODA_OSC_PERIOD_CTRL_OSC_PERIOD_DIVCLK_DIVIDE (4U)           /* No description provided */
#define BITM_PCM_REGS_YODA_OSC_PERIOD_CTRL_OSC_PERIOD_DIVCLK_DIVIDE (0X00F0U)      /* No description provided */
#define BITP_PCM_REGS_YODA_OSC_PERIOD_CTRL_SMPL_CLK_SEL       (8U)           /* No description provided */
#define BITL_PCM_REGS_YODA_OSC_PERIOD_CTRL_SMPL_CLK_SEL       (3U)           /* No description provided */
#define BITM_PCM_REGS_YODA_OSC_PERIOD_CTRL_SMPL_CLK_SEL       (0X0700U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          OSC_PERIOD_RD                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_PCM_REGS_YODA_OSC_PERIOD_RD_SMPL_PERIOD          (0U)           /* No description provided */
#define BITL_PCM_REGS_YODA_OSC_PERIOD_RD_SMPL_PERIOD          (16U)          /* No description provided */
#define BITM_PCM_REGS_YODA_OSC_PERIOD_RD_SMPL_PERIOD          (0XFFFFU)      /* No description provided */

#endif  /* end ifndef PCM_REGS_YODA_ADDR_RDEF_H_ */

/* ====================================================================================================
        DATAPATH Module Instances Address and Mask Definitions
   ==================================================================================================== */
#define INST_DATAPATH_REGS                                       (0X00000D00U)    /* datapath_regs: */


#ifndef DATAPATH_ADDR_RDEF_H_
#define DATAPATH_ADDR_RDEF_H_    /* DATAPATH: Your module description, here. */

#define MASK_DATAPATH                                            (0X000000FFU)    /* DATAPATH: Your module description, here. */

/* ====================================================================================================
        DATAPATH Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_DATAPATH_CORRECTION_CONFIG                          (0X10) 
#define RSTVAL_DATAPATH_USE_CASE_FRAME_CONFIG                      (0X10) 
#define RSTVAL_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL               (0X384) 
#define RSTVAL_DATAPATH_GAIN0                                      (0X0) 
#define RSTVAL_DATAPATH_GAIN1                                      (0X0) 
#define RSTVAL_DATAPATH_GAIN2                                      (0X0) 
#define RSTVAL_DATAPATH_GAIN3                                      (0X0) 
#define RSTVAL_DATAPATH_PARITY_GAIN_MEM                            (0X0) 
#define RSTVAL_DATAPATH_PARITY_LINE_MEM                            (0X0) 
#define RSTVAL_DATAPATH_PP_LFSR                                    (0X0) 
#define RSTVAL_DATAPATH_PP_DECODE_ST_1                             (0X4688) 
#define RSTVAL_DATAPATH_PP_DECODE_ST_2                             (0X4) 
#define RSTVAL_DATAPATH_PP_ENCODE_ST                               (0X26) 
#define RSTVAL_DATAPATH_PP_ENCODE_GT                               (0XE4) 
#define RSTVAL_DATAPATH_DBG_MUX                                    (0X0) 
#define RSTVAL_DATAPATH_GAIN_MARGIN_CONTROL                        (0X0) 
#define RSTVAL_DATAPATH_LINE_MARGIN_CONTROL                        (0X0) 
#define RSTVAL_DATAPATH_ROI_ROW_START                              (0X0) 
#define RSTVAL_DATAPATH_ROI_HEIGHT                                 (0X280) 
#define RSTVAL_DATAPATH_ROI_COLUMN_START                           (0X0) 
#define RSTVAL_DATAPATH_ROI_WIDTH                                  (0X200) 
#define RSTVAL_DATAPATH_PP_USEQ_WRITE                              (0X0) 
#define RSTVAL_DATAPATH_PP_ADC_DELAY                               (0X6) 
#define RSTVAL_DATAPATH_MIPI_BUFF_MARGIN_CONTROL                   (0X0) 
#define RSTVAL_DATAPATH_MIPI_HEADER_WIDTH                          (0X80) 
#define RSTVAL_DATAPATH_FRAME_NUMBER                               (0X0) 
#define RSTVAL_DATAPATH_MICRO_SEQUENCER_FW_VERSION_LSB             (0X0) 
#define RSTVAL_DATAPATH_MICRO_SEQUENCER_FW_VERSION_MSB             (0X0) 
#define RSTVAL_DATAPATH_TS_CAL_VER                                 (0X0) 
#define RSTVAL_DATAPATH_ADC_CAL_VER                                (0X0) 
#define RSTVAL_DATAPATH_REG_0                                      (0X0) 
#define RSTVAL_DATAPATH_REG_1                                      (0X0) 
#define RSTVAL_DATAPATH_REG_2                                      (0X0) 
#define RSTVAL_DATAPATH_REG_3                                      (0X0) 
#define RSTVAL_DATAPATH_REG_4                                      (0X0) 
#define RSTVAL_DATAPATH_REG_5                                      (0X0) 
#define RSTVAL_DATAPATH_REG_6                                      (0X0) 
#define RSTVAL_DATAPATH_REG_7                                      (0X0) 
#define RSTVAL_DATAPATH_PARITY_MIPI_BUFFER                         (0X0) 
#define RSTVAL_DATAPATH_PACKET_COUNT                               (0X0) 
#define RSTVAL_DATAPATH_PACKETS_PER_FRAME                          (0X0) 
#define RSTVAL_DATAPATH_ROW_VECTOR                                 (0X0) 
#define RSTVAL_DATAPATH_ROWS_PER_PACKET_OUT                        (0X0) 
#define RSTVAL_DATAPATH_MIPI_RD_EN_MAX                             (0X0) 
#define RSTVAL_DATAPATH_ANALOG_SS                                  (0X0) 
#define RSTVAL_DATAPATH_MIPI_BUFF_PARITY_ERR_CNT                   (0X0) 
#define RSTVAL_DATAPATH_LINE_MEM_PARITY_ERR_CNT                    (0X0) 
#define RSTVAL_DATAPATH_GAIN_MEM_PARITY_ERR_CNT                    (0X0) 
#define RSTVAL_DATAPATH_IA_SELECT                                  (0X0) 
#define RSTVAL_DATAPATH_IA_ADDR_REG                                (0X0) 
#define RSTVAL_DATAPATH_IA_WRDATA_REG                              (0X0) 
#define RSTVAL_DATAPATH_IA_WRDATA_REG_ALIAS                        (0X0) 
#define RSTVAL_DATAPATH_IA_RDDATA_REG                              (0X0) 
#define RSTVAL_DATAPATH_IA_RDDATA_REG_ALIAS                        (0X0) 
#define RSTVAL_DATAPATH_IA_BANK_TYPE                               (0X1) 

/* ====================================================================================================
        DATAPATH Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          CORRECTION_CONFIG                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_CORRECTION_CONFIG_BYPASS_PC_GAIN        (0U)           /* Bypass per-column gain correction */
#define BITL_DATAPATH_CORRECTION_CONFIG_BYPASS_PC_GAIN        (1U)           /* Bypass per-column gain correction */
#define BITM_DATAPATH_CORRECTION_CONFIG_BYPASS_PC_GAIN        (0X0001U)      /* Bypass per-column gain correction */
#define BITP_DATAPATH_CORRECTION_CONFIG_BYPASS_PC_OFFSET      (1U)           /* Bypass per-column offset correction */
#define BITL_DATAPATH_CORRECTION_CONFIG_BYPASS_PC_OFFSET      (1U)           /* Bypass per-column offset correction */
#define BITM_DATAPATH_CORRECTION_CONFIG_BYPASS_PC_OFFSET      (0X0002U)      /* Bypass per-column offset correction */
#define BITP_DATAPATH_CORRECTION_CONFIG_BYPASS_SCALE          (2U)           /* Bypass gain scaling */
#define BITL_DATAPATH_CORRECTION_CONFIG_BYPASS_SCALE          (1U)           /* Bypass gain scaling */
#define BITM_DATAPATH_CORRECTION_CONFIG_BYPASS_SCALE          (0X0004U)      /* Bypass gain scaling */
#define BITP_DATAPATH_CORRECTION_CONFIG_BYPASS_SATTAG         (3U)           /* Bypass saturation tag while pixel adjustment */
#define BITL_DATAPATH_CORRECTION_CONFIG_BYPASS_SATTAG         (1U)           /* Bypass saturation tag while pixel adjustment */
#define BITM_DATAPATH_CORRECTION_CONFIG_BYPASS_SATTAG         (0X0008U)      /* Bypass saturation tag while pixel adjustment */
#define BITP_DATAPATH_CORRECTION_CONFIG_BINNING_AVG_EN        (4U)           /* 1'b1 = average two rows while digital binning; 1'b0 = add two rows while digital binning */
#define BITL_DATAPATH_CORRECTION_CONFIG_BINNING_AVG_EN        (1U)           /* 1'b1 = average two rows while digital binning; 1'b0 = add two rows while digital binning */
#define BITM_DATAPATH_CORRECTION_CONFIG_BINNING_AVG_EN        (0X0010U)      /* 1'b1 = average two rows while digital binning; 1'b0 = add two rows while digital binning */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_FRAME_CONFIG                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_USE_CASE_FRAME_CONFIG_ADC_9B            (0U)           /* When 1, ADC valid data width is 9 bits */
#define BITL_DATAPATH_USE_CASE_FRAME_CONFIG_ADC_9B            (1U)           /* When 1, ADC valid data width is 9 bits */
#define BITM_DATAPATH_USE_CASE_FRAME_CONFIG_ADC_9B            (0X0001U)      /* When 1, ADC valid data width is 9 bits */
#define BITP_DATAPATH_USE_CASE_FRAME_CONFIG_ALTERNATE_AMP_MUX_POL (1U)           /* 1'b1 = Inverted gain scaling for even ADCs (needed for supply rejection work-around); 1'b0 = No inversion */
#define BITL_DATAPATH_USE_CASE_FRAME_CONFIG_ALTERNATE_AMP_MUX_POL (1U)           /* 1'b1 = Inverted gain scaling for even ADCs (needed for supply rejection work-around); 1'b0 = No inversion */
#define BITM_DATAPATH_USE_CASE_FRAME_CONFIG_ALTERNATE_AMP_MUX_POL (0X0002U)      /* 1'b1 = Inverted gain scaling for even ADCs (needed for supply rejection work-around); 1'b0 = No inversion */
#define BITP_DATAPATH_USE_CASE_FRAME_CONFIG_DIGITAL_BIN_EN    (2U)           /* Enable digital binning between adjacent rows, invalid to set if delta_comp_en is set */
#define BITL_DATAPATH_USE_CASE_FRAME_CONFIG_DIGITAL_BIN_EN    (1U)           /* Enable digital binning between adjacent rows, invalid to set if delta_comp_en is set */
#define BITM_DATAPATH_USE_CASE_FRAME_CONFIG_DIGITAL_BIN_EN    (0X0004U)      /* Enable digital binning between adjacent rows, invalid to set if delta_comp_en is set */
#define BITP_DATAPATH_USE_CASE_FRAME_CONFIG_DELTA_COMP_EN     (3U)           /* Enable delta compression */
#define BITL_DATAPATH_USE_CASE_FRAME_CONFIG_DELTA_COMP_EN     (1U)           /* Enable delta compression */
#define BITM_DATAPATH_USE_CASE_FRAME_CONFIG_DELTA_COMP_EN     (0X0008U)      /* Enable delta compression */
#define BITP_DATAPATH_USE_CASE_FRAME_CONFIG_FIX2FLT_EN        (4U)           /* enable fixed to floating point conversion */
#define BITL_DATAPATH_USE_CASE_FRAME_CONFIG_FIX2FLT_EN        (1U)           /* enable fixed to floating point conversion */
#define BITM_DATAPATH_USE_CASE_FRAME_CONFIG_FIX2FLT_EN        (0X0010U)      /* enable fixed to floating point conversion */
#define BITP_DATAPATH_USE_CASE_FRAME_CONFIG_RAW_MODE          (5U)           /* Enable RAW mode and select how the ADC clipping is handled */
#define BITL_DATAPATH_USE_CASE_FRAME_CONFIG_RAW_MODE          (2U)           /* Enable RAW mode and select how the ADC clipping is handled */
#define BITM_DATAPATH_USE_CASE_FRAME_CONFIG_RAW_MODE          (0X0060U)      /* Enable RAW mode and select how the ADC clipping is handled */
#define BITP_DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH      (7U)           /* Bit width of final output to MIPI */
#define BITL_DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH      (3U)           /* Bit width of final output to MIPI */
#define BITM_DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH      (0X0380U)      /* Bit width of final output to MIPI */
#define BITP_DATAPATH_USE_CASE_FRAME_CONFIG_DARK_ROW_VEC      (10U)          /* Each bit enables the readout of  8 dark rows on the top and the bottom of the array */
#define BITL_DATAPATH_USE_CASE_FRAME_CONFIG_DARK_ROW_VEC      (2U)           /* Each bit enables the readout of  8 dark rows on the top and the bottom of the array */
#define BITM_DATAPATH_USE_CASE_FRAME_CONFIG_DARK_ROW_VEC      (0X0C00U)      /* Each bit enables the readout of  8 dark rows on the top and the bottom of the array */
#define BITP_DATAPATH_USE_CASE_FRAME_CONFIG_MIPI_OUT_8BIT     (12U)          /* 1'b1 = Send RAW14 or RAW16 as two 8 bit RAW pixels to MIPI; when 1'b0 = Send the pixel data as 14 bit or 16 bit value */
#define BITL_DATAPATH_USE_CASE_FRAME_CONFIG_MIPI_OUT_8BIT     (1U)           /* 1'b1 = Send RAW14 or RAW16 as two 8 bit RAW pixels to MIPI; when 1'b0 = Send the pixel data as 14 bit or 16 bit value */
#define BITM_DATAPATH_USE_CASE_FRAME_CONFIG_MIPI_OUT_8BIT     (0X1000U)      /* 1'b1 = Send RAW14 or RAW16 as two 8 bit RAW pixels to MIPI; when 1'b0 = Send the pixel data as 14 bit or 16 bit value */

#define ENUM_DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH_RAW8 (0X0000U)      /* 8 bit Floating point: {sign,exp[2:0],mantissa[3:0]} or raw data */
#define ENUM_DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH_RAW10 (0X0001U)      /* 10 bit Floating point: {sign,exp[2:0],mantissa[5:0]} or raw data */
#define ENUM_DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH_RAW12 (0X0002U)      /* 12 bit Floating point: {sign,exp[2:0],mantissa[7:0]} or raw data */
#define ENUM_DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH_RAW16 (0X0003U)      /* 16 bit binary spread over two 8 bit mipi packets */
#define ENUM_DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH_RAW14 (0X0004U)      /* 14 bit Binary, spread over two 8 bit MIPI packets */
#define ENUM_DATAPATH_USE_CASE_FRAME_CONFIG_RAW_MODE_DISABLE  (0X0000U)      /* Raw mode disabled */
#define ENUM_DATAPATH_USE_CASE_FRAME_CONFIG_RAW_MODE_NO_CLIP  (0X0001U)      /* RAW mode ADC data direct */
#define ENUM_DATAPATH_USE_CASE_FRAME_CONFIG_RAW_MODE_CLIP_ADC_OVR (0X0002U)      /* RAW mode ADC data clipped when overflow flag high */
#define ENUM_DATAPATH_USE_CASE_FRAME_CONFIG_RAW_MODE_CLIP_ALL (0X0003U)      /* RAW mode ADC data clipped when overflow flag or sat_tag high */
#define ENUM_DATAPATH_USE_CASE_FRAME_CONFIG_ALTERNATE_AMP_MUX_POL_NO_INVERT (0X0000U)      /* No description provided */
#define ENUM_DATAPATH_USE_CASE_FRAME_CONFIG_ALTERNATE_AMP_MUX_POL_INVERT_EVEN (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_MIPI_PACKET_CONTROL                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_ROWS_PER_PACKET (0U)           /* Number of rows to be packed in a MIPI  packet. Valid values are 1,2,4,8,16,32,64 */
#define BITL_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_ROWS_PER_PACKET (7U)           /* Number of rows to be packed in a MIPI  packet. Valid values are 1,2,4,8,16,32,64 */
#define BITM_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_ROWS_PER_PACKET (0X007FU)      /* Number of rows to be packed in a MIPI  packet. Valid values are 1,2,4,8,16,32,64 */
#define BITP_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_AUTO_ROWS_PER_PACKET_EN (7U)           /* When 1, datapath calculates the rows to be packed in a MIPI packet; when 0 use the value programmed in rows per packet register */
#define BITL_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_AUTO_ROWS_PER_PACKET_EN (1U)           /* When 1, datapath calculates the rows to be packed in a MIPI packet; when 0 use the value programmed in rows per packet register */
#define BITM_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_AUTO_ROWS_PER_PACKET_EN (0X0080U)      /* When 1, datapath calculates the rows to be packed in a MIPI packet; when 0 use the value programmed in rows per packet register */
#define BITP_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_MIPI_BUFF_RD_LIMIT (8U)           /* Number of buffers to be filled before sending packet request */
#define BITL_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_MIPI_BUFF_RD_LIMIT (2U)           /* Number of buffers to be filled before sending packet request */
#define BITM_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_MIPI_BUFF_RD_LIMIT (0X0300U)      /* Number of buffers to be filled before sending packet request */
#define BITP_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_MIPI_BUFF_RD_CTRL_EN (10U)          /* When 1, reading of MIPI buffers is based on mipi_buff_rd_limit, else done in datapath */
#define BITL_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_MIPI_BUFF_RD_CTRL_EN (1U)           /* When 1, reading of MIPI buffers is based on mipi_buff_rd_limit, else done in datapath */
#define BITM_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_MIPI_BUFF_RD_CTRL_EN (0X0400U)      /* When 1, reading of MIPI buffers is based on mipi_buff_rd_limit, else done in datapath */

/* ----------------------------------------------------------------------------------------------------
          GAIN0                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_GAIN0_GLOBAL_GAIN_SCALE_P0              (0U)           /* Gain scale for GT0 */
#define BITL_DATAPATH_GAIN0_GLOBAL_GAIN_SCALE_P0              (10U)          /* Gain scale for GT0 */
#define BITM_DATAPATH_GAIN0_GLOBAL_GAIN_SCALE_P0              (0X03FFU)      /* Gain scale for GT0 */
#define BITP_DATAPATH_GAIN0_GLOBAL_GAIN_SHIFT_P0              (10U)          /* Gain shift for GT0 */
#define BITL_DATAPATH_GAIN0_GLOBAL_GAIN_SHIFT_P0              (3U)           /* Gain shift for GT0 */
#define BITM_DATAPATH_GAIN0_GLOBAL_GAIN_SHIFT_P0              (0X1C00U)      /* Gain shift for GT0 */

/* ----------------------------------------------------------------------------------------------------
          GAIN1                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_GAIN1_GLOBAL_GAIN_SCALE_P1              (0U)           /* Gain scale for GT1 */
#define BITL_DATAPATH_GAIN1_GLOBAL_GAIN_SCALE_P1              (10U)          /* Gain scale for GT1 */
#define BITM_DATAPATH_GAIN1_GLOBAL_GAIN_SCALE_P1              (0X03FFU)      /* Gain scale for GT1 */
#define BITP_DATAPATH_GAIN1_GLOBAL_GAIN_SHIFT_P1              (10U)          /* Gain shift for GT1 */
#define BITL_DATAPATH_GAIN1_GLOBAL_GAIN_SHIFT_P1              (3U)           /* Gain shift for GT1 */
#define BITM_DATAPATH_GAIN1_GLOBAL_GAIN_SHIFT_P1              (0X1C00U)      /* Gain shift for GT1 */

/* ----------------------------------------------------------------------------------------------------
          GAIN2                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_GAIN2_GLOBAL_GAIN_SCALE_P2              (0U)           /* Gain scale for GT2 */
#define BITL_DATAPATH_GAIN2_GLOBAL_GAIN_SCALE_P2              (10U)          /* Gain scale for GT2 */
#define BITM_DATAPATH_GAIN2_GLOBAL_GAIN_SCALE_P2              (0X03FFU)      /* Gain scale for GT2 */
#define BITP_DATAPATH_GAIN2_GLOBAL_GAIN_SHIFT_P2              (10U)          /* Gain shift for GT2 */
#define BITL_DATAPATH_GAIN2_GLOBAL_GAIN_SHIFT_P2              (3U)           /* Gain shift for GT2 */
#define BITM_DATAPATH_GAIN2_GLOBAL_GAIN_SHIFT_P2              (0X1C00U)      /* Gain shift for GT2 */

/* ----------------------------------------------------------------------------------------------------
          GAIN3                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_GAIN3_GLOBAL_GAIN_SCALE_P3              (0U)           /* Gain scale for GT3 */
#define BITL_DATAPATH_GAIN3_GLOBAL_GAIN_SCALE_P3              (10U)          /* Gain scale for GT3 */
#define BITM_DATAPATH_GAIN3_GLOBAL_GAIN_SCALE_P3              (0X03FFU)      /* Gain scale for GT3 */
#define BITP_DATAPATH_GAIN3_GLOBAL_GAIN_SHIFT_P3              (10U)          /* Gain shift for GT3 */
#define BITL_DATAPATH_GAIN3_GLOBAL_GAIN_SHIFT_P3              (3U)           /* Gain shift for GT3 */
#define BITM_DATAPATH_GAIN3_GLOBAL_GAIN_SHIFT_P3              (0X1C00U)      /* Gain shift for GT3 */

/* ----------------------------------------------------------------------------------------------------
          PARITY_GAIN_MEM                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_PARITY_GAIN_MEM_GAIN_MEM_PARITY_ERR     (0U)           /* Parity error bits from gain correction memories. One bit per bank */
#define BITL_DATAPATH_PARITY_GAIN_MEM_GAIN_MEM_PARITY_ERR     (16U)          /* Parity error bits from gain correction memories. One bit per bank */
#define BITM_DATAPATH_PARITY_GAIN_MEM_GAIN_MEM_PARITY_ERR     (0XFFFFU)      /* Parity error bits from gain correction memories. One bit per bank */

/* ----------------------------------------------------------------------------------------------------
          PARITY_LINE_MEM                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_PARITY_LINE_MEM_LINE_MEM_PARITY_ERR     (0U)           /* Parity error bits from line buffer memories. One bit per bank */
#define BITL_DATAPATH_PARITY_LINE_MEM_LINE_MEM_PARITY_ERR     (2U)           /* Parity error bits from line buffer memories. One bit per bank */
#define BITM_DATAPATH_PARITY_LINE_MEM_LINE_MEM_PARITY_ERR     (0X0003U)      /* Parity error bits from line buffer memories. One bit per bank */

/* ----------------------------------------------------------------------------------------------------
          PP_LFSR                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_PP_LFSR_LFSR_EN                         (0U)           /* Indicates whether the Pixel Packer must work in LFSR mode */
#define BITL_DATAPATH_PP_LFSR_LFSR_EN                         (1U)           /* Indicates whether the Pixel Packer must work in LFSR mode */
#define BITM_DATAPATH_PP_LFSR_LFSR_EN                         (0X0001U)      /* Indicates whether the Pixel Packer must work in LFSR mode */
#define BITP_DATAPATH_PP_LFSR_LFSR_MODE                       (1U)           /* Indicates the LFSR data generation mode */
#define BITL_DATAPATH_PP_LFSR_LFSR_MODE                       (2U)           /* Indicates the LFSR data generation mode */
#define BITM_DATAPATH_PP_LFSR_LFSR_MODE                       (0X0006U)      /* Indicates the LFSR data generation mode */
#define BITP_DATAPATH_PP_LFSR_LFSR_SEED                       (3U)           /* The initial seed value for the data generation in LFSR mode */
#define BITL_DATAPATH_PP_LFSR_LFSR_SEED                       (12U)          /* The initial seed value for the data generation in LFSR mode */
#define BITM_DATAPATH_PP_LFSR_LFSR_SEED                       (0X7FF8U)      /* The initial seed value for the data generation in LFSR mode */

/* ----------------------------------------------------------------------------------------------------
          PP_DECODE_ST_1                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_PP_DECODE_ST_1_ST_DECODE_000            (0U)           /* Mux input for the ST selection */
#define BITL_DATAPATH_PP_DECODE_ST_1_ST_DECODE_000            (3U)           /* Mux input for the ST selection */
#define BITM_DATAPATH_PP_DECODE_ST_1_ST_DECODE_000            (0X0007U)      /* Mux input for the ST selection */
#define BITP_DATAPATH_PP_DECODE_ST_1_ST_DECODE_001            (3U)           /* Mux input for the ST selection */
#define BITL_DATAPATH_PP_DECODE_ST_1_ST_DECODE_001            (3U)           /* Mux input for the ST selection */
#define BITM_DATAPATH_PP_DECODE_ST_1_ST_DECODE_001            (0X0038U)      /* Mux input for the ST selection */
#define BITP_DATAPATH_PP_DECODE_ST_1_ST_DECODE_010            (6U)           /* Mux input for the ST selection */
#define BITL_DATAPATH_PP_DECODE_ST_1_ST_DECODE_010            (3U)           /* Mux input for the ST selection */
#define BITM_DATAPATH_PP_DECODE_ST_1_ST_DECODE_010            (0X01C0U)      /* Mux input for the ST selection */
#define BITP_DATAPATH_PP_DECODE_ST_1_ST_DECODE_011            (9U)           /* Mux input for the ST selection */
#define BITL_DATAPATH_PP_DECODE_ST_1_ST_DECODE_011            (3U)           /* Mux input for the ST selection */
#define BITM_DATAPATH_PP_DECODE_ST_1_ST_DECODE_011            (0X0E00U)      /* Mux input for the ST selection */
#define BITP_DATAPATH_PP_DECODE_ST_1_ST_DECODE_100            (12U)          /* Mux input for the ST selection */
#define BITL_DATAPATH_PP_DECODE_ST_1_ST_DECODE_100            (3U)           /* Mux input for the ST selection */
#define BITM_DATAPATH_PP_DECODE_ST_1_ST_DECODE_100            (0X7000U)      /* Mux input for the ST selection */

/* ----------------------------------------------------------------------------------------------------
          PP_DECODE_ST_2                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_PP_DECODE_ST_2_ST_DECODE_101            (0U)           /* Mux input for the ST selection */
#define BITL_DATAPATH_PP_DECODE_ST_2_ST_DECODE_101            (3U)           /* Mux input for the ST selection */
#define BITM_DATAPATH_PP_DECODE_ST_2_ST_DECODE_101            (0X0007U)      /* Mux input for the ST selection */
#define BITP_DATAPATH_PP_DECODE_ST_2_ST_DECODE_110            (3U)           /* Mux input for the ST selection */
#define BITL_DATAPATH_PP_DECODE_ST_2_ST_DECODE_110            (3U)           /* Mux input for the ST selection */
#define BITM_DATAPATH_PP_DECODE_ST_2_ST_DECODE_110            (0X0038U)      /* Mux input for the ST selection */
#define BITP_DATAPATH_PP_DECODE_ST_2_ST_DECODE_111            (6U)           /* Mux input for the ST selection */
#define BITL_DATAPATH_PP_DECODE_ST_2_ST_DECODE_111            (3U)           /* Mux input for the ST selection */
#define BITM_DATAPATH_PP_DECODE_ST_2_ST_DECODE_111            (0X01C0U)      /* Mux input for the ST selection */

/* ----------------------------------------------------------------------------------------------------
          PP_ENCODE_ST                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_PP_ENCODE_ST_ST_EN_0                    (0U)           /* Remap value for ST value of 0 from analog */
#define BITL_DATAPATH_PP_ENCODE_ST_ST_EN_0                    (1U)           /* Remap value for ST value of 0 from analog */
#define BITM_DATAPATH_PP_ENCODE_ST_ST_EN_0                    (0X0001U)      /* Remap value for ST value of 0 from analog */
#define BITP_DATAPATH_PP_ENCODE_ST_ST_EN_1                    (1U)           /* Remap value for ST value of 1 from analog */
#define BITL_DATAPATH_PP_ENCODE_ST_ST_EN_1                    (1U)           /* Remap value for ST value of 1 from analog */
#define BITM_DATAPATH_PP_ENCODE_ST_ST_EN_1                    (0X0002U)      /* Remap value for ST value of 1 from analog */
#define BITP_DATAPATH_PP_ENCODE_ST_OVERFLOW_ACTIVE            (2U)           /* When 0, inactive and ignored for adc mux */
#define BITL_DATAPATH_PP_ENCODE_ST_OVERFLOW_ACTIVE            (1U)           /* When 0, inactive and ignored for adc mux */
#define BITM_DATAPATH_PP_ENCODE_ST_OVERFLOW_ACTIVE            (0X0004U)      /* When 0, inactive and ignored for adc mux */
#define BITP_DATAPATH_PP_ENCODE_ST_BINNED_AND_OR              (4U)           /* Indicates whether the ST tags are to be ORed together */
#define BITL_DATAPATH_PP_ENCODE_ST_BINNED_AND_OR              (1U)           /* Indicates whether the ST tags are to be ORed together */
#define BITM_DATAPATH_PP_ENCODE_ST_BINNED_AND_OR              (0X0010U)      /* Indicates whether the ST tags are to be ORed together */
#define BITP_DATAPATH_PP_ENCODE_ST_ST_LATCH_ACTIVE_HI         (5U)           /* Indicates whether the st latch is active HIGH */
#define BITL_DATAPATH_PP_ENCODE_ST_ST_LATCH_ACTIVE_HI         (1U)           /* Indicates whether the st latch is active HIGH */
#define BITM_DATAPATH_PP_ENCODE_ST_ST_LATCH_ACTIVE_HI         (0X0020U)      /* Indicates whether the st latch is active HIGH */

/* ----------------------------------------------------------------------------------------------------
          PP_ENCODE_GT                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_PP_ENCODE_GT_GT_EN_00                   (0U)           /* Remap value for GT value of 00 from analog */
#define BITL_DATAPATH_PP_ENCODE_GT_GT_EN_00                   (2U)           /* Remap value for GT value of 00 from analog */
#define BITM_DATAPATH_PP_ENCODE_GT_GT_EN_00                   (0X0003U)      /* Remap value for GT value of 00 from analog */
#define BITP_DATAPATH_PP_ENCODE_GT_GT_EN_01                   (2U)           /* Remap value for GT value of 01 from analog */
#define BITL_DATAPATH_PP_ENCODE_GT_GT_EN_01                   (2U)           /* Remap value for GT value of 01 from analog */
#define BITM_DATAPATH_PP_ENCODE_GT_GT_EN_01                   (0X000CU)      /* Remap value for GT value of 01 from analog */
#define BITP_DATAPATH_PP_ENCODE_GT_GT_EN_10                   (4U)           /* Remap value for GT value of 10 from analog */
#define BITL_DATAPATH_PP_ENCODE_GT_GT_EN_10                   (2U)           /* Remap value for GT value of 10 from analog */
#define BITM_DATAPATH_PP_ENCODE_GT_GT_EN_10                   (0X0030U)      /* Remap value for GT value of 10 from analog */
#define BITP_DATAPATH_PP_ENCODE_GT_GT_EN_11                   (6U)           /* Remap value for GT value of 11 from analog */
#define BITL_DATAPATH_PP_ENCODE_GT_GT_EN_11                   (2U)           /* Remap value for GT value of 11 from analog */
#define BITM_DATAPATH_PP_ENCODE_GT_GT_EN_11                   (0X00C0U)      /* Remap value for GT value of 11 from analog */
#define BITP_DATAPATH_PP_ENCODE_GT_GT_LATCH_ACTIVE_HI         (8U)           /* Indicates whether the gt latch is active HIGH */
#define BITL_DATAPATH_PP_ENCODE_GT_GT_LATCH_ACTIVE_HI         (1U)           /* Indicates whether the gt latch is active HIGH */
#define BITM_DATAPATH_PP_ENCODE_GT_GT_LATCH_ACTIVE_HI         (0X0100U)      /* Indicates whether the gt latch is active HIGH */
#define BITP_DATAPATH_PP_ENCODE_GT_TAG_WAIT                   (9U)           /* Wait time between value change on gt_sel_o signal in number of clock cycles (2-20 clks) */
#define BITL_DATAPATH_PP_ENCODE_GT_TAG_WAIT                   (2U)           /* Wait time between value change on gt_sel_o signal in number of clock cycles (2-20 clks) */
#define BITM_DATAPATH_PP_ENCODE_GT_TAG_WAIT                   (0X0600U)      /* Wait time between value change on gt_sel_o signal in number of clock cycles (2-20 clks) */
#define BITP_DATAPATH_PP_ENCODE_GT_TAG_SCALE                  (11U)          /* Scaling factor for tag wait */
#define BITL_DATAPATH_PP_ENCODE_GT_TAG_SCALE                  (2U)           /* Scaling factor for tag wait */
#define BITM_DATAPATH_PP_ENCODE_GT_TAG_SCALE                  (0X1800U)      /* Scaling factor for tag wait */

/* ----------------------------------------------------------------------------------------------------
          DBG_MUX                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_DBG_MUX_DBG_SEL                         (0U)           /* Internal signal debug mux select signal. Refer to Datapath HRM for mux description */
#define BITL_DATAPATH_DBG_MUX_DBG_SEL                         (5U)           /* Internal signal debug mux select signal. Refer to Datapath HRM for mux description */
#define BITM_DATAPATH_DBG_MUX_DBG_SEL                         (0X001FU)      /* Internal signal debug mux select signal. Refer to Datapath HRM for mux description */
#define BITP_DATAPATH_DBG_MUX_DBG_EN                          (15U)          /* Enable for the Debug mux */
#define BITL_DATAPATH_DBG_MUX_DBG_EN                          (1U)           /* Enable for the Debug mux */
#define BITM_DATAPATH_DBG_MUX_DBG_EN                          (0X8000U)      /* Enable for the Debug mux */

/* ----------------------------------------------------------------------------------------------------
          GAIN_MARGIN_CONTROL                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_GAIN_MARGIN_CONTROL_GAIN_MEM_MARGIN     (0U)           /* Read and write margin control. One bit for each gain memory. */
#define BITL_DATAPATH_GAIN_MARGIN_CONTROL_GAIN_MEM_MARGIN     (16U)          /* Read and write margin control. One bit for each gain memory. */
#define BITM_DATAPATH_GAIN_MARGIN_CONTROL_GAIN_MEM_MARGIN     (0XFFFFU)      /* Read and write margin control. One bit for each gain memory. */

/* ----------------------------------------------------------------------------------------------------
          LINE_MARGIN_CONTROL                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_LINE_MARGIN_CONTROL_LINE_MEM_MARGIN     (0U)           /* Read and write margin control. One bit for each line memory. */
#define BITL_DATAPATH_LINE_MARGIN_CONTROL_LINE_MEM_MARGIN     (2U)           /* Read and write margin control. One bit for each line memory. */
#define BITM_DATAPATH_LINE_MARGIN_CONTROL_LINE_MEM_MARGIN     (0X0003U)      /* Read and write margin control. One bit for each line memory. */

/* ----------------------------------------------------------------------------------------------------
          ROI_ROW_START                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_ROI_ROW_START_START_ROW                 (0U)           /* Start row of ROI. Should be a multiple of 64. Range (0-639) */
#define BITL_DATAPATH_ROI_ROW_START_START_ROW                 (10U)          /* Start row of ROI. Should be a multiple of 64. Range (0-639) */
#define BITM_DATAPATH_ROI_ROW_START_START_ROW                 (0X03FFU)      /* Start row of ROI. Should be a multiple of 64. Range (0-639) */

/* ----------------------------------------------------------------------------------------------------
          ROI_HEIGHT                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_ROI_HEIGHT_ROI_HEIGHT                   (0U)           /* Numbers of rows enabled in ROI */
#define BITL_DATAPATH_ROI_HEIGHT_ROI_HEIGHT                   (10U)          /* Numbers of rows enabled in ROI */
#define BITM_DATAPATH_ROI_HEIGHT_ROI_HEIGHT                   (0X03FFU)      /* Numbers of rows enabled in ROI */

/* ----------------------------------------------------------------------------------------------------
          ROI_COLUMN_START                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_ROI_COLUMN_START_START_COLUMN           (0U)           /* Start column of ROI. Should be a multiple of 16 */
#define BITL_DATAPATH_ROI_COLUMN_START_START_COLUMN           (9U)           /* Start column of ROI. Should be a multiple of 16 */
#define BITM_DATAPATH_ROI_COLUMN_START_START_COLUMN           (0X01FFU)      /* Start column of ROI. Should be a multiple of 16 */

/* ----------------------------------------------------------------------------------------------------
          ROI_WIDTH                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_ROI_WIDTH_ROI_WIDTH                     (0U)           /* Number of columns enabled in ROI */
#define BITL_DATAPATH_ROI_WIDTH_ROI_WIDTH                     (10U)          /* Number of columns enabled in ROI */
#define BITM_DATAPATH_ROI_WIDTH_ROI_WIDTH                     (0X03FFU)      /* Number of columns enabled in ROI */

/* ----------------------------------------------------------------------------------------------------
          PP_USEQ_WRITE                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_PP_USEQ_WRITE_DUMP_START                (0U)           /* Bit to be written by micro-sequencer */
#define BITL_DATAPATH_PP_USEQ_WRITE_DUMP_START                (1U)           /* Bit to be written by micro-sequencer */
#define BITM_DATAPATH_PP_USEQ_WRITE_DUMP_START                (0X0001U)      /* Bit to be written by micro-sequencer */

/* ----------------------------------------------------------------------------------------------------
          PP_ADC_DELAY                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_PP_ADC_DELAY_ADC_DELAY                  (0U)           /* Specifies the number of clock cycles after which the data from adc_bus can read and used on adc_convert positive transition */
#define BITL_DATAPATH_PP_ADC_DELAY_ADC_DELAY                  (6U)           /* Specifies the number of clock cycles after which the data from adc_bus can read and used on adc_convert positive transition */
#define BITM_DATAPATH_PP_ADC_DELAY_ADC_DELAY                  (0X003FU)      /* Specifies the number of clock cycles after which the data from adc_bus can read and used on adc_convert positive transition */

/* ----------------------------------------------------------------------------------------------------
          MIPI_BUFF_MARGIN_CONTROL                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_MIPI_BUFF_MARGIN_CONTROL_MIPI_BUFF_MARGIN (0U)           /* Read and write margin control. One bit for each MIPI buffer */
#define BITL_DATAPATH_MIPI_BUFF_MARGIN_CONTROL_MIPI_BUFF_MARGIN (4U)           /* Read and write margin control. One bit for each MIPI buffer */
#define BITM_DATAPATH_MIPI_BUFF_MARGIN_CONTROL_MIPI_BUFF_MARGIN (0X000FU)      /* Read and write margin control. One bit for each MIPI buffer */

/* ----------------------------------------------------------------------------------------------------
          MIPI_HEADER_WIDTH                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_MIPI_HEADER_WIDTH_METADATA_BYTES        (0U)           /* Number of bytes sent as Metadata */
#define BITL_DATAPATH_MIPI_HEADER_WIDTH_METADATA_BYTES        (8U)           /* Number of bytes sent as Metadata */
#define BITM_DATAPATH_MIPI_HEADER_WIDTH_METADATA_BYTES        (0X00FFU)      /* Number of bytes sent as Metadata */

/* ----------------------------------------------------------------------------------------------------
          FRAME_NUMBER                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_FRAME_NUMBER_FRAME_COUNT                (0U)           /* Frame counter to be incremented by micro-sequencer */
#define BITL_DATAPATH_FRAME_NUMBER_FRAME_COUNT                (16U)          /* Frame counter to be incremented by micro-sequencer */
#define BITM_DATAPATH_FRAME_NUMBER_FRAME_COUNT                (0XFFFFU)      /* Frame counter to be incremented by micro-sequencer */

/* ----------------------------------------------------------------------------------------------------
          MICRO_SEQUENCER_FW_VERSION_LSB                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_MICRO_SEQUENCER_FW_VERSION_LSB_USEQ_FW_VERSION_LSB (0U)           /* LSB bits of micro-sequencer FW version */
#define BITL_DATAPATH_MICRO_SEQUENCER_FW_VERSION_LSB_USEQ_FW_VERSION_LSB (16U)          /* LSB bits of micro-sequencer FW version */
#define BITM_DATAPATH_MICRO_SEQUENCER_FW_VERSION_LSB_USEQ_FW_VERSION_LSB (0XFFFFU)      /* LSB bits of micro-sequencer FW version */

/* ----------------------------------------------------------------------------------------------------
          MICRO_SEQUENCER_FW_VERSION_MSB                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_MICRO_SEQUENCER_FW_VERSION_MSB_USEQ_FW_VERSION_MSB (0U)           /* MSB bits of micro-sequencer FW version */
#define BITL_DATAPATH_MICRO_SEQUENCER_FW_VERSION_MSB_USEQ_FW_VERSION_MSB (16U)          /* MSB bits of micro-sequencer FW version */
#define BITM_DATAPATH_MICRO_SEQUENCER_FW_VERSION_MSB_USEQ_FW_VERSION_MSB (0XFFFFU)      /* MSB bits of micro-sequencer FW version */

/* ----------------------------------------------------------------------------------------------------
          TS_CAL_VER                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_TS_CAL_VER_TS_CAL_VER                   (0U)           /* Temperature sensor calibration version */
#define BITL_DATAPATH_TS_CAL_VER_TS_CAL_VER                   (16U)          /* Temperature sensor calibration version */
#define BITM_DATAPATH_TS_CAL_VER_TS_CAL_VER                   (0XFFFFU)      /* Temperature sensor calibration version */

/* ----------------------------------------------------------------------------------------------------
          ADC_CAL_VER                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_ADC_CAL_VER_ADC_CAL_VER                 (0U)           /* ADC and gain calibration version */
#define BITL_DATAPATH_ADC_CAL_VER_ADC_CAL_VER                 (16U)          /* ADC and gain calibration version */
#define BITM_DATAPATH_ADC_CAL_VER_ADC_CAL_VER                 (0XFFFFU)      /* ADC and gain calibration version */

/* ----------------------------------------------------------------------------------------------------
          REG_0                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_REG_0_REG_0                             (0U)           /* Reserved for MIPI header */
#define BITL_DATAPATH_REG_0_REG_0                             (16U)          /* Reserved for MIPI header */
#define BITM_DATAPATH_REG_0_REG_0                             (0XFFFFU)      /* Reserved for MIPI header */

/* ----------------------------------------------------------------------------------------------------
          REG_1                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_REG_1_REG_1                             (0U)           /* Reserved for MIPI header */
#define BITL_DATAPATH_REG_1_REG_1                             (16U)          /* Reserved for MIPI header */
#define BITM_DATAPATH_REG_1_REG_1                             (0XFFFFU)      /* Reserved for MIPI header */

/* ----------------------------------------------------------------------------------------------------
          REG_2                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_REG_2_REG_2                             (0U)           /* Reserved for MIPI header */
#define BITL_DATAPATH_REG_2_REG_2                             (16U)          /* Reserved for MIPI header */
#define BITM_DATAPATH_REG_2_REG_2                             (0XFFFFU)      /* Reserved for MIPI header */

/* ----------------------------------------------------------------------------------------------------
          REG_3                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_REG_3_REG_3                             (0U)           /* Reserved for MIPI header */
#define BITL_DATAPATH_REG_3_REG_3                             (16U)          /* Reserved for MIPI header */
#define BITM_DATAPATH_REG_3_REG_3                             (0XFFFFU)      /* Reserved for MIPI header */

/* ----------------------------------------------------------------------------------------------------
          REG_4                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_REG_4_REG_4                             (0U)           /* Reserved for MIPI header */
#define BITL_DATAPATH_REG_4_REG_4                             (16U)          /* Reserved for MIPI header */
#define BITM_DATAPATH_REG_4_REG_4                             (0XFFFFU)      /* Reserved for MIPI header */

/* ----------------------------------------------------------------------------------------------------
          REG_5                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_REG_5_REG_5                             (0U)           /* Reserved for MIPI header */
#define BITL_DATAPATH_REG_5_REG_5                             (16U)          /* Reserved for MIPI header */
#define BITM_DATAPATH_REG_5_REG_5                             (0XFFFFU)      /* Reserved for MIPI header */

/* ----------------------------------------------------------------------------------------------------
          REG_6                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_REG_6_REG_6                             (0U)           /* Reserved for MIPI header */
#define BITL_DATAPATH_REG_6_REG_6                             (16U)          /* Reserved for MIPI header */
#define BITM_DATAPATH_REG_6_REG_6                             (0XFFFFU)      /* Reserved for MIPI header */

/* ----------------------------------------------------------------------------------------------------
          REG_7                                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_REG_7_REG_7                             (0U)           /* Reserved for MIPI header */
#define BITL_DATAPATH_REG_7_REG_7                             (16U)          /* Reserved for MIPI header */
#define BITM_DATAPATH_REG_7_REG_7                             (0XFFFFU)      /* Reserved for MIPI header */

/* ----------------------------------------------------------------------------------------------------
          PARITY_MIPI_BUFFER                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_PARITY_MIPI_BUFFER_MIPI_BUFFER_PARITY_ERROR (0U)           /* parity error bits from MIPI buffer memory. 1 per line memory, memory not divided into banks. 4 memories of size 65WX128D */
#define BITL_DATAPATH_PARITY_MIPI_BUFFER_MIPI_BUFFER_PARITY_ERROR (4U)           /* parity error bits from MIPI buffer memory. 1 per line memory, memory not divided into banks. 4 memories of size 65WX128D */
#define BITM_DATAPATH_PARITY_MIPI_BUFFER_MIPI_BUFFER_PARITY_ERROR (0X000FU)      /* parity error bits from MIPI buffer memory. 1 per line memory, memory not divided into banks. 4 memories of size 65WX128D */

/* ----------------------------------------------------------------------------------------------------
          PACKET_COUNT                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_PACKET_COUNT_PACKET_COUNT               (0U)           /* Indicates the packet number being sent to MIPI. */
#define BITL_DATAPATH_PACKET_COUNT_PACKET_COUNT               (10U)          /* Indicates the packet number being sent to MIPI. */
#define BITM_DATAPATH_PACKET_COUNT_PACKET_COUNT               (0X03FFU)      /* Indicates the packet number being sent to MIPI. */

/* ----------------------------------------------------------------------------------------------------
          PACKETS_PER_FRAME                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_PACKETS_PER_FRAME_TOTAL_PACKETS_PER_FRAME (0U)           /* Indicates the total number of data packets in a MIPI frame */
#define BITL_DATAPATH_PACKETS_PER_FRAME_TOTAL_PACKETS_PER_FRAME (10U)          /* Indicates the total number of data packets in a MIPI frame */
#define BITM_DATAPATH_PACKETS_PER_FRAME_TOTAL_PACKETS_PER_FRAME (0X03FFU)      /* Indicates the total number of data packets in a MIPI frame */

/* ----------------------------------------------------------------------------------------------------
          ROW_VECTOR                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_ROW_VECTOR_ROW_VECTOR                   (0U)           /* Each bit is used to indicate that a group of 64 rows of pixel array are enabled. LSB bit used to indicate rows 0-63 */
#define BITL_DATAPATH_ROW_VECTOR_ROW_VECTOR                   (10U)          /* Each bit is used to indicate that a group of 64 rows of pixel array are enabled. LSB bit used to indicate rows 0-63 */
#define BITM_DATAPATH_ROW_VECTOR_ROW_VECTOR                   (0X03FFU)      /* Each bit is used to indicate that a group of 64 rows of pixel array are enabled. LSB bit used to indicate rows 0-63 */

/* ----------------------------------------------------------------------------------------------------
          ROWS_PER_PACKET_OUT                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_ROWS_PER_PACKET_OUT_ROWS_PER_PACKET_OUT (0U)           /* When auto_rows_per_packet_en = 1; readback this value calculated by the datapath */
#define BITL_DATAPATH_ROWS_PER_PACKET_OUT_ROWS_PER_PACKET_OUT (7U)           /* When auto_rows_per_packet_en = 1; readback this value calculated by the datapath */
#define BITM_DATAPATH_ROWS_PER_PACKET_OUT_ROWS_PER_PACKET_OUT (0X007FU)      /* When auto_rows_per_packet_en = 1; readback this value calculated by the datapath */

/* ----------------------------------------------------------------------------------------------------
          MIPI_RD_EN_MAX                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_MIPI_RD_EN_MAX_MIPI_BUFF_READ_ENABLE_COUNT_MAX (0U)           /* Indicates the number of reads done per mipi buffer. This value will be equal to (number of pixels per dump written to the MIPI buffer)/4 */
#define BITL_DATAPATH_MIPI_RD_EN_MAX_MIPI_BUFF_READ_ENABLE_COUNT_MAX (8U)           /* Indicates the number of reads done per mipi buffer. This value will be equal to (number of pixels per dump written to the MIPI buffer)/4 */
#define BITM_DATAPATH_MIPI_RD_EN_MAX_MIPI_BUFF_READ_ENABLE_COUNT_MAX (0X00FFU)      /* Indicates the number of reads done per mipi buffer. This value will be equal to (number of pixels per dump written to the MIPI buffer)/4 */

/* ----------------------------------------------------------------------------------------------------
          ANALOG_SS                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_ANALOG_SS_ANALOG_SS                     (0U)           /* Indicates the sub-sampling factor. Input from DE */
#define BITL_DATAPATH_ANALOG_SS_ANALOG_SS                     (4U)           /* Indicates the sub-sampling factor. Input from DE */
#define BITM_DATAPATH_ANALOG_SS_ANALOG_SS                     (0X000FU)      /* Indicates the sub-sampling factor. Input from DE */

/* ----------------------------------------------------------------------------------------------------
          MIPI_BUFF_PARITY_ERR_CNT                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_MIPI_BUFF_PARITY_ERR_CNT_MIPI_BUFF_PARITY_ERR_COUNT (0U)           /* Count of parity errors indicated for every MIPI buffer read */
#define BITL_DATAPATH_MIPI_BUFF_PARITY_ERR_CNT_MIPI_BUFF_PARITY_ERR_COUNT (16U)          /* Count of parity errors indicated for every MIPI buffer read */
#define BITM_DATAPATH_MIPI_BUFF_PARITY_ERR_CNT_MIPI_BUFF_PARITY_ERR_COUNT (0XFFFFU)      /* Count of parity errors indicated for every MIPI buffer read */

/* ----------------------------------------------------------------------------------------------------
          LINE_MEM_PARITY_ERR_CNT                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_LINE_MEM_PARITY_ERR_CNT_LINE_MEM_PARITY_ERR_COUNT (0U)           /* Count of parity errors indicated for every line memory read */
#define BITL_DATAPATH_LINE_MEM_PARITY_ERR_CNT_LINE_MEM_PARITY_ERR_COUNT (16U)          /* Count of parity errors indicated for every line memory read */
#define BITM_DATAPATH_LINE_MEM_PARITY_ERR_CNT_LINE_MEM_PARITY_ERR_COUNT (0XFFFFU)      /* Count of parity errors indicated for every line memory read */

/* ----------------------------------------------------------------------------------------------------
          GAIN_MEM_PARITY_ERR_CNT                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_GAIN_MEM_PARITY_ERR_CNT_GAIN_MEM_PARITY_ERR_COUNT (0U)           /* Count of parity errors indicated for every gain memory read */
#define BITL_DATAPATH_GAIN_MEM_PARITY_ERR_CNT_GAIN_MEM_PARITY_ERR_COUNT (16U)          /* Count of parity errors indicated for every gain memory read */
#define BITM_DATAPATH_GAIN_MEM_PARITY_ERR_CNT_GAIN_MEM_PARITY_ERR_COUNT (0XFFFFU)      /* Count of parity errors indicated for every gain memory read */

/* ----------------------------------------------------------------------------------------------------
          IA_SELECT                                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_IA_SELECT_IA_ENA                        (0U)           /* Indirect access enable for column correction memory */
#define BITL_DATAPATH_IA_SELECT_IA_ENA                        (1U)           /* Indirect access enable for column correction memory */
#define BITM_DATAPATH_IA_SELECT_IA_ENA                        (0X0001U)      /* Indirect access enable for column correction memory */

/* ----------------------------------------------------------------------------------------------------
          IA_ADDR_REG                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_IA_ADDR_REG_IA_START_ADDR               (0U)           /* Indirect access start address */
#define BITL_DATAPATH_IA_ADDR_REG_IA_START_ADDR               (12U)          /* Indirect access start address */
#define BITM_DATAPATH_IA_ADDR_REG_IA_START_ADDR               (0X0FFFU)      /* Indirect access start address */

/* ----------------------------------------------------------------------------------------------------
          IA_WRDATA_REG                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_IA_WRDATA_REG_IA_WRDATA                 (0U)           /* Indirect access write data */
#define BITL_DATAPATH_IA_WRDATA_REG_IA_WRDATA                 (16U)          /* Indirect access write data */
#define BITM_DATAPATH_IA_WRDATA_REG_IA_WRDATA                 (0XFFFFU)      /* Indirect access write data */

/* ----------------------------------------------------------------------------------------------------
          IA_WRDATA_REG_ALIAS                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_IA_WRDATA_REG_ALIAS_IA_WRDATA_ALIAS     (0U)           /* Indirect access write data (alias for ia_wrdata) */
#define BITL_DATAPATH_IA_WRDATA_REG_ALIAS_IA_WRDATA_ALIAS     (16U)          /* Indirect access write data (alias for ia_wrdata) */
#define BITM_DATAPATH_IA_WRDATA_REG_ALIAS_IA_WRDATA_ALIAS     (0XFFFFU)      /* Indirect access write data (alias for ia_wrdata) */

/* ----------------------------------------------------------------------------------------------------
          IA_RDDATA_REG                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_IA_RDDATA_REG_IA_RDDATA                 (0U)           /* Indirect access read data */
#define BITL_DATAPATH_IA_RDDATA_REG_IA_RDDATA                 (16U)          /* Indirect access read data */
#define BITM_DATAPATH_IA_RDDATA_REG_IA_RDDATA                 (0XFFFFU)      /* Indirect access read data */

/* ----------------------------------------------------------------------------------------------------
          IA_RDDATA_REG_ALIAS                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_IA_RDDATA_REG_ALIAS_IA_RDDATA_ALIAS     (0U)           /* Indirect access read data (alias for ia_rdata) */
#define BITL_DATAPATH_IA_RDDATA_REG_ALIAS_IA_RDDATA_ALIAS     (16U)          /* Indirect access read data (alias for ia_rdata) */
#define BITM_DATAPATH_IA_RDDATA_REG_ALIAS_IA_RDDATA_ALIAS     (0XFFFFU)      /* Indirect access read data (alias for ia_rdata) */

/* ----------------------------------------------------------------------------------------------------
          IA_BANK_TYPE                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DATAPATH_IA_BANK_TYPE_IA_BANK_TYPE               (0U)           /* 1'b1 = LSB bits of address used to select bank in reg mem access module. 1'b0 = have another input which specifies the bank */
#define BITL_DATAPATH_IA_BANK_TYPE_IA_BANK_TYPE               (1U)           /* 1'b1 = LSB bits of address used to select bank in reg mem access module. 1'b0 = have another input which specifies the bank */
#define BITM_DATAPATH_IA_BANK_TYPE_IA_BANK_TYPE               (0X0001U)      /* 1'b1 = LSB bits of address used to select bank in reg mem access module. 1'b0 = have another input which specifies the bank */

#endif  /* end ifndef DATAPATH_ADDR_RDEF_H_ */

/* ====================================================================================================
        DE_REGS_YODA Module Instances Address and Mask Definitions
   ==================================================================================================== */
#define INST_DE_REGS                                             (0X00000E00U)    /* de_regs: */


#ifndef DE_REGS_YODA_ADDR_RDEF_H_
#define DE_REGS_YODA_ADDR_RDEF_H_    /* DE_REGS_YODA: Your module description, here. */

#define MASK_DE_REGS_YODA                                        (0X000000FFU)    /* DE_REGS_YODA: Your module description, here. */

/* ====================================================================================================
        DE_REGS_YODA Module Register ResetValue Definitions
   ==================================================================================================== */
#define RSTVAL_DE_REGS_YODA_DE_CONTROL                             (0X1) 
#define RSTVAL_DE_REGS_YODA_BINNED1X2_REPEAT_COUNT                 (0X1) 
#define RSTVAL_DE_REGS_YODA_OVERRIDE_DATA_REG1                     (0X0) 
#define RSTVAL_DE_REGS_YODA_OVERRIDE_DATA_REG2                     (0X0) 
#define RSTVAL_DE_REGS_YODA_OVERRIDE_DATA_REG3                     (0X0) 
#define RSTVAL_DE_REGS_YODA_BINNED1X2_END                          (0X0) 
#define RSTVAL_DE_REGS_YODA_OVERRIDE_SEL_REG1                      (0X0) 
#define RSTVAL_DE_REGS_YODA_OVERRIDE_SEL_REG2                      (0X0) 
#define RSTVAL_DE_REGS_YODA_OVERRIDE_SEL_REG3                      (0X0) 
#define RSTVAL_DE_REGS_YODA_BINNED1X2_START                        (0X0) 
#define RSTVAL_DE_REGS_YODA_AMP_MUX_SEL_EE_LOW                     (0X41) 
#define RSTVAL_DE_REGS_YODA_AMP_MUX_SEL_EE_HIGH                    (0X0) 
#define RSTVAL_DE_REGS_YODA_AMP_MUX_SEL_EO_LOW                     (0X0) 
#define RSTVAL_DE_REGS_YODA_AMP_MUX_SEL_EO_HIGH                    (0X0) 
#define RSTVAL_DE_REGS_YODA_AMP_MUX_SEL_OE_LOW                     (0X0) 
#define RSTVAL_DE_REGS_YODA_AMP_MUX_SEL_OE_HIGH                    (0X0) 
#define RSTVAL_DE_REGS_YODA_AMP_MUX_SEL_OO_LOW                     (0X0) 
#define RSTVAL_DE_REGS_YODA_AMP_MUX_SEL_OO_HIGH                    (0X0) 
#define RSTVAL_DE_REGS_YODA_AMP_MUX_SEL_SELB_LOW                   (0X0) 
#define RSTVAL_DE_REGS_YODA_AMP_MUX_SEL_SELB_HIGH                  (0X0) 
#define RSTVAL_DE_REGS_YODA_NATIVE_RESOLUTION_START                (0X0) 
#define RSTVAL_DE_REGS_YODA_NATIVE_RESOLUTION_END                  (0X0) 
#define RSTVAL_DE_REGS_YODA_NATIVE_RESOLUTION_REPEAT               (0X0) 
#define RSTVAL_DE_REGS_YODA_SUB_SAMPLED_2X_START                   (0X0) 
#define RSTVAL_DE_REGS_YODA_SUB_SAMPLED_2X_END                     (0X0) 
#define RSTVAL_DE_REGS_YODA_SUB_SAMPLED_2X_REPEAT                  (0X0) 
#define RSTVAL_DE_REGS_YODA_SUB_SAMPLED_4X_START                   (0X0) 
#define RSTVAL_DE_REGS_YODA_SUB_SAMPLED_4X_END                     (0X0) 
#define RSTVAL_DE_REGS_YODA_SUB_SAMPLED_4X_REPEAT                  (0X0) 
#define RSTVAL_DE_REGS_YODA_BINNED_START                           (0X0) 
#define RSTVAL_DE_REGS_YODA_BINNED_END                             (0X0) 
#define RSTVAL_DE_REGS_YODA_BINNED_REPEAT                          (0X0) 
#define RSTVAL_DE_REGS_YODA_DARK_START                             (0X0) 
#define RSTVAL_DE_REGS_YODA_DARK_END                               (0X0) 
#define RSTVAL_DE_REGS_YODA_DARK_REPEAT                            (0X0) 
#define RSTVAL_DE_REGS_YODA_PREAMBLE_START                         (0X0) 
#define RSTVAL_DE_REGS_YODA_PREAMBLE_END                           (0X0) 
#define RSTVAL_DE_REGS_YODA_PREAMBLE_REPEAT                        (0X0) 
#define RSTVAL_DE_REGS_YODA_POSTAMBLE_START                        (0X0) 
#define RSTVAL_DE_REGS_YODA_POSTAMBLE_END                          (0X0) 
#define RSTVAL_DE_REGS_YODA_POSTAMBLE_REPEAT                       (0X0) 
#define RSTVAL_DE_REGS_YODA_ARRAY_INIT_VEC_DARK                    (0X0) 
#define RSTVAL_DE_REGS_YODA_ARRAY_INIT_VEC                         (0X0) 
#define RSTVAL_DE_REGS_YODA_TYPE_OVERRIDE                          (0X0) 
#define RSTVAL_DE_REGS_YODA_MEM_DFT                                (0X0) 
#define RSTVAL_DE_REGS_YODA_DBG_MUX_CONTROL_0                      (0X0) 
#define RSTVAL_DE_REGS_YODA_DBG_MUX_CONTROL_1                      (0X0) 
#define RSTVAL_DE_REGS_YODA_DBG_MUX_CONTROL_2                      (0X0) 
#define RSTVAL_DE_REGS_YODA_DBG_MUX_CONTROL_3                      (0X0) 
#define RSTVAL_DE_REGS_YODA_DBG_MUX_CONTROL_4                      (0X0) 
#define RSTVAL_DE_REGS_YODA_DE_IA_SELECT                           (0X0) 
#define RSTVAL_DE_REGS_YODA_DE_IA_ADDR_REG                         (0X0) 
#define RSTVAL_DE_REGS_YODA_DE_IA_WRDATA_REG                       (0X0) 
#define RSTVAL_DE_REGS_YODA_DE_IA_WRDATA_REG_ALIAS                 (0X0) 
#define RSTVAL_DE_REGS_YODA_DE_IA_RDDATA_REG                       (0X0) 
#define RSTVAL_DE_REGS_YODA_DE_IA_RDDATA_REG_ALIAS                 (0X0) 
#define RSTVAL_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_TOPBOT        (0X0) 
#define RSTVAL_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_MAIN          (0X0) 
#define RSTVAL_DE_REGS_YODA_USE_CASE_0_ROI_0_COLUMN_VEC            (0X0) 
#define RSTVAL_DE_REGS_YODA_USE_CASE_0_ROI_0_ROI_TYPE              (0X0) 
#define RSTVAL_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_TOPBOT        (0X0) 
#define RSTVAL_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_MAIN          (0X0) 
#define RSTVAL_DE_REGS_YODA_USE_CASE_0_ROI_1_COLUMN_VEC            (0X0) 
#define RSTVAL_DE_REGS_YODA_USE_CASE_0_ROI_1_ROI_TYPE              (0X0) 
#define RSTVAL_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_TOPBOT        (0X0) 
#define RSTVAL_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_MAIN          (0X0) 
#define RSTVAL_DE_REGS_YODA_USE_CASE_0_ROI_2_COLUMN_VEC            (0X0) 
#define RSTVAL_DE_REGS_YODA_USE_CASE_0_ROI_2_ROI_TYPE              (0X0) 

/* ====================================================================================================
        DE_REGS_YODA Module Register BitPositions, Lengths, Masks and Enumerations Definitions
   ==================================================================================================== */

/* ----------------------------------------------------------------------------------------------------
          DE_CONTROL                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DE_CONTROL_STATIC_CNTRL             (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_CONTROL_STATIC_CNTRL             (1U)           /* No description provided */
#define BITM_DE_REGS_YODA_DE_CONTROL_STATIC_CNTRL             (0X0001U)      /* No description provided */
#define BITP_DE_REGS_YODA_DE_CONTROL_OVERIDE_CNTRL            (1U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_CONTROL_OVERIDE_CNTRL            (1U)           /* No description provided */
#define BITM_DE_REGS_YODA_DE_CONTROL_OVERIDE_CNTRL            (0X0002U)      /* No description provided */
#define BITP_DE_REGS_YODA_DE_CONTROL_GO_BIT                   (2U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_CONTROL_GO_BIT                   (1U)           /* No description provided */
#define BITM_DE_REGS_YODA_DE_CONTROL_GO_BIT                   (0X0004U)      /* No description provided */
#define BITP_DE_REGS_YODA_DE_CONTROL_BUSY_BIT                 (3U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_CONTROL_BUSY_BIT                 (1U)           /* No description provided */
#define BITM_DE_REGS_YODA_DE_CONTROL_BUSY_BIT                 (0X0008U)      /* No description provided */
#define BITP_DE_REGS_YODA_DE_CONTROL_POSTAMBLE_BIT            (4U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_CONTROL_POSTAMBLE_BIT            (1U)           /* No description provided */
#define BITM_DE_REGS_YODA_DE_CONTROL_POSTAMBLE_BIT            (0X0010U)      /* No description provided */
#define BITP_DE_REGS_YODA_DE_CONTROL_PREAMBLE_BIT             (5U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_CONTROL_PREAMBLE_BIT             (1U)           /* No description provided */
#define BITM_DE_REGS_YODA_DE_CONTROL_PREAMBLE_BIT             (0X0020U)      /* No description provided */
#define BITP_DE_REGS_YODA_DE_CONTROL_MANUAL_MODE_BIT          (6U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_CONTROL_MANUAL_MODE_BIT          (1U)           /* No description provided */
#define BITM_DE_REGS_YODA_DE_CONTROL_MANUAL_MODE_BIT          (0X0040U)      /* No description provided */
#define BITP_DE_REGS_YODA_DE_CONTROL_INIT_VEC_BIT             (7U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_CONTROL_INIT_VEC_BIT             (1U)           /* No description provided */
#define BITM_DE_REGS_YODA_DE_CONTROL_INIT_VEC_BIT             (0X0080U)      /* No description provided */
#define BITP_DE_REGS_YODA_DE_CONTROL_DUMP_DIRECTION_BIT       (8U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_CONTROL_DUMP_DIRECTION_BIT       (1U)           /* No description provided */
#define BITM_DE_REGS_YODA_DE_CONTROL_DUMP_DIRECTION_BIT       (0X0100U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          BINNED1X2_REPEAT_COUNT                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_BINNED1X2_REPEAT_COUNT_REPEAT_COUNT (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_BINNED1X2_REPEAT_COUNT_REPEAT_COUNT (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_BINNED1X2_REPEAT_COUNT_REPEAT_COUNT (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          OVERRIDE_DATA_REG1                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_OVERRIDE_DATA_REG1_OVR_VAL          (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_OVERRIDE_DATA_REG1_OVR_VAL          (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_OVERRIDE_DATA_REG1_OVR_VAL          (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          OVERRIDE_DATA_REG2                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_OVERRIDE_DATA_REG2_OVR_VAL          (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_OVERRIDE_DATA_REG2_OVR_VAL          (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_OVERRIDE_DATA_REG2_OVR_VAL          (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          OVERRIDE_DATA_REG3                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_OVERRIDE_DATA_REG3_OVR_VAL          (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_OVERRIDE_DATA_REG3_OVR_VAL          (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_OVERRIDE_DATA_REG3_OVR_VAL          (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          BINNED1X2_END                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_BINNED1X2_END_END_ADDRESS           (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_BINNED1X2_END_END_ADDRESS           (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_BINNED1X2_END_END_ADDRESS           (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          OVERRIDE_SEL_REG1                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_OVERRIDE_SEL_REG1_OVR_VAL_SEL       (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_OVERRIDE_SEL_REG1_OVR_VAL_SEL       (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_OVERRIDE_SEL_REG1_OVR_VAL_SEL       (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          OVERRIDE_SEL_REG2                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_OVERRIDE_SEL_REG2_OVR_VAL_SEL       (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_OVERRIDE_SEL_REG2_OVR_VAL_SEL       (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_OVERRIDE_SEL_REG2_OVR_VAL_SEL       (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          OVERRIDE_SEL_REG3                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_OVERRIDE_SEL_REG3_OVR_VAL_SEL       (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_OVERRIDE_SEL_REG3_OVR_VAL_SEL       (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_OVERRIDE_SEL_REG3_OVR_VAL_SEL       (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          BINNED1X2_START                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_BINNED1X2_START_START_ADDRESS       (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_BINNED1X2_START_START_ADDRESS       (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_BINNED1X2_START_START_ADDRESS       (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMP_MUX_SEL_EE_LOW                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_AMP_MUX_SEL_EE_LOW_BITS15_0         (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_AMP_MUX_SEL_EE_LOW_BITS15_0         (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_AMP_MUX_SEL_EE_LOW_BITS15_0         (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMP_MUX_SEL_EE_HIGH                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_AMP_MUX_SEL_EE_HIGH_BITS20_16       (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_AMP_MUX_SEL_EE_HIGH_BITS20_16       (5U)           /* No description provided */
#define BITM_DE_REGS_YODA_AMP_MUX_SEL_EE_HIGH_BITS20_16       (0X001FU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMP_MUX_SEL_EO_LOW                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_AMP_MUX_SEL_EO_LOW_BITS15_0         (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_AMP_MUX_SEL_EO_LOW_BITS15_0         (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_AMP_MUX_SEL_EO_LOW_BITS15_0         (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMP_MUX_SEL_EO_HIGH                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_AMP_MUX_SEL_EO_HIGH_BITS20_16       (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_AMP_MUX_SEL_EO_HIGH_BITS20_16       (5U)           /* No description provided */
#define BITM_DE_REGS_YODA_AMP_MUX_SEL_EO_HIGH_BITS20_16       (0X001FU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMP_MUX_SEL_OE_LOW                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_AMP_MUX_SEL_OE_LOW_BITS15_0         (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_AMP_MUX_SEL_OE_LOW_BITS15_0         (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_AMP_MUX_SEL_OE_LOW_BITS15_0         (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMP_MUX_SEL_OE_HIGH                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_AMP_MUX_SEL_OE_HIGH_BITS20_16       (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_AMP_MUX_SEL_OE_HIGH_BITS20_16       (5U)           /* No description provided */
#define BITM_DE_REGS_YODA_AMP_MUX_SEL_OE_HIGH_BITS20_16       (0X001FU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMP_MUX_SEL_OO_LOW                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_AMP_MUX_SEL_OO_LOW_BITS15_0         (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_AMP_MUX_SEL_OO_LOW_BITS15_0         (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_AMP_MUX_SEL_OO_LOW_BITS15_0         (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMP_MUX_SEL_OO_HIGH                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_AMP_MUX_SEL_OO_HIGH_BITS20_16       (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_AMP_MUX_SEL_OO_HIGH_BITS20_16       (5U)           /* No description provided */
#define BITM_DE_REGS_YODA_AMP_MUX_SEL_OO_HIGH_BITS20_16       (0X001FU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMP_MUX_SEL_SELB_LOW                                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_AMP_MUX_SEL_SELB_LOW_BITS15_0       (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_AMP_MUX_SEL_SELB_LOW_BITS15_0       (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_AMP_MUX_SEL_SELB_LOW_BITS15_0       (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          AMP_MUX_SEL_SELB_HIGH                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_AMP_MUX_SEL_SELB_HIGH_BITS20_16     (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_AMP_MUX_SEL_SELB_HIGH_BITS20_16     (5U)           /* No description provided */
#define BITM_DE_REGS_YODA_AMP_MUX_SEL_SELB_HIGH_BITS20_16     (0X001FU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          NATIVE_RESOLUTION_START                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_NATIVE_RESOLUTION_START_ADDRESS     (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_NATIVE_RESOLUTION_START_ADDRESS     (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_NATIVE_RESOLUTION_START_ADDRESS     (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          NATIVE_RESOLUTION_END                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_NATIVE_RESOLUTION_END_ADDRESS       (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_NATIVE_RESOLUTION_END_ADDRESS       (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_NATIVE_RESOLUTION_END_ADDRESS       (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          NATIVE_RESOLUTION_REPEAT                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_NATIVE_RESOLUTION_REPEAT_COUNT      (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_NATIVE_RESOLUTION_REPEAT_COUNT      (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_NATIVE_RESOLUTION_REPEAT_COUNT      (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SUB_SAMPLED_2X_START                                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_SUB_SAMPLED_2X_START_ADDRESS        (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_SUB_SAMPLED_2X_START_ADDRESS        (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_SUB_SAMPLED_2X_START_ADDRESS        (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SUB_SAMPLED_2X_END                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_SUB_SAMPLED_2X_END_ADDRESS          (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_SUB_SAMPLED_2X_END_ADDRESS          (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_SUB_SAMPLED_2X_END_ADDRESS          (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SUB_SAMPLED_2X_REPEAT                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_SUB_SAMPLED_2X_REPEAT_COUNT         (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_SUB_SAMPLED_2X_REPEAT_COUNT         (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_SUB_SAMPLED_2X_REPEAT_COUNT         (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SUB_SAMPLED_4X_START                                  Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_SUB_SAMPLED_4X_START_ADDRESS        (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_SUB_SAMPLED_4X_START_ADDRESS        (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_SUB_SAMPLED_4X_START_ADDRESS        (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SUB_SAMPLED_4X_END                                    Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_SUB_SAMPLED_4X_END_ADDRESS          (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_SUB_SAMPLED_4X_END_ADDRESS          (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_SUB_SAMPLED_4X_END_ADDRESS          (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          SUB_SAMPLED_4X_REPEAT                                 Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_SUB_SAMPLED_4X_REPEAT_COUNT         (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_SUB_SAMPLED_4X_REPEAT_COUNT         (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_SUB_SAMPLED_4X_REPEAT_COUNT         (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          BINNED_START                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_BINNED_START_ADDRESS                (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_BINNED_START_ADDRESS                (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_BINNED_START_ADDRESS                (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          BINNED_END                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_BINNED_END_ADDRESS                  (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_BINNED_END_ADDRESS                  (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_BINNED_END_ADDRESS                  (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          BINNED_REPEAT                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_BINNED_REPEAT_COUNT                 (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_BINNED_REPEAT_COUNT                 (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_BINNED_REPEAT_COUNT                 (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DARK_START                                            Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DARK_START_ADDRESS                  (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DARK_START_ADDRESS                  (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_DARK_START_ADDRESS                  (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DARK_END                                              Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DARK_END_ADDRESS                    (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DARK_END_ADDRESS                    (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_DARK_END_ADDRESS                    (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DARK_REPEAT                                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DARK_REPEAT_COUNT                   (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DARK_REPEAT_COUNT                   (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_DARK_REPEAT_COUNT                   (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PREAMBLE_START                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_PREAMBLE_START_ADDRESS              (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_PREAMBLE_START_ADDRESS              (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_PREAMBLE_START_ADDRESS              (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PREAMBLE_END                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_PREAMBLE_END_ADDRESS                (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_PREAMBLE_END_ADDRESS                (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_PREAMBLE_END_ADDRESS                (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          PREAMBLE_REPEAT                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_PREAMBLE_REPEAT_COUNT               (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_PREAMBLE_REPEAT_COUNT               (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_PREAMBLE_REPEAT_COUNT               (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          POSTAMBLE_START                                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_POSTAMBLE_START_ADDRESS             (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_POSTAMBLE_START_ADDRESS             (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_POSTAMBLE_START_ADDRESS             (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          POSTAMBLE_END                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_POSTAMBLE_END_ADDRESS               (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_POSTAMBLE_END_ADDRESS               (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_POSTAMBLE_END_ADDRESS               (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          POSTAMBLE_REPEAT                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_POSTAMBLE_REPEAT_COUNT              (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_POSTAMBLE_REPEAT_COUNT              (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_POSTAMBLE_REPEAT_COUNT              (0X00FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ARRAY_INIT_VEC_DARK                                   Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_ARRAY_INIT_VEC_DARK_DARK_BITS       (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_ARRAY_INIT_VEC_DARK_DARK_BITS       (2U)           /* No description provided */
#define BITM_DE_REGS_YODA_ARRAY_INIT_VEC_DARK_DARK_BITS       (0X0003U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          ARRAY_INIT_VEC                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_ARRAY_INIT_VEC_ARRAY_BITS           (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_ARRAY_INIT_VEC_ARRAY_BITS           (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_ARRAY_INIT_VEC_ARRAY_BITS           (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          TYPE_OVERRIDE                                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_TYPE_OVERRIDE_ROI_0                 (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_TYPE_OVERRIDE_ROI_0                 (4U)           /* No description provided */
#define BITM_DE_REGS_YODA_TYPE_OVERRIDE_ROI_0                 (0X000FU)      /* No description provided */
#define BITP_DE_REGS_YODA_TYPE_OVERRIDE_ROI_1                 (4U)           /* No description provided */
#define BITL_DE_REGS_YODA_TYPE_OVERRIDE_ROI_1                 (4U)           /* No description provided */
#define BITM_DE_REGS_YODA_TYPE_OVERRIDE_ROI_1                 (0X00F0U)      /* No description provided */
#define BITP_DE_REGS_YODA_TYPE_OVERRIDE_ROI_2                 (8U)           /* No description provided */
#define BITL_DE_REGS_YODA_TYPE_OVERRIDE_ROI_2                 (4U)           /* No description provided */
#define BITM_DE_REGS_YODA_TYPE_OVERRIDE_ROI_2                 (0X0F00U)      /* No description provided */
#define BITP_DE_REGS_YODA_TYPE_OVERRIDE_ANALOG                (12U)          /* No description provided */
#define BITL_DE_REGS_YODA_TYPE_OVERRIDE_ANALOG                (4U)           /* No description provided */
#define BITM_DE_REGS_YODA_TYPE_OVERRIDE_ANALOG                (0XF000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          MEM_DFT                                               Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_MEM_DFT_MARGIN                      (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_MEM_DFT_MARGIN                      (1U)           /* No description provided */
#define BITM_DE_REGS_YODA_MEM_DFT_MARGIN                      (0X0001U)      /* No description provided */
#define BITP_DE_REGS_YODA_MEM_DFT_PARITY_ERR                  (15U)          /* No description provided */
#define BITL_DE_REGS_YODA_MEM_DFT_PARITY_ERR                  (1U)           /* No description provided */
#define BITM_DE_REGS_YODA_MEM_DFT_PARITY_ERR                  (0X8000U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DBG_MUX_CONTROL_0                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DBG_MUX_CONTROL_0_CNTRL_0           (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DBG_MUX_CONTROL_0_CNTRL_0           (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_DBG_MUX_CONTROL_0_CNTRL_0           (0X00FFU)      /* No description provided */
#define BITP_DE_REGS_YODA_DBG_MUX_CONTROL_0_CNTRL_1           (8U)           /* No description provided */
#define BITL_DE_REGS_YODA_DBG_MUX_CONTROL_0_CNTRL_1           (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_DBG_MUX_CONTROL_0_CNTRL_1           (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DBG_MUX_CONTROL_1                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DBG_MUX_CONTROL_1_CNTRL_2           (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DBG_MUX_CONTROL_1_CNTRL_2           (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_DBG_MUX_CONTROL_1_CNTRL_2           (0X00FFU)      /* No description provided */
#define BITP_DE_REGS_YODA_DBG_MUX_CONTROL_1_CNTRL_3           (8U)           /* No description provided */
#define BITL_DE_REGS_YODA_DBG_MUX_CONTROL_1_CNTRL_3           (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_DBG_MUX_CONTROL_1_CNTRL_3           (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DBG_MUX_CONTROL_2                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DBG_MUX_CONTROL_2_CNTRL_4           (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DBG_MUX_CONTROL_2_CNTRL_4           (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_DBG_MUX_CONTROL_2_CNTRL_4           (0X00FFU)      /* No description provided */
#define BITP_DE_REGS_YODA_DBG_MUX_CONTROL_2_CNTRL_5           (8U)           /* No description provided */
#define BITL_DE_REGS_YODA_DBG_MUX_CONTROL_2_CNTRL_5           (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_DBG_MUX_CONTROL_2_CNTRL_5           (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DBG_MUX_CONTROL_3                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DBG_MUX_CONTROL_3_CNTRL_6           (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DBG_MUX_CONTROL_3_CNTRL_6           (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_DBG_MUX_CONTROL_3_CNTRL_6           (0X00FFU)      /* No description provided */
#define BITP_DE_REGS_YODA_DBG_MUX_CONTROL_3_CNTRL_7           (8U)           /* No description provided */
#define BITL_DE_REGS_YODA_DBG_MUX_CONTROL_3_CNTRL_7           (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_DBG_MUX_CONTROL_3_CNTRL_7           (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DBG_MUX_CONTROL_4                                     Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DBG_MUX_CONTROL_4_CNTRL_8           (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DBG_MUX_CONTROL_4_CNTRL_8           (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_DBG_MUX_CONTROL_4_CNTRL_8           (0X00FFU)      /* No description provided */
#define BITP_DE_REGS_YODA_DBG_MUX_CONTROL_4_CNTRL_9           (8U)           /* No description provided */
#define BITL_DE_REGS_YODA_DBG_MUX_CONTROL_4_CNTRL_9           (8U)           /* No description provided */
#define BITM_DE_REGS_YODA_DBG_MUX_CONTROL_4_CNTRL_9           (0XFF00U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DE_IA_SELECT                                          Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DE_IA_SELECT_RAM                    (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_IA_SELECT_RAM                    (1U)           /* No description provided */
#define BITM_DE_REGS_YODA_DE_IA_SELECT_RAM                    (0X0001U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DE_IA_ADDR_REG                                        Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DE_IA_ADDR_REG_RAM_ADDR             (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_IA_ADDR_REG_RAM_ADDR             (10U)          /* No description provided */
#define BITM_DE_REGS_YODA_DE_IA_ADDR_REG_RAM_ADDR             (0X03FFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DE_IA_WRDATA_REG                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DE_IA_WRDATA_REG_RAM_WRDATA         (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_IA_WRDATA_REG_RAM_WRDATA         (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_DE_IA_WRDATA_REG_RAM_WRDATA         (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DE_IA_WRDATA_REG_ALIAS                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DE_IA_WRDATA_REG_ALIAS_RAM_WRDATA_ALIAS (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_IA_WRDATA_REG_ALIAS_RAM_WRDATA_ALIAS (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_DE_IA_WRDATA_REG_ALIAS_RAM_WRDATA_ALIAS (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DE_IA_RDDATA_REG                                      Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DE_IA_RDDATA_REG_RAM_RDDATA         (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_IA_RDDATA_REG_RAM_RDDATA         (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_DE_IA_RDDATA_REG_RAM_RDDATA         (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          DE_IA_RDDATA_REG_ALIAS                                Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_DE_IA_RDDATA_REG_ALIAS_RAM_RDDATA_ALIAS (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_DE_IA_RDDATA_REG_ALIAS_RAM_RDDATA_ALIAS (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_DE_IA_RDDATA_REG_ALIAS_RAM_RDDATA_ALIAS (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_0_ROI_0_ROW_VEC_TOPBOT                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_TOPBOT_BITS17_AND_0 (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_TOPBOT_BITS17_AND_0 (2U)           /* No description provided */
#define BITM_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_TOPBOT_BITS17_AND_0 (0X0003U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_0_ROI_0_ROW_VEC_MAIN                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_MAIN_BITS16_1 (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_MAIN_BITS16_1 (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_MAIN_BITS16_1 (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_0_ROI_0_COLUMN_VEC                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_USE_CASE_0_ROI_0_COLUMN_VEC_BITS15_0 (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_USE_CASE_0_ROI_0_COLUMN_VEC_BITS15_0 (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_USE_CASE_0_ROI_0_COLUMN_VEC_BITS15_0 (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_0_ROI_0_ROI_TYPE                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_USE_CASE_0_ROI_0_ROI_TYPE_BINSS     (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_USE_CASE_0_ROI_0_ROI_TYPE_BINSS     (4U)           /* No description provided */
#define BITM_DE_REGS_YODA_USE_CASE_0_ROI_0_ROI_TYPE_BINSS     (0X000FU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_0_ROI_1_ROW_VEC_TOPBOT                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_TOPBOT_BITS17_AND_0 (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_TOPBOT_BITS17_AND_0 (2U)           /* No description provided */
#define BITM_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_TOPBOT_BITS17_AND_0 (0X0003U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_0_ROI_1_ROW_VEC_MAIN                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_MAIN_BITS16_1 (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_MAIN_BITS16_1 (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_MAIN_BITS16_1 (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_0_ROI_1_COLUMN_VEC                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_USE_CASE_0_ROI_1_COLUMN_VEC_BITS15_0 (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_USE_CASE_0_ROI_1_COLUMN_VEC_BITS15_0 (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_USE_CASE_0_ROI_1_COLUMN_VEC_BITS15_0 (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_0_ROI_1_ROI_TYPE                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_USE_CASE_0_ROI_1_ROI_TYPE_BINSS     (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_USE_CASE_0_ROI_1_ROI_TYPE_BINSS     (4U)           /* No description provided */
#define BITM_DE_REGS_YODA_USE_CASE_0_ROI_1_ROI_TYPE_BINSS     (0X000FU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_0_ROI_2_ROW_VEC_TOPBOT                       Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_TOPBOT_BITS17_AND_0 (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_TOPBOT_BITS17_AND_0 (2U)           /* No description provided */
#define BITM_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_TOPBOT_BITS17_AND_0 (0X0003U)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_0_ROI_2_ROW_VEC_MAIN                         Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_MAIN_BITS16_1 (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_MAIN_BITS16_1 (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_MAIN_BITS16_1 (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_0_ROI_2_COLUMN_VEC                           Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_USE_CASE_0_ROI_2_COLUMN_VEC_BITS15_0 (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_USE_CASE_0_ROI_2_COLUMN_VEC_BITS15_0 (16U)          /* No description provided */
#define BITM_DE_REGS_YODA_USE_CASE_0_ROI_2_COLUMN_VEC_BITS15_0 (0XFFFFU)      /* No description provided */

/* ----------------------------------------------------------------------------------------------------
          USE_CASE_0_ROI_2_ROI_TYPE                             Value             Description
   ---------------------------------------------------------------------------------------------------- */
#define BITP_DE_REGS_YODA_USE_CASE_0_ROI_2_ROI_TYPE_BINSS     (0U)           /* No description provided */
#define BITL_DE_REGS_YODA_USE_CASE_0_ROI_2_ROI_TYPE_BINSS     (4U)           /* No description provided */
#define BITM_DE_REGS_YODA_USE_CASE_0_ROI_2_ROI_TYPE_BINSS     (0X000FU)      /* No description provided */

#endif  /* end ifndef DE_REGS_YODA_ADDR_RDEF_H_ */

#endif /* NEWTON_ADDR_RDEF_H */

