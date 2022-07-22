
/* ================================================================================
     Created by  :   nguthrie
     Created on  :   2020 Apr 03, 08:29 EDT

     Project     :   newton
     File        :   newton_addr_def.h
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

#ifndef NEWTON_ADDR_DEF_H
#define NEWTON_ADDR_DEF_H


#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

/* ====================================================================================================
        USEQ_REGS_MAP1 Module Instances Register Address Defintions
   ==================================================================================================== */
#define ADDR_USEQ_REGS_SEQUENCESTARTADDR                         (0X00000008U)    /* Sequence Start Address Register */
#define ADDR_USEQ_REGS_SEQUENCEENDADDR                           (0X0000000AU)    /* Sequence End Address Register */
#define ADDR_USEQ_REGS_USEQCONTROLREGISTER                       (0X0000000CU)    /* Useq control Register */
#define ADDR_USEQ_REGS_FRAMESYNCCTRL                             (0X0000000EU)    /* FrameSync Control Register */
#define ADDR_USEQ_REGS_BREAKPOINTCTRL                            (0X00000010U)    /* The breakpoint feature is used by the software for debugging purposes. */
#define ADDR_USEQ_REGS_UPDATESTAMP                               (0X00000012U)    /* Update Stamp Value Register */
#define ADDR_USEQ_REGS_DIGPWRDOWN                                (0X00000014U)    /* Controls clock gates for various digital circuit power-down */
#define ADDR_USEQ_REGS_PIXGAINTAG1LATCHCTRL                      (0X00000016U)    /* DEPRECATED: This register is deprecated and should not be used. */
#define ADDR_USEQ_REGS_PIXGAINTAG1READOUTCTRL                    (0X00000018U)    /* DEPRECATED: This register is deprecated and should not be used. */
#define ADDR_USEQ_REGS_PIXSATURATELATCHCTRL                      (0X0000001AU)    /* DEPRECATED: This register is deprecated and should not be used. */
#define ADDR_USEQ_REGS_PIXSATURATEREADOUTCTRL                    (0X0000001CU)    /* DEPRECATED: This register is deprecated and should not be used. */
#define ADDR_USEQ_REGS_ROWCNTINCRCONTROL                         (0X0000001EU)    /* DEPRECATED: This register is deprecated and should not be used. */
#define ADDR_USEQ_REGS_PIXGAINTAG0LATCHCTRL                      (0X00000020U)    /* DEPRECATED: This register is deprecated and should not be used. */
#define ADDR_USEQ_REGS_PIXGAINTAG0READOUTCTRL                    (0X00000022U)    /* DEPRECATED: This register is deprecated and should not be used. */
#define ADDR_USEQ_REGS_I2CCTRL                                   (0X00000024U)    /* Configures Read prefetching on I2C reads */
#define ADDR_USEQ_REGS_SEQUENCESTATUS                            (0X00000026U)    /* Sequence Status Register */
#define ADDR_USEQ_REGS_SYSTEMCLOCKCONTROL                        (0X00000028U)    /* Clock Control Register */
#define ADDR_USEQ_REGS_CALLRPTCOUNT                              (0X0000002AU)    /* Call Repeat Count Value */
#define ADDR_USEQ_REGS_GTSWAP                                    (0X0000002CU)    /* DEPRECATED: This register is deprecated and should not be used. */
#define ADDR_USEQ_REGS_INTERRUPTENABLE                           (0X0000002EU)    /* Interrupt Enable Register */
#define ADDR_USEQ_REGS_ERRORSET                                  (0X00000030U)    /* Error Set Register */
#define ADDR_USEQ_REGS_ERRORSTATUS                               (0X00000032U)    /* Error Status Register */
#define ADDR_USEQ_REGS_GPIOCTRL                                  (0X00000034U)    /* No description provided */
#define ADDR_USEQ_REGS_GPIOINPUT                                 (0X00000036U)    /* No description provided */
#define ADDR_USEQ_REGS_GPIOOUTPUTSET                             (0X00000038U)    /* No description provided */
#define ADDR_USEQ_REGS_GPIOOUTPUTCLR                             (0X0000003AU)    /* No description provided */
#define ADDR_USEQ_REGS_PIXELINTERFACECTRL                        (0X0000003CU)    /* No description provided */
#define ADDR_USEQ_REGS_ROWCNTINCRCONTROL2                        (0X0000003EU)    /* No description provided */
#define ADDR_USEQ_REGS_GPIOFSYNCSNAPSHOT                         (0X00000040U)    /* No description provided */
#define ADDR_USEQ_REGS_WAITFORSYNCSOURCE                         (0X00000044U)    /* No description provided */
#define ADDR_USEQ_REGS_CTIMECTRL                                 (0X00000046U)    /* No description provided */
#define ADDR_USEQ_REGS_CTIME_0                                   (0X00000048U)    /* No description provided */
#define ADDR_USEQ_REGS_CTIME_1                                   (0X0000004AU)    /* No description provided */
#define ADDR_USEQ_REGS_CTIME_2                                   (0X0000004CU)    /* No description provided */
#define ADDR_USEQ_REGS_CTIME_3                                   (0X0000004EU)    /* No description provided */
#define ADDR_USEQ_REGS_CTIME_4                                   (0X00000050U)    /* No description provided */
#define ADDR_USEQ_REGS_CTIME_5                                   (0X00000052U)    /* No description provided */
#define ADDR_USEQ_REGS_SOFT_RESET                                (0X00000054U)    /* No description provided */
#define ADDR_USEQ_REGS_WAITFORSYNCPOLARITY                       (0X00000056U)    /* No description provided */
#define ADDR_USEQ_REGS_USEQ_DFT                                  (0X00000058U)    /* MIcrosequencer ram DFT controls */
#define ADDR_USEQ_REGS_USEQ_PARITY                               (0X0000005AU)    /* Microsequener parity error bits */
#define ADDR_USEQ_REGS_HSP_DFT                                   (0X0000005CU)    /* HSP RAM DFT controls */
#define ADDR_USEQ_REGS_PCCOND                                    (0X0000005EU)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR0                                     (0X00000060U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR1                                     (0X00000062U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR2                                     (0X00000064U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR3                                     (0X00000066U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR4                                     (0X00000068U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR5                                     (0X0000006AU)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR6                                     (0X0000006CU)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR7                                     (0X0000006EU)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR8                                     (0X00000070U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR9                                     (0X00000072U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR10                                    (0X00000074U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR11                                    (0X00000076U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR12                                    (0X00000078U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR13                                    (0X0000007AU)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR14                                    (0X0000007CU)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR15                                    (0X0000007EU)    /* No description provided */
#define ADDR_USEQ_REGS_AMPCLKCTRL                                (0X00000080U)    /* No description provided */
#define ADDR_USEQ_REGS_AMPCLK2CTRL                               (0X00000082U)    /* No description provided */
#define ADDR_USEQ_REGS_AMPCLK3CTRL1                              (0X00000084U)    /* No description provided */
#define ADDR_USEQ_REGS_AMPCLK3CTRL2                              (0X00000086U)    /* No description provided */
#define ADDR_USEQ_REGS_NOISERESETCTRL1                           (0X00000088U)    /* No description provided */
#define ADDR_USEQ_REGS_NOISERESETCTRL2                           (0X0000008AU)    /* No description provided */
#define ADDR_USEQ_REGS_PIXRESETCTRL1                             (0X0000008CU)    /* No description provided */
#define ADDR_USEQ_REGS_PIXRESETCTRL2                             (0X0000008EU)    /* No description provided */
#define ADDR_USEQ_REGS_GPIOPINFUNC1                              (0X00000090U)    /* No description provided */
#define ADDR_USEQ_REGS_GPIOPINFUNC2                              (0X00000092U)    /* No description provided */
#define ADDR_USEQ_REGS_USEQ_DBGMUX                               (0X00000094U)    /* No description provided */
#define ADDR_USEQ_REGS_USEQ_CHIP_DBGMUX                          (0X00000096U)    /* No description provided */
#define ADDR_USEQ_REGS_MM_CTRL                                   (0X00000098U)    /* No description provided */
#define ADDR_USEQ_REGS_ERRJMPADDR                                (0X0000009CU)    /* No description provided */
#define ADDR_USEQ_REGS_STOPERRENA                                (0X0000009EU)    /* No description provided */
#define ADDR_USEQ_REGS_ADCCNVTCTRL1                              (0X000000A0U)    /* No description provided */
#define ADDR_USEQ_REGS_ADCCNVTCTRL2                              (0X000000A2U)    /* No description provided */
#define ADDR_USEQ_REGS_ADCCNVTCTRL3                              (0X000000A4U)    /* No description provided */
#define ADDR_USEQ_REGS_ADCCNVTCTRL4                              (0X000000A6U)    /* No description provided */
#define ADDR_USEQ_REGS_GAINTAG1CLKCTRL1                          (0X000000A8U)    /* No description provided */
#define ADDR_USEQ_REGS_GAINTAG1CLKCTRL2                          (0X000000AAU)    /* No description provided */
#define ADDR_USEQ_REGS_GAINTAGTHRESHCTRL1                        (0X000000ACU)    /* No description provided */
#define ADDR_USEQ_REGS_GAINTAGTHRESHCTRL2                        (0X000000AEU)    /* No description provided */
#define ADDR_USEQ_REGS_GAINTAGTHRESHSEL                          (0X000000B2U)    /* No description provided */
#define ADDR_USEQ_REGS_GAINTAG0CLKCTRL1                          (0X000000B4U)    /* No description provided */
#define ADDR_USEQ_REGS_GAINTAG0CLKCTRL2                          (0X000000B6U)    /* No description provided */
#define ADDR_USEQ_REGS_FORCESFCTRL1                              (0X000000B8U)    /* No description provided */
#define ADDR_USEQ_REGS_FORCESFCTRL2                              (0X000000BAU)    /* No description provided */
#define ADDR_USEQ_REGS_FORCEIPDACTRL1                            (0X000000BCU)    /* No description provided */
#define ADDR_USEQ_REGS_FORCEIPDACTRL2                            (0X000000BEU)    /* No description provided */
#define ADDR_USEQ_REGS_USEQRAMLOADADDR                           (0X000000C0U)    /* Useq Ram Load start Address Register */
#define ADDR_USEQ_REGS_USEQRAMRDSTADDR                           (0X000000C2U)    /* Useq Ram Read start Address Register */
#define ADDR_USEQ_REGS_USEQRAMLOADDATA                           (0X000000C4U)    /* Useq Ram Load Data Register */
#define ADDR_USEQ_REGS_USEQRAMLOADDATAALIAS                      (0X000000C6U)    /* Useq Ram Load Data Register */
#define ADDR_USEQ_REGS_USEQRAMRDDATA                             (0X000000C8U)    /* Useq Ram Read Data Register */
#define ADDR_USEQ_REGS_USEQRAMRDDATAALIAS                        (0X000000CAU)    /* Useq Ram Read Data Register */
#define ADDR_USEQ_REGS_PWM_CTRL_0                                (0X000000CCU)    /* No description provided */
#define ADDR_USEQ_REGS_PWM_CTRL_1                                (0X000000CEU)    /* No description provided */
#define ADDR_USEQ_REGS_FSYNCCTRL                                 (0X000000D0U)    /* No description provided */
#define ADDR_USEQ_REGS_FSYNCSTATUS                               (0X000000D2U)    /* No description provided */
#define ADDR_USEQ_REGS_FSYNCLSMODCNTR_0                          (0X000000D4U)    /* No description provided */
#define ADDR_USEQ_REGS_FSYNCLSMODCNTR_1                          (0X000000D6U)    /* No description provided */
#define ADDR_USEQ_REGS_FSYNCINTCNTR_0                            (0X000000D8U)    /* No description provided */
#define ADDR_USEQ_REGS_FSYNCINTCNTR_1                            (0X000000DAU)    /* No description provided */
#define ADDR_USEQ_REGS_FSYNCSYSCNTR_0                            (0X000000DCU)    /* No description provided */
#define ADDR_USEQ_REGS_FSYNCSYSCNTR_1                            (0X000000DEU)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR16                                    (0X000000E0U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR17                                    (0X000000E2U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR18                                    (0X000000E4U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR19                                    (0X000000E6U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR20                                    (0X000000E8U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR21                                    (0X000000EAU)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR22                                    (0X000000ECU)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR23                                    (0X000000EEU)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR24                                    (0X000000F0U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR25                                    (0X000000F2U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR26                                    (0X000000F4U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR27                                    (0X000000F6U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR28                                    (0X000000F8U)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR29                                    (0X000000FAU)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR30                                    (0X000000FCU)    /* No description provided */
#define ADDR_USEQ_REGS_GPRR31                                    (0X000000FEU)    /* No description provided */

/* ====================================================================================================
        AI_REGS_YODA Module Instances Register Address Defintions
   ==================================================================================================== */
#define ADDR_AI_REGS_ADC_CTRL0_S1                                (0X00000100U)    /* No description provided */
#define ADDR_AI_REGS_ADC_CTRL1_S1                                (0X00000102U)    /* No description provided */
#define ADDR_AI_REGS_ADC_CTRL2_S1                                (0X00000104U)    /* No description provided */
#define ADDR_AI_REGS_ADCPLL_CTRL0_S1                             (0X00000106U)    /* No description provided */
#define ADDR_AI_REGS_ADCPLL_CTRL1_S1                             (0X00000108U)    /* No description provided */
#define ADDR_AI_REGS_ADCPLL_CTRL2_S1                             (0X0000010AU)    /* No description provided */
#define ADDR_AI_REGS_AMP_CTRL0_S1                                (0X0000010CU)    /* No description provided */
#define ADDR_AI_REGS_AMP_CTRL1_S1                                (0X0000010EU)    /* No description provided */
#define ADDR_AI_REGS_AMP_CTRL2_S1                                (0X00000110U)    /* No description provided */
#define ADDR_AI_REGS_CHIP_ID                                     (0X00000112U)    /* No description provided */
#define ADDR_AI_REGS_CKGEN_CTRL                                  (0X00000114U)    /* No description provided */
#define ADDR_AI_REGS_CKGEN_S1                                    (0X00000116U)    /* No description provided */
#define ADDR_AI_REGS_CLK_CTRL                                    (0X00000118U)    /* No description provided */
#define ADDR_AI_REGS_CLK_DE_CTRL_S1                              (0X0000011AU)    /* No description provided */
#define ADDR_AI_REGS_CLK_LVDSTX_S1                               (0X0000011CU)    /* No description provided */
#define ADDR_AI_REGS_CLKTREE0                                    (0X0000011EU)    /* No description provided */
#define ADDR_AI_REGS_CLKTREE_S1                                  (0X00000122U)    /* No description provided */
#define ADDR_AI_REGS_DAC_CTRL1                                   (0X00000126U)    /* No description provided */
#define ADDR_AI_REGS_DAC_CTRL2                                   (0X00000128U)    /* No description provided */
#define ADDR_AI_REGS_DAC_CTRL0_S1                                (0X0000012AU)    /* No description provided */
#define ADDR_AI_REGS_DAC_CTRL1_S1                                (0X0000012CU)    /* No description provided */
#define ADDR_AI_REGS_DAC_CTRL2_S1                                (0X0000012EU)    /* No description provided */
#define ADDR_AI_REGS_DAC_CTRL3_S1                                (0X00000130U)    /* No description provided */
#define ADDR_AI_REGS_DAC_DATA                                    (0X00000132U)    /* No description provided */
#define ADDR_AI_REGS_IPDA_CTRL_S1                                (0X00000134U)    /* No description provided */
#define ADDR_AI_REGS_LS_LVDSTX_S1                                (0X00000136U)    /* No description provided */
#define ADDR_AI_REGS_LSCTRL0_S1                                  (0X00000138U)    /* No description provided */
#define ADDR_AI_REGS_LSMOD_EN                                    (0X0000013AU)    /* No description provided */
#define ADDR_AI_REGS_ROW_CTRL                                    (0X0000013EU)    /* No description provided */
#define ADDR_AI_REGS_PLL_CTRL                                    (0X00000140U)    /* No description provided */
#define ADDR_AI_REGS_PLL_STATUS                                  (0X00000142U)    /* No description provided */
#define ADDR_AI_REGS_POWER_DOWN_0                                (0X00000144U)    /* No description provided */
#define ADDR_AI_REGS_POWER_DOWN_ADC_OTHERS                       (0X00000146U)    /* No description provided */
#define ADDR_AI_REGS_POWER_DOWN_READOUT                          (0X00000148U)    /* No description provided */
#define ADDR_AI_REGS_PUMP_S1                                     (0X0000014CU)    /* No description provided */
#define ADDR_AI_REGS_READOUT_S1                                  (0X0000014EU)    /* No description provided */
#define ADDR_AI_REGS_REGIF_CTRL                                  (0X00000150U)    /* No description provided */
#define ADDR_AI_REGS_REGIF_RDATA                                 (0X00000152U)    /* No description provided */
#define ADDR_AI_REGS_SSPLL_CTRL0_S1                              (0X00000154U)    /* No description provided */
#define ADDR_AI_REGS_SSPLL_CTRL1_S1                              (0X00000156U)    /* No description provided */
#define ADDR_AI_REGS_SSPLL_CTRL2_S1                              (0X00000158U)    /* No description provided */
#define ADDR_AI_REGS_SYSPLL_CTRL0_S1                             (0X0000015AU)    /* No description provided */
#define ADDR_AI_REGS_SYSPLL_CTRL1_S1                             (0X0000015CU)    /* No description provided */
#define ADDR_AI_REGS_SYSPLL_CTRL2_S1                             (0X0000015EU)    /* No description provided */
#define ADDR_AI_REGS_ANA_TEST_MUX_S1                             (0X00000160U)    /* No description provided */
#define ADDR_AI_REGS_TS_CTRL_S1                                  (0X00000162U)    /* No description provided */
#define ADDR_AI_REGS_TS_CTRL                                     (0X00000164U)    /* No description provided */
#define ADDR_AI_REGS_TS_DATA                                     (0X00000166U)    /* No description provided */
#define ADDR_AI_REGS_VLOWENABLE                                  (0X00000168U)    /* No description provided */
#define ADDR_AI_REGS_VLOWREGCTRL0_S2                             (0X0000016AU)    /* No description provided */
#define ADDR_AI_REGS_VLOWREGCTRL1_S2                             (0X0000016CU)    /* No description provided */
#define ADDR_AI_REGS_VLOWREGCTRL2_S2                             (0X0000016EU)    /* No description provided */
#define ADDR_AI_REGS_VLOWREGCTRL3_S2                             (0X00000170U)    /* No description provided */
#define ADDR_AI_REGS_VLOWREGCTRL4_S2                             (0X00000172U)    /* No description provided */
#define ADDR_AI_REGS_VLOWSHOCTRL1                                (0X00000174U)    /* No description provided */
#define ADDR_AI_REGS_VLOWSHOCTRL2                                (0X00000176U)    /* No description provided */
#define ADDR_AI_REGS_VLOWSHOCTRL3                                (0X00000178U)    /* No description provided */
#define ADDR_AI_REGS_VLOWSHODETECT                               (0X0000017AU)    /* No description provided */
#define ADDR_AI_REGS_XOSC_CTRL                                   (0X0000017CU)    /* No description provided */
#define ADDR_AI_REGS_CHAIN1_LEN                                  (0X0000017EU)    /* No description provided */
#define ADDR_AI_REGS_CHAIN2_LEN                                  (0X00000180U)    /* No description provided */
#define ADDR_AI_REGS_MIPITX_CTRL                                 (0X00000182U)    /* No description provided */
#define ADDR_AI_REGS_SSPLL_CTRL3_S1                              (0X00000184U)    /* No description provided */
#define ADDR_AI_REGS_PIXEL_BIAS                                  (0X00000186U)    /* No description provided */
#define ADDR_AI_REGS_DLL_CONTROL                                 (0X00000188U)    /* No description provided */
#define ADDR_AI_REGS_ANA_SPARE_0                                 (0X0000018AU)    /* No description provided */
#define ADDR_AI_REGS_ANA_SPARE_1                                 (0X0000018CU)    /* No description provided */
#define ADDR_AI_REGS_ANA_SERIAL_SPARE_0                          (0X0000018EU)    /* No description provided */
#define ADDR_AI_REGS_ANA_SERIAL_SPARE_1                          (0X00000190U)    /* No description provided */
#define ADDR_AI_REGS_ANA_SERIAL_SPARE_2                          (0X00000192U)    /* No description provided */
#define ADDR_AI_REGS_DEBUG_MUX_CONTROL_REG                       (0X00000194U)    /* No description provided */

/* ====================================================================================================
        USEQ_REGS_MAP2 Module Instances Register Address Defintions
   ==================================================================================================== */
#define ADDR_USEQ_REGS2_SCRATCHPAD_0_                            (0X00000200U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_1_                            (0X00000202U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_2_                            (0X00000204U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_3_                            (0X00000206U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_4_                            (0X00000208U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_5_                            (0X0000020AU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_6_                            (0X0000020CU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_7_                            (0X0000020EU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_8_                            (0X00000210U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_9_                            (0X00000212U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_10_                           (0X00000214U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_11_                           (0X00000216U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_12_                           (0X00000218U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_13_                           (0X0000021AU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_14_                           (0X0000021CU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_15_                           (0X0000021EU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_16_                           (0X00000220U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_17_                           (0X00000222U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_18_                           (0X00000224U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_19_                           (0X00000226U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_20_                           (0X00000228U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_21_                           (0X0000022AU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_22_                           (0X0000022CU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_23_                           (0X0000022EU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_24_                           (0X00000230U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_25_                           (0X00000232U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_26_                           (0X00000234U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_27_                           (0X00000236U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_28_                           (0X00000238U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_29_                           (0X0000023AU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_30_                           (0X0000023CU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_31_                           (0X0000023EU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_32_                           (0X00000240U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_33_                           (0X00000242U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_34_                           (0X00000244U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_35_                           (0X00000246U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_36_                           (0X00000248U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_37_                           (0X0000024AU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_38_                           (0X0000024CU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_39_                           (0X0000024EU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_40_                           (0X00000250U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_41_                           (0X00000252U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_42_                           (0X00000254U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_43_                           (0X00000256U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_44_                           (0X00000258U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_45_                           (0X0000025AU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_46_                           (0X0000025CU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_47_                           (0X0000025EU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_48_                           (0X00000260U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_49_                           (0X00000262U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_50_                           (0X00000264U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_51_                           (0X00000266U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_52_                           (0X00000268U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_53_                           (0X0000026AU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_54_                           (0X0000026CU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_55_                           (0X0000026EU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_56_                           (0X00000270U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_57_                           (0X00000272U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_58_                           (0X00000274U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_59_                           (0X00000276U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_60_                           (0X00000278U)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_61_                           (0X0000027AU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_62_                           (0X0000027CU)    /* No description provided */
#define ADDR_USEQ_REGS2_SCRATCHPAD_63_                           (0X0000027EU)    /* No description provided */
#define ADDR_USEQ_REGS2_CLKGEN_CK1                               (0X00000280U)    /* No description provided */
#define ADDR_USEQ_REGS2_CLKGEN_CK2                               (0X00000282U)    /* No description provided */
#define ADDR_USEQ_REGS2_CLKGEN_CK1REF                            (0X00000284U)    /* No description provided */
#define ADDR_USEQ_REGS2_CLKGEN_CK2REF                            (0X00000286U)    /* No description provided */
#define ADDR_USEQ_REGS2_CLKGEN_L1                                (0X00000288U)    /* No description provided */
#define ADDR_USEQ_REGS2_CLKGEN_L2                                (0X0000028AU)    /* No description provided */
#define ADDR_USEQ_REGS2_CLKGEN_CKX                               (0X0000028CU)    /* No description provided */
#define ADDR_USEQ_REGS2_CLKGEN_CYCLE                             (0X0000028EU)    /* No description provided */
#define ADDR_USEQ_REGS2_CLKGEN_OFFSET                            (0X00000290U)    /* No description provided */
#define ADDR_USEQ_REGS2_CLKGEN_LTOFFSET                          (0X00000292U)    /* No description provided */
#define ADDR_USEQ_REGS2_CLKGEN_BURST_PERIOD                      (0X00000294U)    /* No description provided */

/* ====================================================================================================
        SPIM_REGS Module Instances Register Address Defintions
   ==================================================================================================== */
#define ADDR_SPIM_REGS_CTRLR0                                    (0X00000300U)    /* Control Register 0 */
#define ADDR_SPIM_REGS_CTRLR1                                    (0X00000304U)    /* Control Register 1 */
#define ADDR_SPIM_REGS_SSIENR                                    (0X00000308U)    /* SSI Enable Register */
#define ADDR_SPIM_REGS_MWCR                                      (0X0000030CU)    /* Microwire Control Register */
#define ADDR_SPIM_REGS_SER                                       (0X00000310U)    /* Slave Enable Register */
#define ADDR_SPIM_REGS_BAUDR                                     (0X00000314U)    /* Baud Rate Select */
#define ADDR_SPIM_REGS_TXFTLR                                    (0X00000318U)    /* Transmit FIFO Threshold Level */
#define ADDR_SPIM_REGS_RXFTLR                                    (0X0000031CU)    /* Receive FIFO Threshold Level */
#define ADDR_SPIM_REGS_TXFLR                                     (0X00000320U)    /* Transmit FIFO Level Register */
#define ADDR_SPIM_REGS_RXFLR                                     (0X00000324U)    /* Receive FIFO Level Register */
#define ADDR_SPIM_REGS_SR                                        (0X00000328U)    /* Status Register */
#define ADDR_SPIM_REGS_IMR                                       (0X0000032CU)    /* Interrupt Mask Register */
#define ADDR_SPIM_REGS_ISR                                       (0X00000330U)    /* Interrupt Status Register */
#define ADDR_SPIM_REGS_RISR                                      (0X00000334U)    /* Raw Interrupt Status Register */
#define ADDR_SPIM_REGS_TXOICR                                    (0X00000338U)    /* Transmit FIFO Overflow Interrupt Clear Register Register */
#define ADDR_SPIM_REGS_RXOICR                                    (0X0000033CU)    /* Receive FIFO Overflow Interrupt Clear Register */
#define ADDR_SPIM_REGS_RXUICR                                    (0X00000340U)    /* Receive FIFO Underflow Interrupt Clear Register */
#define ADDR_SPIM_REGS_MSTICR                                    (0X00000344U)    /* Multi-Master Interrupt Clear Register */
#define ADDR_SPIM_REGS_ICR                                       (0X00000348U)    /* Interrupt Clear Register */
#define ADDR_SPIM_REGS_IDR                                       (0X00000358U)    /* Identification Register */
#define ADDR_SPIM_REGS_SSI_VERSION_ID                            (0X0000035CU)    /* Core Kit version ID Register */
#define ADDR_SPIM_REGS_DR0                                       (0X00000360U)    /* DW_apb_ssi Data Register */

/* ====================================================================================================
        CSI2_REGSPEC_TOP_CPU0 Module Instances Register Address Defintions
   ==================================================================================================== */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CFG_NUM_LANES                (0X00000400U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CFG_T_PRE                    (0X00000404U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CFG_T_POST                   (0X00000408U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CFG_TX_GAP                   (0X0000040CU)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CFG_T_CLK_GAP                (0X00000410U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CFG_CONTINUOUS_HS_CLK        (0X00000414U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CFG_TWAKEUP                  (0X00000418U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_ULPS_CLK_ENABLE              (0X0000041CU)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_ULPS_ENABLE                  (0X00000420U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_ULPS_CLK_ACTIVE              (0X00000424U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_ULPS_ACTIVE                  (0X00000428U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_IRQ_STATUS                   (0X0000042CU)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_IRQ_ENABLE                   (0X00000430U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CSI2TX_IRQ_CLR               (0X00000434U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CFG_CLK_LANE_EN              (0X00000438U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CFG_DATA_LANE_EN             (0X0000043CU)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CFG_CPHY_EN                  (0X00000440U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CFG_PPI_16_EN                (0X00000444U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CFG_PACKET_INTERFACE_EN      (0X00000448U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_BASE_CFG_VCX_EN                   (0X0000044CU)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_I       (0X00000450U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_P       (0X00000454U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PLL       (0X00000460U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PLL_CTRL    (0X00000464U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PHY       (0X00000468U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PHY_CTRL    (0X0000046CU)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TST_PLL      (0X00000470U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CN           (0X00000474U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CM           (0X00000478U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CO           (0X0000047CU)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_BYP     (0X00000480U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_BYPASS_PLL    (0X00000484U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_LATCH    (0X00000488U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TX_RCAL      (0X0000048CU)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_AUTO_PD_EN    (0X00000490U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_ENBL    (0X00000494U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK         (0X00000498U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_ZERO    (0X0000049CU)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_ZERO    (0X000004A0U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_TRAIL    (0X000004A4U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_TRAIL    (0X000004A8U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_PREPARE    (0X000004ACU)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_PREPARE    (0X000004B0U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN0    (0X000004B4U)    /* No description provided */
#define ADDR_MIPI_REGS_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN1    (0X000004B8U)    /* No description provided */

/* ====================================================================================================
        EFUSE Module Instances Register Address Defintions
   ==================================================================================================== */
#define ADDR_EFUSE_REGS_PWR_CTRL                                 (0X00000680U)    /* Power Control Register */
#define ADDR_EFUSE_REGS_STATUS                                   (0X00000682U)    /* Program status register */
#define ADDR_EFUSE_REGS_ERR_LOCATION                             (0X00000684U)    /* Error location */
#define ADDR_EFUSE_REGS_TIMING                                   (0X00000686U)    /* Timing Configuration */
#define ADDR_EFUSE_REGS_CHARACTERIZATION                         (0X00000688U)    /* Characterization Options */

/* ====================================================================================================
        LPS_REGS_YODA Module Instances Register Address Defintions
   ==================================================================================================== */
#define ADDR_LPS1_REGS_LPSCTRL                                   (0X00000900U)    /* No description provided */
#define ADDR_LPS1_REGS_LPSWAVEFREQ                               (0X00000902U)    /* No description provided */
#define ADDR_LPS1_REGS_LPSWAVEGENACC                             (0X00000904U)    /* No description provided */
#define ADDR_LPS1_REGS_LPSRAMADDR                                (0X00000906U)    /* No description provided */
#define ADDR_LPS1_REGS_LPSRAMRDCMD                               (0X00000908U)    /* No description provided */
#define ADDR_LPS1_REGS_LPSWAVEGENADDR                            (0X0000090CU)    /* No description provided */
#define ADDR_LPS1_REGS_LPSMARGIN                                 (0X0000090EU)    /* No description provided */
#define ADDR_LPS1_REGS_LPSDBG                                    (0X00000910U)    /* No description provided */
#define ADDR_LPS1_REGS_LPSRAMDATA                                (0X00000920U)    /* No description provided */
#define ADDR_LPS1_REGS_LPSRAMDATA_ALIAS                          (0X00000922U)    /* No description provided */
#define ADDR_LPS2_REGS_LPSCTRL                                   (0X00000A00U)    /* No description provided */
#define ADDR_LPS2_REGS_LPSWAVEFREQ                               (0X00000A02U)    /* No description provided */
#define ADDR_LPS2_REGS_LPSWAVEGENACC                             (0X00000A04U)    /* No description provided */
#define ADDR_LPS2_REGS_LPSRAMADDR                                (0X00000A06U)    /* No description provided */
#define ADDR_LPS2_REGS_LPSRAMRDCMD                               (0X00000A08U)    /* No description provided */
#define ADDR_LPS2_REGS_LPSWAVEGENADDR                            (0X00000A0CU)    /* No description provided */
#define ADDR_LPS2_REGS_LPSMARGIN                                 (0X00000A0EU)    /* No description provided */
#define ADDR_LPS2_REGS_LPSDBG                                    (0X00000A10U)    /* No description provided */
#define ADDR_LPS2_REGS_LPSRAMDATA                                (0X00000A20U)    /* No description provided */
#define ADDR_LPS2_REGS_LPSRAMDATA_ALIAS                          (0X00000A22U)    /* No description provided */

/* ====================================================================================================
        SS_REGS Module Instances Register Address Defintions
   ==================================================================================================== */
#define ADDR_SSWAVEFREQ                                          (0X00000B00U)    /* No description provided */
#define ADDR_SSWAVEPERIOD                                        (0X00000B02U)    /* No description provided */
#define ADDR_SSWAVEAMP                                           (0X00000B04U)    /* No description provided */
#define ADDR_SSPINLO                                             (0X00000B06U)    /* No description provided */
#define ADDR_SSPINHI                                             (0X00000B08U)    /* No description provided */
#define ADDR_SSINTERVLO                                          (0X00000B0AU)    /* No description provided */
#define ADDR_SSINTERVHI                                          (0X00000B0CU)    /* No description provided */
#define ADDR_SSWAVEOFFSET                                        (0X00000B0EU)    /* No description provided */
#define ADDR_SSCTRL                                              (0X00000B10U)    /* No description provided */
#define ADDR_SSMODFREQ                                           (0X00000B12U)    /* No description provided */
#define ADDR_SSMODAMP                                            (0X00000B14U)    /* No description provided */
#define ADDR_SSINTERV_10                                         (0X00000B16U)    /* No description provided */
#define ADDR_SSINTERV_11                                         (0X00000B18U)    /* No description provided */
#define ADDR_SSINTERV_20                                         (0X00000B1AU)    /* No description provided */
#define ADDR_SSINTERV_21                                         (0X00000B1CU)    /* No description provided */
#define ADDR_SSINTERV_30                                         (0X00000B1EU)    /* No description provided */
#define ADDR_SSINTERV_31                                         (0X00000B20U)    /* No description provided */
#define ADDR_SSVALUE_00                                          (0X00000B22U)    /* No description provided */
#define ADDR_SSVALUE_01                                          (0X00000B24U)    /* No description provided */
#define ADDR_SSVALUE_10                                          (0X00000B26U)    /* No description provided */
#define ADDR_SSVALUE_11                                          (0X00000B28U)    /* No description provided */
#define ADDR_SSVALUE_20                                          (0X00000B2AU)    /* No description provided */
#define ADDR_SSVALUE_21                                          (0X00000B2CU)    /* No description provided */
#define ADDR_SSVALUE_30                                          (0X00000B2EU)    /* No description provided */
#define ADDR_SSVALUE_31                                          (0X00000B30U)    /* No description provided */
#define ADDR_SSDBG                                               (0X00000B32U)    /* No description provided */

/* ====================================================================================================
        PCM_REGS_YODA Module Instances Register Address Defintions
   ==================================================================================================== */
#define ADDR_PCM_REGS_PCMCTRL_0                                  (0X00000C00U)    /* No description provided */
#define ADDR_PCM_REGS_PCMCTRL_1                                  (0X00000C02U)    /* No description provided */
#define ADDR_PCM_REGS_PCMOUT                                     (0X00000C04U)    /* No description provided */
#define ADDR_PCM_REGS_OSC_PERIOD_CTRL                            (0X00000C06U)    /* No description provided */
#define ADDR_PCM_REGS_OSC_PERIOD_RD                              (0X00000C08U)    /* No description provided */

/* ====================================================================================================
        DATAPATH Module Instances Register Address Defintions
   ==================================================================================================== */
#define ADDR_DATAPATH_REGS_CORRECTION_CONFIG                     (0X00000D08U)    /* No description provided */
#define ADDR_DATAPATH_REGS_USE_CASE_FRAME_CONFIG                 (0X00000D0AU)    /* No description provided */
#define ADDR_DATAPATH_REGS_USE_CASE_MIPI_PACKET_CONTROL          (0X00000D0CU)    /* No description provided */
#define ADDR_DATAPATH_REGS_GAIN0                                 (0X00000D0EU)    /* No description provided */
#define ADDR_DATAPATH_REGS_GAIN1                                 (0X00000D10U)    /* No description provided */
#define ADDR_DATAPATH_REGS_GAIN2                                 (0X00000D12U)    /* No description provided */
#define ADDR_DATAPATH_REGS_GAIN3                                 (0X00000D14U)    /* No description provided */
#define ADDR_DATAPATH_REGS_PARITY_GAIN_MEM                       (0X00000D16U)    /* No description provided */
#define ADDR_DATAPATH_REGS_PARITY_LINE_MEM                       (0X00000D18U)    /* No description provided */
#define ADDR_DATAPATH_REGS_PP_LFSR                               (0X00000D1AU)    /* No description provided */
#define ADDR_DATAPATH_REGS_PP_DECODE_ST_1                        (0X00000D1CU)    /* No description provided */
#define ADDR_DATAPATH_REGS_PP_DECODE_ST_2                        (0X00000D1EU)    /* No description provided */
#define ADDR_DATAPATH_REGS_PP_ENCODE_ST                          (0X00000D20U)    /* No description provided */
#define ADDR_DATAPATH_REGS_PP_ENCODE_GT                          (0X00000D22U)    /* No description provided */
#define ADDR_DATAPATH_REGS_DBG_MUX                               (0X00000D24U)    /* No description provided */
#define ADDR_DATAPATH_REGS_GAIN_MARGIN_CONTROL                   (0X00000D26U)    /* No description provided */
#define ADDR_DATAPATH_REGS_LINE_MARGIN_CONTROL                   (0X00000D28U)    /* No description provided */
#define ADDR_DATAPATH_REGS_ROI_ROW_START                         (0X00000D2AU)    /* No description provided */
#define ADDR_DATAPATH_REGS_ROI_HEIGHT                            (0X00000D2CU)    /* No description provided */
#define ADDR_DATAPATH_REGS_ROI_COLUMN_START                      (0X00000D2EU)    /* No description provided */
#define ADDR_DATAPATH_REGS_ROI_WIDTH                             (0X00000D30U)    /* No description provided */
#define ADDR_DATAPATH_REGS_PP_USEQ_WRITE                         (0X00000D32U)    /* No description provided */
#define ADDR_DATAPATH_REGS_PP_ADC_DELAY                          (0X00000D34U)    /* No description provided */
#define ADDR_DATAPATH_REGS_MIPI_BUFF_MARGIN_CONTROL              (0X00000D36U)    /* No description provided */
#define ADDR_DATAPATH_REGS_MIPI_HEADER_WIDTH                     (0X00000D38U)    /* No description provided */
#define ADDR_DATAPATH_REGS_FRAME_NUMBER                          (0X00000D3CU)    /* No description provided */
#define ADDR_DATAPATH_REGS_MICRO_SEQUENCER_FW_VERSION_LSB        (0X00000D3EU)    /* No description provided */
#define ADDR_DATAPATH_REGS_MICRO_SEQUENCER_FW_VERSION_MSB        (0X00000D40U)    /* No description provided */
#define ADDR_DATAPATH_REGS_TS_CAL_VER                            (0X00000D42U)    /* No description provided */
#define ADDR_DATAPATH_REGS_ADC_CAL_VER                           (0X00000D44U)    /* No description provided */
#define ADDR_DATAPATH_REGS_REG_0                                 (0X00000D46U)    /* No description provided */
#define ADDR_DATAPATH_REGS_REG_1                                 (0X00000D48U)    /* No description provided */
#define ADDR_DATAPATH_REGS_REG_2                                 (0X00000D4AU)    /* No description provided */
#define ADDR_DATAPATH_REGS_REG_3                                 (0X00000D4CU)    /* No description provided */
#define ADDR_DATAPATH_REGS_REG_4                                 (0X00000D4EU)    /* No description provided */
#define ADDR_DATAPATH_REGS_REG_5                                 (0X00000D50U)    /* No description provided */
#define ADDR_DATAPATH_REGS_REG_6                                 (0X00000D52U)    /* No description provided */
#define ADDR_DATAPATH_REGS_REG_7                                 (0X00000D54U)    /* No description provided */
#define ADDR_DATAPATH_REGS_PARITY_MIPI_BUFFER                    (0X00000D56U)    /* No description provided */
#define ADDR_DATAPATH_REGS_PACKET_COUNT                          (0X00000D58U)    /* No description provided */
#define ADDR_DATAPATH_REGS_PACKETS_PER_FRAME                     (0X00000D5AU)    /* No description provided */
#define ADDR_DATAPATH_REGS_ROW_VECTOR                            (0X00000D5CU)    /* No description provided */
#define ADDR_DATAPATH_REGS_ROWS_PER_PACKET_OUT                   (0X00000D5EU)    /* No description provided */
#define ADDR_DATAPATH_REGS_MIPI_RD_EN_MAX                        (0X00000D60U)    /* No description provided */
#define ADDR_DATAPATH_REGS_ANALOG_SS                             (0X00000D62U)    /* No description provided */
#define ADDR_DATAPATH_REGS_MIPI_BUFF_PARITY_ERR_CNT              (0X00000D64U)    /* No description provided */
#define ADDR_DATAPATH_REGS_LINE_MEM_PARITY_ERR_CNT               (0X00000D66U)    /* No description provided */
#define ADDR_DATAPATH_REGS_GAIN_MEM_PARITY_ERR_CNT               (0X00000D68U)    /* No description provided */
#define ADDR_DATAPATH_REGS_IA_SELECT                             (0X00000D70U)    /* No description provided */
#define ADDR_DATAPATH_REGS_IA_ADDR_REG                           (0X00000D72U)    /* No description provided */
#define ADDR_DATAPATH_REGS_IA_WRDATA_REG                         (0X00000D74U)    /* No description provided */
#define ADDR_DATAPATH_REGS_IA_WRDATA_REG_ALIAS                   (0X00000D76U)    /* No description provided */
#define ADDR_DATAPATH_REGS_IA_RDDATA_REG                         (0X00000D78U)    /* No description provided */
#define ADDR_DATAPATH_REGS_IA_RDDATA_REG_ALIAS                   (0X00000D7AU)    /* No description provided */
#define ADDR_DATAPATH_REGS_IA_BANK_TYPE                          (0X00000D7CU)    /* No description provided */

/* ====================================================================================================
        DE_REGS_YODA Module Instances Register Address Defintions
   ==================================================================================================== */
#define ADDR_DE_REGS_DE_CONTROL                                  (0X00000E00U)    /* No description provided */
#define ADDR_DE_REGS_BINNED1X2_REPEAT_COUNT                      (0X00000E02U)    /* No description provided */
#define ADDR_DE_REGS_OVERRIDE_DATA_REG1                          (0X00000E0CU)    /* No description provided */
#define ADDR_DE_REGS_OVERRIDE_DATA_REG2                          (0X00000E0EU)    /* No description provided */
#define ADDR_DE_REGS_OVERRIDE_DATA_REG3                          (0X00000E10U)    /* No description provided */
#define ADDR_DE_REGS_BINNED1X2_END                               (0X00000E12U)    /* No description provided */
#define ADDR_DE_REGS_OVERRIDE_SEL_REG1                           (0X00000E14U)    /* No description provided */
#define ADDR_DE_REGS_OVERRIDE_SEL_REG2                           (0X00000E16U)    /* No description provided */
#define ADDR_DE_REGS_OVERRIDE_SEL_REG3                           (0X00000E18U)    /* No description provided */
#define ADDR_DE_REGS_BINNED1X2_START                             (0X00000E1AU)    /* No description provided */
#define ADDR_DE_REGS_AMP_MUX_SEL_EE_LOW                          (0X00000E1CU)    /* No description provided */
#define ADDR_DE_REGS_AMP_MUX_SEL_EE_HIGH                         (0X00000E1EU)    /* No description provided */
#define ADDR_DE_REGS_AMP_MUX_SEL_EO_LOW                          (0X00000E20U)    /* No description provided */
#define ADDR_DE_REGS_AMP_MUX_SEL_EO_HIGH                         (0X00000E22U)    /* No description provided */
#define ADDR_DE_REGS_AMP_MUX_SEL_OE_LOW                          (0X00000E24U)    /* No description provided */
#define ADDR_DE_REGS_AMP_MUX_SEL_OE_HIGH                         (0X00000E26U)    /* No description provided */
#define ADDR_DE_REGS_AMP_MUX_SEL_OO_LOW                          (0X00000E28U)    /* No description provided */
#define ADDR_DE_REGS_AMP_MUX_SEL_OO_HIGH                         (0X00000E2AU)    /* No description provided */
#define ADDR_DE_REGS_AMP_MUX_SEL_SELB_LOW                        (0X00000E2CU)    /* No description provided */
#define ADDR_DE_REGS_AMP_MUX_SEL_SELB_HIGH                       (0X00000E2EU)    /* No description provided */
#define ADDR_DE_REGS_NATIVE_RESOLUTION_START                     (0X00000E30U)    /* No description provided */
#define ADDR_DE_REGS_NATIVE_RESOLUTION_END                       (0X00000E32U)    /* No description provided */
#define ADDR_DE_REGS_NATIVE_RESOLUTION_REPEAT                    (0X00000E34U)    /* No description provided */
#define ADDR_DE_REGS_SUB_SAMPLED_2X_START                        (0X00000E36U)    /* No description provided */
#define ADDR_DE_REGS_SUB_SAMPLED_2X_END                          (0X00000E38U)    /* No description provided */
#define ADDR_DE_REGS_SUB_SAMPLED_2X_REPEAT                       (0X00000E3AU)    /* No description provided */
#define ADDR_DE_REGS_SUB_SAMPLED_4X_START                        (0X00000E3CU)    /* No description provided */
#define ADDR_DE_REGS_SUB_SAMPLED_4X_END                          (0X00000E3EU)    /* No description provided */
#define ADDR_DE_REGS_SUB_SAMPLED_4X_REPEAT                       (0X00000E40U)    /* No description provided */
#define ADDR_DE_REGS_BINNED_START                                (0X00000E42U)    /* No description provided */
#define ADDR_DE_REGS_BINNED_END                                  (0X00000E44U)    /* No description provided */
#define ADDR_DE_REGS_BINNED_REPEAT                               (0X00000E46U)    /* No description provided */
#define ADDR_DE_REGS_DARK_START                                  (0X00000E48U)    /* No description provided */
#define ADDR_DE_REGS_DARK_END                                    (0X00000E4AU)    /* No description provided */
#define ADDR_DE_REGS_DARK_REPEAT                                 (0X00000E4CU)    /* No description provided */
#define ADDR_DE_REGS_PREAMBLE_START                              (0X00000E4EU)    /* No description provided */
#define ADDR_DE_REGS_PREAMBLE_END                                (0X00000E50U)    /* No description provided */
#define ADDR_DE_REGS_PREAMBLE_REPEAT                             (0X00000E52U)    /* No description provided */
#define ADDR_DE_REGS_POSTAMBLE_START                             (0X00000E54U)    /* No description provided */
#define ADDR_DE_REGS_POSTAMBLE_END                               (0X00000E56U)    /* No description provided */
#define ADDR_DE_REGS_POSTAMBLE_REPEAT                            (0X00000E58U)    /* No description provided */
#define ADDR_DE_REGS_ARRAY_INIT_VEC_DARK                         (0X00000E5AU)    /* No description provided */
#define ADDR_DE_REGS_ARRAY_INIT_VEC                              (0X00000E5CU)    /* No description provided */
#define ADDR_DE_REGS_TYPE_OVERRIDE                               (0X00000E5EU)    /* No description provided */
#define ADDR_DE_REGS_MEM_DFT                                     (0X00000E60U)    /* No description provided */
#define ADDR_DE_REGS_DBG_MUX_CONTROL_0                           (0X00000E62U)    /* No description provided */
#define ADDR_DE_REGS_DBG_MUX_CONTROL_1                           (0X00000E64U)    /* No description provided */
#define ADDR_DE_REGS_DBG_MUX_CONTROL_2                           (0X00000E66U)    /* No description provided */
#define ADDR_DE_REGS_DBG_MUX_CONTROL_3                           (0X00000E68U)    /* No description provided */
#define ADDR_DE_REGS_DBG_MUX_CONTROL_4                           (0X00000E6AU)    /* No description provided */
#define ADDR_DE_REGS_DE_IA_SELECT                                (0X00000E70U)    /* No description provided */
#define ADDR_DE_REGS_DE_IA_ADDR_REG                              (0X00000E72U)    /* No description provided */
#define ADDR_DE_REGS_DE_IA_WRDATA_REG                            (0X00000E74U)    /* No description provided */
#define ADDR_DE_REGS_DE_IA_WRDATA_REG_ALIAS                      (0X00000E76U)    /* No description provided */
#define ADDR_DE_REGS_DE_IA_RDDATA_REG                            (0X00000E78U)    /* No description provided */
#define ADDR_DE_REGS_DE_IA_RDDATA_REG_ALIAS                      (0X00000E7AU)    /* No description provided */
#define ADDR_DE_REGS_USE_CASE_0_ROI_0_ROW_VEC_TOPBOT             (0X00000E80U)    /* No description provided */
#define ADDR_DE_REGS_USE_CASE_0_ROI_0_ROW_VEC_MAIN               (0X00000E82U)    /* No description provided */
#define ADDR_DE_REGS_USE_CASE_0_ROI_0_COLUMN_VEC                 (0X00000E84U)    /* No description provided */
#define ADDR_DE_REGS_USE_CASE_0_ROI_0_ROI_TYPE                   (0X00000E86U)    /* No description provided */
#define ADDR_DE_REGS_USE_CASE_0_ROI_1_ROW_VEC_TOPBOT             (0X00000E88U)    /* No description provided */
#define ADDR_DE_REGS_USE_CASE_0_ROI_1_ROW_VEC_MAIN               (0X00000E8AU)    /* No description provided */
#define ADDR_DE_REGS_USE_CASE_0_ROI_1_COLUMN_VEC                 (0X00000E8CU)    /* No description provided */
#define ADDR_DE_REGS_USE_CASE_0_ROI_1_ROI_TYPE                   (0X00000E8EU)    /* No description provided */
#define ADDR_DE_REGS_USE_CASE_0_ROI_2_ROW_VEC_TOPBOT             (0X00000E90U)    /* No description provided */
#define ADDR_DE_REGS_USE_CASE_0_ROI_2_ROW_VEC_MAIN               (0X00000E92U)    /* No description provided */
#define ADDR_DE_REGS_USE_CASE_0_ROI_2_COLUMN_VEC                 (0X00000E94U)    /* No description provided */
#define ADDR_DE_REGS_USE_CASE_0_ROI_2_ROI_TYPE                   (0X00000E96U)    /* No description provided */

#endif /* NEWTON_ADDR_DEF_H */

