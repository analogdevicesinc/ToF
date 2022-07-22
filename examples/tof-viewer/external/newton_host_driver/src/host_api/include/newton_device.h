
/* ================================================================================
     Created by  :   nguthrie
     Created on  :   2020 Apr 03, 08:29 EDT

     Project     :   newton
     File        :   newton_device.h
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

#ifndef NEWTON_DEVICE_H
#define NEWTON_DEVICE_H

/* pickup integer types */
#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

/* pickup register bitfield and bit masks */
#include "newton_typedefs.h"

#if defined ( __CC_ARM   )
#pragma push
#pragma anon_unions
#endif

#ifndef ADI_USEQ_REGS_MAP1_DEVICE_H_
#define ADI_USEQ_REGS_MAP1_DEVICE_H_

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_TypeDef
 *  \brief Your module description, here.
 *  ======================================================================== */
typedef struct _ADI_USEQ_REGS_MAP1_TypeDef
{
    volatile const uint16_t RESERVED0[8];                   
    volatile       uint16_t SEQUENCESTARTADDR;               /**< 8 Sequence Start Address Register */
    volatile const uint16_t RESERVED1;                      
    volatile       uint16_t SEQUENCEENDADDR;                 /**< A Sequence End Address Register */
    volatile const uint16_t RESERVED2;                      
    volatile       uint16_t USEQCONTROLREGISTER;             /**< C Useq control Register */
    volatile const uint16_t RESERVED3;                      
    volatile       uint16_t FRAMESYNCCTRL;                   /**< E FrameSync Control Register */
    volatile const uint16_t RESERVED4;                      
    volatile       uint16_t BREAKPOINTCTRL;                  /**< 10 The breakpoint feature is used by the software for debugging purposes. */
    volatile const uint16_t RESERVED5;                      
    volatile       uint16_t UPDATESTAMP;                     /**< 12 Update Stamp Value Register */
    volatile const uint16_t RESERVED6;                      
    volatile       uint16_t DIGPWRDOWN;                      /**< 14 Controls clock gates for various digital circuit power-down */
    volatile const uint16_t RESERVED7;                      
    volatile       uint16_t PIXGAINTAG1LATCHCTRL;            /**< 16 DEPRECATED: This register is deprecated and should not be used. */
    volatile const uint16_t RESERVED8;                      
    volatile       uint16_t PIXGAINTAG1READOUTCTRL;          /**< 18 DEPRECATED: This register is deprecated and should not be used. */
    volatile const uint16_t RESERVED9;                      
    volatile       uint16_t PIXSATURATELATCHCTRL;            /**< 1A DEPRECATED: This register is deprecated and should not be used. */
    volatile const uint16_t RESERVED10;                     
    volatile       uint16_t PIXSATURATEREADOUTCTRL;          /**< 1C DEPRECATED: This register is deprecated and should not be used. */
    volatile const uint16_t RESERVED11;                     
    volatile       uint16_t ROWCNTINCRCONTROL;               /**< 1E DEPRECATED: This register is deprecated and should not be used. */
    volatile const uint16_t RESERVED12;                     
    volatile       uint16_t PIXGAINTAG0LATCHCTRL;            /**< 20 DEPRECATED: This register is deprecated and should not be used. */
    volatile const uint16_t RESERVED13;                     
    volatile       uint16_t PIXGAINTAG0READOUTCTRL;          /**< 22 DEPRECATED: This register is deprecated and should not be used. */
    volatile const uint16_t RESERVED14;                     
    volatile       uint16_t I2CCTRL;                         /**< 24 Configures Read prefetching on I2C reads */
    volatile const uint16_t RESERVED15;                     
    volatile       uint16_t SEQUENCESTATUS;                  /**< 26 Sequence Status Register */
    volatile const uint16_t RESERVED16;                     
    volatile       uint16_t SYSTEMCLOCKCONTROL;              /**< 28 Clock Control Register */
    volatile const uint16_t RESERVED17;                     
    volatile       uint16_t CALLRPTCOUNT;                    /**< 2A Call Repeat Count Value */
    volatile const uint16_t RESERVED18;                     
    volatile       uint16_t GTSWAP;                          /**< 2C DEPRECATED: This register is deprecated and should not be used. */
    volatile const uint16_t RESERVED19;                     
    volatile       uint16_t INTERRUPTENABLE;                 /**< 2E Interrupt Enable Register */
    volatile const uint16_t RESERVED20;                     
    volatile       uint16_t ERRORSET;                        /**< 30 Error Set Register */
    volatile const uint16_t RESERVED21;                     
    volatile       uint16_t ERRORSTATUS;                     /**< 32 Error Status Register */
    volatile const uint16_t RESERVED22;                     
    volatile       uint16_t GPIOCTRL;                        /**< 34 No description provided */
    volatile const uint16_t RESERVED23;                     
    volatile       uint16_t GPIOINPUT;                       /**< 36 No description provided */
    volatile const uint16_t RESERVED24;                     
    volatile       uint16_t GPIOOUTPUTSET;                   /**< 38 No description provided */
    volatile const uint16_t RESERVED25;                     
    volatile       uint16_t GPIOOUTPUTCLR;                   /**< 3A No description provided */
    volatile const uint16_t RESERVED26;                     
    volatile       uint16_t PIXELINTERFACECTRL;              /**< 3C No description provided */
    volatile const uint16_t RESERVED27;                     
    volatile       uint16_t ROWCNTINCRCONTROL2;              /**< 3E No description provided */
    volatile const uint16_t RESERVED28;                     
    volatile       uint16_t GPIOFSYNCSNAPSHOT;               /**< 40 No description provided */
    volatile const uint16_t RESERVED29[3];                  
    volatile       uint16_t WAITFORSYNCSOURCE;               /**< 44 No description provided */
    volatile const uint16_t RESERVED30;                     
    volatile       uint16_t CTIMECTRL;                       /**< 46 No description provided */
    volatile const uint16_t RESERVED31;                     
    volatile       uint16_t CTIME_0;                         /**< 48 No description provided */
    volatile const uint16_t RESERVED32;                     
    volatile       uint16_t CTIME_1;                         /**< 4A No description provided */
    volatile const uint16_t RESERVED33;                     
    volatile       uint16_t CTIME_2;                         /**< 4C No description provided */
    volatile const uint16_t RESERVED34;                     
    volatile       uint16_t CTIME_3;                         /**< 4E No description provided */
    volatile const uint16_t RESERVED35;                     
    volatile       uint16_t CTIME_4;                         /**< 50 No description provided */
    volatile const uint16_t RESERVED36;                     
    volatile       uint16_t CTIME_5;                         /**< 52 No description provided */
    volatile const uint16_t RESERVED37;                     
    volatile       uint16_t SOFT_RESET;                      /**< 54 No description provided */
    volatile const uint16_t RESERVED38;                     
    volatile       uint16_t WAITFORSYNCPOLARITY;             /**< 56 No description provided */
    volatile const uint16_t RESERVED39;                     
    volatile       uint16_t USEQ_DFT;                        /**< 58 MIcrosequencer ram DFT controls */
    volatile const uint16_t RESERVED40;                     
    volatile       uint16_t USEQ_PARITY;                     /**< 5A Microsequener parity error bits */
    volatile const uint16_t RESERVED41;                     
    volatile       uint16_t HSP_DFT;                         /**< 5C HSP RAM DFT controls */
    volatile const uint16_t RESERVED42;                     
    volatile       uint16_t PCCOND;                          /**< 5E No description provided */
    volatile const uint16_t RESERVED43;                     
    volatile       uint16_t GPRR0;                           /**< 60 No description provided */
    volatile const uint16_t RESERVED44;                     
    volatile       uint16_t GPRR1;                           /**< 62 No description provided */
    volatile const uint16_t RESERVED45;                     
    volatile       uint16_t GPRR2;                           /**< 64 No description provided */
    volatile const uint16_t RESERVED46;                     
    volatile       uint16_t GPRR3;                           /**< 66 No description provided */
    volatile const uint16_t RESERVED47;                     
    volatile       uint16_t GPRR4;                           /**< 68 No description provided */
    volatile const uint16_t RESERVED48;                     
    volatile       uint16_t GPRR5;                           /**< 6A No description provided */
    volatile const uint16_t RESERVED49;                     
    volatile       uint16_t GPRR6;                           /**< 6C No description provided */
    volatile const uint16_t RESERVED50;                     
    volatile       uint16_t GPRR7;                           /**< 6E No description provided */
    volatile const uint16_t RESERVED51;                     
    volatile       uint16_t GPRR8;                           /**< 70 No description provided */
    volatile const uint16_t RESERVED52;                     
    volatile       uint16_t GPRR9;                           /**< 72 No description provided */
    volatile const uint16_t RESERVED53;                     
    volatile       uint16_t GPRR10;                          /**< 74 No description provided */
    volatile const uint16_t RESERVED54;                     
    volatile       uint16_t GPRR11;                          /**< 76 No description provided */
    volatile const uint16_t RESERVED55;                     
    volatile       uint16_t GPRR12;                          /**< 78 No description provided */
    volatile const uint16_t RESERVED56;                     
    volatile       uint16_t GPRR13;                          /**< 7A No description provided */
    volatile const uint16_t RESERVED57;                     
    volatile       uint16_t GPRR14;                          /**< 7C No description provided */
    volatile const uint16_t RESERVED58;                     
    volatile       uint16_t GPRR15;                          /**< 7E No description provided */
    volatile const uint16_t RESERVED59;                     
    volatile       uint16_t AMPCLKCTRL;                      /**< 80 No description provided */
    volatile const uint16_t RESERVED60;                     
    volatile       uint16_t AMPCLK2CTRL;                     /**< 82 No description provided */
    volatile const uint16_t RESERVED61;                     
    volatile       uint16_t AMPCLK3CTRL1;                    /**< 84 No description provided */
    volatile const uint16_t RESERVED62;                     
    volatile       uint16_t AMPCLK3CTRL2;                    /**< 86 No description provided */
    volatile const uint16_t RESERVED63;                     
    volatile       uint16_t NOISERESETCTRL1;                 /**< 88 No description provided */
    volatile const uint16_t RESERVED64;                     
    volatile       uint16_t NOISERESETCTRL2;                 /**< 8A No description provided */
    volatile const uint16_t RESERVED65;                     
    volatile       uint16_t PIXRESETCTRL1;                   /**< 8C No description provided */
    volatile const uint16_t RESERVED66;                     
    volatile       uint16_t PIXRESETCTRL2;                   /**< 8E No description provided */
    volatile const uint16_t RESERVED67;                     
    volatile       uint16_t GPIOPINFUNC1;                    /**< 90 No description provided */
    volatile const uint16_t RESERVED68;                     
    volatile       uint16_t GPIOPINFUNC2;                    /**< 92 No description provided */
    volatile const uint16_t RESERVED69;                     
    volatile       uint16_t USEQ_DBGMUX;                     /**< 94 No description provided */
    volatile const uint16_t RESERVED70;                     
    volatile       uint16_t USEQ_CHIP_DBGMUX;                /**< 96 No description provided */
    volatile const uint16_t RESERVED71;                     
    volatile       uint16_t MM_CTRL;                         /**< 98 No description provided */
    volatile const uint16_t RESERVED72[3];                  
    volatile       uint16_t ERRJMPADDR;                      /**< 9C No description provided */
    volatile const uint16_t RESERVED73;                     
    volatile       uint16_t STOPERRENA;                      /**< 9E No description provided */
    volatile const uint16_t RESERVED74;                     
    volatile       uint16_t ADCCNVTCTRL1;                    /**< A0 No description provided */
    volatile const uint16_t RESERVED75;                     
    volatile       uint16_t ADCCNVTCTRL2;                    /**< A2 No description provided */
    volatile const uint16_t RESERVED76;                     
    volatile       uint16_t ADCCNVTCTRL3;                    /**< A4 No description provided */
    volatile const uint16_t RESERVED77;                     
    volatile       uint16_t ADCCNVTCTRL4;                    /**< A6 No description provided */
    volatile const uint16_t RESERVED78;                     
    volatile       uint16_t GAINTAG1CLKCTRL1;                /**< A8 No description provided */
    volatile const uint16_t RESERVED79;                     
    volatile       uint16_t GAINTAG1CLKCTRL2;                /**< AA No description provided */
    volatile const uint16_t RESERVED80;                     
    volatile       uint16_t GAINTAGTHRESHCTRL1;              /**< AC No description provided */
    volatile const uint16_t RESERVED81;                     
    volatile       uint16_t GAINTAGTHRESHCTRL2;              /**< AE No description provided */
    volatile const uint16_t RESERVED82[3];                  
    volatile       uint16_t GAINTAGTHRESHSEL;                /**< B2 No description provided */
    volatile const uint16_t RESERVED83;                     
    volatile       uint16_t GAINTAG0CLKCTRL1;                /**< B4 No description provided */
    volatile const uint16_t RESERVED84;                     
    volatile       uint16_t GAINTAG0CLKCTRL2;                /**< B6 No description provided */
    volatile const uint16_t RESERVED85;                     
    volatile       uint16_t FORCESFCTRL1;                    /**< B8 No description provided */
    volatile const uint16_t RESERVED86;                     
    volatile       uint16_t FORCESFCTRL2;                    /**< BA No description provided */
    volatile const uint16_t RESERVED87;                     
    volatile       uint16_t FORCEIPDACTRL1;                  /**< BC No description provided */
    volatile const uint16_t RESERVED88;                     
    volatile       uint16_t FORCEIPDACTRL2;                  /**< BE No description provided */
    volatile const uint16_t RESERVED89;                     
    volatile       uint16_t USEQRAMLOADADDR;                 /**< C0 Useq Ram Load start Address Register */
    volatile const uint16_t RESERVED90;                     
    volatile       uint16_t USEQRAMRDSTADDR;                 /**< C2 Useq Ram Read start Address Register */
    volatile const uint16_t RESERVED91;                     
    volatile       uint16_t USEQRAMLOADDATA;                 /**< C4 Useq Ram Load Data Register */
    volatile const uint16_t RESERVED92;                     
    volatile       uint16_t USEQRAMLOADDATAALIAS;            /**< C6 Useq Ram Load Data Register */
    volatile const uint16_t RESERVED93;                     
    volatile       uint16_t USEQRAMRDDATA;                   /**< C8 Useq Ram Read Data Register */
    volatile const uint16_t RESERVED94;                     
    volatile       uint16_t USEQRAMRDDATAALIAS;              /**< CA Useq Ram Read Data Register */
    volatile const uint16_t RESERVED95;                     
    volatile       uint16_t PWM_CTRL_0;                      /**< CC No description provided */
    volatile const uint16_t RESERVED96;                     
    volatile       uint16_t PWM_CTRL_1;                      /**< CE No description provided */
    volatile const uint16_t RESERVED97;                     
    volatile       uint16_t FSYNCCTRL;                       /**< D0 No description provided */
    volatile const uint16_t RESERVED98;                     
    volatile       uint16_t FSYNCSTATUS;                     /**< D2 No description provided */
    volatile const uint16_t RESERVED99;                     
    volatile       uint16_t FSYNCLSMODCNTR_0;                /**< D4 No description provided */
    volatile const uint16_t RESERVED100;                    
    volatile       uint16_t FSYNCLSMODCNTR_1;                /**< D6 No description provided */
    volatile const uint16_t RESERVED101;                    
    volatile       uint16_t FSYNCINTCNTR_0;                  /**< D8 No description provided */
    volatile const uint16_t RESERVED102;                    
    volatile       uint16_t FSYNCINTCNTR_1;                  /**< DA No description provided */
    volatile const uint16_t RESERVED103;                    
    volatile       uint16_t FSYNCSYSCNTR_0;                  /**< DC No description provided */
    volatile const uint16_t RESERVED104;                    
    volatile       uint16_t FSYNCSYSCNTR_1;                  /**< DE No description provided */
    volatile const uint16_t RESERVED105;                    
    volatile       uint16_t GPRR16;                          /**< E0 No description provided */
    volatile const uint16_t RESERVED106;                    
    volatile       uint16_t GPRR17;                          /**< E2 No description provided */
    volatile const uint16_t RESERVED107;                    
    volatile       uint16_t GPRR18;                          /**< E4 No description provided */
    volatile const uint16_t RESERVED108;                    
    volatile       uint16_t GPRR19;                          /**< E6 No description provided */
    volatile const uint16_t RESERVED109;                    
    volatile       uint16_t GPRR20;                          /**< E8 No description provided */
    volatile const uint16_t RESERVED110;                    
    volatile       uint16_t GPRR21;                          /**< EA No description provided */
    volatile const uint16_t RESERVED111;                    
    volatile       uint16_t GPRR22;                          /**< EC No description provided */
    volatile const uint16_t RESERVED112;                    
    volatile       uint16_t GPRR23;                          /**< EE No description provided */
    volatile const uint16_t RESERVED113;                    
    volatile       uint16_t GPRR24;                          /**< F0 No description provided */
    volatile const uint16_t RESERVED114;                    
    volatile       uint16_t GPRR25;                          /**< F2 No description provided */
    volatile const uint16_t RESERVED115;                    
    volatile       uint16_t GPRR26;                          /**< F4 No description provided */
    volatile const uint16_t RESERVED116;                    
    volatile       uint16_t GPRR27;                          /**< F6 No description provided */
    volatile const uint16_t RESERVED117;                    
    volatile       uint16_t GPRR28;                          /**< F8 No description provided */
    volatile const uint16_t RESERVED118;                    
    volatile       uint16_t GPRR29;                          /**< FA No description provided */
    volatile const uint16_t RESERVED119;                    
    volatile       uint16_t GPRR30;                          /**< FC No description provided */
    volatile const uint16_t RESERVED120;                    
    volatile       uint16_t GPRR31;                          /**< FE No description provided */
} ADI_USEQ_REGS_MAP1_TypeDef;


#define ADI_USEQ_REGS_BASE                  (0X00000000U)    /* Base address of useq_regs: */
#define pADI_USEQ_REGS                      ((ADI_USEQ_REGS_MAP1_TypeDef *) ADI_USEQ_REGS_BASE )    /* Pointer to  (useq_regs)*/

#endif  /* end ifndef ADI_USEQ_REGS_MAP1_DEVICE_H_ */

#ifndef ADI_AI_REGS_YODA_DEVICE_H_
#define ADI_AI_REGS_YODA_DEVICE_H_

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_TypeDef
 *  \brief Your module description, here.
 *  ======================================================================== */
typedef struct _ADI_AI_REGS_YODA_TypeDef
{
    volatile       uint16_t ADC_CTRL0_S1;                    /**< 0 No description provided */
    volatile const uint16_t RESERVED0;                      
    volatile       uint16_t ADC_CTRL1_S1;                    /**< 2 No description provided */
    volatile const uint16_t RESERVED1;                      
    volatile       uint16_t ADC_CTRL2_S1;                    /**< 4 No description provided */
    volatile const uint16_t RESERVED2;                      
    volatile       uint16_t ADCPLL_CTRL0_S1;                 /**< 6 No description provided */
    volatile const uint16_t RESERVED3;                      
    volatile       uint16_t ADCPLL_CTRL1_S1;                 /**< 8 No description provided */
    volatile const uint16_t RESERVED4;                      
    volatile       uint16_t ADCPLL_CTRL2_S1;                 /**< A No description provided */
    volatile const uint16_t RESERVED5;                      
    volatile       uint16_t AMP_CTRL0_S1;                    /**< C No description provided */
    volatile const uint16_t RESERVED6;                      
    volatile       uint16_t AMP_CTRL1_S1;                    /**< E No description provided */
    volatile const uint16_t RESERVED7;                      
    volatile       uint16_t AMP_CTRL2_S1;                    /**< 10 No description provided */
    volatile const uint16_t RESERVED8;                      
    volatile       uint16_t CHIP_ID;                         /**< 12 No description provided */
    volatile const uint16_t RESERVED9;                      
    volatile       uint16_t CKGEN_CTRL;                      /**< 14 No description provided */
    volatile const uint16_t RESERVED10;                     
    volatile       uint16_t CKGEN_S1;                        /**< 16 No description provided */
    volatile const uint16_t RESERVED11;                     
    volatile       uint16_t CLK_CTRL;                        /**< 18 No description provided */
    volatile const uint16_t RESERVED12;                     
    volatile       uint16_t CLK_DE_CTRL_S1;                  /**< 1A No description provided */
    volatile const uint16_t RESERVED13;                     
    volatile       uint16_t CLK_LVDSTX_S1;                   /**< 1C No description provided */
    volatile const uint16_t RESERVED14;                     
    volatile       uint16_t CLKTREE0;                        /**< 1E No description provided */
    volatile const uint16_t RESERVED15[3];                  
    volatile       uint16_t CLKTREE_S1;                      /**< 22 No description provided */
    volatile const uint16_t RESERVED16[3];                  
    volatile       uint16_t DAC_CTRL1;                       /**< 26 No description provided */
    volatile const uint16_t RESERVED17;                     
    volatile       uint16_t DAC_CTRL2;                       /**< 28 No description provided */
    volatile const uint16_t RESERVED18;                     
    volatile       uint16_t DAC_CTRL0_S1;                    /**< 2A No description provided */
    volatile const uint16_t RESERVED19;                     
    volatile       uint16_t DAC_CTRL1_S1;                    /**< 2C No description provided */
    volatile const uint16_t RESERVED20;                     
    volatile       uint16_t DAC_CTRL2_S1;                    /**< 2E No description provided */
    volatile const uint16_t RESERVED21;                     
    volatile       uint16_t DAC_CTRL3_S1;                    /**< 30 No description provided */
    volatile const uint16_t RESERVED22;                     
    volatile       uint16_t DAC_DATA;                        /**< 32 No description provided */
    volatile const uint16_t RESERVED23;                     
    volatile       uint16_t IPDA_CTRL_S1;                    /**< 34 No description provided */
    volatile const uint16_t RESERVED24;                     
    volatile       uint16_t LS_LVDSTX_S1;                    /**< 36 No description provided */
    volatile const uint16_t RESERVED25;                     
    volatile       uint16_t LSCTRL0_S1;                      /**< 38 No description provided */
    volatile const uint16_t RESERVED26;                     
    volatile       uint16_t LSMOD_EN;                        /**< 3A No description provided */
    volatile const uint16_t RESERVED27[3];                  
    volatile       uint16_t ROW_CTRL;                        /**< 3E No description provided */
    volatile const uint16_t RESERVED28;                     
    volatile       uint16_t PLL_CTRL;                        /**< 40 No description provided */
    volatile const uint16_t RESERVED29;                     
    volatile       uint16_t PLL_STATUS;                      /**< 42 No description provided */
    volatile const uint16_t RESERVED30;                     
    volatile       uint16_t POWER_DOWN_0;                    /**< 44 No description provided */
    volatile const uint16_t RESERVED31;                     
    volatile       uint16_t POWER_DOWN_ADC_OTHERS;           /**< 46 No description provided */
    volatile const uint16_t RESERVED32;                     
    volatile       uint16_t POWER_DOWN_READOUT;              /**< 48 No description provided */
    volatile const uint16_t RESERVED33[3];                  
    volatile       uint16_t PUMP_S1;                         /**< 4C No description provided */
    volatile const uint16_t RESERVED34;                     
    volatile       uint16_t READOUT_S1;                      /**< 4E No description provided */
    volatile const uint16_t RESERVED35;                     
    volatile       uint16_t REGIF_CTRL;                      /**< 50 No description provided */
    volatile const uint16_t RESERVED36;                     
    volatile       uint16_t REGIF_RDATA;                     /**< 52 No description provided */
    volatile const uint16_t RESERVED37;                     
    volatile       uint16_t SSPLL_CTRL0_S1;                  /**< 54 No description provided */
    volatile const uint16_t RESERVED38;                     
    volatile       uint16_t SSPLL_CTRL1_S1;                  /**< 56 No description provided */
    volatile const uint16_t RESERVED39;                     
    volatile       uint16_t SSPLL_CTRL2_S1;                  /**< 58 No description provided */
    volatile const uint16_t RESERVED40;                     
    volatile       uint16_t SYSPLL_CTRL0_S1;                 /**< 5A No description provided */
    volatile const uint16_t RESERVED41;                     
    volatile       uint16_t SYSPLL_CTRL1_S1;                 /**< 5C No description provided */
    volatile const uint16_t RESERVED42;                     
    volatile       uint16_t SYSPLL_CTRL2_S1;                 /**< 5E No description provided */
    volatile const uint16_t RESERVED43;                     
    volatile       uint16_t ANA_TEST_MUX_S1;                 /**< 60 No description provided */
    volatile const uint16_t RESERVED44;                     
    volatile       uint16_t TS_CTRL_S1;                      /**< 62 No description provided */
    volatile const uint16_t RESERVED45;                     
    volatile       uint16_t TS_CTRL;                         /**< 64 No description provided */
    volatile const uint16_t RESERVED46;                     
    volatile       uint16_t TS_DATA;                         /**< 66 No description provided */
    volatile const uint16_t RESERVED47;                     
    volatile       uint16_t VLOWENABLE;                      /**< 68 No description provided */
    volatile const uint16_t RESERVED48;                     
    volatile       uint16_t VLOWREGCTRL0_S2;                 /**< 6A No description provided */
    volatile const uint16_t RESERVED49;                     
    volatile       uint16_t VLOWREGCTRL1_S2;                 /**< 6C No description provided */
    volatile const uint16_t RESERVED50;                     
    volatile       uint16_t VLOWREGCTRL2_S2;                 /**< 6E No description provided */
    volatile const uint16_t RESERVED51;                     
    volatile       uint16_t VLOWREGCTRL3_S2;                 /**< 70 No description provided */
    volatile const uint16_t RESERVED52;                     
    volatile       uint16_t VLOWREGCTRL4_S2;                 /**< 72 No description provided */
    volatile const uint16_t RESERVED53;                     
    volatile       uint16_t VLOWSHOCTRL1;                    /**< 74 No description provided */
    volatile const uint16_t RESERVED54;                     
    volatile       uint16_t VLOWSHOCTRL2;                    /**< 76 No description provided */
    volatile const uint16_t RESERVED55;                     
    volatile       uint16_t VLOWSHOCTRL3;                    /**< 78 No description provided */
    volatile const uint16_t RESERVED56;                     
    volatile       uint16_t VLOWSHODETECT;                   /**< 7A No description provided */
    volatile const uint16_t RESERVED57;                     
    volatile       uint16_t XOSC_CTRL;                       /**< 7C No description provided */
    volatile const uint16_t RESERVED58;                     
    volatile       uint16_t CHAIN1_LEN;                      /**< 7E No description provided */
    volatile const uint16_t RESERVED59;                     
    volatile       uint16_t CHAIN2_LEN;                      /**< 80 No description provided */
    volatile const uint16_t RESERVED60;                     
    volatile       uint16_t MIPITX_CTRL;                     /**< 82 No description provided */
    volatile const uint16_t RESERVED61;                     
    volatile       uint16_t SSPLL_CTRL3_S1;                  /**< 84 No description provided */
    volatile const uint16_t RESERVED62;                     
    volatile       uint16_t PIXEL_BIAS;                      /**< 86 No description provided */
    volatile const uint16_t RESERVED63;                     
    volatile       uint16_t DLL_CONTROL;                     /**< 88 No description provided */
    volatile const uint16_t RESERVED64;                     
    volatile       uint16_t ANA_SPARE_0;                     /**< 8A No description provided */
    volatile const uint16_t RESERVED65;                     
    volatile       uint16_t ANA_SPARE_1;                     /**< 8C No description provided */
    volatile const uint16_t RESERVED66;                     
    volatile       uint16_t ANA_SERIAL_SPARE_0;              /**< 8E No description provided */
    volatile const uint16_t RESERVED67;                     
    volatile       uint16_t ANA_SERIAL_SPARE_1;              /**< 90 No description provided */
    volatile const uint16_t RESERVED68;                     
    volatile       uint16_t ANA_SERIAL_SPARE_2;              /**< 92 No description provided */
    volatile const uint16_t RESERVED69;                     
    volatile       uint16_t DEBUG_MUX_CONTROL_REG;           /**< 94 No description provided */
} ADI_AI_REGS_YODA_TypeDef;


#define ADI_AI_REGS_BASE                    (0X00000100U)    /* Base address of ai_regs: */
#define pADI_AI_REGS                        ((ADI_AI_REGS_YODA_TypeDef *) ADI_AI_REGS_BASE )    /* Pointer to  (ai_regs)*/

#endif  /* end ifndef ADI_AI_REGS_YODA_DEVICE_H_ */

#ifndef ADI_USEQ_REGS_MAP2_DEVICE_H_
#define ADI_USEQ_REGS_MAP2_DEVICE_H_

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP2_TypeDef
 *  \brief Your module description, here.
 *  ======================================================================== */
typedef struct _ADI_USEQ_REGS_MAP2_TypeDef
{
    volatile       uint16_t SCRATCHPAD_0_;                   /**< 0 No description provided */
    volatile const uint16_t RESERVED0;                      
    volatile       uint16_t SCRATCHPAD_1_;                   /**< 2 No description provided */
    volatile const uint16_t RESERVED1;                      
    volatile       uint16_t SCRATCHPAD_2_;                   /**< 4 No description provided */
    volatile const uint16_t RESERVED2;                      
    volatile       uint16_t SCRATCHPAD_3_;                   /**< 6 No description provided */
    volatile const uint16_t RESERVED3;                      
    volatile       uint16_t SCRATCHPAD_4_;                   /**< 8 No description provided */
    volatile const uint16_t RESERVED4;                      
    volatile       uint16_t SCRATCHPAD_5_;                   /**< A No description provided */
    volatile const uint16_t RESERVED5;                      
    volatile       uint16_t SCRATCHPAD_6_;                   /**< C No description provided */
    volatile const uint16_t RESERVED6;                      
    volatile       uint16_t SCRATCHPAD_7_;                   /**< E No description provided */
    volatile const uint16_t RESERVED7;                      
    volatile       uint16_t SCRATCHPAD_8_;                   /**< 10 No description provided */
    volatile const uint16_t RESERVED8;                      
    volatile       uint16_t SCRATCHPAD_9_;                   /**< 12 No description provided */
    volatile const uint16_t RESERVED9;                      
    volatile       uint16_t SCRATCHPAD_10_;                  /**< 14 No description provided */
    volatile const uint16_t RESERVED10;                     
    volatile       uint16_t SCRATCHPAD_11_;                  /**< 16 No description provided */
    volatile const uint16_t RESERVED11;                     
    volatile       uint16_t SCRATCHPAD_12_;                  /**< 18 No description provided */
    volatile const uint16_t RESERVED12;                     
    volatile       uint16_t SCRATCHPAD_13_;                  /**< 1A No description provided */
    volatile const uint16_t RESERVED13;                     
    volatile       uint16_t SCRATCHPAD_14_;                  /**< 1C No description provided */
    volatile const uint16_t RESERVED14;                     
    volatile       uint16_t SCRATCHPAD_15_;                  /**< 1E No description provided */
    volatile const uint16_t RESERVED15;                     
    volatile       uint16_t SCRATCHPAD_16_;                  /**< 20 No description provided */
    volatile const uint16_t RESERVED16;                     
    volatile       uint16_t SCRATCHPAD_17_;                  /**< 22 No description provided */
    volatile const uint16_t RESERVED17;                     
    volatile       uint16_t SCRATCHPAD_18_;                  /**< 24 No description provided */
    volatile const uint16_t RESERVED18;                     
    volatile       uint16_t SCRATCHPAD_19_;                  /**< 26 No description provided */
    volatile const uint16_t RESERVED19;                     
    volatile       uint16_t SCRATCHPAD_20_;                  /**< 28 No description provided */
    volatile const uint16_t RESERVED20;                     
    volatile       uint16_t SCRATCHPAD_21_;                  /**< 2A No description provided */
    volatile const uint16_t RESERVED21;                     
    volatile       uint16_t SCRATCHPAD_22_;                  /**< 2C No description provided */
    volatile const uint16_t RESERVED22;                     
    volatile       uint16_t SCRATCHPAD_23_;                  /**< 2E No description provided */
    volatile const uint16_t RESERVED23;                     
    volatile       uint16_t SCRATCHPAD_24_;                  /**< 30 No description provided */
    volatile const uint16_t RESERVED24;                     
    volatile       uint16_t SCRATCHPAD_25_;                  /**< 32 No description provided */
    volatile const uint16_t RESERVED25;                     
    volatile       uint16_t SCRATCHPAD_26_;                  /**< 34 No description provided */
    volatile const uint16_t RESERVED26;                     
    volatile       uint16_t SCRATCHPAD_27_;                  /**< 36 No description provided */
    volatile const uint16_t RESERVED27;                     
    volatile       uint16_t SCRATCHPAD_28_;                  /**< 38 No description provided */
    volatile const uint16_t RESERVED28;                     
    volatile       uint16_t SCRATCHPAD_29_;                  /**< 3A No description provided */
    volatile const uint16_t RESERVED29;                     
    volatile       uint16_t SCRATCHPAD_30_;                  /**< 3C No description provided */
    volatile const uint16_t RESERVED30;                     
    volatile       uint16_t SCRATCHPAD_31_;                  /**< 3E No description provided */
    volatile const uint16_t RESERVED31;                     
    volatile       uint16_t SCRATCHPAD_32_;                  /**< 40 No description provided */
    volatile const uint16_t RESERVED32;                     
    volatile       uint16_t SCRATCHPAD_33_;                  /**< 42 No description provided */
    volatile const uint16_t RESERVED33;                     
    volatile       uint16_t SCRATCHPAD_34_;                  /**< 44 No description provided */
    volatile const uint16_t RESERVED34;                     
    volatile       uint16_t SCRATCHPAD_35_;                  /**< 46 No description provided */
    volatile const uint16_t RESERVED35;                     
    volatile       uint16_t SCRATCHPAD_36_;                  /**< 48 No description provided */
    volatile const uint16_t RESERVED36;                     
    volatile       uint16_t SCRATCHPAD_37_;                  /**< 4A No description provided */
    volatile const uint16_t RESERVED37;                     
    volatile       uint16_t SCRATCHPAD_38_;                  /**< 4C No description provided */
    volatile const uint16_t RESERVED38;                     
    volatile       uint16_t SCRATCHPAD_39_;                  /**< 4E No description provided */
    volatile const uint16_t RESERVED39;                     
    volatile       uint16_t SCRATCHPAD_40_;                  /**< 50 No description provided */
    volatile const uint16_t RESERVED40;                     
    volatile       uint16_t SCRATCHPAD_41_;                  /**< 52 No description provided */
    volatile const uint16_t RESERVED41;                     
    volatile       uint16_t SCRATCHPAD_42_;                  /**< 54 No description provided */
    volatile const uint16_t RESERVED42;                     
    volatile       uint16_t SCRATCHPAD_43_;                  /**< 56 No description provided */
    volatile const uint16_t RESERVED43;                     
    volatile       uint16_t SCRATCHPAD_44_;                  /**< 58 No description provided */
    volatile const uint16_t RESERVED44;                     
    volatile       uint16_t SCRATCHPAD_45_;                  /**< 5A No description provided */
    volatile const uint16_t RESERVED45;                     
    volatile       uint16_t SCRATCHPAD_46_;                  /**< 5C No description provided */
    volatile const uint16_t RESERVED46;                     
    volatile       uint16_t SCRATCHPAD_47_;                  /**< 5E No description provided */
    volatile const uint16_t RESERVED47;                     
    volatile       uint16_t SCRATCHPAD_48_;                  /**< 60 No description provided */
    volatile const uint16_t RESERVED48;                     
    volatile       uint16_t SCRATCHPAD_49_;                  /**< 62 No description provided */
    volatile const uint16_t RESERVED49;                     
    volatile       uint16_t SCRATCHPAD_50_;                  /**< 64 No description provided */
    volatile const uint16_t RESERVED50;                     
    volatile       uint16_t SCRATCHPAD_51_;                  /**< 66 No description provided */
    volatile const uint16_t RESERVED51;                     
    volatile       uint16_t SCRATCHPAD_52_;                  /**< 68 No description provided */
    volatile const uint16_t RESERVED52;                     
    volatile       uint16_t SCRATCHPAD_53_;                  /**< 6A No description provided */
    volatile const uint16_t RESERVED53;                     
    volatile       uint16_t SCRATCHPAD_54_;                  /**< 6C No description provided */
    volatile const uint16_t RESERVED54;                     
    volatile       uint16_t SCRATCHPAD_55_;                  /**< 6E No description provided */
    volatile const uint16_t RESERVED55;                     
    volatile       uint16_t SCRATCHPAD_56_;                  /**< 70 No description provided */
    volatile const uint16_t RESERVED56;                     
    volatile       uint16_t SCRATCHPAD_57_;                  /**< 72 No description provided */
    volatile const uint16_t RESERVED57;                     
    volatile       uint16_t SCRATCHPAD_58_;                  /**< 74 No description provided */
    volatile const uint16_t RESERVED58;                     
    volatile       uint16_t SCRATCHPAD_59_;                  /**< 76 No description provided */
    volatile const uint16_t RESERVED59;                     
    volatile       uint16_t SCRATCHPAD_60_;                  /**< 78 No description provided */
    volatile const uint16_t RESERVED60;                     
    volatile       uint16_t SCRATCHPAD_61_;                  /**< 7A No description provided */
    volatile const uint16_t RESERVED61;                     
    volatile       uint16_t SCRATCHPAD_62_;                  /**< 7C No description provided */
    volatile const uint16_t RESERVED62;                     
    volatile       uint16_t SCRATCHPAD_63_;                  /**< 7E No description provided */
    volatile const uint16_t RESERVED63;                     
    volatile       uint16_t CLKGEN_CK1;                      /**< 80 No description provided */
    volatile const uint16_t RESERVED64;                     
    volatile       uint16_t CLKGEN_CK2;                      /**< 82 No description provided */
    volatile const uint16_t RESERVED65;                     
    volatile       uint16_t CLKGEN_CK1REF;                   /**< 84 No description provided */
    volatile const uint16_t RESERVED66;                     
    volatile       uint16_t CLKGEN_CK2REF;                   /**< 86 No description provided */
    volatile const uint16_t RESERVED67;                     
    volatile       uint16_t CLKGEN_L1;                       /**< 88 No description provided */
    volatile const uint16_t RESERVED68;                     
    volatile       uint16_t CLKGEN_L2;                       /**< 8A No description provided */
    volatile const uint16_t RESERVED69;                     
    volatile       uint16_t CLKGEN_CKX;                      /**< 8C No description provided */
    volatile const uint16_t RESERVED70;                     
    volatile       uint16_t CLKGEN_CYCLE;                    /**< 8E No description provided */
    volatile const uint16_t RESERVED71;                     
    volatile       uint16_t CLKGEN_OFFSET;                   /**< 90 No description provided */
    volatile const uint16_t RESERVED72;                     
    volatile       uint16_t CLKGEN_LTOFFSET;                 /**< 92 No description provided */
    volatile const uint16_t RESERVED73;                     
    volatile       uint16_t CLKGEN_BURST_PERIOD;             /**< 94 No description provided */
} ADI_USEQ_REGS_MAP2_TypeDef;


#define ADI_USEQ_REGS2_BASE                 (0X00000200U)    /* Base address of useq_regs2: */
#define pADI_USEQ_REGS2                     ((ADI_USEQ_REGS_MAP2_TypeDef *) ADI_USEQ_REGS2_BASE )    /* Pointer to  (useq_regs2)*/

#endif  /* end ifndef ADI_USEQ_REGS_MAP2_DEVICE_H_ */

#ifndef ADI_SPIM_REGS_DEVICE_H_
#define ADI_SPIM_REGS_DEVICE_H_

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_TypeDef
 *  \brief Your module description, here.
 *  ======================================================================== */
typedef struct _ADI_SPIM_REGS_TypeDef
{
    volatile       uint16_t CTRLR0;                          /**< 0 Control Register 0 */
    volatile const uint16_t RESERVED0[3];                   
    volatile       uint16_t CTRLR1;                          /**< 4 Control Register 1 */
    volatile const uint16_t RESERVED1[3];                   
    volatile       uint16_t SSIENR;                          /**< 8 SSI Enable Register */
    volatile const uint16_t RESERVED2[3];                   
    volatile       uint16_t MWCR;                            /**< C Microwire Control Register */
    volatile const uint16_t RESERVED3[3];                   
    volatile       uint16_t SER;                             /**< 10 Slave Enable Register */
    volatile const uint16_t RESERVED4[3];                   
    volatile       uint16_t BAUDR;                           /**< 14 Baud Rate Select */
    volatile const uint16_t RESERVED5[3];                   
    volatile       uint16_t TXFTLR;                          /**< 18 Transmit FIFO Threshold Level */
    volatile const uint16_t RESERVED6[3];                   
    volatile       uint16_t RXFTLR;                          /**< 1C Receive FIFO Threshold Level */
    volatile const uint16_t RESERVED7[3];                   
    volatile       uint16_t TXFLR;                           /**< 20 Transmit FIFO Level Register */
    volatile const uint16_t RESERVED8[3];                   
    volatile       uint16_t RXFLR;                           /**< 24 Receive FIFO Level Register */
    volatile const uint16_t RESERVED9[3];                   
    volatile       uint16_t SR;                              /**< 28 Status Register */
    volatile const uint16_t RESERVED10[3];                  
    volatile       uint16_t IMR;                             /**< 2C Interrupt Mask Register */
    volatile const uint16_t RESERVED11[3];                  
    volatile       uint16_t ISR;                             /**< 30 Interrupt Status Register */
    volatile const uint16_t RESERVED12[3];                  
    volatile       uint16_t RISR;                            /**< 34 Raw Interrupt Status Register */
    volatile const uint16_t RESERVED13[3];                  
    volatile       uint16_t TXOICR;                          /**< 38 Transmit FIFO Overflow Interrupt Clear Register Register */
    volatile const uint16_t RESERVED14[3];                  
    volatile       uint16_t RXOICR;                          /**< 3C Receive FIFO Overflow Interrupt Clear Register */
    volatile const uint16_t RESERVED15[3];                  
    volatile       uint16_t RXUICR;                          /**< 40 Receive FIFO Underflow Interrupt Clear Register */
    volatile const uint16_t RESERVED16[3];                  
    volatile       uint16_t MSTICR;                          /**< 44 Multi-Master Interrupt Clear Register */
    volatile const uint16_t RESERVED17[3];                  
    volatile       uint16_t ICR;                             /**< 48 Interrupt Clear Register */
    volatile const uint16_t RESERVED18[15];                 
    volatile       uint16_t IDR;                             /**< 58 Identification Register */
    volatile const uint16_t RESERVED19[3];                  
    volatile       uint16_t SSI_VERSION_ID;                  /**< 5C Core Kit version ID Register */
    volatile const uint16_t RESERVED20[3];                  
    volatile       uint16_t DR0;                             /**< 60 DW_apb_ssi Data Register */
} ADI_SPIM_REGS_TypeDef;


#define ADI_SPIM_REGS_BASE                  (0X00000300U)    /* Base address of spim_regs: */
#define pADI_SPIM_REGS                      ((ADI_SPIM_REGS_TypeDef *) ADI_SPIM_REGS_BASE )    /* Pointer to  (spim_regs)*/

#endif  /* end ifndef ADI_SPIM_REGS_DEVICE_H_ */

#ifndef ADI_CSI2_REGSPEC_TOP_CPU0_DEVICE_H_
#define ADI_CSI2_REGSPEC_TOP_CPU0_DEVICE_H_

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_TypeDef
 *  \brief Your module description, here.
 *  ======================================================================== */
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_TypeDef
{
    volatile       uint32_t CSI2_TX_BASE_CFG_NUM_LANES;      /**< 0 No description provided */
    volatile       uint32_t CSI2_TX_BASE_CFG_T_PRE;          /**< 4 No description provided */
    volatile       uint32_t CSI2_TX_BASE_CFG_T_POST;         /**< 8 No description provided */
    volatile       uint32_t CSI2_TX_BASE_CFG_TX_GAP;         /**< C No description provided */
    volatile       uint32_t CSI2_TX_BASE_CFG_T_CLK_GAP;      /**< 10 No description provided */
    volatile       uint32_t CSI2_TX_BASE_CFG_CONTINUOUS_HS_CLK; /**< 14 No description provided */
    volatile       uint32_t CSI2_TX_BASE_CFG_TWAKEUP;        /**< 18 No description provided */
    volatile       uint32_t CSI2_TX_BASE_ULPS_CLK_ENABLE;    /**< 1C No description provided */
    volatile       uint32_t CSI2_TX_BASE_ULPS_ENABLE;        /**< 20 No description provided */
    volatile       uint32_t CSI2_TX_BASE_ULPS_CLK_ACTIVE;    /**< 24 No description provided */
    volatile       uint32_t CSI2_TX_BASE_ULPS_ACTIVE;        /**< 28 No description provided */
    volatile       uint32_t CSI2_TX_BASE_IRQ_STATUS;         /**< 2C No description provided */
    volatile       uint32_t CSI2_TX_BASE_IRQ_ENABLE;         /**< 30 No description provided */
    volatile       uint32_t CSI2_TX_BASE_CSI2TX_IRQ_CLR;     /**< 34 No description provided */
    volatile       uint32_t CSI2_TX_BASE_CFG_CLK_LANE_EN;    /**< 38 No description provided */
    volatile       uint32_t CSI2_TX_BASE_CFG_DATA_LANE_EN;   /**< 3C No description provided */
    volatile       uint32_t CSI2_TX_BASE_CFG_CPHY_EN;        /**< 40 No description provided */
    volatile       uint32_t CSI2_TX_BASE_CFG_PPI_16_EN;      /**< 44 No description provided */
    volatile       uint32_t CSI2_TX_BASE_CFG_PACKET_INTERFACE_EN; /**< 48 No description provided */
    volatile       uint32_t CSI2_TX_BASE_CFG_VCX_EN;         /**< 4C No description provided */
    volatile       uint32_t CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_I; /**< 50 No description provided */
    volatile       uint32_t CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_P; /**< 54 No description provided */
    volatile const uint8_t RESERVED0[8];                   
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PLL; /**< 60 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PLL_CTRL; /**< 64 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PHY; /**< 68 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PHY_CTRL; /**< 6C No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TST_PLL; /**< 70 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CN; /**< 74 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CM; /**< 78 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CO; /**< 7C No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_BYP; /**< 80 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_BYPASS_PLL; /**< 84 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_LATCH; /**< 88 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TX_RCAL; /**< 8C No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_AUTO_PD_EN; /**< 90 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_ENBL; /**< 94 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK; /**< 98 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_ZERO; /**< 9C No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_ZERO; /**< A0 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_TRAIL; /**< A4 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_TRAIL; /**< A8 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_PREPARE; /**< AC No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_PREPARE; /**< B0 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN0; /**< B4 No description provided */
    volatile       uint32_t CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN1; /**< B8 No description provided */
} ADI_CSI2_REGSPEC_TOP_CPU0_TypeDef;


#define ADI_MIPI_REGS_BASE                  (0X00000400U)    /* Base address of mipi_regs: */
#define pADI_MIPI_REGS                      ((ADI_CSI2_REGSPEC_TOP_CPU0_TypeDef *) ADI_MIPI_REGS_BASE )    /* Pointer to  (mipi_regs)*/

#endif  /* end ifndef ADI_CSI2_REGSPEC_TOP_CPU0_DEVICE_H_ */

#ifndef ADI_EFUSE_DEVICE_H_
#define ADI_EFUSE_DEVICE_H_

/*! ========================================================================
 *  \struct ADI_EFUSE_TypeDef
 *  \brief eFuse
 *  ======================================================================== */
typedef struct _ADI_EFUSE_TypeDef
{
    volatile const uint16_t RESERVED0[128];                 
    volatile       uint16_t PWR_CTRL;                        /**< 80 Power Control Register */
    volatile const uint16_t RESERVED1;                      
    volatile       uint16_t STATUS;                          /**< 82 Program status register */
    volatile const uint16_t RESERVED2;                      
    volatile       uint16_t ERR_LOCATION;                    /**< 84 Error location */
    volatile const uint16_t RESERVED3;                      
    volatile       uint16_t TIMING;                          /**< 86 Timing Configuration */
    volatile const uint16_t RESERVED4;                      
    volatile       uint16_t CHARACTERIZATION;                /**< 88 Characterization Options */
} ADI_EFUSE_TypeDef;


#define ADI_EFUSE_REGS_BASE                 (0X00000600U)    /* Base address of efuse_regs: */
#define pADI_EFUSE_REGS                     ((ADI_EFUSE_TypeDef *) ADI_EFUSE_REGS_BASE )    /* Pointer to  (efuse_regs)*/

#endif  /* end ifndef ADI_EFUSE_DEVICE_H_ */

#ifndef ADI_LPS_REGS_YODA_DEVICE_H_
#define ADI_LPS_REGS_YODA_DEVICE_H_

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_TypeDef
 *  \brief Your module description, here.
 *  ======================================================================== */
typedef struct _ADI_LPS_REGS_YODA_TypeDef
{
    volatile       uint16_t LPSCTRL;                         /**< 0 No description provided */
    volatile const uint16_t RESERVED0;                      
    volatile       uint16_t LPSWAVEFREQ;                     /**< 2 No description provided */
    volatile const uint16_t RESERVED1;                      
    volatile       uint16_t LPSWAVEGENACC;                   /**< 4 No description provided */
    volatile const uint16_t RESERVED2;                      
    volatile       uint16_t LPSRAMADDR;                      /**< 6 No description provided */
    volatile const uint16_t RESERVED3;                      
    volatile       uint16_t LPSRAMRDCMD;                     /**< 8 No description provided */
    volatile const uint16_t RESERVED4[3];                   
    volatile       uint16_t LPSWAVEGENADDR;                  /**< C No description provided */
    volatile const uint16_t RESERVED5;                      
    volatile       uint16_t LPSMARGIN;                       /**< E No description provided */
    volatile const uint16_t RESERVED6;                      
    volatile       uint16_t LPSDBG;                          /**< 10 No description provided */
    volatile const uint16_t RESERVED7[15];                  
    volatile       uint16_t LPSRAMDATA;                      /**< 20 No description provided */
    volatile const uint16_t RESERVED8;                      
    volatile       uint16_t LPSRAMDATA_ALIAS;                /**< 22 No description provided */
} ADI_LPS_REGS_YODA_TypeDef;


#define ADI_LPS1_REGS_BASE                  (0X00000900U)    /* Base address of lps1_regs: */
#define pADI_LPS1_REGS                      ((ADI_LPS_REGS_YODA_TypeDef *) ADI_LPS1_REGS_BASE )    /* Pointer to  (lps1_regs)*/
#define ADI_LPS2_REGS_BASE                  (0X00000A00U)    /* Base address of lps2_regs: */
#define pADI_LPS2_REGS                      ((ADI_LPS_REGS_YODA_TypeDef *) ADI_LPS2_REGS_BASE )    /* Pointer to  (lps2_regs)*/

#endif  /* end ifndef ADI_LPS_REGS_YODA_DEVICE_H_ */

#ifndef ADI_SS_REGS_DEVICE_H_
#define ADI_SS_REGS_DEVICE_H_

/*! ========================================================================
 *  \struct ADI_SS_REGS_TypeDef
 *  \brief Your module description, here.
 *  ======================================================================== */
typedef struct _ADI_SS_REGS_TypeDef
{
    volatile       uint16_t SSWAVEFREQ;                      /**< 0 No description provided */
    volatile const uint16_t RESERVED0;                      
    volatile       uint16_t SSWAVEPERIOD;                    /**< 2 No description provided */
    volatile const uint16_t RESERVED1;                      
    volatile       uint16_t SSWAVEAMP;                       /**< 4 No description provided */
    volatile const uint16_t RESERVED2;                      
    volatile       uint16_t SSPINLO;                         /**< 6 No description provided */
    volatile const uint16_t RESERVED3;                      
    volatile       uint16_t SSPINHI;                         /**< 8 No description provided */
    volatile const uint16_t RESERVED4;                      
    volatile       uint16_t SSINTERVLO;                      /**< A No description provided */
    volatile const uint16_t RESERVED5;                      
    volatile       uint16_t SSINTERVHI;                      /**< C No description provided */
    volatile const uint16_t RESERVED6;                      
    volatile       uint16_t SSWAVEOFFSET;                    /**< E No description provided */
    volatile const uint16_t RESERVED7;                      
    volatile       uint16_t SSCTRL;                          /**< 10 No description provided */
    volatile const uint16_t RESERVED8;                      
    volatile       uint16_t SSMODFREQ;                       /**< 12 No description provided */
    volatile const uint16_t RESERVED9;                      
    volatile       uint16_t SSMODAMP;                        /**< 14 No description provided */
    volatile const uint16_t RESERVED10;                     
    volatile       uint16_t SSINTERV_10;                     /**< 16 No description provided */
    volatile const uint16_t RESERVED11;                     
    volatile       uint16_t SSINTERV_11;                     /**< 18 No description provided */
    volatile const uint16_t RESERVED12;                     
    volatile       uint16_t SSINTERV_20;                     /**< 1A No description provided */
    volatile const uint16_t RESERVED13;                     
    volatile       uint16_t SSINTERV_21;                     /**< 1C No description provided */
    volatile const uint16_t RESERVED14;                     
    volatile       uint16_t SSINTERV_30;                     /**< 1E No description provided */
    volatile const uint16_t RESERVED15;                     
    volatile       uint16_t SSINTERV_31;                     /**< 20 No description provided */
    volatile const uint16_t RESERVED16;                     
    volatile       uint16_t SSVALUE_00;                      /**< 22 No description provided */
    volatile const uint16_t RESERVED17;                     
    volatile       uint16_t SSVALUE_01;                      /**< 24 No description provided */
    volatile const uint16_t RESERVED18;                     
    volatile       uint16_t SSVALUE_10;                      /**< 26 No description provided */
    volatile const uint16_t RESERVED19;                     
    volatile       uint16_t SSVALUE_11;                      /**< 28 No description provided */
    volatile const uint16_t RESERVED20;                     
    volatile       uint16_t SSVALUE_20;                      /**< 2A No description provided */
    volatile const uint16_t RESERVED21;                     
    volatile       uint16_t SSVALUE_21;                      /**< 2C No description provided */
    volatile const uint16_t RESERVED22;                     
    volatile       uint16_t SSVALUE_30;                      /**< 2E No description provided */
    volatile const uint16_t RESERVED23;                     
    volatile       uint16_t SSVALUE_31;                      /**< 30 No description provided */
    volatile const uint16_t RESERVED24;                     
    volatile       uint16_t SSDBG;                           /**< 32 No description provided */
} ADI_SS_REGS_TypeDef;


#define ADI_SS_REGS_BASE                    (0X00000B00U)    /* Base address of ss_regs: */
#define pADI_SS_REGS                        ((ADI_SS_REGS_TypeDef *) ADI_SS_REGS_BASE )    /* Pointer to  (ss_regs)*/

#endif  /* end ifndef ADI_SS_REGS_DEVICE_H_ */

#ifndef ADI_PCM_REGS_YODA_DEVICE_H_
#define ADI_PCM_REGS_YODA_DEVICE_H_

/*! ========================================================================
 *  \struct ADI_PCM_REGS_YODA_TypeDef
 *  \brief Your module description, here.
 *  ======================================================================== */
typedef struct _ADI_PCM_REGS_YODA_TypeDef
{
    volatile       uint16_t PCMCTRL_0;                       /**< 0 No description provided */
    volatile const uint16_t RESERVED0;                      
    volatile       uint16_t PCMCTRL_1;                       /**< 2 No description provided */
    volatile const uint16_t RESERVED1;                      
    volatile       uint16_t PCMOUT;                          /**< 4 No description provided */
    volatile const uint16_t RESERVED2;                      
    volatile       uint16_t OSC_PERIOD_CTRL;                 /**< 6 No description provided */
    volatile const uint16_t RESERVED3;                      
    volatile       uint16_t OSC_PERIOD_RD;                   /**< 8 No description provided */
} ADI_PCM_REGS_YODA_TypeDef;


#define ADI_PCM_REGS_BASE                   (0X00000C00U)    /* Base address of pcm_regs: */
#define pADI_PCM_REGS                       ((ADI_PCM_REGS_YODA_TypeDef *) ADI_PCM_REGS_BASE )    /* Pointer to  (pcm_regs)*/

#endif  /* end ifndef ADI_PCM_REGS_YODA_DEVICE_H_ */

#ifndef ADI_DATAPATH_DEVICE_H_
#define ADI_DATAPATH_DEVICE_H_

/*! ========================================================================
 *  \struct ADI_DATAPATH_TypeDef
 *  \brief Your module description, here.
 *  ======================================================================== */
typedef struct _ADI_DATAPATH_TypeDef
{
    volatile const uint16_t RESERVED0[8];                   
    volatile       uint16_t CORRECTION_CONFIG;               /**< 8 No description provided */
    volatile const uint16_t RESERVED1;                      
    volatile       uint16_t USE_CASE_FRAME_CONFIG;           /**< A No description provided */
    volatile const uint16_t RESERVED2;                      
    volatile       uint16_t USE_CASE_MIPI_PACKET_CONTROL;    /**< C No description provided */
    volatile const uint16_t RESERVED3;                      
    volatile       uint16_t GAIN0;                           /**< E No description provided */
    volatile const uint16_t RESERVED4;                      
    volatile       uint16_t GAIN1;                           /**< 10 No description provided */
    volatile const uint16_t RESERVED5;                      
    volatile       uint16_t GAIN2;                           /**< 12 No description provided */
    volatile const uint16_t RESERVED6;                      
    volatile       uint16_t GAIN3;                           /**< 14 No description provided */
    volatile const uint16_t RESERVED7;                      
    volatile       uint16_t PARITY_GAIN_MEM;                 /**< 16 No description provided */
    volatile const uint16_t RESERVED8;                      
    volatile       uint16_t PARITY_LINE_MEM;                 /**< 18 No description provided */
    volatile const uint16_t RESERVED9;                      
    volatile       uint16_t PP_LFSR;                         /**< 1A No description provided */
    volatile const uint16_t RESERVED10;                     
    volatile       uint16_t PP_DECODE_ST_1;                  /**< 1C No description provided */
    volatile const uint16_t RESERVED11;                     
    volatile       uint16_t PP_DECODE_ST_2;                  /**< 1E No description provided */
    volatile const uint16_t RESERVED12;                     
    volatile       uint16_t PP_ENCODE_ST;                    /**< 20 No description provided */
    volatile const uint16_t RESERVED13;                     
    volatile       uint16_t PP_ENCODE_GT;                    /**< 22 No description provided */
    volatile const uint16_t RESERVED14;                     
    volatile       uint16_t DBG_MUX;                         /**< 24 No description provided */
    volatile const uint16_t RESERVED15;                     
    volatile       uint16_t GAIN_MARGIN_CONTROL;             /**< 26 No description provided */
    volatile const uint16_t RESERVED16;                     
    volatile       uint16_t LINE_MARGIN_CONTROL;             /**< 28 No description provided */
    volatile const uint16_t RESERVED17;                     
    volatile       uint16_t ROI_ROW_START;                   /**< 2A No description provided */
    volatile const uint16_t RESERVED18;                     
    volatile       uint16_t ROI_HEIGHT;                      /**< 2C No description provided */
    volatile const uint16_t RESERVED19;                     
    volatile       uint16_t ROI_COLUMN_START;                /**< 2E No description provided */
    volatile const uint16_t RESERVED20;                     
    volatile       uint16_t ROI_WIDTH;                       /**< 30 No description provided */
    volatile const uint16_t RESERVED21;                     
    volatile       uint16_t PP_USEQ_WRITE;                   /**< 32 No description provided */
    volatile const uint16_t RESERVED22;                     
    volatile       uint16_t PP_ADC_DELAY;                    /**< 34 No description provided */
    volatile const uint16_t RESERVED23;                     
    volatile       uint16_t MIPI_BUFF_MARGIN_CONTROL;        /**< 36 No description provided */
    volatile const uint16_t RESERVED24;                     
    volatile       uint16_t MIPI_HEADER_WIDTH;               /**< 38 No description provided */
    volatile const uint16_t RESERVED25[3];                  
    volatile       uint16_t FRAME_NUMBER;                    /**< 3C No description provided */
    volatile const uint16_t RESERVED26;                     
    volatile       uint16_t MICRO_SEQUENCER_FW_VERSION_LSB;  /**< 3E No description provided */
    volatile const uint16_t RESERVED27;                     
    volatile       uint16_t MICRO_SEQUENCER_FW_VERSION_MSB;  /**< 40 No description provided */
    volatile const uint16_t RESERVED28;                     
    volatile       uint16_t TS_CAL_VER;                      /**< 42 No description provided */
    volatile const uint16_t RESERVED29;                     
    volatile       uint16_t ADC_CAL_VER;                     /**< 44 No description provided */
    volatile const uint16_t RESERVED30;                     
    volatile       uint16_t REG_0;                           /**< 46 No description provided */
    volatile const uint16_t RESERVED31;                     
    volatile       uint16_t REG_1;                           /**< 48 No description provided */
    volatile const uint16_t RESERVED32;                     
    volatile       uint16_t REG_2;                           /**< 4A No description provided */
    volatile const uint16_t RESERVED33;                     
    volatile       uint16_t REG_3;                           /**< 4C No description provided */
    volatile const uint16_t RESERVED34;                     
    volatile       uint16_t REG_4;                           /**< 4E No description provided */
    volatile const uint16_t RESERVED35;                     
    volatile       uint16_t REG_5;                           /**< 50 No description provided */
    volatile const uint16_t RESERVED36;                     
    volatile       uint16_t REG_6;                           /**< 52 No description provided */
    volatile const uint16_t RESERVED37;                     
    volatile       uint16_t REG_7;                           /**< 54 No description provided */
    volatile const uint16_t RESERVED38;                     
    volatile       uint16_t PARITY_MIPI_BUFFER;              /**< 56 No description provided */
    volatile const uint16_t RESERVED39;                     
    volatile       uint16_t PACKET_COUNT;                    /**< 58 No description provided */
    volatile const uint16_t RESERVED40;                     
    volatile       uint16_t PACKETS_PER_FRAME;               /**< 5A No description provided */
    volatile const uint16_t RESERVED41;                     
    volatile       uint16_t ROW_VECTOR;                      /**< 5C No description provided */
    volatile const uint16_t RESERVED42;                     
    volatile       uint16_t ROWS_PER_PACKET_OUT;             /**< 5E No description provided */
    volatile const uint16_t RESERVED43;                     
    volatile       uint16_t MIPI_RD_EN_MAX;                  /**< 60 No description provided */
    volatile const uint16_t RESERVED44;                     
    volatile       uint16_t ANALOG_SS;                       /**< 62 No description provided */
    volatile const uint16_t RESERVED45;                     
    volatile       uint16_t MIPI_BUFF_PARITY_ERR_CNT;        /**< 64 No description provided */
    volatile const uint16_t RESERVED46;                     
    volatile       uint16_t LINE_MEM_PARITY_ERR_CNT;         /**< 66 No description provided */
    volatile const uint16_t RESERVED47;                     
    volatile       uint16_t GAIN_MEM_PARITY_ERR_CNT;         /**< 68 No description provided */
    volatile const uint16_t RESERVED48[7];                  
    volatile       uint16_t IA_SELECT;                       /**< 70 No description provided */
    volatile const uint16_t RESERVED49;                     
    volatile       uint16_t IA_ADDR_REG;                     /**< 72 No description provided */
    volatile const uint16_t RESERVED50;                     
    volatile       uint16_t IA_WRDATA_REG;                   /**< 74 No description provided */
    volatile const uint16_t RESERVED51;                     
    volatile       uint16_t IA_WRDATA_REG_ALIAS;             /**< 76 No description provided */
    volatile const uint16_t RESERVED52;                     
    volatile       uint16_t IA_RDDATA_REG;                   /**< 78 No description provided */
    volatile const uint16_t RESERVED53;                     
    volatile       uint16_t IA_RDDATA_REG_ALIAS;             /**< 7A No description provided */
    volatile const uint16_t RESERVED54;                     
    volatile       uint16_t IA_BANK_TYPE;                    /**< 7C No description provided */
} ADI_DATAPATH_TypeDef;


#define ADI_DATAPATH_REGS_BASE              (0X00000D00U)    /* Base address of datapath_regs: */
#define pADI_DATAPATH_REGS                  ((ADI_DATAPATH_TypeDef *) ADI_DATAPATH_REGS_BASE )    /* Pointer to  (datapath_regs)*/

#endif  /* end ifndef ADI_DATAPATH_DEVICE_H_ */

#ifndef ADI_DE_REGS_YODA_DEVICE_H_
#define ADI_DE_REGS_YODA_DEVICE_H_

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_TypeDef
 *  \brief Your module description, here.
 *  ======================================================================== */
typedef struct _ADI_DE_REGS_YODA_TypeDef
{
    volatile       uint16_t DE_CONTROL;                      /**< 0 No description provided */
    volatile const uint16_t RESERVED0;                      
    volatile       uint16_t BINNED1X2_REPEAT_COUNT;          /**< 2 No description provided */
    volatile const uint16_t RESERVED1[9];                   
    volatile       uint16_t OVERRIDE_DATA_REG1;              /**< C No description provided */
    volatile const uint16_t RESERVED2;                      
    volatile       uint16_t OVERRIDE_DATA_REG2;              /**< E No description provided */
    volatile const uint16_t RESERVED3;                      
    volatile       uint16_t OVERRIDE_DATA_REG3;              /**< 10 No description provided */
    volatile const uint16_t RESERVED4;                      
    volatile       uint16_t BINNED1X2_END;                   /**< 12 No description provided */
    volatile const uint16_t RESERVED5;                      
    volatile       uint16_t OVERRIDE_SEL_REG1;               /**< 14 No description provided */
    volatile const uint16_t RESERVED6;                      
    volatile       uint16_t OVERRIDE_SEL_REG2;               /**< 16 No description provided */
    volatile const uint16_t RESERVED7;                      
    volatile       uint16_t OVERRIDE_SEL_REG3;               /**< 18 No description provided */
    volatile const uint16_t RESERVED8;                      
    volatile       uint16_t BINNED1X2_START;                 /**< 1A No description provided */
    volatile const uint16_t RESERVED9;                      
    volatile       uint16_t AMP_MUX_SEL_EE_LOW;              /**< 1C No description provided */
    volatile const uint16_t RESERVED10;                     
    volatile       uint16_t AMP_MUX_SEL_EE_HIGH;             /**< 1E No description provided */
    volatile const uint16_t RESERVED11;                     
    volatile       uint16_t AMP_MUX_SEL_EO_LOW;              /**< 20 No description provided */
    volatile const uint16_t RESERVED12;                     
    volatile       uint16_t AMP_MUX_SEL_EO_HIGH;             /**< 22 No description provided */
    volatile const uint16_t RESERVED13;                     
    volatile       uint16_t AMP_MUX_SEL_OE_LOW;              /**< 24 No description provided */
    volatile const uint16_t RESERVED14;                     
    volatile       uint16_t AMP_MUX_SEL_OE_HIGH;             /**< 26 No description provided */
    volatile const uint16_t RESERVED15;                     
    volatile       uint16_t AMP_MUX_SEL_OO_LOW;              /**< 28 No description provided */
    volatile const uint16_t RESERVED16;                     
    volatile       uint16_t AMP_MUX_SEL_OO_HIGH;             /**< 2A No description provided */
    volatile const uint16_t RESERVED17;                     
    volatile       uint16_t AMP_MUX_SEL_SELB_LOW;            /**< 2C No description provided */
    volatile const uint16_t RESERVED18;                     
    volatile       uint16_t AMP_MUX_SEL_SELB_HIGH;           /**< 2E No description provided */
    volatile const uint16_t RESERVED19;                     
    volatile       uint16_t NATIVE_RESOLUTION_START;         /**< 30 No description provided */
    volatile const uint16_t RESERVED20;                     
    volatile       uint16_t NATIVE_RESOLUTION_END;           /**< 32 No description provided */
    volatile const uint16_t RESERVED21;                     
    volatile       uint16_t NATIVE_RESOLUTION_REPEAT;        /**< 34 No description provided */
    volatile const uint16_t RESERVED22;                     
    volatile       uint16_t SUB_SAMPLED_2X_START;            /**< 36 No description provided */
    volatile const uint16_t RESERVED23;                     
    volatile       uint16_t SUB_SAMPLED_2X_END;              /**< 38 No description provided */
    volatile const uint16_t RESERVED24;                     
    volatile       uint16_t SUB_SAMPLED_2X_REPEAT;           /**< 3A No description provided */
    volatile const uint16_t RESERVED25;                     
    volatile       uint16_t SUB_SAMPLED_4X_START;            /**< 3C No description provided */
    volatile const uint16_t RESERVED26;                     
    volatile       uint16_t SUB_SAMPLED_4X_END;              /**< 3E No description provided */
    volatile const uint16_t RESERVED27;                     
    volatile       uint16_t SUB_SAMPLED_4X_REPEAT;           /**< 40 No description provided */
    volatile const uint16_t RESERVED28;                     
    volatile       uint16_t BINNED_START;                    /**< 42 No description provided */
    volatile const uint16_t RESERVED29;                     
    volatile       uint16_t BINNED_END;                      /**< 44 No description provided */
    volatile const uint16_t RESERVED30;                     
    volatile       uint16_t BINNED_REPEAT;                   /**< 46 No description provided */
    volatile const uint16_t RESERVED31;                     
    volatile       uint16_t DARK_START;                      /**< 48 No description provided */
    volatile const uint16_t RESERVED32;                     
    volatile       uint16_t DARK_END;                        /**< 4A No description provided */
    volatile const uint16_t RESERVED33;                     
    volatile       uint16_t DARK_REPEAT;                     /**< 4C No description provided */
    volatile const uint16_t RESERVED34;                     
    volatile       uint16_t PREAMBLE_START;                  /**< 4E No description provided */
    volatile const uint16_t RESERVED35;                     
    volatile       uint16_t PREAMBLE_END;                    /**< 50 No description provided */
    volatile const uint16_t RESERVED36;                     
    volatile       uint16_t PREAMBLE_REPEAT;                 /**< 52 No description provided */
    volatile const uint16_t RESERVED37;                     
    volatile       uint16_t POSTAMBLE_START;                 /**< 54 No description provided */
    volatile const uint16_t RESERVED38;                     
    volatile       uint16_t POSTAMBLE_END;                   /**< 56 No description provided */
    volatile const uint16_t RESERVED39;                     
    volatile       uint16_t POSTAMBLE_REPEAT;                /**< 58 No description provided */
    volatile const uint16_t RESERVED40;                     
    volatile       uint16_t ARRAY_INIT_VEC_DARK;             /**< 5A No description provided */
    volatile const uint16_t RESERVED41;                     
    volatile       uint16_t ARRAY_INIT_VEC;                  /**< 5C No description provided */
    volatile const uint16_t RESERVED42;                     
    volatile       uint16_t TYPE_OVERRIDE;                   /**< 5E No description provided */
    volatile const uint16_t RESERVED43;                     
    volatile       uint16_t MEM_DFT;                         /**< 60 No description provided */
    volatile const uint16_t RESERVED44;                     
    volatile       uint16_t DBG_MUX_CONTROL_0;               /**< 62 No description provided */
    volatile const uint16_t RESERVED45;                     
    volatile       uint16_t DBG_MUX_CONTROL_1;               /**< 64 No description provided */
    volatile const uint16_t RESERVED46;                     
    volatile       uint16_t DBG_MUX_CONTROL_2;               /**< 66 No description provided */
    volatile const uint16_t RESERVED47;                     
    volatile       uint16_t DBG_MUX_CONTROL_3;               /**< 68 No description provided */
    volatile const uint16_t RESERVED48;                     
    volatile       uint16_t DBG_MUX_CONTROL_4;               /**< 6A No description provided */
    volatile const uint16_t RESERVED49[5];                  
    volatile       uint16_t DE_IA_SELECT;                    /**< 70 No description provided */
    volatile const uint16_t RESERVED50;                     
    volatile       uint16_t DE_IA_ADDR_REG;                  /**< 72 No description provided */
    volatile const uint16_t RESERVED51;                     
    volatile       uint16_t DE_IA_WRDATA_REG;                /**< 74 No description provided */
    volatile const uint16_t RESERVED52;                     
    volatile       uint16_t DE_IA_WRDATA_REG_ALIAS;          /**< 76 No description provided */
    volatile const uint16_t RESERVED53;                     
    volatile       uint16_t DE_IA_RDDATA_REG;                /**< 78 No description provided */
    volatile const uint16_t RESERVED54;                     
    volatile       uint16_t DE_IA_RDDATA_REG_ALIAS;          /**< 7A No description provided */
    volatile const uint16_t RESERVED55[5];                  
    volatile       uint16_t USE_CASE_0_ROI_0_ROW_VEC_TOPBOT; /**< 80 No description provided */
    volatile const uint16_t RESERVED56;                     
    volatile       uint16_t USE_CASE_0_ROI_0_ROW_VEC_MAIN;   /**< 82 No description provided */
    volatile const uint16_t RESERVED57;                     
    volatile       uint16_t USE_CASE_0_ROI_0_COLUMN_VEC;     /**< 84 No description provided */
    volatile const uint16_t RESERVED58;                     
    volatile       uint16_t USE_CASE_0_ROI_0_ROI_TYPE;       /**< 86 No description provided */
    volatile const uint16_t RESERVED59;                     
    volatile       uint16_t USE_CASE_0_ROI_1_ROW_VEC_TOPBOT; /**< 88 No description provided */
    volatile const uint16_t RESERVED60;                     
    volatile       uint16_t USE_CASE_0_ROI_1_ROW_VEC_MAIN;   /**< 8A No description provided */
    volatile const uint16_t RESERVED61;                     
    volatile       uint16_t USE_CASE_0_ROI_1_COLUMN_VEC;     /**< 8C No description provided */
    volatile const uint16_t RESERVED62;                     
    volatile       uint16_t USE_CASE_0_ROI_1_ROI_TYPE;       /**< 8E No description provided */
    volatile const uint16_t RESERVED63;                     
    volatile       uint16_t USE_CASE_0_ROI_2_ROW_VEC_TOPBOT; /**< 90 No description provided */
    volatile const uint16_t RESERVED64;                     
    volatile       uint16_t USE_CASE_0_ROI_2_ROW_VEC_MAIN;   /**< 92 No description provided */
    volatile const uint16_t RESERVED65;                     
    volatile       uint16_t USE_CASE_0_ROI_2_COLUMN_VEC;     /**< 94 No description provided */
    volatile const uint16_t RESERVED66;                     
    volatile       uint16_t USE_CASE_0_ROI_2_ROI_TYPE;       /**< 96 No description provided */
} ADI_DE_REGS_YODA_TypeDef;


#define ADI_DE_REGS_BASE                    (0X00000E00U)    /* Base address of de_regs: */
#define pADI_DE_REGS                        ((ADI_DE_REGS_YODA_TypeDef *) ADI_DE_REGS_BASE )    /* Pointer to  (de_regs)*/

#endif  /* end ifndef ADI_DE_REGS_YODA_DEVICE_H_ */

#if defined (__CC_ARM)
#pragma pop
#endif

#endif /* NEWTON_DEVICE_H */

