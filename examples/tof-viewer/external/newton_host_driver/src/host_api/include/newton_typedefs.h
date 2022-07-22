
/* ================================================================================
     Created by  :   saskauka
     Created on  :   2020 Apr 27, 12:15 IST

     Project     :   newton
     File        :   newton_typedefs.h
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
       Last modified : 5-Apr-2020
   ================================================================================ */

#ifndef NEWTON_TYPEDEFS_H
#define NEWTON_TYPEDEFS_H

/* pickup integer types */
#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

#if defined ( __CC_ARM   )
#pragma push
#pragma anon_unions
#endif

#ifndef ADI_USEQ_REGS_MAP1_TYPEDEFS_H_
#define ADI_USEQ_REGS_MAP1_TYPEDEFS_H_

/** @defgroup SEQUENCESTARTADDR Sequence Start Address Register (SEQUENCESTARTADDR) Register
 *  Sequence Start Address Register (SEQUENCESTARTADDR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_SEQUENCESTARTADDR_t
 *  \brief Sequence Start Address Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_SEQUENCESTARTADDR_t {
    union {
        struct {
             unsigned int SEQUENCE_START_ADDR : 12; /**< No description provided */
             unsigned int RESERVED12       : 4; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_SEQUENCESTARTADDR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SEQUENCEENDADDR Sequence End Address Register (SEQUENCEENDADDR) Register
 *  Sequence End Address Register (SEQUENCEENDADDR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_SEQUENCEENDADDR_t
 *  \brief Sequence End Address Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_SEQUENCEENDADDR_t {
    union {
        struct {
             unsigned int SEQUENCE_END_ADDR : 12; /**< No description provided */
             unsigned int RESERVED12       : 4; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_SEQUENCEENDADDR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USEQCONTROLREGISTER Useq control Register (USEQCONTROLREGISTER) Register
 *  Useq control Register (USEQCONTROLREGISTER) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQCONTROLREGISTER_t
 *  \brief Useq control Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_USEQCONTROLREGISTER_t {
    union {
        struct {
             unsigned int START_EXEC       : 1; /**< No description provided */
             unsigned int STOP_EXEC        : 1; /**< No description provided */
             unsigned int LOAD_PAD         : 1; /**< No description provided */
             unsigned int UPDATE_SEQRAM    : 1; /**< No description provided */
             unsigned int UPDATE_WAVERAM   : 1; /**< No description provided */
             unsigned int UPDATE_MAPRAM    : 1; /**< No description provided */
             unsigned int LOAD_LC          : 1; /**< No description provided */
             unsigned int LOAD_POKE        : 1; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQCONTROLREGISTER_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FRAMESYNCCTRL FrameSync Control Register (FRAMESYNCCTRL) Register
 *  FrameSync Control Register (FRAMESYNCCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_FRAMESYNCCTRL_t
 *  \brief FrameSync Control Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_FRAMESYNCCTRL_t {
    union {
        struct {
             unsigned int EN_EXT_SYNC      : 1; /**< No description provided */
             unsigned int EXT_SYNC_MODE    : 1; /**< No description provided */
             unsigned int EXT_SYNC_CNT     : 4; /**< No description provided */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_FRAMESYNCCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup BREAKPOINTCTRL The breakpoint feature is used by the software for debugging purposes. (BREAKPOINTCTRL) Register
 *  The breakpoint feature is used by the software for debugging purposes. (BREAKPOINTCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_BREAKPOINTCTRL_t
 *  \brief The breakpoint feature is used by the software for debugging purposes.
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_BREAKPOINTCTRL_t {
    union {
        struct {
             unsigned int BRK              : 1; /**< No description provided */
             unsigned int BRK_RES          : 1; /**< No description provided */
             unsigned int SEQ_ENABLE       : 1; /**< No description provided */
             unsigned int BRK_SEQ_RAM_ADDR : 12; /**< No description provided */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_BREAKPOINTCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup UPDATESTAMP Update Stamp Value Register (UPDATESTAMP) Register
 *  Update Stamp Value Register (UPDATESTAMP) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_UPDATESTAMP_t
 *  \brief Update Stamp Value Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_UPDATESTAMP_t {
    union {
        struct {
             unsigned int UPDATE_STAMP     : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_UPDATESTAMP_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DIGPWRDOWN Controls clock gates for various digital circuit power-down (DIGPWRDOWN) Register
 *  Controls clock gates for various digital circuit power-down (DIGPWRDOWN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_DIGPWRDOWN_t
 *  \brief Controls clock gates for various digital circuit power-down
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_DIGPWRDOWN_t {
    union {
        struct {
             unsigned int PD_DE            : 1; /**< No description provided */
             unsigned int RESERVED1        : 2; /**< Reserved */
             unsigned int PD_CSI           : 1; /**< No description provided */
             unsigned int PD_DATAPATH      : 1; /**< No description provided */
             unsigned int RESERVED5        : 3; /**< Reserved */
             unsigned int PD_SPI_MASTER    : 1; /**< No description provided */
             unsigned int RESERVED9        : 2; /**< Reserved */
             unsigned int PD_PCM           : 1; /**< No description provided */
             unsigned int PD_LPS2          : 1; /**< No description provided */
             unsigned int PD_SS            : 1; /**< No description provided */
             unsigned int PD_LPS1          : 1; /**< No description provided */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_DIGPWRDOWN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PIXGAINTAG1LATCHCTRL DEPRECATED: This register is deprecated and should not be used. (PIXGAINTAG1LATCHCTRL) Register
 *  DEPRECATED: This register is deprecated and should not be used. (PIXGAINTAG1LATCHCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_t
 *  \brief DEPRECATED: This register is deprecated and should not be used.
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_t {
    union {
        struct {
             unsigned int PIXGAINTAG1_LATCH_EN : 1; /**< No description provided */
             unsigned int PIXGAINTAG1_LATCH_RISE_CNVT_CNT : 2; /**< No description provided */
             unsigned int PIXGAINTAG1_LATCH_RISE_CLK_CNT : 4; /**< No description provided */
             unsigned int PIXGAINTAG1_LATCH_FALL_CNVT_CNT : 2; /**< No description provided */
             unsigned int PIXGAINTAG1_LATCH_FALL_CLK_CNT : 4; /**< No description provided */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_PIXGAINTAG1LATCHCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PIXGAINTAG1READOUTCTRL DEPRECATED: This register is deprecated and should not be used. (PIXGAINTAG1READOUTCTRL) Register
 *  DEPRECATED: This register is deprecated and should not be used. (PIXGAINTAG1READOUTCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_PIXGAINTAG1READOUTCTRL_t
 *  \brief DEPRECATED: This register is deprecated and should not be used.
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_PIXGAINTAG1READOUTCTRL_t {
    union {
        struct {
             unsigned int PIXGAINTAG1_READOUT_CNVT_CNT : 2; /**< No description provided */
             unsigned int PIXGAINTAG1_READOUT_CLK_CNT : 4; /**< No description provided */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_PIXGAINTAG1READOUTCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PIXSATURATELATCHCTRL DEPRECATED: This register is deprecated and should not be used. (PIXSATURATELATCHCTRL) Register
 *  DEPRECATED: This register is deprecated and should not be used. (PIXSATURATELATCHCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_t
 *  \brief DEPRECATED: This register is deprecated and should not be used.
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_t {
    union {
        struct {
             unsigned int PIXSATURATE_LATCH_EN : 1; /**< No description provided */
             unsigned int PIXSATURATE_LATCH_RISE_CNVT_CNT : 2; /**< No description provided */
             unsigned int PIXSATURATE_LATCH_RISE_CLK_CNT : 4; /**< No description provided */
             unsigned int PIXSATURATE_LATCH_FALL_CNVT_CNT : 2; /**< No description provided */
             unsigned int PIXSATURATE_LATCH_FALL_CLK_CNT : 4; /**< No description provided */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_PIXSATURATELATCHCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PIXSATURATEREADOUTCTRL DEPRECATED: This register is deprecated and should not be used. (PIXSATURATEREADOUTCTRL) Register
 *  DEPRECATED: This register is deprecated and should not be used. (PIXSATURATEREADOUTCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_PIXSATURATEREADOUTCTRL_t
 *  \brief DEPRECATED: This register is deprecated and should not be used.
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_PIXSATURATEREADOUTCTRL_t {
    union {
        struct {
             unsigned int PIXSATURATE_READOUT_CNVT_CNT : 2; /**< No description provided */
             unsigned int PIXSATURATE_READOUT_CLK_CNT : 4; /**< No description provided */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_PIXSATURATEREADOUTCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ROWCNTINCRCONTROL DEPRECATED: This register is deprecated and should not be used. (ROWCNTINCRCONTROL) Register
 *  DEPRECATED: This register is deprecated and should not be used. (ROWCNTINCRCONTROL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_t
 *  \brief DEPRECATED: This register is deprecated and should not be used.
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_t {
    union {
        struct {
             unsigned int AUTO_ROW_CNT_INCR_EN : 1; /**< No description provided */
             unsigned int NO_OF_ADC_CONVERT : 3; /**< No description provided */
             unsigned int NO_OF_POSEDGE_CLKCYCLE : 4; /**< No description provided */
             unsigned int AUTO_ROW_INCR_STRIDE : 4; /**< No description provided */
             unsigned int AUTO_ROW_INCR_DIR : 2; /**< No description provided */
             unsigned int RESERVED14       : 2; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_ROWCNTINCRCONTROL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PIXGAINTAG0LATCHCTRL DEPRECATED: This register is deprecated and should not be used. (PIXGAINTAG0LATCHCTRL) Register
 *  DEPRECATED: This register is deprecated and should not be used. (PIXGAINTAG0LATCHCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_t
 *  \brief DEPRECATED: This register is deprecated and should not be used.
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_t {
    union {
        struct {
             unsigned int PIXGAINTAG0_LATCH_EN : 1; /**< No description provided */
             unsigned int PIXGAINTAG0_LATCH_RISE_CNVT_CNT : 2; /**< No description provided */
             unsigned int PIXGAINTAG0_LATCH_RISE_CLK_CNT : 4; /**< No description provided */
             unsigned int PIXGAINTAG0_LATCH_FALL_CNVT_CNT : 2; /**< No description provided */
             unsigned int PIXGAINTAG0_LATCH_FALL_CLK_CNT : 4; /**< No description provided */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_PIXGAINTAG0LATCHCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PIXGAINTAG0READOUTCTRL DEPRECATED: This register is deprecated and should not be used. (PIXGAINTAG0READOUTCTRL) Register
 *  DEPRECATED: This register is deprecated and should not be used. (PIXGAINTAG0READOUTCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_PIXGAINTAG0READOUTCTRL_t
 *  \brief DEPRECATED: This register is deprecated and should not be used.
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_PIXGAINTAG0READOUTCTRL_t {
    union {
        struct {
             unsigned int PIXGAINTAG0_READOUT_CNVT_CNT : 2; /**< No description provided */
             unsigned int PIXGAINTAG0_READOUT_CLK_CNT : 4; /**< No description provided */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_PIXGAINTAG0READOUTCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup I2CCTRL Configures Read prefetching on I2C reads (I2CCTRL) Register
 *  Configures Read prefetching on I2C reads (I2CCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_I2CCTRL_t
 *  \brief Configures Read prefetching on I2C reads
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_I2CCTRL_t {
    union {
        struct {
             unsigned int I2CRDPREFETCHENABLE : 1; /**< No description provided */
             unsigned int I2CBURSTMODE     : 2; /**< No description provided */
             unsigned int RESERVED3        : 13; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_I2CCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SEQUENCESTATUS Sequence Status Register (SEQUENCESTATUS) Register
 *  Sequence Status Register (SEQUENCESTATUS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_SEQUENCESTATUS_t
 *  \brief Sequence Status Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_SEQUENCESTATUS_t {
    union {
        struct {
             unsigned int CALL_STACK_NO_LSB : 3; /**< No description provided */
             unsigned int CALL_RPT_CNT_LSB : 8; /**< No description provided */
             unsigned int SEQ_STATE_MACHINE_STS : 1; /**< No description provided */
             unsigned int RESERVED12       : 3; /**< Reserved */
             unsigned int CALL_STACK_NO_MSB : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_SEQUENCESTATUS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SYSTEMCLOCKCONTROL Clock Control Register (SYSTEMCLOCKCONTROL) Register
 *  Clock Control Register (SYSTEMCLOCKCONTROL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_SYSTEMCLOCKCONTROL_t
 *  \brief Clock Control Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_SYSTEMCLOCKCONTROL_t {
    union {
        struct {
             unsigned int CLK_SYS_SEL      : 1; /**< No description provided */
             unsigned int RESERVED1        : 1; /**< Reserved */
             unsigned int CLK_PROC_SEL     : 1; /**< No description provided */
             unsigned int RESERVED3        : 1; /**< Reserved */
             unsigned int CLK_DE_SEL       : 1; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_SYSTEMCLOCKCONTROL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CALLRPTCOUNT Call Repeat Count Value (CALLRPTCOUNT) Register
 *  Call Repeat Count Value (CALLRPTCOUNT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_CALLRPTCOUNT_t
 *  \brief Call Repeat Count Value
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_CALLRPTCOUNT_t {
    union {
        struct {
             unsigned int CALL_RPT_CNT     : 12; /**< No description provided */
             unsigned int RESERVED12       : 4; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_CALLRPTCOUNT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GTSWAP DEPRECATED: This register is deprecated and should not be used. (GTSWAP) Register
 *  DEPRECATED: This register is deprecated and should not be used. (GTSWAP) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GTSWAP_t
 *  \brief DEPRECATED: This register is deprecated and should not be used.
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GTSWAP_t {
    union {
        struct {
             unsigned int GT01_SWAP        : 1; /**< No description provided */
             unsigned int AMP_CLK_FINAL_INVERT : 1; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GTSWAP_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup INTERRUPTENABLE Interrupt Enable Register (INTERRUPTENABLE) Register
 *  Interrupt Enable Register (INTERRUPTENABLE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_INTERRUPTENABLE_t
 *  \brief Interrupt Enable Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_INTERRUPTENABLE_t {
    union {
        struct {
             unsigned int INTERRUPT_PSEUDO_ERR_EN : 1; /**< No description provided */
             unsigned int INTERRUPT_CAL_STACK_OVRFLOW_ERR_EN : 1; /**< No description provided */
             unsigned int INTERRUPT_CAL_STACK_UNDR_RUN_ERR_EN : 1; /**< No description provided */
             unsigned int INTERRUPT_INVALID_OPCOD_ERR_EN : 1; /**< No description provided */
             unsigned int INTERRUPT_CSI_TX_PKT_CMD_ERR_EN : 1; /**< No description provided */
             unsigned int INTERRUPT_FIRMWARE_PARITY_ERR_EN : 1; /**< No description provided */
             unsigned int INTERRUPT_APB_TIMEOUT_ERR_EN : 1; /**< No description provided */
             unsigned int INTERRUPT_COLCORRECT_PARITY_ERR_EN : 1; /**< No description provided */
             unsigned int INTERRUPT_MIPI_CSI2_UNDERFLOW_ERR_EN : 1; /**< No description provided */
             unsigned int INTERRUPT_COMPRESSION_PARITY_ERR_EN : 1; /**< No description provided */
             unsigned int INTERRUPT_REG_WR_ERR_EN : 1; /**< No description provided */
             unsigned int INTERRUPT_INVALID_OPERAND_ERR_EN : 1; /**< No description provided */
             unsigned int INTERRUPT_USER_DEFINED_ERR_EN : 4; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_INTERRUPTENABLE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ERRORSET Error Set Register (ERRORSET) Register
 *  Error Set Register (ERRORSET) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_ERRORSET_t
 *  \brief Error Set Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_ERRORSET_t {
    union {
        struct {
             unsigned int SET_PSEUDO_ERROR : 1; /**< No description provided */
             unsigned int RESERVED1        : 11; /**< Reserved */
             unsigned int SET_USER_DEFINED_ERROR : 4; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_ERRORSET_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ERRORSTATUS Error Status Register (ERRORSTATUS) Register
 *  Error Status Register (ERRORSTATUS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_ERRORSTATUS_t
 *  \brief Error Status Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_ERRORSTATUS_t {
    union {
        struct {
             unsigned int PSEUDO_ERROR     : 1; /**< No description provided */
             unsigned int CALL_STACK_OVERFLOW_ERROR : 1; /**< No description provided */
             unsigned int CALL_STACK_UNDERRUN_ERROR : 1; /**< No description provided */
             unsigned int INVALID_OPCODE_ERROR : 1; /**< No description provided */
             unsigned int BUF_CSI_TX_PKT_CMD_ERROR : 1; /**< No description provided */
             unsigned int FIRMWARE_PARITY_ERROR : 1; /**< No description provided */
             unsigned int APB_TIMEOUT_ERROR : 1; /**< No description provided */
             unsigned int COLCORRECT_PARITY_ERROR : 1; /**< No description provided */
             unsigned int MIPI_CSI_2_UNDERFLOW_ERROR : 1; /**< No description provided */
             unsigned int COMPRESSION_PARITY_ERROR : 1; /**< No description provided */
             unsigned int REG_WR_ERROR     : 1; /**< No description provided */
             unsigned int INVALID_OPERAND_ERROR : 1; /**< No description provided */
             unsigned int USER_DEFINED_ERROR : 4; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_ERRORSTATUS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPIOCTRL No description provided (GPIOCTRL) Register
 *  No description provided (GPIOCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPIOCTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPIOCTRL_t {
    union {
        struct {
             unsigned int GPIO_DIRECTION   : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPIOCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPIOINPUT No description provided (GPIOINPUT) Register
 *  No description provided (GPIOINPUT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPIOINPUT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPIOINPUT_t {
    union {
        struct {
             unsigned int GPIO_IN          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPIOINPUT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPIOOUTPUTSET No description provided (GPIOOUTPUTSET) Register
 *  No description provided (GPIOOUTPUTSET) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPIOOUTPUTSET_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPIOOUTPUTSET_t {
    union {
        struct {
             unsigned int GPIO_OUT         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPIOOUTPUTSET_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPIOOUTPUTCLR No description provided (GPIOOUTPUTCLR) Register
 *  No description provided (GPIOOUTPUTCLR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPIOOUTPUTCLR_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPIOOUTPUTCLR_t {
    union {
        struct {
             unsigned int GPIO_OUT         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPIOOUTPUTCLR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PIXELINTERFACECTRL No description provided (PIXELINTERFACECTRL) Register
 *  No description provided (PIXELINTERFACECTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_PIXELINTERFACECTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_PIXELINTERFACECTRL_t {
    union {
        struct {
             unsigned int READ             : 1; /**< No description provided */
             unsigned int GLOBAL_RESET     : 1; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_PIXELINTERFACECTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ROWCNTINCRCONTROL2 No description provided (ROWCNTINCRCONTROL2) Register
 *  No description provided (ROWCNTINCRCONTROL2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_ROWCNTINCRCONTROL2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_ROWCNTINCRCONTROL2_t {
    union {
        struct {
             unsigned int RESERVED0        : 4; /**< Reserved */
             unsigned int AUTO_ROW_INCR_STRIDE_MSB : 5; /**< No description provided */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_ROWCNTINCRCONTROL2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPIOFSYNCSNAPSHOT No description provided (GPIOFSYNCSNAPSHOT) Register
 *  No description provided (GPIOFSYNCSNAPSHOT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPIOFSYNCSNAPSHOT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPIOFSYNCSNAPSHOT_t {
    union {
        struct {
             unsigned int GPIO_FSYNC_SNAPSHOT : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPIOFSYNCSNAPSHOT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup WAITFORSYNCSOURCE No description provided (WAITFORSYNCSOURCE) Register
 *  No description provided (WAITFORSYNCSOURCE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_t {
    union {
        struct {
             unsigned int DATAPATH_DONE_SOURCE : 1; /**< No description provided */
             unsigned int SHIFT_CHAIN_DONE_SOURCE : 1; /**< No description provided */
             unsigned int MIPI_ULPS_END_SOURCE : 1; /**< No description provided */
             unsigned int DE_DONE_SOURCE   : 1; /**< No description provided */
             unsigned int SPIM_WRITE_DONE_SOURCE : 1; /**< No description provided */
             unsigned int SPIM_READ_DONE   : 1; /**< No description provided */
             unsigned int ADCPLL_LOCK_SOURCE : 1; /**< No description provided */
             unsigned int TEMP_SENSOR_DONE_SOURCE : 1; /**< No description provided */
             unsigned int GPIO2_SOURCE     : 1; /**< No description provided */
             unsigned int GPIO3_SOURCE     : 1; /**< No description provided */
             unsigned int GPIO4_SOURCE     : 1; /**< No description provided */
             unsigned int GPIO5_SOURCE     : 1; /**< No description provided */
             unsigned int GPIO6_SOURCE     : 1; /**< No description provided */
             unsigned int GPIO7_SOURCE     : 1; /**< No description provided */
             unsigned int GPIO8_SOURCE     : 1; /**< No description provided */
             unsigned int GPIO9_SOURCE     : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_WAITFORSYNCSOURCE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CTIMECTRL No description provided (CTIMECTRL) Register
 *  No description provided (CTIMECTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_CTIMECTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_CTIMECTRL_t {
    union {
        struct {
             unsigned int CTIME_ENABLE     : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_CTIMECTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CTIME_0 No description provided (CTIME_0) Register
 *  No description provided (CTIME_0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_CTIME_0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_CTIME_0_t {
    union {
        struct {
             unsigned int CTIME_LO         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_CTIME_0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CTIME_1 No description provided (CTIME_1) Register
 *  No description provided (CTIME_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_CTIME_1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_CTIME_1_t {
    union {
        struct {
             unsigned int CTIME_HI         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_CTIME_1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CTIME_2 No description provided (CTIME_2) Register
 *  No description provided (CTIME_2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_CTIME_2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_CTIME_2_t {
    union {
        struct {
             unsigned int CAPTURE_START_TIME_LO : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_CTIME_2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CTIME_3 No description provided (CTIME_3) Register
 *  No description provided (CTIME_3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_CTIME_3_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_CTIME_3_t {
    union {
        struct {
             unsigned int CAPTURE_START_TIME_HI : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_CTIME_3_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CTIME_4 No description provided (CTIME_4) Register
 *  No description provided (CTIME_4) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_CTIME_4_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_CTIME_4_t {
    union {
        struct {
             unsigned int CAPTURE_END_TIME_LO : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_CTIME_4_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CTIME_5 No description provided (CTIME_5) Register
 *  No description provided (CTIME_5) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_CTIME_5_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_CTIME_5_t {
    union {
        struct {
             unsigned int CAPTURE_END_TIME_HI : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_CTIME_5_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SOFT_RESET No description provided (SOFT_RESET) Register
 *  No description provided (SOFT_RESET) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_SOFT_RESET_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_SOFT_RESET_t {
    union {
        struct {
             unsigned int SOFT_RESET_DE    : 1; /**< No description provided */
             unsigned int RESERVED1        : 2; /**< Reserved */
             unsigned int SOFT_RESET_CSI   : 1; /**< No description provided */
             unsigned int SOFT_RESET_DATAPATH : 1; /**< No description provided */
             unsigned int RESERVED5        : 3; /**< Reserved */
             unsigned int SOFT_RESET_SPI_MASTER : 1; /**< No description provided */
             unsigned int SOFT_RESET_BOOT_MEM : 1; /**< No description provided */
             unsigned int RESERVED10       : 1; /**< Reserved */
             unsigned int SOFT_RESET_PCM   : 1; /**< No description provided */
             unsigned int SOFT_RESET_LPS2  : 1; /**< No description provided */
             unsigned int SOFT_RESET_SS    : 1; /**< No description provided */
             unsigned int SOFT_RESET_LPS1  : 1; /**< No description provided */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_SOFT_RESET_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup WAITFORSYNCPOLARITY No description provided (WAITFORSYNCPOLARITY) Register
 *  No description provided (WAITFORSYNCPOLARITY) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_WAITFORSYNCPOLARITY_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_WAITFORSYNCPOLARITY_t {
    union {
        struct {
             unsigned int WAIT_FOR_SYNC_POL : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_WAITFORSYNCPOLARITY_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USEQ_DFT MIcrosequencer ram DFT controls (USEQ_DFT) Register
 *  MIcrosequencer ram DFT controls (USEQ_DFT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQ_DFT_t
 *  \brief MIcrosequencer ram DFT controls
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_USEQ_DFT_t {
    union {
        struct {
             unsigned int SEQRAM_MARGIN    : 2; /**< SeqRAM Margin */
             unsigned int SEQRAM_DST       : 1; /**< SeqRAM DST (Disable Self Timing) Register */
             unsigned int RESERVED3        : 1; /**< Reserved */
             unsigned int WAVERAM_MARGIN   : 2; /**< WaveRAM Margin */
             unsigned int WAVERAM_DST      : 1; /**< WaveRAM DST (Disable Self Timing) Register */
             unsigned int RESERVED7        : 1; /**< Reserved */
             unsigned int MAPRAM_MARGIN    : 1; /**< WaveRAM Margin */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQ_DFT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USEQ_PARITY Microsequener parity error bits (USEQ_PARITY) Register
 *  Microsequener parity error bits (USEQ_PARITY) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQ_PARITY_t
 *  \brief Microsequener parity error bits
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_USEQ_PARITY_t {
    union {
        struct {
             unsigned int SEQRAM_PARITY_ERR : 1; /**< Parity Error in Sequence RAM */
             unsigned int MAPRAM_PARITY_ERR : 1; /**< Parity error in map RAM */
             unsigned int WAVERAM_PARITY_ERR : 1; /**< Parity error in wave RAM */
             unsigned int RESERVED3        : 13; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQ_PARITY_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup HSP_DFT HSP RAM DFT controls (HSP_DFT) Register
 *  HSP RAM DFT controls (HSP_DFT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_HSP_DFT_t
 *  \brief HSP RAM DFT controls
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_HSP_DFT_t {
    union {
        struct {
             unsigned int HSP_SPRAM_MARGIN : 2; /**< HSP SPRAM Margin */
             unsigned int HSP_SPRAM_DST    : 1; /**< HSP SPRAM DST (Disable Self Timing) Register */
             unsigned int RESERVED3        : 1; /**< Reserved */
             unsigned int HSP_SPROM_MARGIN : 1; /**< HSP SPROM Margin */
             unsigned int HSP_1024X38_MARGIN : 1; /**< HSP 1024x38 Margin */
             unsigned int HSP_256X76_MARGIN : 1; /**< HSP 256x76 Margin */
             unsigned int HSP_128X76_MARGIN : 1; /**< HSP 128x76 Margin */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_HSP_DFT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PCCOND No description provided (PCCOND) Register
 *  No description provided (PCCOND) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_PCCOND_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_PCCOND_t {
    union {
        struct {
             unsigned int PC               : 12; /**< No description provided */
             unsigned int RESERVED12       : 3; /**< Reserved */
             unsigned int COND             : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_PCCOND_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR0 No description provided (GPRR0) Register
 *  No description provided (GPRR0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR0_t {
    union {
        struct {
             unsigned int GPR_R0           : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR1 No description provided (GPRR1) Register
 *  No description provided (GPRR1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR1_t {
    union {
        struct {
             unsigned int GPR_R1           : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR2 No description provided (GPRR2) Register
 *  No description provided (GPRR2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR2_t {
    union {
        struct {
             unsigned int GPR_R2           : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR3 No description provided (GPRR3) Register
 *  No description provided (GPRR3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR3_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR3_t {
    union {
        struct {
             unsigned int GPR_R3           : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR3_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR4 No description provided (GPRR4) Register
 *  No description provided (GPRR4) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR4_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR4_t {
    union {
        struct {
             unsigned int GPR_R4           : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR4_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR5 No description provided (GPRR5) Register
 *  No description provided (GPRR5) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR5_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR5_t {
    union {
        struct {
             unsigned int GPR_R5           : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR5_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR6 No description provided (GPRR6) Register
 *  No description provided (GPRR6) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR6_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR6_t {
    union {
        struct {
             unsigned int GPR_R6           : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR6_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR7 No description provided (GPRR7) Register
 *  No description provided (GPRR7) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR7_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR7_t {
    union {
        struct {
             unsigned int GPR_R7           : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR7_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR8 No description provided (GPRR8) Register
 *  No description provided (GPRR8) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR8_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR8_t {
    union {
        struct {
             unsigned int GPR_R8           : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR8_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR9 No description provided (GPRR9) Register
 *  No description provided (GPRR9) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR9_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR9_t {
    union {
        struct {
             unsigned int GPR_R9           : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR9_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR10 No description provided (GPRR10) Register
 *  No description provided (GPRR10) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR10_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR10_t {
    union {
        struct {
             unsigned int GPR_R10          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR10_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR11 No description provided (GPRR11) Register
 *  No description provided (GPRR11) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR11_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR11_t {
    union {
        struct {
             unsigned int GPR_R11          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR11_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR12 No description provided (GPRR12) Register
 *  No description provided (GPRR12) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR12_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR12_t {
    union {
        struct {
             unsigned int GPR_R12          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR12_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR13 No description provided (GPRR13) Register
 *  No description provided (GPRR13) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR13_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR13_t {
    union {
        struct {
             unsigned int GPR_R13          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR13_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR14 No description provided (GPRR14) Register
 *  No description provided (GPRR14) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR14_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR14_t {
    union {
        struct {
             unsigned int GPR_R14          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR14_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR15 No description provided (GPRR15) Register
 *  No description provided (GPRR15) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR15_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR15_t {
    union {
        struct {
             unsigned int GPR_R15          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR15_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMPCLKCTRL No description provided (AMPCLKCTRL) Register
 *  No description provided (AMPCLKCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_AMPCLKCTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_AMPCLKCTRL_t {
    union {
        struct {
             unsigned int AMP_CLK_RISE_CNVT_CNT : 2; /**< No description provided */
             unsigned int AMP_CLK_RISE_CLK_CNT : 4; /**< No description provided */
             unsigned int AMP_CLK_FALL_CNVT_CNT : 2; /**< No description provided */
             unsigned int AMP_CLK_FALL_CLK_CNT : 4; /**< No description provided */
             unsigned int RESERVED12       : 4; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_AMPCLKCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMPCLK2CTRL No description provided (AMPCLK2CTRL) Register
 *  No description provided (AMPCLK2CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_AMPCLK2CTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_AMPCLK2CTRL_t {
    union {
        struct {
             unsigned int AMP_CLK2_RISE_CNVT_CNT : 2; /**< No description provided */
             unsigned int AMP_CLK2_RISE_CLK_CNT : 4; /**< No description provided */
             unsigned int AMP_CLK2_FALL_CNVT_CNT : 2; /**< No description provided */
             unsigned int AMP_CLK2_FALL_CLK_CNT : 4; /**< No description provided */
             unsigned int RESERVED12       : 4; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_AMPCLK2CTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMPCLK3CTRL1 No description provided (AMPCLK3CTRL1) Register
 *  No description provided (AMPCLK3CTRL1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_AMPCLK3CTRL1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_AMPCLK3CTRL1_t {
    union {
        struct {
             unsigned int AMP_CLK3_RISE_CLK_CNT : 8; /**< No description provided */
             unsigned int AMP_CLK3_FALL_CLK_CNT : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_AMPCLK3CTRL1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMPCLK3CTRL2 No description provided (AMPCLK3CTRL2) Register
 *  No description provided (AMPCLK3CTRL2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_AMPCLK3CTRL2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_AMPCLK3CTRL2_t {
    union {
        struct {
             unsigned int AMP_CLK3_RISE_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED2        : 4; /**< Reserved */
             unsigned int AMP_CLK3_FALL_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_AMPCLK3CTRL2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup NOISERESETCTRL1 No description provided (NOISERESETCTRL1) Register
 *  No description provided (NOISERESETCTRL1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_NOISERESETCTRL1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_NOISERESETCTRL1_t {
    union {
        struct {
             unsigned int NOISE_RESET_RISE_CLK_CNT : 8; /**< No description provided */
             unsigned int NOISE_RESET_FALL_CLK_CNT : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_NOISERESETCTRL1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup NOISERESETCTRL2 No description provided (NOISERESETCTRL2) Register
 *  No description provided (NOISERESETCTRL2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_NOISERESETCTRL2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_NOISERESETCTRL2_t {
    union {
        struct {
             unsigned int NOISE_RESET_RISE_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED2        : 4; /**< Reserved */
             unsigned int NOISE_RESET_FALL_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_NOISERESETCTRL2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PIXRESETCTRL1 No description provided (PIXRESETCTRL1) Register
 *  No description provided (PIXRESETCTRL1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_PIXRESETCTRL1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_PIXRESETCTRL1_t {
    union {
        struct {
             unsigned int PIX_RESET_RISE_CLK_CNT : 8; /**< No description provided */
             unsigned int PIX_RESET_FALL_CLK_CNT : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_PIXRESETCTRL1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PIXRESETCTRL2 No description provided (PIXRESETCTRL2) Register
 *  No description provided (PIXRESETCTRL2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_PIXRESETCTRL2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_PIXRESETCTRL2_t {
    union {
        struct {
             unsigned int PIX_RESET_RISE_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED2        : 4; /**< Reserved */
             unsigned int PIX_RESET_FALL_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_PIXRESETCTRL2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPIOPINFUNC1 No description provided (GPIOPINFUNC1) Register
 *  No description provided (GPIOPINFUNC1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPIOPINFUNC1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPIOPINFUNC1_t {
    union {
        struct {
             unsigned int GPIO0_PIN_FUNC   : 2; /**< No description provided */
             unsigned int GPIO1_PIN_FUNC   : 2; /**< No description provided */
             unsigned int GPIO2_PIN_FUNC   : 2; /**< No description provided */
             unsigned int GPIO3_PIN_FUNC   : 2; /**< No description provided */
             unsigned int GPIO4_PIN_FUNC   : 2; /**< No description provided */
             unsigned int GPIO5_PIN_FUNC   : 2; /**< No description provided */
             unsigned int GPIO6_PIN_FUNC   : 2; /**< No description provided */
             unsigned int GPIO7_PIN_FUNC   : 2; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPIOPINFUNC1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPIOPINFUNC2 No description provided (GPIOPINFUNC2) Register
 *  No description provided (GPIOPINFUNC2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPIOPINFUNC2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPIOPINFUNC2_t {
    union {
        struct {
             unsigned int GPIO8_PIN_FUNC   : 2; /**< No description provided */
             unsigned int GPIO9_PIN_FUNC   : 2; /**< No description provided */
             unsigned int GPIO10_PIN_FUNC  : 2; /**< No description provided */
             unsigned int GPIO11_PIN_FUNC  : 2; /**< No description provided */
             unsigned int GPIO12_PIN_FUNC  : 2; /**< No description provided */
             unsigned int GPIO13_PIN_FUNC  : 2; /**< No description provided */
             unsigned int GPIO14_PIN_FUNC  : 2; /**< No description provided */
             unsigned int GPIO15_PIN_FUNC  : 2; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPIOPINFUNC2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USEQ_DBGMUX No description provided (USEQ_DBGMUX) Register
 *  No description provided (USEQ_DBGMUX) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQ_DBGMUX_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_USEQ_DBGMUX_t {
    union {
        struct {
             unsigned int MUXGRP_SEL       : 3; /**< No description provided */
             unsigned int RESERVED3        : 5; /**< Reserved */
             unsigned int CMN_SEL0         : 3; /**< No description provided */
             unsigned int CMN_SEL1         : 3; /**< No description provided */
             unsigned int RESERVED14       : 1; /**< Reserved */
             unsigned int DBG_ENABLE       : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQ_DBGMUX_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USEQ_CHIP_DBGMUX No description provided (USEQ_CHIP_DBGMUX) Register
 *  No description provided (USEQ_CHIP_DBGMUX) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_t {
    union {
        struct {
             unsigned int IPMUX_SEL        : 4; /**< No description provided */
             unsigned int RESERVED4        : 3; /**< Reserved */
             unsigned int DBG_ENABLE       : 1; /**< No description provided */
             unsigned int MM_IPMUX_SEL     : 2; /**< No description provided */
             unsigned int RESERVED10       : 5; /**< Reserved */
             unsigned int MM_ENABLE        : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQ_CHIP_DBGMUX_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup MM_CTRL No description provided (MM_CTRL) Register
 *  No description provided (MM_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_MM_CTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_MM_CTRL_t {
    union {
        struct {
             unsigned int MM_OUT_SEL       : 4; /**< No description provided */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_MM_CTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ERRJMPADDR No description provided (ERRJMPADDR) Register
 *  No description provided (ERRJMPADDR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_ERRJMPADDR_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_ERRJMPADDR_t {
    union {
        struct {
             unsigned int USEQERRJUMPADDR  : 12; /**< No description provided */
             unsigned int RESERVED12       : 4; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_ERRJMPADDR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup STOPERRENA No description provided (STOPERRENA) Register
 *  No description provided (STOPERRENA) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_STOPERRENA_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_STOPERRENA_t {
    union {
        struct {
             unsigned int PSEUDO_ERREN     : 1; /**< No description provided */
             unsigned int CALL_STACK_OVERFLOW_ERREN : 1; /**< No description provided */
             unsigned int CALL_STACK_UNDERRUN_ERREN : 1; /**< No description provided */
             unsigned int INVALID_OPCODE_ERREN : 1; /**< No description provided */
             unsigned int BUF_MEMORYT_OVERFLOW_ERREN : 1; /**< No description provided */
             unsigned int FIRMWARE_PARITY_ERREN : 1; /**< No description provided */
             unsigned int FRAME_HEADERT_OVERFLOW_ERREN : 1; /**< No description provided */
             unsigned int COLCORRECT_PARITY_ERREN : 1; /**< No description provided */
             unsigned int MIPI_CSI_2T_UNDERFLOW_ERREN : 1; /**< No description provided */
             unsigned int COMPRESSON_PARITY_ERREN : 1; /**< No description provided */
             unsigned int REG_WR_ERREN     : 1; /**< No description provided */
             unsigned int INVALID_OPERAND_ERREN : 1; /**< No description provided */
             unsigned int USER_DEFINED_ERREN : 4; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_STOPERRENA_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ADCCNVTCTRL1 No description provided (ADCCNVTCTRL1) Register
 *  No description provided (ADCCNVTCTRL1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_ADCCNVTCTRL1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_ADCCNVTCTRL1_t {
    union {
        struct {
             unsigned int ASTART           : 1; /**< No description provided */
             unsigned int REPEAT_COUNT     : 12; /**< No description provided */
             unsigned int RESERVED13       : 1; /**< Reserved */
             unsigned int POL              : 1; /**< No description provided */
             unsigned int ABUSY            : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_ADCCNVTCTRL1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ADCCNVTCTRL2 No description provided (ADCCNVTCTRL2) Register
 *  No description provided (ADCCNVTCTRL2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_ADCCNVTCTRL2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_ADCCNVTCTRL2_t {
    union {
        struct {
             unsigned int EXEC_LENGTH      : 14; /**< No description provided */
             unsigned int CDIV             : 2; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_ADCCNVTCTRL2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ADCCNVTCTRL3 No description provided (ADCCNVTCTRL3) Register
 *  No description provided (ADCCNVTCTRL3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_ADCCNVTCTRL3_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_ADCCNVTCTRL3_t {
    union {
        struct {
             unsigned int FALL_TIME        : 8; /**< No description provided */
             unsigned int RISE_TIME        : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_ADCCNVTCTRL3_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ADCCNVTCTRL4 No description provided (ADCCNVTCTRL4) Register
 *  No description provided (ADCCNVTCTRL4) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_ADCCNVTCTRL4_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_ADCCNVTCTRL4_t {
    union {
        struct {
             unsigned int ADC_CNVT_DELAY   : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_ADCCNVTCTRL4_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GAINTAG1CLKCTRL1 No description provided (GAINTAG1CLKCTRL1) Register
 *  No description provided (GAINTAG1CLKCTRL1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GAINTAG1CLKCTRL1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GAINTAG1CLKCTRL1_t {
    union {
        struct {
             unsigned int GAINTAG1_CLK_RISE_CLK_CNT : 8; /**< No description provided */
             unsigned int GAINTAG1_CLK_FALL_CLK_CNT : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GAINTAG1CLKCTRL1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GAINTAG1CLKCTRL2 No description provided (GAINTAG1CLKCTRL2) Register
 *  No description provided (GAINTAG1CLKCTRL2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GAINTAG1CLKCTRL2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GAINTAG1CLKCTRL2_t {
    union {
        struct {
             unsigned int GAINTAG1_CLK_RISE_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED2        : 4; /**< Reserved */
             unsigned int GAINTAG1_CLK_FALL_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GAINTAG1CLKCTRL2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GAINTAGTHRESHCTRL1 No description provided (GAINTAGTHRESHCTRL1) Register
 *  No description provided (GAINTAGTHRESHCTRL1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL1_t {
    union {
        struct {
             unsigned int GAINTAG_THRESH_RISE_CLK_CNT : 8; /**< No description provided */
             unsigned int GAINTAG_THRESH_FALL_CLK_CNT : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GAINTAGTHRESHCTRL2 No description provided (GAINTAGTHRESHCTRL2) Register
 *  No description provided (GAINTAGTHRESHCTRL2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL2_t {
    union {
        struct {
             unsigned int GAINTAG_THRESH_RISE_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED2        : 4; /**< Reserved */
             unsigned int GAINTAG_THRESH_FALL_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GAINTAGTHRESHCTRL2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GAINTAGTHRESHSEL No description provided (GAINTAGTHRESHSEL) Register
 *  No description provided (GAINTAGTHRESHSEL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GAINTAGTHRESHSEL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GAINTAGTHRESHSEL_t {
    union {
        struct {
             unsigned int GAINTAG_THRESH_SEL : 1; /**< No description provided */
             unsigned int GAINTAG_THRESH_STATIC : 1; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GAINTAGTHRESHSEL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GAINTAG0CLKCTRL1 No description provided (GAINTAG0CLKCTRL1) Register
 *  No description provided (GAINTAG0CLKCTRL1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GAINTAG0CLKCTRL1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GAINTAG0CLKCTRL1_t {
    union {
        struct {
             unsigned int GAINTAG0_CLK_RISE_CLK_CNT : 8; /**< No description provided */
             unsigned int GAINTAG0_CLK_FALL_CLK_CNT : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GAINTAG0CLKCTRL1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GAINTAG0CLKCTRL2 No description provided (GAINTAG0CLKCTRL2) Register
 *  No description provided (GAINTAG0CLKCTRL2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GAINTAG0CLKCTRL2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GAINTAG0CLKCTRL2_t {
    union {
        struct {
             unsigned int GAINTAG0_CLK_RISE_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED2        : 4; /**< Reserved */
             unsigned int GAINTAG0_CLK_FALL_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GAINTAG0CLKCTRL2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FORCESFCTRL1 No description provided (FORCESFCTRL1) Register
 *  No description provided (FORCESFCTRL1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_FORCESFCTRL1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_FORCESFCTRL1_t {
    union {
        struct {
             unsigned int FORCE_SF_RISE_CLK_CNT : 8; /**< No description provided */
             unsigned int FORCE_SF_FALL_CLK_CNT : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_FORCESFCTRL1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FORCESFCTRL2 No description provided (FORCESFCTRL2) Register
 *  No description provided (FORCESFCTRL2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_FORCESFCTRL2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_FORCESFCTRL2_t {
    union {
        struct {
             unsigned int FORCE_SF_RISE_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED2        : 4; /**< Reserved */
             unsigned int FORCE_SF_FALL_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_FORCESFCTRL2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FORCEIPDACTRL1 No description provided (FORCEIPDACTRL1) Register
 *  No description provided (FORCEIPDACTRL1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_FORCEIPDACTRL1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_FORCEIPDACTRL1_t {
    union {
        struct {
             unsigned int FORCE_IPDA_RISE_CLK_CNT : 8; /**< No description provided */
             unsigned int FORCE_IPDA_FALL_CLK_CNT : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_FORCEIPDACTRL1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FORCEIPDACTRL2 No description provided (FORCEIPDACTRL2) Register
 *  No description provided (FORCEIPDACTRL2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_FORCEIPDACTRL2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_FORCEIPDACTRL2_t {
    union {
        struct {
             unsigned int FORCE_IPDA_RISE_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED2        : 4; /**< Reserved */
             unsigned int FORCE_IPDA_FALL_CNVT_CNT : 2; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_FORCEIPDACTRL2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USEQRAMLOADADDR Useq Ram Load start Address Register (USEQRAMLOADADDR) Register
 *  Useq Ram Load start Address Register (USEQRAMLOADADDR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_t
 *  \brief Useq Ram Load start Address Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_t {
    union {
        struct {
             unsigned int LD_RAM_SEL       : 2; /**< No description provided */
             unsigned int LD_ADDR          : 12; /**< No description provided */
             unsigned int RESERVED14       : 2; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USEQRAMRDSTADDR Useq Ram Read start Address Register (USEQRAMRDSTADDR) Register
 *  Useq Ram Read start Address Register (USEQRAMRDSTADDR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQRAMRDSTADDR_t
 *  \brief Useq Ram Read start Address Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_USEQRAMRDSTADDR_t {
    union {
        struct {
             unsigned int RD_RAM_SEL       : 2; /**< No description provided */
             unsigned int RD_ADDR          : 12; /**< No description provided */
             unsigned int RESERVED14       : 2; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQRAMRDSTADDR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USEQRAMLOADDATA Useq Ram Load Data Register (USEQRAMLOADDATA) Register
 *  Useq Ram Load Data Register (USEQRAMLOADDATA) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQRAMLOADDATA_t
 *  \brief Useq Ram Load Data Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_USEQRAMLOADDATA_t {
    union {
        struct {
             unsigned int LD_DATA          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQRAMLOADDATA_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USEQRAMLOADDATAALIAS Useq Ram Load Data Register (USEQRAMLOADDATAALIAS) Register
 *  Useq Ram Load Data Register (USEQRAMLOADDATAALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQRAMLOADDATAALIAS_t
 *  \brief Useq Ram Load Data Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_USEQRAMLOADDATAALIAS_t {
    union {
        struct {
             unsigned int LD_DATA_ALIAS    : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQRAMLOADDATAALIAS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USEQRAMRDDATA Useq Ram Read Data Register (USEQRAMRDDATA) Register
 *  Useq Ram Read Data Register (USEQRAMRDDATA) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQRAMRDDATA_t
 *  \brief Useq Ram Read Data Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_USEQRAMRDDATA_t {
    union {
        struct {
             unsigned int USEQ_RAM_RD_DATA : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQRAMRDDATA_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USEQRAMRDDATAALIAS Useq Ram Read Data Register (USEQRAMRDDATAALIAS) Register
 *  Useq Ram Read Data Register (USEQRAMRDDATAALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQRAMRDDATAALIAS_t
 *  \brief Useq Ram Read Data Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_USEQRAMRDDATAALIAS_t {
    union {
        struct {
             unsigned int USEQ_RAM_RD_DATA_ALIAS : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQRAMRDDATAALIAS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PWM_CTRL_0 No description provided (PWM_CTRL_0) Register
 *  No description provided (PWM_CTRL_0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_PWM_CTRL_0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_PWM_CTRL_0_t {
    union {
        struct {
             unsigned int PWM_PERIOD       : 8; /**< No description provided */
             unsigned int PWM_STEP_SIZE    : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_PWM_CTRL_0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PWM_CTRL_1 No description provided (PWM_CTRL_1) Register
 *  No description provided (PWM_CTRL_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_PWM_CTRL_1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_PWM_CTRL_1_t {
    union {
        struct {
             unsigned int PWM_REPEAT_PER_STEP : 8; /**< No description provided */
             unsigned int RESERVED8        : 6; /**< Reserved */
             unsigned int CURRENT_CG       : 1; /**< No description provided */
             unsigned int PWM_BUSY         : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_PWM_CTRL_1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FSYNCCTRL No description provided (FSYNCCTRL) Register
 *  No description provided (FSYNCCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_FSYNCCTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_FSYNCCTRL_t {
    union {
        struct {
             unsigned int FSYNC_OUT_EN     : 1; /**< No description provided */
             unsigned int FSYNC_OUT_MODE   : 1; /**< No description provided */
             unsigned int EXT_FSYNC_ESH    : 1; /**< No description provided */
             unsigned int RESERVED3        : 1; /**< Reserved */
             unsigned int LSMODCTR_ESH     : 1; /**< No description provided */
             unsigned int LSMODCTR_AUTORESTART : 1; /**< No description provided */
             unsigned int LSMODCTR_FREEZE  : 1; /**< No description provided */
             unsigned int LSMODCTR_START   : 1; /**< No description provided */
             unsigned int INTCTR_ESH       : 1; /**< No description provided */
             unsigned int INTCTR_AUTORESTART : 1; /**< No description provided */
             unsigned int INTCTR_FREEZE    : 1; /**< No description provided */
             unsigned int INTCTR_START     : 1; /**< No description provided */
             unsigned int SYSCTR_ESH       : 1; /**< No description provided */
             unsigned int SYSCTR_AUTORESTART : 1; /**< No description provided */
             unsigned int SYSCTR_FREEZE    : 1; /**< No description provided */
             unsigned int SYSCTR_START     : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_FSYNCCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FSYNCSTATUS No description provided (FSYNCSTATUS) Register
 *  No description provided (FSYNCSTATUS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_FSYNCSTATUS_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_FSYNCSTATUS_t {
    union {
        struct {
             unsigned int FSYNC_FLAG       : 1; /**< No description provided */
             unsigned int LSMODCTR_FLAG    : 1; /**< No description provided */
             unsigned int INTCTR_FLAG      : 1; /**< No description provided */
             unsigned int SYSCTR_FLAG      : 1; /**< No description provided */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_FSYNCSTATUS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FSYNCLSMODCNTR_0 No description provided (FSYNCLSMODCNTR_0) Register
 *  No description provided (FSYNCLSMODCNTR_0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_FSYNCLSMODCNTR_0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_FSYNCLSMODCNTR_0_t {
    union {
        struct {
             unsigned int FSYNC_LSMOD_COUNTER_0 : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_FSYNCLSMODCNTR_0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FSYNCLSMODCNTR_1 No description provided (FSYNCLSMODCNTR_1) Register
 *  No description provided (FSYNCLSMODCNTR_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_FSYNCLSMODCNTR_1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_FSYNCLSMODCNTR_1_t {
    union {
        struct {
             unsigned int FSYNC_LSMOD_COUNTER_1 : 4; /**< No description provided */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_FSYNCLSMODCNTR_1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FSYNCINTCNTR_0 No description provided (FSYNCINTCNTR_0) Register
 *  No description provided (FSYNCINTCNTR_0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_FSYNCINTCNTR_0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_FSYNCINTCNTR_0_t {
    union {
        struct {
             unsigned int FSYNC_INT_COUNTER_0 : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_FSYNCINTCNTR_0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FSYNCINTCNTR_1 No description provided (FSYNCINTCNTR_1) Register
 *  No description provided (FSYNCINTCNTR_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_FSYNCINTCNTR_1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_FSYNCINTCNTR_1_t {
    union {
        struct {
             unsigned int FSYNC_INT_COUNTER_1 : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_FSYNCINTCNTR_1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FSYNCSYSCNTR_0 No description provided (FSYNCSYSCNTR_0) Register
 *  No description provided (FSYNCSYSCNTR_0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_FSYNCSYSCNTR_0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_FSYNCSYSCNTR_0_t {
    union {
        struct {
             unsigned int FSYNC_SYS_COUNTER_0 : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_FSYNCSYSCNTR_0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FSYNCSYSCNTR_1 No description provided (FSYNCSYSCNTR_1) Register
 *  No description provided (FSYNCSYSCNTR_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_FSYNCSYSCNTR_1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_FSYNCSYSCNTR_1_t {
    union {
        struct {
             unsigned int FSYNC_SYS_COUNTER_1 : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_FSYNCSYSCNTR_1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR16 No description provided (GPRR16) Register
 *  No description provided (GPRR16) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR16_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR16_t {
    union {
        struct {
             unsigned int GPR_R16          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR16_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR17 No description provided (GPRR17) Register
 *  No description provided (GPRR17) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR17_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR17_t {
    union {
        struct {
             unsigned int GPR_R17          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR17_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR18 No description provided (GPRR18) Register
 *  No description provided (GPRR18) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR18_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR18_t {
    union {
        struct {
             unsigned int GPR_R18          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR18_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR19 No description provided (GPRR19) Register
 *  No description provided (GPRR19) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR19_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR19_t {
    union {
        struct {
             unsigned int GPR_R19          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR19_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR20 No description provided (GPRR20) Register
 *  No description provided (GPRR20) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR20_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR20_t {
    union {
        struct {
             unsigned int GPR_R20          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR20_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR21 No description provided (GPRR21) Register
 *  No description provided (GPRR21) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR21_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR21_t {
    union {
        struct {
             unsigned int GPR_R21          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR21_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR22 No description provided (GPRR22) Register
 *  No description provided (GPRR22) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR22_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR22_t {
    union {
        struct {
             unsigned int GPR_R22          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR22_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR23 No description provided (GPRR23) Register
 *  No description provided (GPRR23) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR23_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR23_t {
    union {
        struct {
             unsigned int GPR_R23          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR23_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR24 No description provided (GPRR24) Register
 *  No description provided (GPRR24) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR24_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR24_t {
    union {
        struct {
             unsigned int GPR_R24          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR24_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR25 No description provided (GPRR25) Register
 *  No description provided (GPRR25) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR25_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR25_t {
    union {
        struct {
             unsigned int GPR_R25          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR25_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR26 No description provided (GPRR26) Register
 *  No description provided (GPRR26) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR26_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR26_t {
    union {
        struct {
             unsigned int GPR_R26          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR26_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR27 No description provided (GPRR27) Register
 *  No description provided (GPRR27) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR27_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR27_t {
    union {
        struct {
             unsigned int GPR_R27          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR27_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR28 No description provided (GPRR28) Register
 *  No description provided (GPRR28) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR28_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR28_t {
    union {
        struct {
             unsigned int GPR_R28          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR28_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR29 No description provided (GPRR29) Register
 *  No description provided (GPRR29) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR29_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR29_t {
    union {
        struct {
             unsigned int GPR_R29          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR29_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR30 No description provided (GPRR30) Register
 *  No description provided (GPRR30) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR30_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR30_t {
    union {
        struct {
             unsigned int GPR_R30          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR30_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GPRR31 No description provided (GPRR31) Register
 *  No description provided (GPRR31) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_GPRR31_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP1_GPRR31_t {
    union {
        struct {
             unsigned int GPR_R31          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP1_GPRR31_t;
#ifdef SWIG
%}
#endif

/*@}*/

#endif  /* end ifndef ADI_USEQ_REGS_MAP1_TYPEDEFS_H_ */

#ifndef ADI_AI_REGS_YODA_TYPEDEFS_H_
#define ADI_AI_REGS_YODA_TYPEDEFS_H_

/** @defgroup ADC_CTRL0_S1 No description provided (ADC_CTRL0_S1) Register
 *  No description provided (ADC_CTRL0_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_ADC_CTRL0_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_ADC_CTRL0_S1_t {
    union {
        struct {
             unsigned int CLK_DE_MODE      : 1; /**< No description provided */
             unsigned int ADC_EXT_RESET_SEL : 1; /**< No description provided */
             unsigned int ADC_EN_10B       : 1; /**< No description provided */
             unsigned int RESERVED3        : 1; /**< Reserved */
             unsigned int ADC_MUX          : 3; /**< No description provided */
             unsigned int RESERVED7        : 9; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_ADC_CTRL0_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ADC_CTRL1_S1 No description provided (ADC_CTRL1_S1) Register
 *  No description provided (ADC_CTRL1_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_ADC_CTRL1_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_ADC_CTRL1_S1_t {
    union {
        struct {
             unsigned int ADC_DN_DELAY     : 8; /**< No description provided */
             unsigned int ADC_UP_DELAY     : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_ADC_CTRL1_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ADC_CTRL2_S1 No description provided (ADC_CTRL2_S1) Register
 *  No description provided (ADC_CTRL2_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_ADC_CTRL2_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_ADC_CTRL2_S1_t {
    union {
        struct {
             unsigned int ADC_IRAMP        : 9; /**< No description provided */
             unsigned int ADC_REGRESET     : 1; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_ADC_CTRL2_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ADCPLL_CTRL0_S1 No description provided (ADCPLL_CTRL0_S1) Register
 *  No description provided (ADCPLL_CTRL0_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_ADCPLL_CTRL0_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_ADCPLL_CTRL0_S1_t {
    union {
        struct {
             unsigned int ADCPLL_PHASE_LOCK_DELAY : 8; /**< No description provided */
             unsigned int ADCPLL_QP        : 5; /**< No description provided */
             unsigned int ADCPLL_RZ        : 2; /**< No description provided */
             unsigned int ADCPLL_DLPF      : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_ADCPLL_CTRL0_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ADCPLL_CTRL1_S1 No description provided (ADCPLL_CTRL1_S1) Register
 *  No description provided (ADCPLL_CTRL1_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_ADCPLL_CTRL1_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_ADCPLL_CTRL1_S1_t {
    union {
        struct {
             unsigned int ADCPLL_LOCK_ACC  : 8; /**< No description provided */
             unsigned int ADCPLL_UNLOCK_ACC : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_ADCPLL_CTRL1_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ADCPLL_CTRL2_S1 No description provided (ADCPLL_CTRL2_S1) Register
 *  No description provided (ADCPLL_CTRL2_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_ADCPLL_CTRL2_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_ADCPLL_CTRL2_S1_t {
    union {
        struct {
             unsigned int ADCPLL_C         : 5; /**< No description provided */
             unsigned int RESERVED5        : 3; /**< Reserved */
             unsigned int ADCPLL_END5      : 4; /**< No description provided */
             unsigned int ADCPLL_TESTMUX   : 2; /**< No description provided */
             unsigned int ADCPLL_TEST_SEL  : 2; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_ADCPLL_CTRL2_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMP_CTRL0_S1 No description provided (AMP_CTRL0_S1) Register
 *  No description provided (AMP_CTRL0_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_AMP_CTRL0_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_AMP_CTRL0_S1_t {
    union {
        struct {
             unsigned int AMP_CFBA         : 4; /**< No description provided */
             unsigned int AMP_CFBB         : 4; /**< No description provided */
             unsigned int AMP_CFBC         : 4; /**< No description provided */
             unsigned int AMP_CFBD         : 4; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_AMP_CTRL0_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMP_CTRL1_S1 No description provided (AMP_CTRL1_S1) Register
 *  No description provided (AMP_CTRL1_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_AMP_CTRL1_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_AMP_CTRL1_S1_t {
    union {
        struct {
             unsigned int AMP_CINA         : 2; /**< No description provided */
             unsigned int RESERVED2        : 2; /**< Reserved */
             unsigned int AMP_CINB         : 2; /**< No description provided */
             unsigned int RESERVED6        : 2; /**< Reserved */
             unsigned int AMP_CINC         : 2; /**< No description provided */
             unsigned int RESERVED10       : 2; /**< Reserved */
             unsigned int AMP_CIND         : 2; /**< No description provided */
             unsigned int RESERVED14       : 2; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_AMP_CTRL1_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMP_CTRL2_S1 No description provided (AMP_CTRL2_S1) Register
 *  No description provided (AMP_CTRL2_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_AMP_CTRL2_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_AMP_CTRL2_S1_t {
    union {
        struct {
             unsigned int AMP_S            : 2; /**< No description provided */
             unsigned int AMP_TESTSELP     : 2; /**< No description provided */
             unsigned int AMP_TESTSELN     : 2; /**< No description provided */
             unsigned int GAINTAG_LATCH_INHIBIT : 3; /**< No description provided */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_AMP_CTRL2_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CHIP_ID No description provided (CHIP_ID) Register
 *  No description provided (CHIP_ID) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_CHIP_ID_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_CHIP_ID_t {
    union {
        struct {
             unsigned int CHIP_ID          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_CHIP_ID_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CKGEN_CTRL No description provided (CKGEN_CTRL) Register
 *  No description provided (CKGEN_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_CKGEN_CTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_CKGEN_CTRL_t {
    union {
        struct {
             unsigned int CG_STATE         : 1; /**< No description provided */
             unsigned int CG_QBUF_STATE    : 1; /**< No description provided */
             unsigned int CG_LIGHT_STATE   : 1; /**< No description provided */
             unsigned int CG_RSTN          : 1; /**< No description provided */
             unsigned int CG_XTAL_PRESCALE : 2; /**< No description provided */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_CKGEN_CTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CKGEN_S1 No description provided (CKGEN_S1) Register
 *  No description provided (CKGEN_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_CKGEN_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_CKGEN_S1_t {
    union {
        struct {
             unsigned int CG_INVERT_CLK    : 4; /**< No description provided */
             unsigned int RESERVED4        : 4; /**< Reserved */
             unsigned int CLK_TEST_SEL     : 6; /**< No description provided */
             unsigned int RESERVED14       : 2; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_CKGEN_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLK_CTRL No description provided (CLK_CTRL) Register
 *  No description provided (CLK_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_CLK_CTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_CLK_CTRL_t {
    union {
        struct {
             unsigned int CLKDE_RESET      : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_CLK_CTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLK_DE_CTRL_S1 No description provided (CLK_DE_CTRL_S1) Register
 *  No description provided (CLK_DE_CTRL_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_CLK_DE_CTRL_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_CLK_DE_CTRL_S1_t {
    union {
        struct {
             unsigned int CLK_DE_C         : 5; /**< No description provided */
             unsigned int RESERVED5        : 3; /**< Reserved */
             unsigned int CLK_DE_END5      : 4; /**< No description provided */
             unsigned int RESERVED12       : 4; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_CLK_DE_CTRL_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLK_LVDSTX_S1 No description provided (CLK_LVDSTX_S1) Register
 *  No description provided (CLK_LVDSTX_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_CLK_LVDSTX_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_CLK_LVDSTX_S1_t {
    union {
        struct {
             unsigned int RESERVED0        : 1; /**< Reserved */
             unsigned int CLK_LVDSTX_I2X   : 1; /**< No description provided */
             unsigned int CLK_LVDSTX_TE    : 1; /**< No description provided */
             unsigned int CLK_LVDSTX_TEST  : 1; /**< No description provided */
             unsigned int CLK_LVDSTX_TESTD : 1; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_CLK_LVDSTX_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLKTREE0 No description provided (CLKTREE0) Register
 *  No description provided (CLKTREE0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_CLKTREE0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_CLKTREE0_t {
    union {
        struct {
             unsigned int DLL_CLKA_PARK    : 1; /**< No description provided */
             unsigned int DLL_CLKB_PARK    : 1; /**< No description provided */
             unsigned int DLL_CLKIN_EN     : 1; /**< No description provided */
             unsigned int DLL_LPF_EN       : 1; /**< No description provided */
             unsigned int DLL_REST_EN      : 1; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_CLKTREE0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLKTREE_S1 No description provided (CLKTREE_S1) Register
 *  No description provided (CLKTREE_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_CLKTREE_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_CLKTREE_S1_t {
    union {
        struct {
             unsigned int DLL_LEAK         : 5; /**< No description provided */
             unsigned int DLL_CPLF         : 7; /**< No description provided */
             unsigned int RESERVED12       : 4; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_CLKTREE_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DAC_CTRL1 No description provided (DAC_CTRL1) Register
 *  No description provided (DAC_CTRL1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_DAC_CTRL1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_DAC_CTRL1_t {
    union {
        struct {
             unsigned int DAC_LATCH        : 9; /**< No description provided */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_DAC_CTRL1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DAC_CTRL2 No description provided (DAC_CTRL2) Register
 *  No description provided (DAC_CTRL2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_DAC_CTRL2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_DAC_CTRL2_t {
    union {
        struct {
             unsigned int DAC_PD           : 10; /**< No description provided */
             unsigned int DAC_LOWLOAD      : 4; /**< No description provided */
             unsigned int RESERVED14       : 1; /**< Reserved */
             unsigned int DAC_GLOBAL_PD    : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_DAC_CTRL2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DAC_CTRL0_S1 No description provided (DAC_CTRL0_S1) Register
 *  No description provided (DAC_CTRL0_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_DAC_CTRL0_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_DAC_CTRL0_S1_t {
    union {
        struct {
             unsigned int DAC_SPARE1       : 8; /**< No description provided */
             unsigned int DAC_SHARE        : 1; /**< No description provided */
             unsigned int COMP_REF_DISABLE : 6; /**< No description provided */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_DAC_CTRL0_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DAC_CTRL1_S1 No description provided (DAC_CTRL1_S1) Register
 *  No description provided (DAC_CTRL1_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_DAC_CTRL1_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_DAC_CTRL1_S1_t {
    union {
        struct {
             unsigned int DAC_SPARE2       : 1; /**< No description provided */
             unsigned int DAC_SPARE0       : 2; /**< No description provided */
             unsigned int RESERVED3        : 13; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_DAC_CTRL1_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DAC_CTRL2_S1 No description provided (DAC_CTRL2_S1) Register
 *  No description provided (DAC_CTRL2_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_DAC_CTRL2_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_DAC_CTRL2_S1_t {
    union {
        struct {
             unsigned int COMP_REF_DAC1    : 6; /**< No description provided */
             unsigned int RESERVED6        : 2; /**< Reserved */
             unsigned int COMP_REF_DAC2    : 6; /**< No description provided */
             unsigned int RESERVED14       : 2; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_DAC_CTRL2_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DAC_CTRL3_S1 No description provided (DAC_CTRL3_S1) Register
 *  No description provided (DAC_CTRL3_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_DAC_CTRL3_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_DAC_CTRL3_S1_t {
    union {
        struct {
             unsigned int COMP_REF_DAC3    : 6; /**< No description provided */
             unsigned int RESERVED6        : 2; /**< Reserved */
             unsigned int COMP_REF_DAC4    : 6; /**< No description provided */
             unsigned int RESERVED14       : 2; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_DAC_CTRL3_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DAC_DATA No description provided (DAC_DATA) Register
 *  No description provided (DAC_DATA) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_DAC_DATA_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_DAC_DATA_t {
    union {
        struct {
             unsigned int DAC_IN           : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_DAC_DATA_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup IPDA_CTRL_S1 No description provided (IPDA_CTRL_S1) Register
 *  No description provided (IPDA_CTRL_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_IPDA_CTRL_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_IPDA_CTRL_S1_t {
    union {
        struct {
             unsigned int IBOUT_ADJ        : 4; /**< No description provided */
             unsigned int IPDA_GAIN        : 1; /**< No description provided */
             unsigned int IPDA_SPARE0      : 1; /**< No description provided */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_IPDA_CTRL_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup LS_LVDSTX_S1 No description provided (LS_LVDSTX_S1) Register
 *  No description provided (LS_LVDSTX_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_LS_LVDSTX_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_LS_LVDSTX_S1_t {
    union {
        struct {
             unsigned int RESERVED0        : 1; /**< Reserved */
             unsigned int LS_LVDSTX_I2X    : 1; /**< No description provided */
             unsigned int LS_LVDSTX_TE     : 1; /**< No description provided */
             unsigned int LS_LVDSTX_TEST   : 1; /**< No description provided */
             unsigned int LS_LVDSTX_TESTD  : 1; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_LS_LVDSTX_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup LSCTRL0_S1 No description provided (LSCTRL0_S1) Register
 *  No description provided (LSCTRL0_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_LSCTRL0_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_LSCTRL0_S1_t {
    union {
        struct {
             unsigned int LIGHTMUX_SEL     : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_LSCTRL0_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup LSMOD_EN No description provided (LSMOD_EN) Register
 *  No description provided (LSMOD_EN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_LSMOD_EN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_LSMOD_EN_t {
    union {
        struct {
             unsigned int CLK_LVDS_OE      : 1; /**< No description provided */
             unsigned int CLK_LVDSTX_PD    : 1; /**< No description provided */
             unsigned int LS_LVDS_OE       : 1; /**< No description provided */
             unsigned int LS_LVDSTX_PD     : 1; /**< No description provided */
             unsigned int CLK_LVDSTX_HZ    : 1; /**< No description provided */
             unsigned int LS_LVDSTX_HZ     : 1; /**< No description provided */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_LSMOD_EN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ROW_CTRL No description provided (ROW_CTRL) Register
 *  No description provided (ROW_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_ROW_CTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_ROW_CTRL_t {
    union {
        struct {
             unsigned int GLOBAL_RESET     : 1; /**< No description provided */
             unsigned int ROW_WRAPEN       : 1; /**< No description provided */
             unsigned int RESERVED2        : 6; /**< Reserved */
             unsigned int ROW_READ_DEF     : 1; /**< No description provided */
             unsigned int ROW_RESET_DEF    : 1; /**< No description provided */
             unsigned int ROW_CAPODD_DEF   : 1; /**< No description provided */
             unsigned int ROW_CAPEVEN_DEF  : 1; /**< No description provided */
             unsigned int ROW_TX_PD_DEF    : 1; /**< No description provided */
             unsigned int ROW_TX_DEF       : 1; /**< No description provided */
             unsigned int RESERVED14       : 2; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_ROW_CTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PLL_CTRL No description provided (PLL_CTRL) Register
 *  No description provided (PLL_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_PLL_CTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_PLL_CTRL_t {
    union {
        struct {
             unsigned int SSPLL_PD         : 1; /**< No description provided */
             unsigned int SSPLL_RST        : 1; /**< No description provided */
             unsigned int RESERVED2        : 1; /**< Reserved */
             unsigned int SSPLL_RESET_LOCK_LOST : 1; /**< No description provided */
             unsigned int ADCPLL_PD        : 1; /**< No description provided */
             unsigned int ADCPLL_RST       : 1; /**< No description provided */
             unsigned int RESERVED6        : 1; /**< Reserved */
             unsigned int ADCPLL_RESET_LOCK_LOST : 1; /**< No description provided */
             unsigned int SYSPLL_PD        : 1; /**< No description provided */
             unsigned int RESERVED9        : 2; /**< Reserved */
             unsigned int SYSPLL_RESET_LOCK_LOST : 1; /**< No description provided */
             unsigned int RESERVED12       : 4; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_PLL_CTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PLL_STATUS No description provided (PLL_STATUS) Register
 *  No description provided (PLL_STATUS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_PLL_STATUS_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_PLL_STATUS_t {
    union {
        struct {
             unsigned int SSPLL_PHASE_LOCK_REG : 1; /**< No description provided */
             unsigned int SSPLL_LOCK_LOST  : 1; /**< No description provided */
             unsigned int RESERVED2        : 2; /**< Reserved */
             unsigned int ADCPLL_PHASE_LOCK_REG : 1; /**< No description provided */
             unsigned int ADCPLL_LOCK_LOST : 1; /**< No description provided */
             unsigned int RESERVED6        : 2; /**< Reserved */
             unsigned int SYSPLL_PHASE_LOCK_REG : 1; /**< No description provided */
             unsigned int SYSPLL_LOCK_LOST : 1; /**< No description provided */
             unsigned int SYSPLL_ACTIVE    : 1; /**< No description provided */
             unsigned int RESERVED11       : 5; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_PLL_STATUS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup POWER_DOWN_0 No description provided (POWER_DOWN_0) Register
 *  No description provided (POWER_DOWN_0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_POWER_DOWN_0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_POWER_DOWN_0_t {
    union {
        struct {
             unsigned int ROW_VECTOR_LD    : 1; /**< No description provided */
             unsigned int PUMP_BYPASS      : 1; /**< No description provided */
             unsigned int DLL_PD           : 1; /**< No description provided */
             unsigned int RESERVED3        : 13; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_POWER_DOWN_0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup POWER_DOWN_ADC_OTHERS No description provided (POWER_DOWN_ADC_OTHERS) Register
 *  No description provided (POWER_DOWN_ADC_OTHERS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_POWER_DOWN_ADC_OTHERS_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_POWER_DOWN_ADC_OTHERS_t {
    union {
        struct {
             unsigned int ADC_RAMP_PD      : 1; /**< No description provided */
             unsigned int COL_BIAS_PD      : 1; /**< No description provided */
             unsigned int ROW_UP_DNB       : 1; /**< No description provided */
             unsigned int SAT_DETECT_PD    : 1; /**< No description provided */
             unsigned int COL_COMPARATOR_PD : 1; /**< No description provided */
             unsigned int AMP_PD           : 1; /**< No description provided */
             unsigned int ADC_PD           : 1; /**< No description provided */
             unsigned int REFGEN_BGR_PD    : 1; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_POWER_DOWN_ADC_OTHERS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup POWER_DOWN_READOUT No description provided (POWER_DOWN_READOUT) Register
 *  No description provided (POWER_DOWN_READOUT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_POWER_DOWN_READOUT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_POWER_DOWN_READOUT_t {
    union {
        struct {
             unsigned int READOUT_PD       : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_POWER_DOWN_READOUT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PUMP_S1 No description provided (PUMP_S1) Register
 *  No description provided (PUMP_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_PUMP_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_PUMP_S1_t {
    union {
        struct {
             unsigned int PUMP_ADJ         : 2; /**< No description provided */
             unsigned int RESERVED2        : 6; /**< Reserved */
             unsigned int QPD              : 6; /**< No description provided */
             unsigned int RESERVED14       : 2; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_PUMP_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup READOUT_S1 No description provided (READOUT_S1) Register
 *  No description provided (READOUT_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_READOUT_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_READOUT_S1_t {
    union {
        struct {
             unsigned int BITLINE_TURBO    : 2; /**< No description provided */
             unsigned int RESERVED2        : 1; /**< Reserved */
             unsigned int NCDS_MODE        : 1; /**< No description provided */
             unsigned int SATDETECT_S      : 1; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_READOUT_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup REGIF_CTRL No description provided (REGIF_CTRL) Register
 *  No description provided (REGIF_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_REGIF_CTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_REGIF_CTRL_t {
    union {
        struct {
             unsigned int REGIF_RESETB1    : 1; /**< No description provided */
             unsigned int REGIF_RD_WRB     : 1; /**< No description provided */
             unsigned int REGIF_START1     : 1; /**< No description provided */
             unsigned int REGIF_BUSY1      : 1; /**< No description provided */
             unsigned int RESERVED4        : 4; /**< Reserved */
             unsigned int REGIF_RESETB2    : 1; /**< No description provided */
             unsigned int RESERVED9        : 1; /**< Reserved */
             unsigned int REGIF_START2     : 1; /**< No description provided */
             unsigned int REGIF_BUSY2      : 1; /**< No description provided */
             unsigned int RESERVED12       : 3; /**< Reserved */
             unsigned int REGIF_READ_SHIFT : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_REGIF_CTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup REGIF_RDATA No description provided (REGIF_RDATA) Register
 *  No description provided (REGIF_RDATA) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_REGIF_RDATA_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_REGIF_RDATA_t {
    union {
        struct {
             unsigned int REGIF_READ_DATA  : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_REGIF_RDATA_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSPLL_CTRL0_S1 No description provided (SSPLL_CTRL0_S1) Register
 *  No description provided (SSPLL_CTRL0_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_SSPLL_CTRL0_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_SSPLL_CTRL0_S1_t {
    union {
        struct {
             unsigned int SSPLL_PHASE_LOCK_DELAY : 8; /**< No description provided */
             unsigned int SSPLL_QP         : 5; /**< No description provided */
             unsigned int SSPLL_RZ         : 2; /**< No description provided */
             unsigned int SSPLL_DLPF       : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_SSPLL_CTRL0_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSPLL_CTRL1_S1 No description provided (SSPLL_CTRL1_S1) Register
 *  No description provided (SSPLL_CTRL1_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_SSPLL_CTRL1_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_SSPLL_CTRL1_S1_t {
    union {
        struct {
             unsigned int SSPLL_LOCK_ACC   : 8; /**< No description provided */
             unsigned int SSPLL_UNLOCK_ACC : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_SSPLL_CTRL1_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSPLL_CTRL2_S1 No description provided (SSPLL_CTRL2_S1) Register
 *  No description provided (SSPLL_CTRL2_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_SSPLL_CTRL2_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_SSPLL_CTRL2_S1_t {
    union {
        struct {
             unsigned int SSPLL_C          : 5; /**< No description provided */
             unsigned int RESERVED5        : 3; /**< Reserved */
             unsigned int SSPLL_END5       : 4; /**< No description provided */
             unsigned int SSPLL_TESTMUX    : 2; /**< No description provided */
             unsigned int SSPLL_TEST_SEL   : 2; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_SSPLL_CTRL2_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SYSPLL_CTRL0_S1 No description provided (SYSPLL_CTRL0_S1) Register
 *  No description provided (SYSPLL_CTRL0_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_SYSPLL_CTRL0_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_SYSPLL_CTRL0_S1_t {
    union {
        struct {
             unsigned int SYSPLL_PHASE_LOCK_DELAY : 8; /**< No description provided */
             unsigned int SYSPLL_QP        : 5; /**< No description provided */
             unsigned int SYSPLL_RZ        : 2; /**< No description provided */
             unsigned int SYSPLL_DLPF      : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_SYSPLL_CTRL0_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SYSPLL_CTRL1_S1 No description provided (SYSPLL_CTRL1_S1) Register
 *  No description provided (SYSPLL_CTRL1_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_SYSPLL_CTRL1_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_SYSPLL_CTRL1_S1_t {
    union {
        struct {
             unsigned int SYSPLL_LOCK_ACC  : 8; /**< No description provided */
             unsigned int SYSPLL_UNLOCK_ACC : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_SYSPLL_CTRL1_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SYSPLL_CTRL2_S1 No description provided (SYSPLL_CTRL2_S1) Register
 *  No description provided (SYSPLL_CTRL2_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_SYSPLL_CTRL2_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_SYSPLL_CTRL2_S1_t {
    union {
        struct {
             unsigned int SYSPLL_M         : 6; /**< No description provided */
             unsigned int RESERVED6        : 1; /**< Reserved */
             unsigned int SYSPLL_N         : 1; /**< No description provided */
             unsigned int SYSPLL_P         : 6; /**< No description provided */
             unsigned int RESERVED14       : 1; /**< Reserved */
             unsigned int SYSPLL_LX        : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_SYSPLL_CTRL2_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ANA_TEST_MUX_S1 No description provided (ANA_TEST_MUX_S1) Register
 *  No description provided (ANA_TEST_MUX_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_ANA_TEST_MUX_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_ANA_TEST_MUX_S1_t {
    union {
        struct {
             unsigned int ANA_TEST_MUX     : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_ANA_TEST_MUX_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup TS_CTRL_S1 No description provided (TS_CTRL_S1) Register
 *  No description provided (TS_CTRL_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_TS_CTRL_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_TS_CTRL_S1_t {
    union {
        struct {
             unsigned int TS_CALIB_ENABLE  : 1; /**< No description provided */
             unsigned int TS_CALIB_INPUTS  : 1; /**< No description provided */
             unsigned int RESERVED2        : 1; /**< Reserved */
             unsigned int TS_FREQ_SELECT   : 1; /**< No description provided */
             unsigned int RESERVED4        : 4; /**< Reserved */
             unsigned int TS_FSADJ         : 7; /**< No description provided */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_TS_CTRL_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup TS_CTRL No description provided (TS_CTRL) Register
 *  No description provided (TS_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_TS_CTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_TS_CTRL_t {
    union {
        struct {
             unsigned int TS_INIT_LOGIC    : 1; /**< No description provided */
             unsigned int TS_RST           : 1; /**< No description provided */
             unsigned int TS_PD            : 1; /**< No description provided */
             unsigned int RESERVED3        : 12; /**< Reserved */
             unsigned int TS_EOC_REG       : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_TS_CTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup TS_DATA No description provided (TS_DATA) Register
 *  No description provided (TS_DATA) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_TS_DATA_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_TS_DATA_t {
    union {
        struct {
             unsigned int TS_Q_REG         : 12; /**< No description provided */
             unsigned int TS_OVRFL_REG     : 1; /**< No description provided */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_TS_DATA_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup VLOWENABLE No description provided (VLOWENABLE) Register
 *  No description provided (VLOWENABLE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_VLOWENABLE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_VLOWENABLE_t {
    union {
        struct {
             unsigned int VLOWENABLECTRL   : 3; /**< No description provided */
             unsigned int RESERVED3        : 5; /**< Reserved */
             unsigned int VLOW_LIN_EN      : 1; /**< No description provided */
             unsigned int RESERVED9        : 6; /**< Reserved */
             unsigned int VLOW_LIN_PD      : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_VLOWENABLE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup VLOWREGCTRL0_S2 No description provided (VLOWREGCTRL0_S2) Register
 *  No description provided (VLOWREGCTRL0_S2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_VLOWREGCTRL0_S2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_VLOWREGCTRL0_S2_t {
    union {
        struct {
             unsigned int VLOWSETPOINTCTRL : 7; /**< No description provided */
             unsigned int VLOW_GMPS2X      : 1; /**< No description provided */
             unsigned int VLOW_SPARE       : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_VLOWREGCTRL0_S2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup VLOWREGCTRL1_S2 No description provided (VLOWREGCTRL1_S2) Register
 *  No description provided (VLOWREGCTRL1_S2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_VLOWREGCTRL1_S2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_VLOWREGCTRL1_S2_t {
    union {
        struct {
             unsigned int VLOWMISCCTRL     : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_VLOWREGCTRL1_S2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup VLOWREGCTRL2_S2 No description provided (VLOWREGCTRL2_S2) Register
 *  No description provided (VLOWREGCTRL2_S2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_VLOWREGCTRL2_S2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_VLOWREGCTRL2_S2_t {
    union {
        struct {
             unsigned int VLOWCLKLOCNT     : 8; /**< No description provided */
             unsigned int VLOWCLKHICNT     : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_VLOWREGCTRL2_S2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup VLOWREGCTRL3_S2 No description provided (VLOWREGCTRL3_S2) Register
 *  No description provided (VLOWREGCTRL3_S2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_VLOWREGCTRL3_S2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_VLOWREGCTRL3_S2_t {
    union {
        struct {
             unsigned int VREGMISCCTRL     : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_VLOWREGCTRL3_S2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup VLOWREGCTRL4_S2 No description provided (VLOWREGCTRL4_S2) Register
 *  No description provided (VLOWREGCTRL4_S2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_VLOWREGCTRL4_S2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_VLOWREGCTRL4_S2_t {
    union {
        struct {
             unsigned int VLOW_SW_VADJ     : 6; /**< No description provided */
             unsigned int RESERVED6        : 2; /**< Reserved */
             unsigned int VLOW_LIN_VADJ    : 6; /**< No description provided */
             unsigned int RESERVED14       : 2; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_VLOWREGCTRL4_S2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup VLOWSHOCTRL1 No description provided (VLOWSHOCTRL1) Register
 *  No description provided (VLOWSHOCTRL1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_VLOWSHOCTRL1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_VLOWSHOCTRL1_t {
    union {
        struct {
             unsigned int VLOW_IC_START    : 1; /**< No description provided */
             unsigned int VLOW_IGNORE_CNT  : 15; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_VLOWSHOCTRL1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup VLOWSHOCTRL2 No description provided (VLOWSHOCTRL2) Register
 *  No description provided (VLOWSHOCTRL2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_VLOWSHOCTRL2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_VLOWSHOCTRL2_t {
    union {
        struct {
             unsigned int VLOW_HITIME_CTR  : 8; /**< No description provided */
             unsigned int VLOW_EVENT_CTR   : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_VLOWSHOCTRL2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup VLOWSHOCTRL3 No description provided (VLOWSHOCTRL3) Register
 *  No description provided (VLOWSHOCTRL3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_VLOWSHOCTRL3_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_VLOWSHOCTRL3_t {
    union {
        struct {
             unsigned int VLOW_CTR_RD      : 1; /**< No description provided */
             unsigned int VLOW_CTR_RD_RDY  : 1; /**< No description provided */
             unsigned int VLOW_CTR_RESET   : 1; /**< No description provided */
             unsigned int RESERVED3        : 13; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_VLOWSHOCTRL3_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup VLOWSHODETECT No description provided (VLOWSHODETECT) Register
 *  No description provided (VLOWSHODETECT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_VLOWSHODETECT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_VLOWSHODETECT_t {
    union {
        struct {
             unsigned int SHO_VLOW         : 1; /**< No description provided */
             unsigned int VLOW_SHO_DETECT  : 1; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_VLOWSHODETECT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup XOSC_CTRL No description provided (XOSC_CTRL) Register
 *  No description provided (XOSC_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_XOSC_CTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_XOSC_CTRL_t {
    union {
        struct {
             unsigned int RESERVED0        : 4; /**< Reserved */
             unsigned int XIDLE_HOST_SET   : 1; /**< No description provided */
             unsigned int XIDLE_HOST_CLEAR : 1; /**< No description provided */
             unsigned int XIDLE_HOST       : 1; /**< No description provided */
             unsigned int RESERVED7        : 1; /**< Reserved */
             unsigned int XIDLE_YEATS_SET  : 1; /**< No description provided */
             unsigned int XIDLE_YEATS_CLEAR : 1; /**< No description provided */
             unsigned int XIDLE_YEATS      : 1; /**< No description provided */
             unsigned int RESERVED11       : 5; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_XOSC_CTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CHAIN1_LEN No description provided (CHAIN1_LEN) Register
 *  No description provided (CHAIN1_LEN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_CHAIN1_LEN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_CHAIN1_LEN_t {
    union {
        struct {
             unsigned int CHAIN1_LEN       : 9; /**< No description provided */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_CHAIN1_LEN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CHAIN2_LEN No description provided (CHAIN2_LEN) Register
 *  No description provided (CHAIN2_LEN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_CHAIN2_LEN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_CHAIN2_LEN_t {
    union {
        struct {
             unsigned int CHAIN2_LEN       : 9; /**< No description provided */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_CHAIN2_LEN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSPLL_CTRL3_S1 No description provided (SSPLL_CTRL3_S1) Register
 *  No description provided (SSPLL_CTRL3_S1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_SSPLL_CTRL3_S1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_SSPLL_CTRL3_S1_t {
    union {
        struct {
             unsigned int SSPLL_ACOFF      : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_SSPLL_CTRL3_S1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PIXEL_BIAS No description provided (PIXEL_BIAS) Register
 *  No description provided (PIXEL_BIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_PIXEL_BIAS_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_PIXEL_BIAS_t {
    union {
        struct {
             unsigned int PIXEL_BIAS_CDN   : 1; /**< No description provided */
             unsigned int PIXEL_BIAS_SDN   : 1; /**< No description provided */
             unsigned int PIXEL_BIAS_DIR   : 1; /**< No description provided */
             unsigned int PIXEL_BIAS_PD_DEF : 1; /**< No description provided */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_PIXEL_BIAS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DLL_CONTROL No description provided (DLL_CONTROL) Register
 *  No description provided (DLL_CONTROL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_DLL_CONTROL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_DLL_CONTROL_t {
    union {
        struct {
             unsigned int RESERVED0        : 2; /**< Reserved */
             unsigned int DLL_COL_SREG_DIR : 1; /**< No description provided */
             unsigned int DLL_COL_EN_SDN   : 1; /**< No description provided */
             unsigned int DLL_COL_EN_CDN   : 1; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_DLL_CONTROL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ANA_SPARE_0 No description provided (ANA_SPARE_0) Register
 *  No description provided (ANA_SPARE_0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_ANA_SPARE_0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_ANA_SPARE_0_t {
    union {
        struct {
             unsigned int AMP_CLK2_DUMMY   : 1; /**< No description provided */
             unsigned int SEL_D2ORD3N_LOAD1 : 1; /**< No description provided */
             unsigned int XA               : 2; /**< No description provided */
             unsigned int XAGCOFF          : 1; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_ANA_SPARE_0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ANA_SPARE_1 No description provided (ANA_SPARE_1) Register
 *  No description provided (ANA_SPARE_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_ANA_SPARE_1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_ANA_SPARE_1_t {
    union {
        struct {
             unsigned int TS_SOC_CAL       : 1; /**< No description provided */
             unsigned int DLL_OPTION_1     : 1; /**< No description provided */
             unsigned int SEL_D2ORD3N_LOAD2 : 1; /**< No description provided */
             unsigned int DLL_OPTION_2     : 1; /**< No description provided */
             unsigned int ADC_RAMP_CM_SEL  : 2; /**< No description provided */
             unsigned int ANA_SPARE_I1     : 9; /**< No description provided */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_ANA_SPARE_1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ANA_SERIAL_SPARE_0 No description provided (ANA_SERIAL_SPARE_0) Register
 *  No description provided (ANA_SERIAL_SPARE_0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_ANA_SERIAL_SPARE_0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_ANA_SERIAL_SPARE_0_t {
    union {
        struct {
             unsigned int ROW_SPARE        : 2; /**< No description provided */
             unsigned int PUMP_SPARE       : 2; /**< No description provided */
             unsigned int TS_SPARE         : 2; /**< No description provided */
             unsigned int SYSPLL_SPARE     : 2; /**< No description provided */
             unsigned int ADCPLL_SPARE     : 2; /**< No description provided */
             unsigned int SSPLL_SPARE      : 2; /**< No description provided */
             unsigned int DAC_SPARE        : 2; /**< No description provided */
             unsigned int DLL_SPARE        : 2; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_ANA_SERIAL_SPARE_0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ANA_SERIAL_SPARE_1 No description provided (ANA_SERIAL_SPARE_1) Register
 *  No description provided (ANA_SERIAL_SPARE_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_ANA_SERIAL_SPARE_1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_ANA_SERIAL_SPARE_1_t {
    union {
        struct {
             unsigned int LSMOD_SPARE      : 2; /**< No description provided */
             unsigned int CG_SPARE         : 2; /**< No description provided */
             unsigned int LS_SPARE         : 2; /**< No description provided */
             unsigned int LS_SPARE2        : 1; /**< No description provided */
             unsigned int LS_SPARE3        : 1; /**< No description provided */
             unsigned int IPDA_SPARE1      : 1; /**< No description provided */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_ANA_SERIAL_SPARE_1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ANA_SERIAL_SPARE_2 No description provided (ANA_SERIAL_SPARE_2) Register
 *  No description provided (ANA_SERIAL_SPARE_2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_ANA_SERIAL_SPARE_2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_ANA_SERIAL_SPARE_2_t {
    union {
        struct {
             unsigned int ARRAY_SPARE      : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_ANA_SERIAL_SPARE_2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DEBUG_MUX_CONTROL_REG No description provided (DEBUG_MUX_CONTROL_REG) Register
 *  No description provided (DEBUG_MUX_CONTROL_REG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_AI_REGS_YODA_DEBUG_MUX_CONTROL_REG_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_AI_REGS_YODA_DEBUG_MUX_CONTROL_REG_t {
    union {
        struct {
             unsigned int DEBUG_MUX_CONTROL : 6; /**< No description provided */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_AI_REGS_YODA_DEBUG_MUX_CONTROL_REG_t;
#ifdef SWIG
%}
#endif

/*@}*/

#endif  /* end ifndef ADI_AI_REGS_YODA_TYPEDEFS_H_ */

#ifndef ADI_USEQ_REGS_MAP2_TYPEDEFS_H_
#define ADI_USEQ_REGS_MAP2_TYPEDEFS_H_

/** @defgroup SCRATCHPAD_0_ No description provided (SCRATCHPAD_0_) Register
 *  No description provided (SCRATCHPAD_0_) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP2_SCRATCHPAD_0__t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP2_SCRATCHPAD_0__t {
    union {
        struct {
             unsigned int SCRATCHPAD       : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP2_SCRATCHPAD_0__t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLKGEN_CK1 No description provided (CLKGEN_CK1) Register
 *  No description provided (CLKGEN_CK1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP2_CLKGEN_CK1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP2_CLKGEN_CK1_t {
    union {
        struct {
             unsigned int CK1RISE          : 7; /**< No description provided */
             unsigned int RESERVED7        : 1; /**< Reserved */
             unsigned int CK1FALL          : 7; /**< No description provided */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP2_CLKGEN_CK1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLKGEN_CK2 No description provided (CLKGEN_CK2) Register
 *  No description provided (CLKGEN_CK2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP2_CLKGEN_CK2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP2_CLKGEN_CK2_t {
    union {
        struct {
             unsigned int CK2RISE          : 7; /**< No description provided */
             unsigned int RESERVED7        : 1; /**< Reserved */
             unsigned int CK2FALL          : 7; /**< No description provided */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP2_CLKGEN_CK2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLKGEN_CK1REF No description provided (CLKGEN_CK1REF) Register
 *  No description provided (CLKGEN_CK1REF) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP2_CLKGEN_CK1REF_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP2_CLKGEN_CK1REF_t {
    union {
        struct {
             unsigned int CK1REFRISE       : 7; /**< No description provided */
             unsigned int RESERVED7        : 1; /**< Reserved */
             unsigned int CK1REFFALL       : 7; /**< No description provided */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP2_CLKGEN_CK1REF_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLKGEN_CK2REF No description provided (CLKGEN_CK2REF) Register
 *  No description provided (CLKGEN_CK2REF) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP2_CLKGEN_CK2REF_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP2_CLKGEN_CK2REF_t {
    union {
        struct {
             unsigned int CK2REFRISE       : 7; /**< No description provided */
             unsigned int RESERVED7        : 1; /**< Reserved */
             unsigned int CK2REFFALL       : 7; /**< No description provided */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP2_CLKGEN_CK2REF_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLKGEN_L1 No description provided (CLKGEN_L1) Register
 *  No description provided (CLKGEN_L1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP2_CLKGEN_L1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP2_CLKGEN_L1_t {
    union {
        struct {
             unsigned int L1RISE           : 7; /**< No description provided */
             unsigned int RESERVED7        : 1; /**< Reserved */
             unsigned int L1FALL           : 7; /**< No description provided */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP2_CLKGEN_L1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLKGEN_L2 No description provided (CLKGEN_L2) Register
 *  No description provided (CLKGEN_L2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP2_CLKGEN_L2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP2_CLKGEN_L2_t {
    union {
        struct {
             unsigned int L2RISE           : 7; /**< No description provided */
             unsigned int RESERVED7        : 1; /**< Reserved */
             unsigned int L2FALL           : 7; /**< No description provided */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP2_CLKGEN_L2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLKGEN_CKX No description provided (CLKGEN_CKX) Register
 *  No description provided (CLKGEN_CKX) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP2_CLKGEN_CKX_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP2_CLKGEN_CKX_t {
    union {
        struct {
             unsigned int CK1RISE_MSB      : 2; /**< No description provided */
             unsigned int CK1FALL_MSB      : 2; /**< No description provided */
             unsigned int CK2RISE_MSB      : 2; /**< No description provided */
             unsigned int CK2FALL_MSB      : 2; /**< No description provided */
             unsigned int CK1REFRISE_MSB   : 2; /**< No description provided */
             unsigned int CK1REFFALL_MSB   : 2; /**< No description provided */
             unsigned int CK2REFRISE_MSB   : 2; /**< No description provided */
             unsigned int CK2REFFALL_MSB   : 2; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP2_CLKGEN_CKX_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLKGEN_CYCLE No description provided (CLKGEN_CYCLE) Register
 *  No description provided (CLKGEN_CYCLE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP2_CLKGEN_CYCLE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP2_CLKGEN_CYCLE_t {
    union {
        struct {
             unsigned int CYCLE_MST        : 7; /**< No description provided */
             unsigned int CGBM             : 1; /**< No description provided */
             unsigned int L1RISE_MSB       : 2; /**< No description provided */
             unsigned int L1FALL_MSB       : 2; /**< No description provided */
             unsigned int L2RISE_MSB       : 2; /**< No description provided */
             unsigned int L2FALL_MSB       : 2; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP2_CLKGEN_CYCLE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLKGEN_OFFSET No description provided (CLKGEN_OFFSET) Register
 *  No description provided (CLKGEN_OFFSET) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP2_CLKGEN_OFFSET_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP2_CLKGEN_OFFSET_t {
    union {
        struct {
             unsigned int CLKOFFSET        : 8; /**< No description provided */
             unsigned int REFCLKOFFSET     : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP2_CLKGEN_OFFSET_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLKGEN_LTOFFSET No description provided (CLKGEN_LTOFFSET) Register
 *  No description provided (CLKGEN_LTOFFSET) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP2_CLKGEN_LTOFFSET_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP2_CLKGEN_LTOFFSET_t {
    union {
        struct {
             unsigned int LTOFFSET         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP2_CLKGEN_LTOFFSET_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CLKGEN_BURST_PERIOD No description provided (CLKGEN_BURST_PERIOD) Register
 *  No description provided (CLKGEN_BURST_PERIOD) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP2_CLKGEN_BURST_PERIOD_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_USEQ_REGS_MAP2_CLKGEN_BURST_PERIOD_t {
    union {
        struct {
             unsigned int BURSTPERIOD      : 11; /**< No description provided */
             unsigned int RESERVED11       : 5; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_USEQ_REGS_MAP2_CLKGEN_BURST_PERIOD_t;
#ifdef SWIG
%}
#endif

/*@}*/

#endif  /* end ifndef ADI_USEQ_REGS_MAP2_TYPEDEFS_H_ */

#ifndef ADI_SPIM_REGS_TYPEDEFS_H_
#define ADI_SPIM_REGS_TYPEDEFS_H_

/** @defgroup CTRLR0 Control Register 0 (CTRLR0) Register
 *  Control Register 0 (CTRLR0) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_CTRLR0_DFS
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_CTRLR0_DFS_FRAME_04BITS = 3,    /**< 04-bit serial data transfer */
     SPIM_REGS_CTRLR0_DFS_FRAME_05BITS = 4,    /**< 05-bit serial data transfer */
     SPIM_REGS_CTRLR0_DFS_FRAME_06BITS = 5,    /**< 06-bit serial data transfer */
     SPIM_REGS_CTRLR0_DFS_FRAME_07BITS = 6,    /**< 07-bit serial data transfer */
     SPIM_REGS_CTRLR0_DFS_FRAME_08BITS = 7,    /**< 08-bit serial data transfer */
     SPIM_REGS_CTRLR0_DFS_FRAME_09BITS = 8,    /**< 09-bit serial data transfer */
     SPIM_REGS_CTRLR0_DFS_FRAME_10BITS = 9,    /**< 10-bit serial data transfer */
     SPIM_REGS_CTRLR0_DFS_FRAME_11BITS = 10,    /**< 11-bit serial data transfer */
     SPIM_REGS_CTRLR0_DFS_FRAME_12BITS = 11,    /**< 12-bit serial data transfer */
     SPIM_REGS_CTRLR0_DFS_FRAME_13BITS = 12,    /**< 13-bit serial data transfer */
     SPIM_REGS_CTRLR0_DFS_FRAME_14BITS = 13,    /**< 14-bit serial data transfer */
     SPIM_REGS_CTRLR0_DFS_FRAME_15BITS = 14,    /**< 15-bit serial data transfer */
     SPIM_REGS_CTRLR0_DFS_FRAME_16BITS = 15     /**< 16-bit serial data transfer */
}  ADI_SPIM_REGS_CTRLR0_DFS;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_CTRLR0_FRF
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_CTRLR0_FRF_MOTOROLLA_SPI = 0,    /**< Motorolla SPI Frame Format */
     SPIM_REGS_CTRLR0_FRF_TEXAS_SSP   = 1,    /**< Texas Instruments SSP Frame Format */
     SPIM_REGS_CTRLR0_FRF_NS_MICROWIRE = 2     /**< National Microwire Frame Format */
}  ADI_SPIM_REGS_CTRLR0_FRF;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_CTRLR0_SCPH
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_CTRLR0_SCPH_MIDDLE_BIT = 0,    /**< Clock toggles in middle of first bit */
     SPIM_REGS_CTRLR0_SCPH_START_BIT  = 1     /**< Clock toggles at start  of first bit */
}  ADI_SPIM_REGS_CTRLR0_SCPH;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_CTRLR0_SCPOL
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_CTRLR0_SCPOL_INACTIVE_HIGH = 0,    /**< Clock is inactive when high */
     SPIM_REGS_CTRLR0_SCPOL_INACTIVE_LOW = 1     /**< Clock is inactive when low */
}  ADI_SPIM_REGS_CTRLR0_SCPOL;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_CTRLR0_TMOD
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_CTRLR0_TMOD_TX_AND_RX  = 0,    /**< Transmit & receive */
     SPIM_REGS_CTRLR0_TMOD_TX_ONLY    = 1,    /**< Transmit only mode */
     SPIM_REGS_CTRLR0_TMOD_RX_ONLY    = 2,    /**< Receive  only mode */
     SPIM_REGS_CTRLR0_TMOD_EEPROM_READ = 3     /**< EEPROM  Read  mode */
}  ADI_SPIM_REGS_CTRLR0_TMOD;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_CTRLR0_SRL
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_CTRLR0_SRL_NORMAL_MODE = 0,    /**< Normal mode operation */
     SPIM_REGS_CTRLR0_SRL_TESTING_MODE = 1     /**< Test mode: Tx & Rx shift reg connected */
}  ADI_SPIM_REGS_CTRLR0_SRL;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_CTRLR0_CFS
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_CTRLR0_CFS_SIZE_01_BIT = 0,    /**< 01-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_02_BIT = 1,    /**< 02-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_03_BIT = 2,    /**< 03-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_04_BIT = 3,    /**< 04-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_05_BIT = 4,    /**< 05-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_06_BIT = 5,    /**< 06-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_07_BIT = 6,    /**< 07-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_08_BIT = 7,    /**< 08-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_09_BIT = 8,    /**< 09-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_10_BIT = 9,    /**< 10-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_11_BIT = 10,    /**< 11-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_12_BIT = 11,    /**< 12-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_13_BIT = 12,    /**< 13-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_14_BIT = 13,    /**< 14-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_15_BIT = 14,    /**< 15-bit Control Word */
     SPIM_REGS_CTRLR0_CFS_SIZE_16_BIT = 15     /**< 16-bit Control Word */
}  ADI_SPIM_REGS_CTRLR0_CFS;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_CTRLR0_t
 *  \brief Control Register 0
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_CTRLR0_t {
    union {
        struct {
             unsigned int DFS              : 4; /**< No description provided */
             unsigned int FRF              : 2; /**< No description provided */
             unsigned int SCPH             : 1; /**< No description provided */
             unsigned int SCPOL            : 1; /**< No description provided */
             unsigned int TMOD             : 2; /**< No description provided */
             unsigned int NOT_USED         : 1; /**< No description provided */
             unsigned int SRL              : 1; /**< No description provided */
             unsigned int CFS              : 4; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_CTRLR0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CTRLR1 Control Register 1 (CTRLR1) Register
 *  Control Register 1 (CTRLR1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_CTRLR1_t
 *  \brief Control Register 1
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_CTRLR1_t {
    union {
        struct {
             unsigned int NDF              : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_CTRLR1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSIENR SSI Enable Register (SSIENR) Register
 *  SSI Enable Register (SSIENR) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_SSIENR_SSI_EN
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_SSIENR_SSI_EN_DISABLE  = 0,    /**< Disables Serial Transfer */
     SPIM_REGS_SSIENR_SSI_EN_ENABLED  = 1     /**< Enables  Serial Transfer */
}  ADI_SPIM_REGS_SSIENR_SSI_EN;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_SSIENR_t
 *  \brief SSI Enable Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_SSIENR_t {
    union {
        struct {
             unsigned int SSI_EN           : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_SSIENR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup MWCR Microwire Control Register (MWCR) Register
 *  Microwire Control Register (MWCR) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_MWCR_MWMOD
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_MWCR_MWMOD_NON_SEQUENTIAL = 0,    /**< Non-Sequential Microwire Transfer */
     SPIM_REGS_MWCR_MWMOD_SEQUENTIAL  = 1     /**< Sequential Microwire Transfer */
}  ADI_SPIM_REGS_MWCR_MWMOD;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_MWCR_MDD
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_MWCR_MDD_RECEIVE       = 0,    /**< SSI  receives data */
     SPIM_REGS_MWCR_MDD_TRANSMIT      = 1     /**< SSI transmits data */
}  ADI_SPIM_REGS_MWCR_MDD;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_MWCR_MHS
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_MWCR_MHS_DISABLE       = 0,    /**< Disables Handshaking IF */
     SPIM_REGS_MWCR_MHS_ENABLED       = 1     /**< Enables  Handshaking IF */
}  ADI_SPIM_REGS_MWCR_MHS;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_MWCR_t
 *  \brief Microwire Control Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_MWCR_t {
    union {
        struct {
             unsigned int MWMOD            : 1; /**< No description provided */
             unsigned int MDD              : 1; /**< No description provided */
             unsigned int MHS              : 1; /**< No description provided */
             unsigned int RESERVED3        : 13; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_MWCR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SER Slave Enable Register (SER) Register
 *  Slave Enable Register (SER) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_SER_SER
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_SER_SER_NOT_SELECTED   = 0,    /**< No slave selected */
     SPIM_REGS_SER_SER_SELECTED       = 1     /**< Slave is selected */
}  ADI_SPIM_REGS_SER_SER;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_SER_t
 *  \brief Slave Enable Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_SER_t {
    union {
        struct {
             unsigned int SER              : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_SER_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup BAUDR Baud Rate Select (BAUDR) Register
 *  Baud Rate Select (BAUDR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_BAUDR_t
 *  \brief Baud Rate Select
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_BAUDR_t {
    union {
        struct {
             unsigned int SCKDV            : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_BAUDR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup TXFTLR Transmit FIFO Threshold Level (TXFTLR) Register
 *  Transmit FIFO Threshold Level (TXFTLR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_TXFTLR_t
 *  \brief Transmit FIFO Threshold Level
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_TXFTLR_t {
    union {
        struct {
             unsigned int TFT              : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_TXFTLR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup RXFTLR Receive FIFO Threshold Level (RXFTLR) Register
 *  Receive FIFO Threshold Level (RXFTLR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_RXFTLR_t
 *  \brief Receive FIFO Threshold Level
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_RXFTLR_t {
    union {
        struct {
             unsigned int RFT              : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_RXFTLR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup TXFLR Transmit FIFO Level Register (TXFLR) Register
 *  Transmit FIFO Level Register (TXFLR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_TXFLR_t
 *  \brief Transmit FIFO Level Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_TXFLR_t {
    union {
        struct {
             unsigned int TXTFL            : 3; /**< No description provided */
             unsigned int RESERVED3        : 13; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_TXFLR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup RXFLR Receive FIFO Level Register (RXFLR) Register
 *  Receive FIFO Level Register (RXFLR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_RXFLR_t
 *  \brief Receive FIFO Level Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_RXFLR_t {
    union {
        struct {
             unsigned int RXTFL            : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_RXFLR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SR Status Register (SR) Register
 *  Status Register (SR) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_SR_BUSY
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_SR_BUSY_INACTIVE       = 0,    /**< SSI is Idle/Disabled */
     SPIM_REGS_SR_BUSY_ACTIVE         = 1     /**< SSI is Active */
}  ADI_SPIM_REGS_SR_BUSY;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_SR_TFNF
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_SR_TFNF_FULL           = 0,    /**< Tx FIFO is full */
     SPIM_REGS_SR_TFNF_NOT_FULL       = 1     /**< Tx FIFO is not Full */
}  ADI_SPIM_REGS_SR_TFNF;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_SR_TFE
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_SR_TFE_NOT_EMPTY       = 0,    /**< Tx FIFO is nonempty */
     SPIM_REGS_SR_TFE_EMPTY           = 1     /**< Tx FIFO  is   empty */
}  ADI_SPIM_REGS_SR_TFE;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_SR_RFNE
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_SR_RFNE_EMPTY          = 0,    /**< Rx FIFO  is   empty */
     SPIM_REGS_SR_RFNE_NOT_EMPTY      = 1     /**< Rx FIFO is nonempty */
}  ADI_SPIM_REGS_SR_RFNE;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_SR_RFF
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_SR_RFF_NOT_FULL        = 0,    /**< RX FIFO is not full */
     SPIM_REGS_SR_RFF_FULL            = 1     /**< Rx FIFO is full */
}  ADI_SPIM_REGS_SR_RFF;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_SR_DCOL
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_SR_DCOL_NO_ERROR_CONDITION = 0,    /**< No Data Error Condition */
     SPIM_REGS_SR_DCOL_TX_COLLISION_ERROR = 1     /**< TX Data Collision Error */
}  ADI_SPIM_REGS_SR_DCOL;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_SR_t
 *  \brief Status Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_SR_t {
    union {
        struct {
             unsigned int BUSY             : 1; /**< No description provided */
             unsigned int TFNF             : 1; /**< No description provided */
             unsigned int TFE              : 1; /**< No description provided */
             unsigned int RFNE             : 1; /**< No description provided */
             unsigned int RFF              : 1; /**< No description provided */
             unsigned int RESERVED5        : 1; /**< Reserved */
             unsigned int DCOL             : 1; /**< No description provided */
             unsigned int RESERVED7        : 9; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_SR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup IMR Interrupt Mask Register (IMR) Register
 *  Interrupt Mask Register (IMR) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_IMR_TXEIM
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_IMR_TXEIM_MASKED       = 0,    /**< TX FIFO Empty Interrupt is masked */
     SPIM_REGS_IMR_TXEIM_UNMASKED     = 1     /**< TX FIFO Empty Interrupt un-masked */
}  ADI_SPIM_REGS_IMR_TXEIM;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_IMR_TXOIM
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_IMR_TXOIM_MASKED       = 0,    /**< TX FIFO Overflow Interrupt is masked */
     SPIM_REGS_IMR_TXOIM_UNMASKED     = 1     /**< TX FIFO Overflow Interrupt un-masked */
}  ADI_SPIM_REGS_IMR_TXOIM;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_IMR_RXUIM
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_IMR_RXUIM_MASKED       = 0,    /**< RX FIFO Underflow Interrupt is masked */
     SPIM_REGS_IMR_RXUIM_UNMASKED     = 1     /**< RX FIFO Underflow Interrupt un-masked */
}  ADI_SPIM_REGS_IMR_RXUIM;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_IMR_RXOIM
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_IMR_RXOIM_MASKED       = 0,    /**< RX FIFO Overflow Interrupt is masked */
     SPIM_REGS_IMR_RXOIM_UNMASKED     = 1     /**< RX FIFO Overflow Interrupt un-masked */
}  ADI_SPIM_REGS_IMR_RXOIM;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_IMR_RXFIM
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_IMR_RXFIM_MASKED       = 0,    /**< RX FIFO Full Interrupt is masked */
     SPIM_REGS_IMR_RXFIM_UNMASKED     = 1     /**< RX FIFO Full Interrupt un-masked */
}  ADI_SPIM_REGS_IMR_RXFIM;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_IMR_MSTIM
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_IMR_MSTIM_MASKED       = 0,    /**< Multi-Master Contention Interrupt is masked */
     SPIM_REGS_IMR_MSTIM_UNMASKED     = 1     /**< Multi-Master Contention Interrupt un-masked */
}  ADI_SPIM_REGS_IMR_MSTIM;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_IMR_t
 *  \brief Interrupt Mask Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_IMR_t {
    union {
        struct {
             unsigned int TXEIM            : 1; /**< No description provided */
             unsigned int TXOIM            : 1; /**< No description provided */
             unsigned int RXUIM            : 1; /**< No description provided */
             unsigned int RXOIM            : 1; /**< No description provided */
             unsigned int RXFIM            : 1; /**< No description provided */
             unsigned int MSTIM            : 1; /**< No description provided */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_IMR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ISR Interrupt Status Register (ISR) Register
 *  Interrupt Status Register (ISR) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_ISR_TXEIS
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_ISR_TXEIS_INACTIVE     = 0,    /**< TX FIFO Empty Interrupt nonactive */
     SPIM_REGS_ISR_TXEIS_ACTIVE       = 1     /**< TX FIFO Empty Interrupt is active */
}  ADI_SPIM_REGS_ISR_TXEIS;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_ISR_TXOIS
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_ISR_TXOIS_INACTIVE     = 0,    /**< TX FIFO Overflow Interrupt nonactive */
     SPIM_REGS_ISR_TXOIS_ACTIVE       = 1     /**< TX FIFO Overflow Interrupt is active */
}  ADI_SPIM_REGS_ISR_TXOIS;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_ISR_RXUIS
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_ISR_RXUIS_INACTIVE     = 0,    /**< RX FIFO Underflow Interrupt nonactive */
     SPIM_REGS_ISR_RXUIS_ACTIVE       = 1     /**< RX FIFO underflow Interrupt is active */
}  ADI_SPIM_REGS_ISR_RXUIS;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_ISR_RXOIS
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_ISR_RXOIS_INACTIVE     = 0,    /**< RX FIFO Overflow Interrupt nonactive */
     SPIM_REGS_ISR_RXOIS_ACTIVE       = 1     /**< RX FIFO Overflow Interrupt is active */
}  ADI_SPIM_REGS_ISR_RXOIS;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_ISR_RXFIS
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_ISR_RXFIS_INACTIVE     = 0,    /**< RX FIFO Full Interrupt nonactive */
     SPIM_REGS_ISR_RXFIS_ACTIVE       = 1     /**< RX FIFO Full Interrupt is active */
}  ADI_SPIM_REGS_ISR_RXFIS;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_ISR_MSTIS
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_ISR_MSTIS_INACTIVE     = 0,    /**< Multi-master Contention Interrupt nonactive */
     SPIM_REGS_ISR_MSTIS_ACTIVE       = 1     /**< Multi-master Contention Interrupt is active */
}  ADI_SPIM_REGS_ISR_MSTIS;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_ISR_t
 *  \brief Interrupt Status Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_ISR_t {
    union {
        struct {
             unsigned int TXEIS            : 1; /**< No description provided */
             unsigned int TXOIS            : 1; /**< No description provided */
             unsigned int RXUIS            : 1; /**< No description provided */
             unsigned int RXOIS            : 1; /**< No description provided */
             unsigned int RXFIS            : 1; /**< No description provided */
             unsigned int MSTIS            : 1; /**< No description provided */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_ISR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup RISR Raw Interrupt Status Register (RISR) Register
 *  Raw Interrupt Status Register (RISR) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_RISR_TXEIR
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_RISR_TXEIR_INACTIVE    = 0,    /**< Raw TX FIFO Empty Interrupt nonactive */
     SPIM_REGS_RISR_TXEIR_ACTIVE      = 1     /**< Raw TX FIFO Empty Interrupt is active */
}  ADI_SPIM_REGS_RISR_TXEIR;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_RISR_TXOIR
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_RISR_TXOIR_INACTIVE    = 0,    /**< Raw TX FIFO Overflow Interrupt nonactive */
     SPIM_REGS_RISR_TXOIR_ACTIVE      = 1     /**< Raw TX FIFO Overflow Interrupt is active */
}  ADI_SPIM_REGS_RISR_TXOIR;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_RISR_RXUIR
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_RISR_RXUIR_INACTIVE    = 0,    /**< Raw RX FIFO Underflow Interrupt nonactive */
     SPIM_REGS_RISR_RXUIR_ACTIVE      = 1     /**< Raw RX FIFO underflow Interrupt is active */
}  ADI_SPIM_REGS_RISR_RXUIR;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_RISR_RXOIR
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_RISR_RXOIR_INACTIVE    = 0,    /**< Raw RX FIFO Overflow Interrupt nonactive */
     SPIM_REGS_RISR_RXOIR_ACTIVE      = 1     /**< Raw RX FIFO Overflow Interrupt is active */
}  ADI_SPIM_REGS_RISR_RXOIR;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_RISR_RXFIR
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_RISR_RXFIR_INACTIVE    = 0,    /**< Raw RX FIFO Full Interrupt nonactive */
     SPIM_REGS_RISR_RXFIR_ACTIVE      = 1     /**< Raw RX FIFO Full Interrupt is active */
}  ADI_SPIM_REGS_RISR_RXFIR;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_SPIM_REGS_RISR_MSTIR
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     SPIM_REGS_RISR_MSTIR_INACTIVE    = 0,    /**< Raw Multi-master Contention Interrupt nonactive */
     SPIM_REGS_RISR_MSTIR_ACTIVE      = 1     /**< Raw Multi-master Contention Interrupt is active */
}  ADI_SPIM_REGS_RISR_MSTIR;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_RISR_t
 *  \brief Raw Interrupt Status Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_RISR_t {
    union {
        struct {
             unsigned int TXEIR            : 1; /**< No description provided */
             unsigned int TXOIR            : 1; /**< No description provided */
             unsigned int RXUIR            : 1; /**< No description provided */
             unsigned int RXOIR            : 1; /**< No description provided */
             unsigned int RXFIR            : 1; /**< No description provided */
             unsigned int MSTIR            : 1; /**< No description provided */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_RISR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup TXOICR Transmit FIFO Overflow Interrupt Clear Register Register (TXOICR) Register
 *  Transmit FIFO Overflow Interrupt Clear Register Register (TXOICR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_TXOICR_t
 *  \brief Transmit FIFO Overflow Interrupt Clear Register Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_TXOICR_t {
    union {
        struct {
             unsigned int TXOICR           : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_TXOICR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup RXOICR Receive FIFO Overflow Interrupt Clear Register (RXOICR) Register
 *  Receive FIFO Overflow Interrupt Clear Register (RXOICR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_RXOICR_t
 *  \brief Receive FIFO Overflow Interrupt Clear Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_RXOICR_t {
    union {
        struct {
             unsigned int RXOICR           : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_RXOICR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup RXUICR Receive FIFO Underflow Interrupt Clear Register (RXUICR) Register
 *  Receive FIFO Underflow Interrupt Clear Register (RXUICR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_RXUICR_t
 *  \brief Receive FIFO Underflow Interrupt Clear Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_RXUICR_t {
    union {
        struct {
             unsigned int RXUICR           : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_RXUICR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup MSTICR Multi-Master Interrupt Clear Register (MSTICR) Register
 *  Multi-Master Interrupt Clear Register (MSTICR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_MSTICR_t
 *  \brief Multi-Master Interrupt Clear Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_MSTICR_t {
    union {
        struct {
             unsigned int MSTICR           : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_MSTICR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ICR Interrupt Clear Register (ICR) Register
 *  Interrupt Clear Register (ICR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_ICR_t
 *  \brief Interrupt Clear Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_ICR_t {
    union {
        struct {
             unsigned int ICR              : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_ICR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup IDR Identification Register (IDR) Register
 *  Identification Register (IDR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_IDR_t
 *  \brief Identification Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_IDR_t {
    union {
        struct {
             unsigned int IDCODE           : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_IDR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSI_VERSION_ID Core Kit version ID Register (SSI_VERSION_ID) Register
 *  Core Kit version ID Register (SSI_VERSION_ID) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_SSI_VERSION_ID_t
 *  \brief Core Kit version ID Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_SSI_VERSION_ID_t {
    union {
        struct {
             unsigned int SSI_COMP_VERSION : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_SSI_VERSION_ID_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DR0 DW_apb_ssi Data Register (DR0) Register
 *  DW_apb_ssi Data Register (DR0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SPIM_REGS_DR0_t
 *  \brief DW_apb_ssi Data Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SPIM_REGS_DR0_t {
    union {
        struct {
             unsigned int DR0              : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SPIM_REGS_DR0_t;
#ifdef SWIG
%}
#endif

/*@}*/

#endif  /* end ifndef ADI_SPIM_REGS_TYPEDEFS_H_ */

#ifndef ADI_CSI2_REGSPEC_TOP_CPU0_TYPEDEFS_H_
#define ADI_CSI2_REGSPEC_TOP_CPU0_TYPEDEFS_H_

/** @defgroup CSI2_TX_BASE_CFG_NUM_LANES No description provided (CSI2_TX_BASE_CFG_NUM_LANES) Register
 *  No description provided (CSI2_TX_BASE_CFG_NUM_LANES) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_NUM_LANES_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_NUM_LANES_t {
    union {
        struct {
             unsigned int CFG_NUM_LANES    : 4; /**< No description provided */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_NUM_LANES_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_CFG_T_PRE No description provided (CSI2_TX_BASE_CFG_T_PRE) Register
 *  No description provided (CSI2_TX_BASE_CFG_T_PRE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_T_PRE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_T_PRE_t {
    union {
        struct {
             unsigned int CFG_T_PRE        : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_T_PRE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_CFG_T_POST No description provided (CSI2_TX_BASE_CFG_T_POST) Register
 *  No description provided (CSI2_TX_BASE_CFG_T_POST) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_T_POST_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_T_POST_t {
    union {
        struct {
             unsigned int CFG_T_POST       : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_T_POST_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_CFG_TX_GAP No description provided (CSI2_TX_BASE_CFG_TX_GAP) Register
 *  No description provided (CSI2_TX_BASE_CFG_TX_GAP) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_TX_GAP_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_TX_GAP_t {
    union {
        struct {
             unsigned int CFG_TX_GAP       : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_TX_GAP_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_CFG_T_CLK_GAP No description provided (CSI2_TX_BASE_CFG_T_CLK_GAP) Register
 *  No description provided (CSI2_TX_BASE_CFG_T_CLK_GAP) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_T_CLK_GAP_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_T_CLK_GAP_t {
    union {
        struct {
             unsigned int CFG_T_CLK_GAP    : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_T_CLK_GAP_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_CFG_CONTINUOUS_HS_CLK No description provided (CSI2_TX_BASE_CFG_CONTINUOUS_HS_CLK) Register
 *  No description provided (CSI2_TX_BASE_CFG_CONTINUOUS_HS_CLK) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_CONTINUOUS_HS_CLK_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_CONTINUOUS_HS_CLK_t {
    union {
        struct {
             unsigned int CFG_CONTINUOUS_HS_CLK : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_CONTINUOUS_HS_CLK_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_CFG_TWAKEUP No description provided (CSI2_TX_BASE_CFG_TWAKEUP) Register
 *  No description provided (CSI2_TX_BASE_CFG_TWAKEUP) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_TWAKEUP_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_TWAKEUP_t {
    union {
        struct {
             unsigned int CFG_TWAKEUP      : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_TWAKEUP_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_ULPS_CLK_ENABLE No description provided (CSI2_TX_BASE_ULPS_CLK_ENABLE) Register
 *  No description provided (CSI2_TX_BASE_ULPS_CLK_ENABLE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_CLK_ENABLE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_CLK_ENABLE_t {
    union {
        struct {
             unsigned int ULPS_CLK_ENABLE  : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_CLK_ENABLE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_ULPS_ENABLE No description provided (CSI2_TX_BASE_ULPS_ENABLE) Register
 *  No description provided (CSI2_TX_BASE_ULPS_ENABLE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_ENABLE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_ENABLE_t {
    union {
        struct {
             unsigned int ULPS_ENABLE      : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_ENABLE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_ULPS_CLK_ACTIVE No description provided (CSI2_TX_BASE_ULPS_CLK_ACTIVE) Register
 *  No description provided (CSI2_TX_BASE_ULPS_CLK_ACTIVE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_CLK_ACTIVE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_CLK_ACTIVE_t {
    union {
        struct {
             unsigned int ULPS_CLK_ACTIVE  : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_CLK_ACTIVE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_ULPS_ACTIVE No description provided (CSI2_TX_BASE_ULPS_ACTIVE) Register
 *  No description provided (CSI2_TX_BASE_ULPS_ACTIVE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_ACTIVE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_ACTIVE_t {
    union {
        struct {
             unsigned int ULPS_ACTIVE      : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_ULPS_ACTIVE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_IRQ_STATUS No description provided (CSI2_TX_BASE_IRQ_STATUS) Register
 *  No description provided (CSI2_TX_BASE_IRQ_STATUS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_IRQ_STATUS_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_IRQ_STATUS_t {
    union {
        struct {
             unsigned int IRQ_STATUS       : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_IRQ_STATUS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_IRQ_ENABLE No description provided (CSI2_TX_BASE_IRQ_ENABLE) Register
 *  No description provided (CSI2_TX_BASE_IRQ_ENABLE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_IRQ_ENABLE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_IRQ_ENABLE_t {
    union {
        struct {
             unsigned int IRQ_ENABLE       : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_IRQ_ENABLE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_CSI2TX_IRQ_CLR No description provided (CSI2_TX_BASE_CSI2TX_IRQ_CLR) Register
 *  No description provided (CSI2_TX_BASE_CSI2TX_IRQ_CLR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CSI2TX_IRQ_CLR_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CSI2TX_IRQ_CLR_t {
    union {
        struct {
             unsigned int IRQ_CLR          : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CSI2TX_IRQ_CLR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_CFG_CLK_LANE_EN No description provided (CSI2_TX_BASE_CFG_CLK_LANE_EN) Register
 *  No description provided (CSI2_TX_BASE_CFG_CLK_LANE_EN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_CLK_LANE_EN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_CLK_LANE_EN_t {
    union {
        struct {
             unsigned int CFG_CLK_LANE_EN  : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_CLK_LANE_EN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_CFG_DATA_LANE_EN No description provided (CSI2_TX_BASE_CFG_DATA_LANE_EN) Register
 *  No description provided (CSI2_TX_BASE_CFG_DATA_LANE_EN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_DATA_LANE_EN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_DATA_LANE_EN_t {
    union {
        struct {
             unsigned int CFG_DATA_LANE_EN : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_DATA_LANE_EN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_CFG_CPHY_EN No description provided (CSI2_TX_BASE_CFG_CPHY_EN) Register
 *  No description provided (CSI2_TX_BASE_CFG_CPHY_EN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_CPHY_EN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_CPHY_EN_t {
    union {
        struct {
             unsigned int CFG_CPHY_EN      : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_CPHY_EN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_CFG_PPI_16_EN No description provided (CSI2_TX_BASE_CFG_PPI_16_EN) Register
 *  No description provided (CSI2_TX_BASE_CFG_PPI_16_EN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_PPI_16_EN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_PPI_16_EN_t {
    union {
        struct {
             unsigned int CFG_PPI_16_EN    : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_PPI_16_EN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_CFG_PACKET_INTERFACE_EN No description provided (CSI2_TX_BASE_CFG_PACKET_INTERFACE_EN) Register
 *  No description provided (CSI2_TX_BASE_CFG_PACKET_INTERFACE_EN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_PACKET_INTERFACE_EN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_PACKET_INTERFACE_EN_t {
    union {
        struct {
             unsigned int CFG_PACKET_INTERFACE_EN : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_PACKET_INTERFACE_EN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_BASE_CFG_VCX_EN No description provided (CSI2_TX_BASE_CFG_VCX_EN) Register
 *  No description provided (CSI2_TX_BASE_CFG_VCX_EN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_VCX_EN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_VCX_EN_t {
    union {
        struct {
             unsigned int CFG_VCX_EN       : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_BASE_CFG_VCX_EN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_I No description provided (CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_I) Register
 *  No description provided (CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_I) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_I_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_I_t {
    union {
        struct {
             unsigned int CFG_SKEWCAL_TIME_I : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_I_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_P No description provided (CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_P) Register
 *  No description provided (CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_P) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_P_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_P_t {
    union {
        struct {
             unsigned int CFG_SKEWCAL_TIME_P : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_SKEW_CAL_CFG_SKEWCAL_TIME_P_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PLL_PHY No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PLL_PHY) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PLL_PHY) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PLL_PHY_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PLL_PHY_t {
    union {
        struct {
             unsigned int CFG_MIXEL_PD_PHY : 1; /**< No description provided */
             unsigned int CFG_MIXEL_PD_PLL : 1; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_PD_PLL_PHY_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PLL_CTRL No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PLL_CTRL) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PLL_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PLL_CTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PLL_CTRL_t {
    union {
        struct {
             unsigned int CFG_MIXEL_ULPS_PLL_CTRL : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PLL_CTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PHY_CTRL No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PHY_CTRL) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PHY_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PHY_CTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PHY_CTRL_t {
    union {
        struct {
             unsigned int CFG_MIXEL_ULPS_PHY_CTRL : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_ULPS_PHY_CTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TST_PLL No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TST_PLL) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TST_PLL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TST_PLL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TST_PLL_t {
    union {
        struct {
             unsigned int CFG_MIXEL_TST_PLL : 4; /**< No description provided */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TST_PLL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CN No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CN) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CN_t {
    union {
        struct {
             unsigned int CFG_MIXEL_CN     : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CM No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CM) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CM) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CM_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CM_t {
    union {
        struct {
             unsigned int CFG_MIXEL_CM     : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CM_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CO No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CO) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CO) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CO_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CO_t {
    union {
        struct {
             unsigned int CFG_MIXEL_CO     : 3; /**< No description provided */
             unsigned int RESERVED3        : 13; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_CO_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_BYP No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_BYP) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_BYP) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_BYP_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_BYP_t {
    union {
        struct {
             unsigned int CFG_MIXEL_LOCK_BYP : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_BYP_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_BYPASS_PLL No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_BYPASS_PLL) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_BYPASS_PLL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_BYPASS_PLL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_BYPASS_PLL_t {
    union {
        struct {
             unsigned int CFG_MIXEL_BYPASS_PLL : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_BYPASS_PLL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_LATCH No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_LATCH) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_LATCH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_LATCH_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_LATCH_t {
    union {
        struct {
             unsigned int CFG_MIXEL_LOCK_LATCH : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_LATCH_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_AUTO_PD_EN No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_AUTO_PD_EN) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_AUTO_PD_EN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_AUTO_PD_EN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_AUTO_PD_EN_t {
    union {
        struct {
             unsigned int CFG_MIXEL_AUTO_PD_EN : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_AUTO_PD_EN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_ENBL No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_ENBL) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_ENBL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_ENBL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_ENBL_t {
    union {
        struct {
             unsigned int CFG_MIXEL_TEST_ENBL : 6; /**< No description provided */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_ENBL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_t {
    union {
        struct {
             unsigned int CFG_MIXEL_LOCK   : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_LOCK_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_ZERO No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_ZERO) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_ZERO) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_ZERO_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_ZERO_t {
    union {
        struct {
             unsigned int CFG_MIXEL_M_PRG_HS_ZERO : 6; /**< No description provided */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_ZERO_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_ZERO No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_ZERO) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_ZERO) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_ZERO_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_ZERO_t {
    union {
        struct {
             unsigned int CFG_MIXEL_MC_PRG_HS_ZERO : 7; /**< No description provided */
             unsigned int RESERVED7        : 9; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_ZERO_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_TRAIL No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_TRAIL) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_TRAIL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_TRAIL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_TRAIL_t {
    union {
        struct {
             unsigned int CFG_MIXEL_M_PRG_HS_TRAIL : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_TRAIL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_TRAIL No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_TRAIL) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_TRAIL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_TRAIL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_TRAIL_t {
    union {
        struct {
             unsigned int CFG_MIXEL_MC_PRG_HS_TRAIL : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_TRAIL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_PREPARE No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_PREPARE) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_PREPARE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_PREPARE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_PREPARE_t {
    union {
        struct {
             unsigned int CFG_MIXEL_M_PRG_HS_PREPARE : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_M_PRG_HS_PREPARE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_PREPARE No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_PREPARE) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_PREPARE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_PREPARE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_PREPARE_t {
    union {
        struct {
             unsigned int CFG_MIXEL_MC_PRG_HS_PREPARE : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_MC_PRG_HS_PREPARE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN0 No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN0) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN0_t {
    union {
        struct {
             unsigned int CFG_MIXEL_TEST_PATTERN : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN1 No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN1) Register
 *  No description provided (CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN1_t {
    union {
        struct {
             unsigned int CFG_MIXEL_TEST_PATTERN : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_CSI2_REGSPEC_TOP_CPU0_CSI2_TX_MIXEL_DPHY_CFG_MIXEL_TEST_PATTERN1_t;
#ifdef SWIG
%}
#endif

/*@}*/

#endif  /* end ifndef ADI_CSI2_REGSPEC_TOP_CPU0_TYPEDEFS_H_ */

#ifndef ADI_EFUSE_TYPEDEFS_H_
#define ADI_EFUSE_TYPEDEFS_H_

/** @defgroup PWR_CTRL Power Control Register (PWR_CTRL) Register
 *  Power Control Register (PWR_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_EFUSE_PWR_CTRL_POWER_CONTROL
 *  \brief Power Control
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     EFUSE_PWR_CTRL_POWER_CONTROL_POWERDOWN = 0,    /**< POWERDOWN: Lowest power state. All accesses incur extra 800ns wait to enter standby state */
     EFUSE_PWR_CTRL_POWER_CONTROL_STANDBY = 1     /**< STANDBY: Ready to program or read */
}  ADI_EFUSE_PWR_CTRL_POWER_CONTROL;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \struct ADI_EFUSE_PWR_CTRL_t
 *  \brief Power Control Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_EFUSE_PWR_CTRL_t {
    union {
        struct {
             unsigned int POWER_CONTROL    : 1; /**< Power Control */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_EFUSE_PWR_CTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup STATUS Program status register (STATUS) Register
 *  Program status register (STATUS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_EFUSE_STATUS_t
 *  \brief Program status register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_EFUSE_STATUS_t {
    union {
        struct {
             unsigned int FUSE_NOT_BURNED  : 1; /**< Post-programming error: fuse not burned */
             unsigned int FUSE_ACCIDENTAL_BURN : 1; /**< Post-programming error: fuse accidentally burned */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_EFUSE_STATUS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ERR_LOCATION Error location (ERR_LOCATION) Register
 *  Error location (ERR_LOCATION) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_EFUSE_ERR_LOCATION_t
 *  \brief Error location
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_EFUSE_ERR_LOCATION_t {
    union {
        struct {
             unsigned int PGM_ERR_BIT_ADDR : 12; /**< Error location */
             unsigned int RESERVED12       : 4; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_EFUSE_ERR_LOCATION_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup TIMING Timing Configuration (TIMING) Register
 *  Timing Configuration (TIMING) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_EFUSE_TIMING_t
 *  \brief Timing Configuration
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_EFUSE_TIMING_t {
    union {
        struct {
             unsigned int TPGM_PULSE       : 8; /**< Program STROBE pulse width */
             unsigned int TREAD_PULSE      : 2; /**< Read STROBE pulse width */
             unsigned int TUNIT            : 2; /**< Efuse timing diagram step size */
             unsigned int TPDPS            : 4; /**< PD to PS setup time */
        };
        uint16_t VALUE16;
    };
} ADI_EFUSE_TIMING_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup CHARACTERIZATION Characterization Options (CHARACTERIZATION) Register
 *  Characterization Options (CHARACTERIZATION) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_EFUSE_CHARACTERIZATION_PGM_MODE
 *  \brief Program mode
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     EFUSE_CHARACTERIZATION_PGM_MODE_FUNCTIONAL_PGM = 0,    /**< Functional mode */
     EFUSE_CHARACTERIZATION_PGM_MODE_DEBUG_PGM = 1     /**< Debug Mode */
}  ADI_EFUSE_CHARACTERIZATION_PGM_MODE;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_EFUSE_CHARACTERIZATION_MARGIN
 *  \brief Set the margin level in the sense amplifiers
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     EFUSE_CHARACTERIZATION_MARGIN_MARGIN_AUTO = 0,    /**< AUTO: Margin set automatically by state machine */
     EFUSE_CHARACTERIZATION_MARGIN_MARGIN_LOW = 1,    /**< LOW: Applies lowest margin. Used at time 0 to test for unusual high resistance fuse */
     EFUSE_CHARACTERIZATION_MARGIN_MARGIN_NORMAL = 2,    /**< NOMINAL: Applies nominal margin level */
     EFUSE_CHARACTERIZATION_MARGIN_MARGIN_HIGH = 3     /**< HIGH: Applies highest margin. Used in post program readback to ensure no weakly blown fuses. */
}  ADI_EFUSE_CHARACTERIZATION_MARGIN;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \struct ADI_EFUSE_CHARACTERIZATION_t
 *  \brief Characterization Options
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_EFUSE_CHARACTERIZATION_t {
    union {
        struct {
             unsigned int RM_ENABLE        : 1; /**< Enable resistance measurement mode */
             unsigned int RESERVED1        : 3; /**< Reserved */
             unsigned int PGM_MODE         : 1; /**< Program mode */
             unsigned int RESERVED5        : 3; /**< Reserved */
             unsigned int MARGIN           : 2; /**< Set the margin level in the sense amplifiers */
             unsigned int RESERVED10       : 2; /**< Reserved */
             unsigned int FIXED_ROW_READ   : 1; /**< Read from Fixed Row */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_EFUSE_CHARACTERIZATION_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup BLANK_CHECK Blank Check Register (BLANK_CHECK) Register
 *  Blank Check Register (BLANK_CHECK) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_EFUSE_BLANK_CHECK_t
 *  \brief Blank Check Register
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_EFUSE_BLANK_CHECK_t {
    union {
        struct {
             unsigned int BLANK_CHECK      : 1; /**< Initiate read of whole fuse macro to check if blank */
             unsigned int BLANK_CHECK_FAIL : 1; /**< Blank Check Failed */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_EFUSE_BLANK_CHECK_t;
#ifdef SWIG
%}
#endif

/*@}*/

#endif  /* end ifndef ADI_EFUSE_TYPEDEFS_H_ */

#ifndef ADI_LPS_REGS_YODA_TYPEDEFS_H_
#define ADI_LPS_REGS_YODA_TYPEDEFS_H_

/** @defgroup LPSCTRL No description provided (LPSCTRL) Register
 *  No description provided (LPSCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSCTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_LPS_REGS_YODA_LPSCTRL_t {
    union {
        struct {
             unsigned int LPS_ENA          : 1; /**< No description provided */
             unsigned int RESERVED1        : 1; /**< No description provided */
             unsigned int LPS_BUSY         : 1; /**< No description provided */
             unsigned int RESERVED         : 1; /**< No description provided */
             unsigned int LPS_START_EN     : 1; /**< No description provided */
             unsigned int LPS_RPT_EN       : 1; /**< No description provided */
             unsigned int LPS_ON           : 1; /**< No description provided */
             unsigned int LPS_OFF          : 1; /**< No description provided */
             unsigned int LPS_ON_IGNORE    : 1; /**< No description provided */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup LPSWAVEFREQ No description provided (LPSWAVEFREQ) Register
 *  No description provided (LPSWAVEFREQ) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSWAVEFREQ_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_LPS_REGS_YODA_LPSWAVEFREQ_t {
    union {
        struct {
             unsigned int LPS_WAVE_FREQ    : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSWAVEFREQ_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup LPSWAVEGENACC No description provided (LPSWAVEGENACC) Register
 *  No description provided (LPSWAVEGENACC) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSWAVEGENACC_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_LPS_REGS_YODA_LPSWAVEGENACC_t {
    union {
        struct {
             unsigned int LPS_WG_ACC       : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSWAVEGENACC_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup LPSRAMADDR No description provided (LPSRAMADDR) Register
 *  No description provided (LPSRAMADDR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSRAMADDR_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_LPS_REGS_YODA_LPSRAMADDR_t {
    union {
        struct {
             unsigned int LPS_RAM_ADDR     : 9; /**< No description provided */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSRAMADDR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup LPSRAMRDCMD No description provided (LPSRAMRDCMD) Register
 *  No description provided (LPSRAMRDCMD) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSRAMRDCMD_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_LPS_REGS_YODA_LPSRAMRDCMD_t {
    union {
        struct {
             unsigned int LPS_RAM_READ_EN  : 1; /**< No description provided */
             unsigned int LPS_RAM_READ_RDY : 1; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSRAMRDCMD_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup LPSWAVEGENADDR No description provided (LPSWAVEGENADDR) Register
 *  No description provided (LPSWAVEGENADDR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSWAVEGENADDR_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_LPS_REGS_YODA_LPSWAVEGENADDR_t {
    union {
        struct {
             unsigned int LPS_RAM_START    : 8; /**< No description provided */
             unsigned int LPS_RAM_END      : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSWAVEGENADDR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup LPSMARGIN No description provided (LPSMARGIN) Register
 *  No description provided (LPSMARGIN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSMARGIN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_LPS_REGS_YODA_LPSMARGIN_t {
    union {
        struct {
             unsigned int LPS_PORT0_MARGIN : 1; /**< No description provided */
             unsigned int LPS_PORT1_MARGIN : 1; /**< No description provided */
             unsigned int RESERVED2        : 13; /**< Reserved */
             unsigned int LPS_PARITY_ERR   : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSMARGIN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup LPSDBG No description provided (LPSDBG) Register
 *  No description provided (LPSDBG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSDBG_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_LPS_REGS_YODA_LPSDBG_t {
    union {
        struct {
             unsigned int LPSDBGSEL        : 3; /**< No description provided */
             unsigned int RESERVED3        : 5; /**< Reserved */
             unsigned int LPSDBGCOM        : 2; /**< No description provided */
             unsigned int RESERVED10       : 5; /**< Reserved */
             unsigned int LPSDBGEN         : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSDBG_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup LPSRAMDATA No description provided (LPSRAMDATA) Register
 *  No description provided (LPSRAMDATA) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSRAMDATA_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_LPS_REGS_YODA_LPSRAMDATA_t {
    union {
        struct {
             unsigned int LPS_RAM_DATA     : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSRAMDATA_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup LPSRAMDATA_ALIAS No description provided (LPSRAMDATA_ALIAS) Register
 *  No description provided (LPSRAMDATA_ALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSRAMDATA_ALIAS_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_LPS_REGS_YODA_LPSRAMDATA_ALIAS_t {
    union {
        struct {
             unsigned int LPS_RAM_DATA_ALIAS : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSRAMDATA_ALIAS_t;
#ifdef SWIG
%}
#endif

/*@}*/

#endif  /* end ifndef ADI_LPS_REGS_YODA_TYPEDEFS_H_ */

#ifndef ADI_SS_REGS_TYPEDEFS_H_
#define ADI_SS_REGS_TYPEDEFS_H_

/** @defgroup SSWAVEFREQ No description provided (SSWAVEFREQ) Register
 *  No description provided (SSWAVEFREQ) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSWAVEFREQ_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSWAVEFREQ_t {
    union {
        struct {
             unsigned int WAVE_FREQ        : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSWAVEFREQ_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSWAVEPERIOD No description provided (SSWAVEPERIOD) Register
 *  No description provided (SSWAVEPERIOD) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSWAVEPERIOD_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSWAVEPERIOD_t {
    union {
        struct {
             unsigned int WAVE_PERIOD      : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSWAVEPERIOD_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSWAVEAMP No description provided (SSWAVEAMP) Register
 *  No description provided (SSWAVEAMP) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSWAVEAMP_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSWAVEAMP_t {
    union {
        struct {
             unsigned int WAVE_AMPLITUDE   : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSWAVEAMP_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSPINLO No description provided (SSPINLO) Register
 *  No description provided (SSPINLO) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSPINLO_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSPINLO_t {
    union {
        struct {
             unsigned int PI_N_LSBS        : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSPINLO_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSPINHI No description provided (SSPINHI) Register
 *  No description provided (SSPINHI) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSPINHI_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSPINHI_t {
    union {
        struct {
             unsigned int PI_N_MSBS        : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSPINHI_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSINTERVLO No description provided (SSINTERVLO) Register
 *  No description provided (SSINTERVLO) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSINTERVLO_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSINTERVLO_t {
    union {
        struct {
             unsigned int INTERVAL_LSBS    : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSINTERVLO_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSINTERVHI No description provided (SSINTERVHI) Register
 *  No description provided (SSINTERVHI) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSINTERVHI_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSINTERVHI_t {
    union {
        struct {
             unsigned int INTERVAL_MSBS    : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSINTERVHI_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSWAVEOFFSET No description provided (SSWAVEOFFSET) Register
 *  No description provided (SSWAVEOFFSET) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSWAVEOFFSET_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSWAVEOFFSET_t {
    union {
        struct {
             unsigned int WAVE_OFFSET      : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSWAVEOFFSET_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSCTRL No description provided (SSCTRL) Register
 *  No description provided (SSCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSCTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSCTRL_t {
    union {
        struct {
             unsigned int SS_ENA           : 1; /**< No description provided */
             unsigned int WTYPE            : 2; /**< No description provided */
             unsigned int RESERVED3        : 13; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSCTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSMODFREQ No description provided (SSMODFREQ) Register
 *  No description provided (SSMODFREQ) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSMODFREQ_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSMODFREQ_t {
    union {
        struct {
             unsigned int MOD_FREQ         : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSMODFREQ_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSMODAMP No description provided (SSMODAMP) Register
 *  No description provided (SSMODAMP) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSMODAMP_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSMODAMP_t {
    union {
        struct {
             unsigned int MOD_AMP          : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSMODAMP_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSINTERV_10 No description provided (SSINTERV_10) Register
 *  No description provided (SSINTERV_10) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSINTERV_10_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSINTERV_10_t {
    union {
        struct {
             unsigned int INTERV_1_0       : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSINTERV_10_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSINTERV_11 No description provided (SSINTERV_11) Register
 *  No description provided (SSINTERV_11) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSINTERV_11_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSINTERV_11_t {
    union {
        struct {
             unsigned int INTERV_1_1       : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSINTERV_11_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSINTERV_20 No description provided (SSINTERV_20) Register
 *  No description provided (SSINTERV_20) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSINTERV_20_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSINTERV_20_t {
    union {
        struct {
             unsigned int INTERV_2_0       : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSINTERV_20_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSINTERV_21 No description provided (SSINTERV_21) Register
 *  No description provided (SSINTERV_21) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSINTERV_21_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSINTERV_21_t {
    union {
        struct {
             unsigned int INTERV_2_1       : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSINTERV_21_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSINTERV_30 No description provided (SSINTERV_30) Register
 *  No description provided (SSINTERV_30) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSINTERV_30_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSINTERV_30_t {
    union {
        struct {
             unsigned int INTERV_3_0       : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSINTERV_30_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSINTERV_31 No description provided (SSINTERV_31) Register
 *  No description provided (SSINTERV_31) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSINTERV_31_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSINTERV_31_t {
    union {
        struct {
             unsigned int INTERV_3_1       : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSINTERV_31_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSVALUE_00 No description provided (SSVALUE_00) Register
 *  No description provided (SSVALUE_00) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSVALUE_00_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSVALUE_00_t {
    union {
        struct {
             unsigned int VALUE_0_0        : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSVALUE_00_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSVALUE_01 No description provided (SSVALUE_01) Register
 *  No description provided (SSVALUE_01) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSVALUE_01_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSVALUE_01_t {
    union {
        struct {
             unsigned int VALUE_0_1        : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSVALUE_01_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSVALUE_10 No description provided (SSVALUE_10) Register
 *  No description provided (SSVALUE_10) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSVALUE_10_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSVALUE_10_t {
    union {
        struct {
             unsigned int VALUE_1_0        : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSVALUE_10_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSVALUE_11 No description provided (SSVALUE_11) Register
 *  No description provided (SSVALUE_11) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSVALUE_11_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSVALUE_11_t {
    union {
        struct {
             unsigned int VALUE_1_1        : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSVALUE_11_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSVALUE_20 No description provided (SSVALUE_20) Register
 *  No description provided (SSVALUE_20) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSVALUE_20_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSVALUE_20_t {
    union {
        struct {
             unsigned int VALUE_2_0        : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSVALUE_20_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSVALUE_21 No description provided (SSVALUE_21) Register
 *  No description provided (SSVALUE_21) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSVALUE_21_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSVALUE_21_t {
    union {
        struct {
             unsigned int VALUE_2_1        : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSVALUE_21_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSVALUE_30 No description provided (SSVALUE_30) Register
 *  No description provided (SSVALUE_30) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSVALUE_30_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSVALUE_30_t {
    union {
        struct {
             unsigned int VALUE_3_0        : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSVALUE_30_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSVALUE_31 No description provided (SSVALUE_31) Register
 *  No description provided (SSVALUE_31) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSVALUE_31_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSVALUE_31_t {
    union {
        struct {
             unsigned int VALUE_3_1        : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSVALUE_31_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SSDBG No description provided (SSDBG) Register
 *  No description provided (SSDBG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_SS_REGS_SSDBG_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_SS_REGS_SSDBG_t {
    union {
        struct {
             unsigned int SSDBGSEL         : 2; /**< No description provided */
             unsigned int RESERVED2        : 6; /**< Reserved */
             unsigned int SSDBGCOM         : 2; /**< No description provided */
             unsigned int RESERVED10       : 5; /**< Reserved */
             unsigned int SSDBGEN          : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_SS_REGS_SSDBG_t;
#ifdef SWIG
%}
#endif

/*@}*/

#endif  /* end ifndef ADI_SS_REGS_TYPEDEFS_H_ */

#ifndef ADI_PCM_REGS_YODA_TYPEDEFS_H_
#define ADI_PCM_REGS_YODA_TYPEDEFS_H_

/** @defgroup PCMCTRL_0 No description provided (PCMCTRL_0) Register
 *  No description provided (PCMCTRL_0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_PCM_REGS_YODA_PCMCTRL_0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_PCM_REGS_YODA_PCMCTRL_0_t {
    union {
        struct {
             unsigned int PCM_START        : 1; /**< No description provided */
             unsigned int PCM_SELECT       : 5; /**< No description provided */
             unsigned int PCM_ENABLE_ALL   : 1; /**< No description provided */
             unsigned int RESERVED7        : 9; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_PCM_REGS_YODA_PCMCTRL_0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PCMCTRL_1 No description provided (PCMCTRL_1) Register
 *  No description provided (PCMCTRL_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_PCM_REGS_YODA_PCMCTRL_1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_PCM_REGS_YODA_PCMCTRL_1_t {
    union {
        struct {
             unsigned int PCM_CLK_COUNT    : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_PCM_REGS_YODA_PCMCTRL_1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PCMOUT No description provided (PCMOUT) Register
 *  No description provided (PCMOUT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_PCM_REGS_YODA_PCMOUT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_PCM_REGS_YODA_PCMOUT_t {
    union {
        struct {
             unsigned int PCM_OUT          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_PCM_REGS_YODA_PCMOUT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup OSC_PERIOD_CTRL No description provided (OSC_PERIOD_CTRL) Register
 *  No description provided (OSC_PERIOD_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_PCM_REGS_YODA_OSC_PERIOD_CTRL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_PCM_REGS_YODA_OSC_PERIOD_CTRL_t {
    union {
        struct {
             unsigned int OSC_PERIOD_ENABLE : 1; /**< No description provided */
             unsigned int RESERVED1        : 3; /**< No description provided */
             unsigned int OSC_PERIOD_DIVCLK_DIVIDE : 4; /**< No description provided */
             unsigned int SMPL_CLK_SEL     : 3; /**< No description provided */
             unsigned int RESERVED11       : 5; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_PCM_REGS_YODA_OSC_PERIOD_CTRL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup OSC_PERIOD_RD No description provided (OSC_PERIOD_RD) Register
 *  No description provided (OSC_PERIOD_RD) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_PCM_REGS_YODA_OSC_PERIOD_RD_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_PCM_REGS_YODA_OSC_PERIOD_RD_t {
    union {
        struct {
             unsigned int SMPL_PERIOD      : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_PCM_REGS_YODA_OSC_PERIOD_RD_t;
#ifdef SWIG
%}
#endif

/*@}*/

#endif  /* end ifndef ADI_PCM_REGS_YODA_TYPEDEFS_H_ */

#ifndef ADI_DATAPATH_TYPEDEFS_H_
#define ADI_DATAPATH_TYPEDEFS_H_

/** @defgroup CORRECTION_CONFIG No description provided (CORRECTION_CONFIG) Register
 *  No description provided (CORRECTION_CONFIG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_CORRECTION_CONFIG_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_CORRECTION_CONFIG_t {
    union {
        struct {
             unsigned int BYPASS_PC_GAIN   : 1; /**< Bypass per-column gain correction */
             unsigned int BYPASS_PC_OFFSET : 1; /**< Bypass per-column offset correction */
             unsigned int BYPASS_SCALE     : 1; /**< Bypass gain scaling */
             unsigned int BYPASS_SATTAG    : 1; /**< Bypass saturation tag while pixel adjustment */
             unsigned int BINNING_AVG_EN   : 1; /**< 1'b1 = average two rows while digital binning (use floor output); 1'b0 = add two rows while digital binning */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_CORRECTION_CONFIG_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_FRAME_CONFIG No description provided (USE_CASE_FRAME_CONFIG) Register
 *  No description provided (USE_CASE_FRAME_CONFIG) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_DATAPATH_USE_CASE_FRAME_CONFIG_ALTERNATE_AMP_MUX_POL
 *  \brief 1'b1 = Inverted gain scaling for even ADCs (needed for supply rejection work-around); 1'b0 = No inversion
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     DATAPATH_USE_CASE_FRAME_CONFIG_ALTERNATE_AMP_MUX_POL_NO_INVERT = 0,    /**< No description provided */
     DATAPATH_USE_CASE_FRAME_CONFIG_ALTERNATE_AMP_MUX_POL_INVERT_EVEN = 1     /**< No description provided */
}  ADI_DATAPATH_USE_CASE_FRAME_CONFIG_ALTERNATE_AMP_MUX_POL;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_DATAPATH_USE_CASE_FRAME_CONFIG_RAW_MODE
 *  \brief Enable RAW mode and select how the ADC clipping is handled
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     DATAPATH_USE_CASE_FRAME_CONFIG_RAW_MODE_DISABLE = 0,    /**< Raw mode disabled */
     DATAPATH_USE_CASE_FRAME_CONFIG_RAW_MODE_NO_CLIP = 1,    /**< RAW mode ADC data direct */
     DATAPATH_USE_CASE_FRAME_CONFIG_RAW_MODE_CLIP_ADC_OVR = 2,    /**< RAW mode ADC data clipped when overflow flag high */
     DATAPATH_USE_CASE_FRAME_CONFIG_RAW_MODE_CLIP_ALL = 3     /**< RAW mode ADC data clipped when overflow flag or sat_tag high */
}  ADI_DATAPATH_USE_CASE_FRAME_CONFIG_RAW_MODE;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \enum ADI_DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH
 *  \brief Bit width of final output to MIPI
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef enum
{
     DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH_RAW8 = 0,    /**< 8 bit Floating point: {sign,exp[2:0],mantissa[3:0]} or raw data */
     DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH_RAW10 = 1,    /**< 10 bit Floating point: {sign,exp[2:0],mantissa[5:0]} or raw data */
     DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH_RAW12 = 2,    /**< 12 bit Floating point: {sign,exp[2:0],mantissa[7:0]} or raw data */
     DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH_RAW16 = 3,    /**< 16 bit binary spread over two 8 bit mipi packets */
     DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH_RAW14 = 4     /**< 14 bit Binary, spread over two 8 bit MIPI packets */
}  ADI_DATAPATH_USE_CASE_FRAME_CONFIG_OUTPUT_WIDTH;
#ifdef SWIG
%}
#endif

/*! ========================================================================
 *  \struct ADI_DATAPATH_USE_CASE_FRAME_CONFIG_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_USE_CASE_FRAME_CONFIG_t {
    union {
        struct {
             unsigned int ADC_9B           : 1; /**< When 1, ADC valid data width is 9 bits; when 0, ADC width is 10 bits */
             unsigned int ALTERNATE_AMP_MUX_POL : 1; /**< 1'b1 = Inverted gain scaling for even ADCs (needed for supply rejection work-around); 1'b0 = No inversion */
             unsigned int DIGITAL_BIN_EN   : 1; /**< Enable digital binning between adjacent rows, invalid to set if delta_comp_en is set */
             unsigned int DELTA_COMP_EN    : 1; /**< Enable delta compression */
             unsigned int FIX2FLT_EN       : 1; /**< Enable fixed to floating point conversion. When raw_mode = 2'b00; this can be disabled only when output width is 16, to get gain corrected data without fixed to float conversion */
             unsigned int RAW_MODE         : 2; /**< Enable RAW mode and select how the ADC clipping is handled */
             unsigned int OUTPUT_WIDTH     : 3; /**< Bit width of final output to MIPI */
             unsigned int DARK_ROW_VEC     : 2; /**< Each bit enables the readout of  8 dark rows on the top and the bottom of the array */
             unsigned int MIPI_OUT_8BIT    : 1; /**< 1'b1 = Send RAW14 or RAW16 as two 8 bit RAW pixels to MIPI; when 1'b0 = Send the pixel data as 14 bit or 16 bit value */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_USE_CASE_FRAME_CONFIG_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_MIPI_PACKET_CONTROL No description provided (USE_CASE_MIPI_PACKET_CONTROL) Register
 *  No description provided (USE_CASE_MIPI_PACKET_CONTROL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_t {
    union {
        struct {
             unsigned int ROWS_PER_PACKET  : 7; /**< Number of rows to be packed in a MIPI  packet. Valid values are 1,2,4,8,16,32,64 */
             unsigned int AUTO_ROWS_PER_PACKET_EN : 1; /**< When 1, datapath calculates the rows/dumps to be packed in a MIPI packet; when 0 use the value programmed in rows per packet register */
             unsigned int MIPI_BUFF_RD_LIMIT : 2; /**< Number of buffers to be filled before sending packet request. Refer to programming guide to program this register */
             unsigned int MIPI_BUFF_RD_CTRL_EN : 1; /**< When 1, reading of MIPI buffers is based on mipi_buff_rd_limit, else done in datapath. */
             unsigned int RESERVED11       : 5; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GAIN0 No description provided (GAIN0) Register
 *  No description provided (GAIN0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_GAIN0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_GAIN0_t {
    union {
        struct {
             unsigned int GLOBAL_GAIN_SCALE_P0 : 10; /**< Gain scale for GT0 */
             unsigned int GLOBAL_GAIN_SHIFT_P0 : 3; /**< Gain shift for GT0 */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_GAIN0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GAIN1 No description provided (GAIN1) Register
 *  No description provided (GAIN1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_GAIN1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_GAIN1_t {
    union {
        struct {
             unsigned int GLOBAL_GAIN_SCALE_P1 : 10; /**< Gain scale for GT1 */
             unsigned int GLOBAL_GAIN_SHIFT_P1 : 3; /**< Gain shift for GT1 */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_GAIN1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GAIN2 No description provided (GAIN2) Register
 *  No description provided (GAIN2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_GAIN2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_GAIN2_t {
    union {
        struct {
             unsigned int GLOBAL_GAIN_SCALE_P2 : 10; /**< Gain scale for GT2 */
             unsigned int GLOBAL_GAIN_SHIFT_P2 : 3; /**< Gain shift for GT2 */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_GAIN2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GAIN3 No description provided (GAIN3) Register
 *  No description provided (GAIN3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_GAIN3_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_GAIN3_t {
    union {
        struct {
             unsigned int GLOBAL_GAIN_SCALE_P3 : 10; /**< Gain scale for GT3 */
             unsigned int GLOBAL_GAIN_SHIFT_P3 : 3; /**< Gain shift for GT3 */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_GAIN3_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PP_LFSR No description provided (PP_LFSR) Register
 *  No description provided (PP_LFSR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_LFSR_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_PP_LFSR_t {
    union {
        struct {
             unsigned int LFSR_EN          : 1; /**< Indicates whether the Pixel Packer must work in LFSR mode */
             unsigned int LFSR_MODE        : 2; /**< Indicates the LFSR data generation mode */
             unsigned int LFSR_SEED        : 12; /**< The initial seed value for the data generation in LFSR mode */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_PP_LFSR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PP_DECODE_ST_1 No description provided (PP_DECODE_ST_1) Register
 *  No description provided (PP_DECODE_ST_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_DECODE_ST_1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_PP_DECODE_ST_1_t {
    union {
        struct {
             unsigned int ST_DECODE_000    : 3; /**< Mux input for the ST selection */
             unsigned int ST_DECODE_001    : 3; /**< Mux input for the ST selection */
             unsigned int ST_DECODE_010    : 3; /**< Mux input for the ST selection */
             unsigned int ST_DECODE_011    : 3; /**< Mux input for the ST selection */
             unsigned int ST_DECODE_100    : 3; /**< Mux input for the ST selection */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_PP_DECODE_ST_1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PP_DECODE_ST_2 No description provided (PP_DECODE_ST_2) Register
 *  No description provided (PP_DECODE_ST_2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_DECODE_ST_2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_PP_DECODE_ST_2_t {
    union {
        struct {
             unsigned int ST_DECODE_101    : 3; /**< Mux input for the ST selection */
             unsigned int ST_DECODE_110    : 3; /**< Mux input for the ST selection */
             unsigned int ST_DECODE_111    : 3; /**< Mux input for the ST selection */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_PP_DECODE_ST_2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PP_ENCODE_ST No description provided (PP_ENCODE_ST) Register
 *  No description provided (PP_ENCODE_ST) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_ENCODE_ST_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_PP_ENCODE_ST_t {
    union {
        struct {
             unsigned int ST_EN_0          : 1; /**< Remap value for ST value of 0 from analog */
             unsigned int ST_EN_1          : 1; /**< Remap value for ST value of 1 from analog */
             unsigned int OVERFLOW_ACTIVE  : 1; /**< When 0, inactive and ignored for adc mux */
             unsigned int RESERVED3        : 1; /**< Reserved */
             unsigned int BINNED_AND_OR    : 1; /**< Indicates whether the ST tags are to be ORed together */
             unsigned int ST_LATCH_ACTIVE_HI : 1; /**< Indicates whether the st latch is active HIGH */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_PP_ENCODE_ST_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PP_ENCODE_GT No description provided (PP_ENCODE_GT) Register
 *  No description provided (PP_ENCODE_GT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_ENCODE_GT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_PP_ENCODE_GT_t {
    union {
        struct {
             unsigned int GT_EN_00         : 2; /**< Remap value for GT value of 00 from analog */
             unsigned int GT_EN_01         : 2; /**< Remap value for GT value of 01 from analog */
             unsigned int GT_EN_10         : 2; /**< Remap value for GT value of 10 from analog */
             unsigned int GT_EN_11         : 2; /**< Remap value for GT value of 11 from analog */
             unsigned int GT_LATCH_ACTIVE_HI : 1; /**< Indicates whether the gt latch is active HIGH */
             unsigned int TAG_WAIT         : 2; /**< Wait time between value change on gt_sel_o signal in number of clock cycles (2-20 clks) */
             unsigned int TAG_SCALE        : 2; /**< Scaling factor for tag wait */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_PP_ENCODE_GT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DBG_MUX No description provided (DBG_MUX) Register
 *  No description provided (DBG_MUX) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_DBG_MUX_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_DBG_MUX_t {
    union {
        struct {
             unsigned int DBG_SEL          : 5; /**< Internal signal debug mux select signal. Refer to Datapath HRM for mux description */
             unsigned int RESERVED5        : 10; /**< Reserved */
             unsigned int DBG_EN           : 1; /**< Enable for the Debug mux */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_DBG_MUX_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup GAIN_MARGIN_CONTROL No description provided (GAIN_MARGIN_CONTROL) Register
 *  No description provided (GAIN_MARGIN_CONTROL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_GAIN_MARGIN_CONTROL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_GAIN_MARGIN_CONTROL_t {
    union {
        struct {
             unsigned int GAIN_MEM_MARGIN  : 16; /**< Read and write margin control for gain and offset correction memory. One bit for each gain memory. */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_GAIN_MARGIN_CONTROL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup LINE_MARGIN_CONTROL No description provided (LINE_MARGIN_CONTROL) Register
 *  No description provided (LINE_MARGIN_CONTROL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_LINE_MARGIN_CONTROL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_LINE_MARGIN_CONTROL_t {
    union {
        struct {
             unsigned int LINE_MEM_MARGIN  : 2; /**< Read and write margin control. One bit for each line memory. */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_LINE_MARGIN_CONTROL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ROI_ROW_START No description provided (ROI_ROW_START) Register
 *  No description provided (ROI_ROW_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ROI_ROW_START_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_ROI_ROW_START_t {
    union {
        struct {
             unsigned int START_ROW        : 10; /**< Start row of ROI. Should be a multiple of 64. Range (0-639) */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_ROI_ROW_START_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ROI_HEIGHT No description provided (ROI_HEIGHT) Register
 *  No description provided (ROI_HEIGHT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ROI_HEIGHT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_ROI_HEIGHT_t {
    union {
        struct {
             unsigned int ROI_HEIGHT       : 10; /**< Numbers of rows enabled in ROI */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_ROI_HEIGHT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ROI_COLUMN_START No description provided (ROI_COLUMN_START) Register
 *  No description provided (ROI_COLUMN_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ROI_COLUMN_START_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_ROI_COLUMN_START_t {
    union {
        struct {
             unsigned int START_COLUMN     : 9; /**< Start column of ROI. Should be a multiple of 16 */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_ROI_COLUMN_START_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ROI_WIDTH No description provided (ROI_WIDTH) Register
 *  No description provided (ROI_WIDTH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ROI_WIDTH_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_ROI_WIDTH_t {
    union {
        struct {
             unsigned int ROI_WIDTH        : 10; /**< Number of columns enabled in ROI */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_ROI_WIDTH_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PP_USEQ_WRITE No description provided (PP_USEQ_WRITE) Register
 *  No description provided (PP_USEQ_WRITE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_USEQ_WRITE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_PP_USEQ_WRITE_t {
    union {
        struct {
             unsigned int DUMP_START       : 1; /**< Bit to be written by micro-sequencer */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_PP_USEQ_WRITE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PP_ADC_DELAY No description provided (PP_ADC_DELAY) Register
 *  No description provided (PP_ADC_DELAY) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_ADC_DELAY_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_PP_ADC_DELAY_t {
    union {
        struct {
             unsigned int ADC_DELAY        : 6; /**< Specifies the number of clock cycles after which the data from adc_bus can read and used on adc_convert positive transition */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_PP_ADC_DELAY_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup MIPI_BUFF_MARGIN_CONTROL No description provided (MIPI_BUFF_MARGIN_CONTROL) Register
 *  No description provided (MIPI_BUFF_MARGIN_CONTROL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_MIPI_BUFF_MARGIN_CONTROL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_MIPI_BUFF_MARGIN_CONTROL_t {
    union {
        struct {
             unsigned int MIPI_BUFF_MARGIN : 4; /**< Read and write margin control for line memory. One bit for each MIPI buffer */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_MIPI_BUFF_MARGIN_CONTROL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup MIPI_HEADER_WIDTH No description provided (MIPI_HEADER_WIDTH) Register
 *  No description provided (MIPI_HEADER_WIDTH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_MIPI_HEADER_WIDTH_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_MIPI_HEADER_WIDTH_t {
    union {
        struct {
             unsigned int METADATA_BYTES   : 8; /**< Number of bytes sent as Metadata. Valid values are 0 or 128 */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_MIPI_HEADER_WIDTH_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup FRAME_NUMBER No description provided (FRAME_NUMBER) Register
 *  No description provided (FRAME_NUMBER) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_FRAME_NUMBER_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_FRAME_NUMBER_t {
    union {
        struct {
             unsigned int FRAME_COUNT      : 16; /**< Depth Frame counter to be incremented by micro-sequencer */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_FRAME_NUMBER_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup MICRO_SEQUENCER_FW_VERSION_LSB No description provided (MICRO_SEQUENCER_FW_VERSION_LSB) Register
 *  No description provided (MICRO_SEQUENCER_FW_VERSION_LSB) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_MICRO_SEQUENCER_FW_VERSION_LSB_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_MICRO_SEQUENCER_FW_VERSION_LSB_t {
    union {
        struct {
             unsigned int USEQ_FW_VERSION_LSB : 16; /**< LSB bits of micro-sequencer FW version */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_MICRO_SEQUENCER_FW_VERSION_LSB_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup MICRO_SEQUENCER_FW_VERSION_MSB No description provided (MICRO_SEQUENCER_FW_VERSION_MSB) Register
 *  No description provided (MICRO_SEQUENCER_FW_VERSION_MSB) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_MICRO_SEQUENCER_FW_VERSION_MSB_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_MICRO_SEQUENCER_FW_VERSION_MSB_t {
    union {
        struct {
             unsigned int USEQ_FW_VERSION_MSB : 16; /**< MSB bits of micro-sequencer FW version */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_MICRO_SEQUENCER_FW_VERSION_MSB_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup TS_CAL_VER No description provided (TS_CAL_VER) Register
 *  No description provided (TS_CAL_VER) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_TS_CAL_VER_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_TS_CAL_VER_t {
    union {
        struct {
             unsigned int TS_CAL_VER       : 16; /**< Temperature sensor calibration version */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_TS_CAL_VER_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ADC_CAL_VER No description provided (ADC_CAL_VER) Register
 *  No description provided (ADC_CAL_VER) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ADC_CAL_VER_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_ADC_CAL_VER_t {
    union {
        struct {
             unsigned int ADC_CAL_VER      : 16; /**< ADC and gain calibration version */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_ADC_CAL_VER_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup REG_0 No description provided (REG_0) Register
 *  No description provided (REG_0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_REG_0_t {
    union {
        struct {
             unsigned int REG_0            : 16; /**< Reserved for MIPI header */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_REG_0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup REG_1 No description provided (REG_1) Register
 *  No description provided (REG_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_REG_1_t {
    union {
        struct {
             unsigned int REG_1            : 16; /**< Reserved for MIPI header */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_REG_1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup REG_2 No description provided (REG_2) Register
 *  No description provided (REG_2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_REG_2_t {
    union {
        struct {
             unsigned int REG_2            : 16; /**< Reserved for MIPI header */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_REG_2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup REG_3 No description provided (REG_3) Register
 *  No description provided (REG_3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_3_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_REG_3_t {
    union {
        struct {
             unsigned int REG_3            : 16; /**< Reserved for MIPI header */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_REG_3_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup REG_4 No description provided (REG_4) Register
 *  No description provided (REG_4) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_4_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_REG_4_t {
    union {
        struct {
             unsigned int REG_4            : 16; /**< Reserved for MIPI header */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_REG_4_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup REG_5 No description provided (REG_5) Register
 *  No description provided (REG_5) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_5_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_REG_5_t {
    union {
        struct {
             unsigned int REG_5            : 16; /**< Reserved for MIPI header */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_REG_5_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup REG_6 No description provided (REG_6) Register
 *  No description provided (REG_6) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_6_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_REG_6_t {
    union {
        struct {
             unsigned int REG_6            : 16; /**< Reserved for MIPI header */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_REG_6_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup REG_7 No description provided (REG_7) Register
 *  No description provided (REG_7) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_7_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_REG_7_t {
    union {
        struct {
             unsigned int REG_7            : 16; /**< Reserved for MIPI header */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_REG_7_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PACKET_COUNT No description provided (PACKET_COUNT) Register
 *  No description provided (PACKET_COUNT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PACKET_COUNT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_PACKET_COUNT_t {
    union {
        struct {
             unsigned int PACKET_COUNT     : 10; /**< Indicates the packet number being sent to MIPI. Will be continously updated when a packet is sent to MIPI */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_PACKET_COUNT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PACKETS_PER_FRAME No description provided (PACKETS_PER_FRAME) Register
 *  No description provided (PACKETS_PER_FRAME) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PACKETS_PER_FRAME_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_PACKETS_PER_FRAME_t {
    union {
        struct {
             unsigned int TOTAL_PACKETS_PER_FRAME : 10; /**< Indicates the total number of data packets in a MIPI frame */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_PACKETS_PER_FRAME_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ROW_VECTOR No description provided (ROW_VECTOR) Register
 *  No description provided (ROW_VECTOR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ROW_VECTOR_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_ROW_VECTOR_t {
    union {
        struct {
             unsigned int ROW_VECTOR       : 10; /**< Each bit is used to indicate that a group of 64 rows of pixel array are enabled. LSB bit used to indicate rows 0-63 */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_ROW_VECTOR_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ROWS_PER_PACKET_OUT No description provided (ROWS_PER_PACKET_OUT) Register
 *  No description provided (ROWS_PER_PACKET_OUT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ROWS_PER_PACKET_OUT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_ROWS_PER_PACKET_OUT_t {
    union {
        struct {
             unsigned int ROWS_PER_PACKET_OUT : 7; /**< When auto_rows_per_packet_en = 1; readback this value calculated by the datapath. Updated on dump start */
             unsigned int RESERVED7        : 9; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_ROWS_PER_PACKET_OUT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup MIPI_RD_EN_MAX No description provided (MIPI_RD_EN_MAX) Register
 *  No description provided (MIPI_RD_EN_MAX) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_MIPI_RD_EN_MAX_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_MIPI_RD_EN_MAX_t {
    union {
        struct {
             unsigned int MIPI_BUFF_READ_ENABLE_COUNT_MAX : 9; /**< Indicates the number of reads done per mipi buffer. This value will be equal to (number of pixels per dump written to the MIPI buffer)/4 */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_MIPI_RD_EN_MAX_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ANALOG_SS No description provided (ANALOG_SS) Register
 *  No description provided (ANALOG_SS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ANALOG_SS_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_ANALOG_SS_t {
    union {
        struct {
             unsigned int ANALOG_SS        : 4; /**< Indicates the sub-sampling factor. Input from DE */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_ANALOG_SS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup IA_SELECT No description provided (IA_SELECT) Register
 *  No description provided (IA_SELECT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_SELECT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_IA_SELECT_t {
    union {
        struct {
             unsigned int IA_ENA           : 1; /**< Indirect access enable for column correction memory */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_IA_SELECT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup IA_ADDR_REG No description provided (IA_ADDR_REG) Register
 *  No description provided (IA_ADDR_REG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_ADDR_REG_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_IA_ADDR_REG_t {
    union {
        struct {
             unsigned int IA_START_ADDR    : 12; /**< Indirect access start address */
             unsigned int RESERVED12       : 4; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_IA_ADDR_REG_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup IA_WRDATA_REG No description provided (IA_WRDATA_REG) Register
 *  No description provided (IA_WRDATA_REG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_WRDATA_REG_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_IA_WRDATA_REG_t {
    union {
        struct {
             unsigned int IA_WRDATA        : 16; /**< Indirect access write data */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_IA_WRDATA_REG_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup IA_WRDATA_REG_ALIAS No description provided (IA_WRDATA_REG_ALIAS) Register
 *  No description provided (IA_WRDATA_REG_ALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_WRDATA_REG_ALIAS_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_IA_WRDATA_REG_ALIAS_t {
    union {
        struct {
             unsigned int IA_WRDATA_ALIAS  : 16; /**< Indirect access write data (alias for ia_wrdata) */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_IA_WRDATA_REG_ALIAS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup IA_RDDATA_REG No description provided (IA_RDDATA_REG) Register
 *  No description provided (IA_RDDATA_REG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_RDDATA_REG_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_IA_RDDATA_REG_t {
    union {
        struct {
             unsigned int IA_RDDATA        : 16; /**< Indirect access read data */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_IA_RDDATA_REG_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup IA_RDDATA_REG_ALIAS No description provided (IA_RDDATA_REG_ALIAS) Register
 *  No description provided (IA_RDDATA_REG_ALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_RDDATA_REG_ALIAS_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_IA_RDDATA_REG_ALIAS_t {
    union {
        struct {
             unsigned int IA_RDDATA_ALIAS  : 16; /**< Indirect access read data (alias for ia_rdata) */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_IA_RDDATA_REG_ALIAS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup IA_BANK_TYPE No description provided (IA_BANK_TYPE) Register
 *  No description provided (IA_BANK_TYPE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_BANK_TYPE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_IA_BANK_TYPE_t {
    union {
        struct {
             unsigned int IA_BANK_TYPE     : 1; /**< 1'b1 = LSB bits of address used to select bank in reg mem access module. 1'b0 = have another input which specifies the bank */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_IA_BANK_TYPE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PARITY_COUNT No description provided (PARITY_COUNT) Register
 *  No description provided (PARITY_COUNT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PARITY_COUNT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_PARITY_COUNT_t {
    union {
        struct {
             unsigned int GAIN_MEM_PARITY_ERR_COUNT : 6; /**< Indicates the parity error count of gain correction memory */
             unsigned int LINE_MEM_PARITY_ERR_COUNT : 5; /**< Indicates the parity error count of line memory */
             unsigned int MIPI_BUFF_PARITY_ERR_COUNT : 5; /**< Indicates parity error count of MIPI buffer memories */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_PARITY_COUNT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup INTERRUPT No description provided (INTERRUPT) Register
 *  No description provided (INTERRUPT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_INTERRUPT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_INTERRUPT_t {
    union {
        struct {
             unsigned int MIPI_CMD_ERR_INTR : 1; /**< Command error received from MIPI. Reset only if datapath is reset */
             unsigned int TIMING_ISSUE_INTR_PP : 1; /**< timing error reported by pixel packer. reset when datapath is reset */
             unsigned int TIMING_ISSUE_INTR_DP : 1; /**< Timing error interrupt seen by the datapath. Reset when datapath is reset */
             unsigned int RESERVED3        : 13; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_INTERRUPT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup MIPI_SKEW_CAL No description provided (MIPI_SKEW_CAL) Register
 *  No description provided (MIPI_SKEW_CAL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_MIPI_SKEW_CAL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_MIPI_SKEW_CAL_t {
    union {
        struct {
             unsigned int L_SKEW_CAL_I     : 1; /**< Initiate an initial skew calibration packet. Microsequencer will write to this register on powerup. This will be reset by the skew_cal_done input from MIPI */
             unsigned int L_SKEW_CAL_P     : 1; /**< Initiate a periodic skew calibration packet. */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_MIPI_SKEW_CAL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup MIPI_ESC_CLK_DIV No description provided (MIPI_ESC_CLK_DIV) Register
 *  No description provided (MIPI_ESC_CLK_DIV) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_MIPI_ESC_CLK_DIV_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DATAPATH_MIPI_ESC_CLK_DIV_t {
    union {
        struct {
             unsigned int ESC_CLK_BYTE_HALF_PERIOD : 4; /**< Sets period of MIPI escape clock */
             unsigned int ESC_CLK_SYS_HALF_PERIOD : 4; /**< Sets period of MIPI escape clock */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DATAPATH_MIPI_ESC_CLK_DIV_t;
#ifdef SWIG
%}
#endif

/*@}*/

#endif  /* end ifndef ADI_DATAPATH_TYPEDEFS_H_ */

#ifndef ADI_DE_REGS_YODA_TYPEDEFS_H_
#define ADI_DE_REGS_YODA_TYPEDEFS_H_

/** @defgroup DE_CONTROL No description provided (DE_CONTROL) Register
 *  No description provided (DE_CONTROL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_CONTROL_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DE_CONTROL_t {
    union {
        struct {
             unsigned int STATIC_CNTRL     : 1; /**< No description provided */
             unsigned int OVERIDE_CNTRL    : 1; /**< No description provided */
             unsigned int GO_BIT           : 1; /**< No description provided */
             unsigned int BUSY_BIT         : 1; /**< No description provided */
             unsigned int POSTAMBLE_BIT    : 1; /**< No description provided */
             unsigned int PREAMBLE_BIT     : 1; /**< No description provided */
             unsigned int MANUAL_MODE_BIT  : 1; /**< No description provided */
             unsigned int INIT_VEC_BIT     : 1; /**< No description provided */
             unsigned int DUMP_DIRECTION_BIT : 1; /**< No description provided */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DE_CONTROL_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup BINNED1X2_REPEAT_COUNT No description provided (BINNED1X2_REPEAT_COUNT) Register
 *  No description provided (BINNED1X2_REPEAT_COUNT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_BINNED1X2_REPEAT_COUNT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_BINNED1X2_REPEAT_COUNT_t {
    union {
        struct {
             unsigned int REPEAT_COUNT     : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_BINNED1X2_REPEAT_COUNT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup OVERRIDE_DATA_REG1 No description provided (OVERRIDE_DATA_REG1) Register
 *  No description provided (OVERRIDE_DATA_REG1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_OVERRIDE_DATA_REG1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_OVERRIDE_DATA_REG1_t {
    union {
        struct {
             unsigned int OVR_VAL          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_OVERRIDE_DATA_REG1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup OVERRIDE_DATA_REG2 No description provided (OVERRIDE_DATA_REG2) Register
 *  No description provided (OVERRIDE_DATA_REG2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_OVERRIDE_DATA_REG2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_OVERRIDE_DATA_REG2_t {
    union {
        struct {
             unsigned int OVR_VAL          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_OVERRIDE_DATA_REG2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup OVERRIDE_DATA_REG3 No description provided (OVERRIDE_DATA_REG3) Register
 *  No description provided (OVERRIDE_DATA_REG3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_OVERRIDE_DATA_REG3_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_OVERRIDE_DATA_REG3_t {
    union {
        struct {
             unsigned int OVR_VAL          : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_OVERRIDE_DATA_REG3_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup BINNED1X2_END No description provided (BINNED1X2_END) Register
 *  No description provided (BINNED1X2_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_BINNED1X2_END_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_BINNED1X2_END_t {
    union {
        struct {
             unsigned int END_ADDRESS      : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_BINNED1X2_END_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup OVERRIDE_SEL_REG1 No description provided (OVERRIDE_SEL_REG1) Register
 *  No description provided (OVERRIDE_SEL_REG1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_OVERRIDE_SEL_REG1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_OVERRIDE_SEL_REG1_t {
    union {
        struct {
             unsigned int OVR_VAL_SEL      : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_OVERRIDE_SEL_REG1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup OVERRIDE_SEL_REG2 No description provided (OVERRIDE_SEL_REG2) Register
 *  No description provided (OVERRIDE_SEL_REG2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_OVERRIDE_SEL_REG2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_OVERRIDE_SEL_REG2_t {
    union {
        struct {
             unsigned int OVR_VAL_SEL      : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_OVERRIDE_SEL_REG2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup OVERRIDE_SEL_REG3 No description provided (OVERRIDE_SEL_REG3) Register
 *  No description provided (OVERRIDE_SEL_REG3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_OVERRIDE_SEL_REG3_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_OVERRIDE_SEL_REG3_t {
    union {
        struct {
             unsigned int OVR_VAL_SEL      : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_OVERRIDE_SEL_REG3_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup BINNED1X2_START No description provided (BINNED1X2_START) Register
 *  No description provided (BINNED1X2_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_BINNED1X2_START_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_BINNED1X2_START_t {
    union {
        struct {
             unsigned int START_ADDRESS    : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_BINNED1X2_START_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMP_MUX_SEL_EE_LOW No description provided (AMP_MUX_SEL_EE_LOW) Register
 *  No description provided (AMP_MUX_SEL_EE_LOW) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_EE_LOW_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_EE_LOW_t {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_EE_LOW_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMP_MUX_SEL_EE_HIGH No description provided (AMP_MUX_SEL_EE_HIGH) Register
 *  No description provided (AMP_MUX_SEL_EE_HIGH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_EE_HIGH_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_EE_HIGH_t {
    union {
        struct {
             unsigned int BITS20_16        : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_EE_HIGH_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMP_MUX_SEL_EO_LOW No description provided (AMP_MUX_SEL_EO_LOW) Register
 *  No description provided (AMP_MUX_SEL_EO_LOW) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_EO_LOW_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_EO_LOW_t {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_EO_LOW_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMP_MUX_SEL_EO_HIGH No description provided (AMP_MUX_SEL_EO_HIGH) Register
 *  No description provided (AMP_MUX_SEL_EO_HIGH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_EO_HIGH_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_EO_HIGH_t {
    union {
        struct {
             unsigned int BITS20_16        : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_EO_HIGH_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMP_MUX_SEL_OE_LOW No description provided (AMP_MUX_SEL_OE_LOW) Register
 *  No description provided (AMP_MUX_SEL_OE_LOW) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_OE_LOW_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_OE_LOW_t {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_OE_LOW_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMP_MUX_SEL_OE_HIGH No description provided (AMP_MUX_SEL_OE_HIGH) Register
 *  No description provided (AMP_MUX_SEL_OE_HIGH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_OE_HIGH_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_OE_HIGH_t {
    union {
        struct {
             unsigned int BITS20_16        : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_OE_HIGH_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMP_MUX_SEL_OO_LOW No description provided (AMP_MUX_SEL_OO_LOW) Register
 *  No description provided (AMP_MUX_SEL_OO_LOW) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_OO_LOW_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_OO_LOW_t {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_OO_LOW_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMP_MUX_SEL_OO_HIGH No description provided (AMP_MUX_SEL_OO_HIGH) Register
 *  No description provided (AMP_MUX_SEL_OO_HIGH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_OO_HIGH_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_OO_HIGH_t {
    union {
        struct {
             unsigned int BITS20_16        : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_OO_HIGH_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMP_MUX_SEL_SELB_LOW No description provided (AMP_MUX_SEL_SELB_LOW) Register
 *  No description provided (AMP_MUX_SEL_SELB_LOW) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_SELB_LOW_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_SELB_LOW_t {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_SELB_LOW_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup AMP_MUX_SEL_SELB_HIGH No description provided (AMP_MUX_SEL_SELB_HIGH) Register
 *  No description provided (AMP_MUX_SEL_SELB_HIGH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_SELB_HIGH_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_SELB_HIGH_t {
    union {
        struct {
             unsigned int BITS20_16        : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_SELB_HIGH_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup NATIVE_RESOLUTION_START No description provided (NATIVE_RESOLUTION_START) Register
 *  No description provided (NATIVE_RESOLUTION_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_NATIVE_RESOLUTION_START_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_NATIVE_RESOLUTION_START_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_NATIVE_RESOLUTION_START_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup NATIVE_RESOLUTION_END No description provided (NATIVE_RESOLUTION_END) Register
 *  No description provided (NATIVE_RESOLUTION_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_NATIVE_RESOLUTION_END_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_NATIVE_RESOLUTION_END_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_NATIVE_RESOLUTION_END_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup NATIVE_RESOLUTION_REPEAT No description provided (NATIVE_RESOLUTION_REPEAT) Register
 *  No description provided (NATIVE_RESOLUTION_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_NATIVE_RESOLUTION_REPEAT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_NATIVE_RESOLUTION_REPEAT_t {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_NATIVE_RESOLUTION_REPEAT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SUB_SAMPLED_2X_START No description provided (SUB_SAMPLED_2X_START) Register
 *  No description provided (SUB_SAMPLED_2X_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_SUB_SAMPLED_2X_START_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_SUB_SAMPLED_2X_START_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_SUB_SAMPLED_2X_START_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SUB_SAMPLED_2X_END No description provided (SUB_SAMPLED_2X_END) Register
 *  No description provided (SUB_SAMPLED_2X_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_SUB_SAMPLED_2X_END_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_SUB_SAMPLED_2X_END_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_SUB_SAMPLED_2X_END_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SUB_SAMPLED_2X_REPEAT No description provided (SUB_SAMPLED_2X_REPEAT) Register
 *  No description provided (SUB_SAMPLED_2X_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_SUB_SAMPLED_2X_REPEAT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_SUB_SAMPLED_2X_REPEAT_t {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_SUB_SAMPLED_2X_REPEAT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SUB_SAMPLED_4X_START No description provided (SUB_SAMPLED_4X_START) Register
 *  No description provided (SUB_SAMPLED_4X_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_SUB_SAMPLED_4X_START_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_SUB_SAMPLED_4X_START_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_SUB_SAMPLED_4X_START_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SUB_SAMPLED_4X_END No description provided (SUB_SAMPLED_4X_END) Register
 *  No description provided (SUB_SAMPLED_4X_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_SUB_SAMPLED_4X_END_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_SUB_SAMPLED_4X_END_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_SUB_SAMPLED_4X_END_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup SUB_SAMPLED_4X_REPEAT No description provided (SUB_SAMPLED_4X_REPEAT) Register
 *  No description provided (SUB_SAMPLED_4X_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_SUB_SAMPLED_4X_REPEAT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_SUB_SAMPLED_4X_REPEAT_t {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_SUB_SAMPLED_4X_REPEAT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup BINNED_START No description provided (BINNED_START) Register
 *  No description provided (BINNED_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_BINNED_START_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_BINNED_START_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_BINNED_START_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup BINNED_END No description provided (BINNED_END) Register
 *  No description provided (BINNED_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_BINNED_END_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_BINNED_END_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_BINNED_END_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup BINNED_REPEAT No description provided (BINNED_REPEAT) Register
 *  No description provided (BINNED_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_BINNED_REPEAT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_BINNED_REPEAT_t {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_BINNED_REPEAT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DARK_START No description provided (DARK_START) Register
 *  No description provided (DARK_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DARK_START_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DARK_START_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DARK_START_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DARK_END No description provided (DARK_END) Register
 *  No description provided (DARK_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DARK_END_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DARK_END_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DARK_END_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DARK_REPEAT No description provided (DARK_REPEAT) Register
 *  No description provided (DARK_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DARK_REPEAT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DARK_REPEAT_t {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DARK_REPEAT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PREAMBLE_START No description provided (PREAMBLE_START) Register
 *  No description provided (PREAMBLE_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_PREAMBLE_START_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_PREAMBLE_START_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_PREAMBLE_START_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PREAMBLE_END No description provided (PREAMBLE_END) Register
 *  No description provided (PREAMBLE_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_PREAMBLE_END_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_PREAMBLE_END_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_PREAMBLE_END_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup PREAMBLE_REPEAT No description provided (PREAMBLE_REPEAT) Register
 *  No description provided (PREAMBLE_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_PREAMBLE_REPEAT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_PREAMBLE_REPEAT_t {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_PREAMBLE_REPEAT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup POSTAMBLE_START No description provided (POSTAMBLE_START) Register
 *  No description provided (POSTAMBLE_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_POSTAMBLE_START_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_POSTAMBLE_START_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_POSTAMBLE_START_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup POSTAMBLE_END No description provided (POSTAMBLE_END) Register
 *  No description provided (POSTAMBLE_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_POSTAMBLE_END_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_POSTAMBLE_END_t {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_POSTAMBLE_END_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup POSTAMBLE_REPEAT No description provided (POSTAMBLE_REPEAT) Register
 *  No description provided (POSTAMBLE_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_POSTAMBLE_REPEAT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_POSTAMBLE_REPEAT_t {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_POSTAMBLE_REPEAT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ARRAY_INIT_VEC_DARK No description provided (ARRAY_INIT_VEC_DARK) Register
 *  No description provided (ARRAY_INIT_VEC_DARK) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_ARRAY_INIT_VEC_DARK_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_ARRAY_INIT_VEC_DARK_t {
    union {
        struct {
             unsigned int DARK_BITS        : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_ARRAY_INIT_VEC_DARK_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup ARRAY_INIT_VEC No description provided (ARRAY_INIT_VEC) Register
 *  No description provided (ARRAY_INIT_VEC) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_ARRAY_INIT_VEC_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_ARRAY_INIT_VEC_t {
    union {
        struct {
             unsigned int ARRAY_BITS       : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_ARRAY_INIT_VEC_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup TYPE_OVERRIDE No description provided (TYPE_OVERRIDE) Register
 *  No description provided (TYPE_OVERRIDE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_TYPE_OVERRIDE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_TYPE_OVERRIDE_t {
    union {
        struct {
             unsigned int ROI_0            : 4; /**< No description provided */
             unsigned int ROI_1            : 4; /**< No description provided */
             unsigned int ROI_2            : 4; /**< No description provided */
             unsigned int ANALOG           : 4; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_TYPE_OVERRIDE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup MEM_DFT No description provided (MEM_DFT) Register
 *  No description provided (MEM_DFT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_MEM_DFT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_MEM_DFT_t {
    union {
        struct {
             unsigned int MARGIN           : 1; /**< No description provided */
             unsigned int RESERVED1        : 14; /**< Reserved */
             unsigned int PARITY_ERR       : 1; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_MEM_DFT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DBG_MUX_CONTROL_0 No description provided (DBG_MUX_CONTROL_0) Register
 *  No description provided (DBG_MUX_CONTROL_0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DBG_MUX_CONTROL_0_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DBG_MUX_CONTROL_0_t {
    union {
        struct {
             unsigned int CNTRL_0          : 8; /**< No description provided */
             unsigned int CNTRL_1          : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DBG_MUX_CONTROL_0_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DBG_MUX_CONTROL_1 No description provided (DBG_MUX_CONTROL_1) Register
 *  No description provided (DBG_MUX_CONTROL_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DBG_MUX_CONTROL_1_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DBG_MUX_CONTROL_1_t {
    union {
        struct {
             unsigned int CNTRL_2          : 8; /**< No description provided */
             unsigned int CNTRL_3          : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DBG_MUX_CONTROL_1_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DBG_MUX_CONTROL_2 No description provided (DBG_MUX_CONTROL_2) Register
 *  No description provided (DBG_MUX_CONTROL_2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DBG_MUX_CONTROL_2_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DBG_MUX_CONTROL_2_t {
    union {
        struct {
             unsigned int CNTRL_4          : 8; /**< No description provided */
             unsigned int CNTRL_5          : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DBG_MUX_CONTROL_2_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DBG_MUX_CONTROL_3 No description provided (DBG_MUX_CONTROL_3) Register
 *  No description provided (DBG_MUX_CONTROL_3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DBG_MUX_CONTROL_3_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DBG_MUX_CONTROL_3_t {
    union {
        struct {
             unsigned int CNTRL_6          : 8; /**< No description provided */
             unsigned int CNTRL_7          : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DBG_MUX_CONTROL_3_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DBG_MUX_CONTROL_4 No description provided (DBG_MUX_CONTROL_4) Register
 *  No description provided (DBG_MUX_CONTROL_4) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DBG_MUX_CONTROL_4_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DBG_MUX_CONTROL_4_t {
    union {
        struct {
             unsigned int CNTRL_8          : 8; /**< No description provided */
             unsigned int CNTRL_9          : 8; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DBG_MUX_CONTROL_4_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DE_IA_SELECT No description provided (DE_IA_SELECT) Register
 *  No description provided (DE_IA_SELECT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_IA_SELECT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DE_IA_SELECT_t {
    union {
        struct {
             unsigned int RAM              : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DE_IA_SELECT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DE_IA_ADDR_REG No description provided (DE_IA_ADDR_REG) Register
 *  No description provided (DE_IA_ADDR_REG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_IA_ADDR_REG_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DE_IA_ADDR_REG_t {
    union {
        struct {
             unsigned int RAM_ADDR         : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DE_IA_ADDR_REG_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DE_IA_WRDATA_REG No description provided (DE_IA_WRDATA_REG) Register
 *  No description provided (DE_IA_WRDATA_REG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_IA_WRDATA_REG_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DE_IA_WRDATA_REG_t {
    union {
        struct {
             unsigned int RAM_WRDATA       : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DE_IA_WRDATA_REG_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DE_IA_WRDATA_REG_ALIAS No description provided (DE_IA_WRDATA_REG_ALIAS) Register
 *  No description provided (DE_IA_WRDATA_REG_ALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_IA_WRDATA_REG_ALIAS_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DE_IA_WRDATA_REG_ALIAS_t {
    union {
        struct {
             unsigned int RAM_WRDATA_ALIAS : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DE_IA_WRDATA_REG_ALIAS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DE_IA_RDDATA_REG No description provided (DE_IA_RDDATA_REG) Register
 *  No description provided (DE_IA_RDDATA_REG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_IA_RDDATA_REG_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DE_IA_RDDATA_REG_t {
    union {
        struct {
             unsigned int RAM_RDDATA       : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DE_IA_RDDATA_REG_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup DE_IA_RDDATA_REG_ALIAS No description provided (DE_IA_RDDATA_REG_ALIAS) Register
 *  No description provided (DE_IA_RDDATA_REG_ALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_IA_RDDATA_REG_ALIAS_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_DE_IA_RDDATA_REG_ALIAS_t {
    union {
        struct {
             unsigned int RAM_RDDATA_ALIAS : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_DE_IA_RDDATA_REG_ALIAS_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_0_ROI_0_ROW_VEC_TOPBOT No description provided (USE_CASE_0_ROI_0_ROW_VEC_TOPBOT) Register
 *  No description provided (USE_CASE_0_ROI_0_ROW_VEC_TOPBOT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_TOPBOT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_TOPBOT_t {
    union {
        struct {
             unsigned int BITS17_AND_0     : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_TOPBOT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_0_ROI_0_ROW_VEC_MAIN No description provided (USE_CASE_0_ROI_0_ROW_VEC_MAIN) Register
 *  No description provided (USE_CASE_0_ROI_0_ROW_VEC_MAIN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_MAIN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_MAIN_t {
    union {
        struct {
             unsigned int BITS16_1         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_MAIN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_0_ROI_0_COLUMN_VEC No description provided (USE_CASE_0_ROI_0_COLUMN_VEC) Register
 *  No description provided (USE_CASE_0_ROI_0_COLUMN_VEC) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_COLUMN_VEC_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_COLUMN_VEC_t {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_COLUMN_VEC_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_0_ROI_0_ROI_TYPE No description provided (USE_CASE_0_ROI_0_ROI_TYPE) Register
 *  No description provided (USE_CASE_0_ROI_0_ROI_TYPE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROI_TYPE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROI_TYPE_t {
    union {
        struct {
             unsigned int BINSS            : 4; /**< No description provided */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROI_TYPE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_0_ROI_1_ROW_VEC_TOPBOT No description provided (USE_CASE_0_ROI_1_ROW_VEC_TOPBOT) Register
 *  No description provided (USE_CASE_0_ROI_1_ROW_VEC_TOPBOT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_TOPBOT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_TOPBOT_t {
    union {
        struct {
             unsigned int BITS17_AND_0     : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_TOPBOT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_0_ROI_1_ROW_VEC_MAIN No description provided (USE_CASE_0_ROI_1_ROW_VEC_MAIN) Register
 *  No description provided (USE_CASE_0_ROI_1_ROW_VEC_MAIN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_MAIN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_MAIN_t {
    union {
        struct {
             unsigned int BITS16_1         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_MAIN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_0_ROI_1_COLUMN_VEC No description provided (USE_CASE_0_ROI_1_COLUMN_VEC) Register
 *  No description provided (USE_CASE_0_ROI_1_COLUMN_VEC) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_COLUMN_VEC_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_COLUMN_VEC_t {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_COLUMN_VEC_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_0_ROI_1_ROI_TYPE No description provided (USE_CASE_0_ROI_1_ROI_TYPE) Register
 *  No description provided (USE_CASE_0_ROI_1_ROI_TYPE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROI_TYPE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROI_TYPE_t {
    union {
        struct {
             unsigned int BINSS            : 4; /**< No description provided */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROI_TYPE_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_0_ROI_2_ROW_VEC_TOPBOT No description provided (USE_CASE_0_ROI_2_ROW_VEC_TOPBOT) Register
 *  No description provided (USE_CASE_0_ROI_2_ROW_VEC_TOPBOT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_TOPBOT_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_TOPBOT_t {
    union {
        struct {
             unsigned int BITS17_AND_0     : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_TOPBOT_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_0_ROI_2_ROW_VEC_MAIN No description provided (USE_CASE_0_ROI_2_ROW_VEC_MAIN) Register
 *  No description provided (USE_CASE_0_ROI_2_ROW_VEC_MAIN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_MAIN_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_MAIN_t {
    union {
        struct {
             unsigned int BITS16_1         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_MAIN_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_0_ROI_2_COLUMN_VEC No description provided (USE_CASE_0_ROI_2_COLUMN_VEC) Register
 *  No description provided (USE_CASE_0_ROI_2_COLUMN_VEC) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_COLUMN_VEC_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_COLUMN_VEC_t {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_COLUMN_VEC_t;
#ifdef SWIG
%}
#endif

/*@}*/

/** @defgroup USE_CASE_0_ROI_2_ROI_TYPE No description provided (USE_CASE_0_ROI_2_ROI_TYPE) Register
 *  No description provided (USE_CASE_0_ROI_2_ROI_TYPE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROI_TYPE_t
 *  \brief No description provided
 *  ======================================================================== */
#ifdef SWIG
%inline %{
#endif
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROI_TYPE_t {
    union {
        struct {
             unsigned int BINSS            : 4; /**< No description provided */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROI_TYPE_t;
#ifdef SWIG
%}
#endif

/*@}*/

#endif  /* end ifndef ADI_DE_REGS_YODA_TYPEDEFS_H_ */

#if defined (__CC_ARM)
#pragma pop
#endif

#endif /* NEWTON_TYPEDEFS_H */

