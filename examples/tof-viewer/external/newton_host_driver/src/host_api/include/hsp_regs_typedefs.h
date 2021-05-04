
/* ================================================================================
     Created by  :   wpeet
     Created on  :   2020 Apr 03, 15:33 EDT

     Project     :   hsp_regs
     File        :   hsp_regs_typedefs.h
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

#ifndef HSP_REGS_TYPEDEFS_H
#define HSP_REGS_TYPEDEFS_H

/* pickup integer types */
#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

#if defined ( __CC_ARM   )
#pragma push
#pragma anon_unions
#endif

#ifndef ADI_HSP_REGS_TYPEDEFS_H_
#define ADI_HSP_REGS_TYPEDEFS_H_

/** @defgroup ADI_S2H_MBX_CTRL System to HSP Mailbox Interrupt Control (ADI_S2H_MBX_CTRL) Register
 *  System to HSP Mailbox Interrupt Control (ADI_S2H_MBX_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_FULL_INT_EN
 *  \brief FIFO full Interrupt Enable
 *  ======================================================================== */
typedef enum
{
     HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_FULL_INT_EN_INTERRUPTDISABLE = 0,    /**< No description provided */
     HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_FULL_INT_EN_INTERRUPTENABLE = 1     /**< No description provided */
}  ADI_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_FULL_INT_EN;

/*! ========================================================================
 *  \enum ADI_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_NEMPTY_INT_EN
 *  \brief FIFO Not-empty Interrupt Enable
 *  ======================================================================== */
typedef enum
{
     HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_NEMPTY_INT_EN_INTERRUPTDISABLE = 0,    /**< No description provided */
     HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_NEMPTY_INT_EN_INTERRUPTENABLE = 1     /**< No description provided */
}  ADI_HSP_REGS_ADI_S2H_MBX_CTRL_S2H_FIFO_NEMPTY_INT_EN;

/*! ========================================================================
 *  \struct ADI_HSP_REGS_ADI_S2H_MBX_CTRL_t
 *  \brief System to HSP Mailbox Interrupt Control
 *  ======================================================================== */
typedef struct _ADI_HSP_REGS_ADI_S2H_MBX_CTRL_t {
    union {
        struct {
             unsigned int S2H_FIFO_FULL_INT_EN : 1; /**< FIFO full Interrupt Enable */
             unsigned int S2H_FIFO_NEMPTY_INT_EN : 1; /**< FIFO Not-empty Interrupt Enable */
             unsigned int S2H_FIFO_VALID_INT_EN : 1; /**< S2H FIFO valid interrupt enable */
             unsigned int S2H_ERR_INT_EN   : 1; /**< S2H error interrupt enable */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_HSP_REGS_ADI_S2H_MBX_CTRL_t;

/*@}*/

/** @defgroup ADI_S2H_MBX_STS System to HSP Mailbox Interrupt Status (ADI_S2H_MBX_STS) Register
 *  System to HSP Mailbox Interrupt Status (ADI_S2H_MBX_STS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_HSP_REGS_ADI_S2H_MBX_STS_t
 *  \brief System to HSP Mailbox Interrupt Status
 *  ======================================================================== */
typedef struct _ADI_HSP_REGS_ADI_S2H_MBX_STS_t {
    union {
        struct {
             unsigned int S2H_FIFO_FULL_INSTS : 1; /**< FIFO Full Status */
             unsigned int S2H_FIFO_NEMPTY_INSTS : 1; /**< FIFO Not empty Status */
             unsigned int S2H_FIFO_VALID   : 1; /**< S2H FIFO Valid */
             unsigned int S2H_ERR_BIT      : 1; /**< S2H Error Status Bit */
             unsigned int RESERVED4        : 4; /**< Reserved */
             unsigned int S2H_FIFO_CNT     : 8; /**< S2H FIFO Count */
        };
        uint16_t VALUE16;
    };
} ADI_HSP_REGS_ADI_S2H_MBX_STS_t;

/*@}*/

/** @defgroup ADI_S2H_MBX_FIFO_PUSH Write to this register pushes data into the S2H FIFO (ADI_S2H_MBX_FIFO_PUSH) Register
 *  Write to this register pushes data into the S2H FIFO (ADI_S2H_MBX_FIFO_PUSH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH_t
 *  \brief Write to this register pushes data into the S2H FIFO
 *  ======================================================================== */
typedef struct _ADI_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH_t {
    union {
        struct {
             unsigned int S2H_FIFO_DATA_PUSH : 16; /**< Write to this register pushes data into the S2H FIFO */
        };
        uint16_t VALUE16;
    };
} ADI_HSP_REGS_ADI_S2H_MBX_FIFO_PUSH_t;

/*@}*/

/** @defgroup ADI_S2H_MBX_FIFO_POP System Write Data into the FIFO. (ADI_S2H_MBX_FIFO_POP) Register
 *  System Write Data into the FIFO. (ADI_S2H_MBX_FIFO_POP) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_HSP_REGS_ADI_S2H_MBX_FIFO_POP_t
 *  \brief System Write Data into the FIFO.
 *  ======================================================================== */
typedef struct _ADI_HSP_REGS_ADI_S2H_MBX_FIFO_POP_t {
    union {
        struct {
             unsigned int S2H_FIFO_DATA_POP : 16; /**< The data available at the S2H FIFO output */
        };
        uint16_t VALUE16;
    };
} ADI_HSP_REGS_ADI_S2H_MBX_FIFO_POP_t;

/*@}*/

/** @defgroup ADI_H2S_MBX_CTRL HSP to System Mailbox Interrupt Control (ADI_H2S_MBX_CTRL) Register
 *  HSP to System Mailbox Interrupt Control (ADI_H2S_MBX_CTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_FULL_INT_EN
 *  \brief FIFO full Interrupt Enable
 *  ======================================================================== */
typedef enum
{
     HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_FULL_INT_EN_INTERRUPTDISABLE = 0,    /**< No description provided */
     HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_FULL_INT_EN_INTERRUPTENABLE = 1     /**< No description provided */
}  ADI_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_FULL_INT_EN;

/*! ========================================================================
 *  \enum ADI_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_NEMPTY_INT_EN
 *  \brief FIFO Not-empty Interrupt Enable
 *  ======================================================================== */
typedef enum
{
     HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_NEMPTY_INT_EN_INTERRUPTDISABLE = 0,    /**< No description provided */
     HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_NEMPTY_INT_EN_INTERRUPTENABLE = 1     /**< No description provided */
}  ADI_HSP_REGS_ADI_H2S_MBX_CTRL_H2S_FIFO_NEMPTY_INT_EN;

/*! ========================================================================
 *  \struct ADI_HSP_REGS_ADI_H2S_MBX_CTRL_t
 *  \brief HSP to System Mailbox Interrupt Control
 *  ======================================================================== */
typedef struct _ADI_HSP_REGS_ADI_H2S_MBX_CTRL_t {
    union {
        struct {
             unsigned int H2S_FIFO_FULL_INT_EN : 1; /**< FIFO full Interrupt Enable */
             unsigned int H2S_FIFO_NEMPTY_INT_EN : 1; /**< FIFO Not-empty Interrupt Enable */
             unsigned int H2S_FIFO_VALID_INT_EN : 1; /**< H2S FIFO valid interrupt enable */
             unsigned int H2S_ERR_INT_EN   : 1; /**< H2S error interrupt enable */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_HSP_REGS_ADI_H2S_MBX_CTRL_t;

/*@}*/

/** @defgroup ADI_H2S_MBX_INSTS HSP to System Mailbox Interrupt Statu (ADI_H2S_MBX_INSTS) Register
 *  HSP to System Mailbox Interrupt Statu (ADI_H2S_MBX_INSTS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_HSP_REGS_ADI_H2S_MBX_INSTS_t
 *  \brief HSP to System Mailbox Interrupt Statu
 *  ======================================================================== */
typedef struct _ADI_HSP_REGS_ADI_H2S_MBX_INSTS_t {
    union {
        struct {
             unsigned int H2S_FIFO_FULL_INSTS : 1; /**< FIFO Full Status */
             unsigned int H2S_FIFO_NEMPTY_INSTS : 1; /**< FIFO Not empty Status */
             unsigned int H2S_FIFO_VALID   : 1; /**< H2S FIFO Valid */
             unsigned int H2S_ERR_BIT      : 1; /**< H2S error status bit */
             unsigned int RESERVED4        : 4; /**< Reserved */
             unsigned int H2S_FIFO_CNT     : 8; /**< H2S FIFI Count */
        };
        uint16_t VALUE16;
    };
} ADI_HSP_REGS_ADI_H2S_MBX_INSTS_t;

/*@}*/

/** @defgroup ADI_H2S_MBX_FIFO_PUSH HSP to System Mailbox Write Data into the FIFO. (ADI_H2S_MBX_FIFO_PUSH) Register
 *  HSP to System Mailbox Write Data into the FIFO. (ADI_H2S_MBX_FIFO_PUSH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_HSP_REGS_ADI_H2S_MBX_FIFO_PUSH_t
 *  \brief HSP to System Mailbox Write Data into the FIFO.
 *  ======================================================================== */
typedef struct _ADI_HSP_REGS_ADI_H2S_MBX_FIFO_PUSH_t {
    union {
        struct {
             unsigned int H2S_FIFO_DATA_PUSH : 16; /**< Write to this register pushes data into the S2H FIFO */
        };
        uint16_t VALUE16;
    };
} ADI_HSP_REGS_ADI_H2S_MBX_FIFO_PUSH_t;

/*@}*/

/** @defgroup ADI_H2S_MBX_FIFO_POP HSP to System Mailbox Read Data from the FIFO (ADI_H2S_MBX_FIFO_POP) Register
 *  HSP to System Mailbox Read Data from the FIFO (ADI_H2S_MBX_FIFO_POP) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_HSP_REGS_ADI_H2S_MBX_FIFO_POP_t
 *  \brief HSP to System Mailbox Read Data from the FIFO
 *  ======================================================================== */
typedef struct _ADI_HSP_REGS_ADI_H2S_MBX_FIFO_POP_t {
    union {
        struct {
             unsigned int H2S_FIFO_DATA_POP : 16; /**< The data available at the S2H FIFO output */
        };
        uint16_t VALUE16;
    };
} ADI_HSP_REGS_ADI_H2S_MBX_FIFO_POP_t;

/*@}*/

#endif  /* end ifndef ADI_HSP_REGS_TYPEDEFS_H_ */

#if defined (__CC_ARM)
#pragma pop
#endif

#endif /* HSP_REGS_TYPEDEFS_H */

