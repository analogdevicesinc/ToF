
/* ================================================================================
     Created by  :   wpeet
     Created on  :   2020 Apr 09, 09:54 EDT

     Project     :   fpga_backdoor
     File        :   fpga_backdoor_typedefs.h
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

#ifndef FPGA_BACKDOOR_TYPEDEFS_H
#define FPGA_BACKDOOR_TYPEDEFS_H

/* pickup integer types */
#if defined(_LANGUAGE_C) || (defined(__GNUC__) && !defined(__ASSEMBLER__))
#include <stdint.h>
#endif /* _LANGUAGE_C */

#if defined ( __CC_ARM   )
#pragma push
#pragma anon_unions
#endif

#ifndef ADI_FPGA_BACKDOOR_TYPEDEFS_H_
#define ADI_FPGA_BACKDOOR_TYPEDEFS_H_

/** @defgroup REGMAP1_BACKDOOR_RESET Control HSP Reset During Backdoor (REGMAP1_BACKDOOR_RESET) Register
 *  Control HSP Reset During Backdoor (REGMAP1_BACKDOOR_RESET) Register
 *  @{
 */

/*! ========================================================================
 *  \enum ADI_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET_HSP_SOFTRESETB_STICKY
 *  \brief HSP Soft Reset Active Low (FPGA)
 *  ======================================================================== */
typedef enum
{
     FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET_HSP_SOFTRESETB_STICKY_DISABLE = 0,    /**< No description provided */
     FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET_HSP_SOFTRESETB_STICKY_ENABLE = 1     /**< No description provided */
}  ADI_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET_HSP_SOFTRESETB_STICKY;

/*! ========================================================================
 *  \struct ADI_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET_t
 *  \brief Control HSP Reset During Backdoor
 *  ======================================================================== */
typedef struct _ADI_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET_t {
    union {
        struct {
             unsigned int HSP_SOFTRESETB_STICKY : 1; /**< HSP Soft Reset Active Low (FPGA) */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_FPGA_BACKDOOR_REGMAP1_BACKDOOR_RESET_t;

/*@}*/

/** @defgroup REGMAP1_BACKDOOR_ENABLE Selects Memory for Backdoor Load (REGMAP1_BACKDOOR_ENABLE) Register
 *  Selects Memory for Backdoor Load (REGMAP1_BACKDOOR_ENABLE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_t
 *  \brief Selects Memory for Backdoor Load
 *  ======================================================================== */
typedef struct _ADI_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_t {
    union {
        struct {
             unsigned int HSPROM_BDEN      : 1; /**< Enables Backdoor Load of HSP ROM */
             unsigned int FMCEFUSE_BDEN    : 1; /**< Enables Backdoor Load of Secure Fuse */
             unsigned int HSPRAM_BDEN      : 1; /**< Enables Backdoor Load of HSP RAM */
             unsigned int RESERVED3        : 13; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_FPGA_BACKDOOR_REGMAP1_BACKDOOR_ENABLE_t;

/*@}*/

/** @defgroup REGMAP1_FPGA_DEBUG1 FPGA Debug Register for HSP AEB (REGMAP1_FPGA_DEBUG1) Register
 *  FPGA Debug Register for HSP AEB (REGMAP1_FPGA_DEBUG1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_FPGA_BACKDOOR_REGMAP1_FPGA_DEBUG1_t
 *  \brief FPGA Debug Register for HSP AEB
 *  ======================================================================== */
typedef struct _ADI_FPGA_BACKDOOR_REGMAP1_FPGA_DEBUG1_t {
    union {
        struct {
             unsigned int HSP_AEB          : 16; /**< HSP Access Enable Bits */
        };
        uint16_t VALUE16;
    };
} ADI_FPGA_BACKDOOR_REGMAP1_FPGA_DEBUG1_t;

/*@}*/

/** @defgroup REGMAP1_FPGA_SCRATCH0 FPGA Scratch Register (REGMAP1_FPGA_SCRATCH0) Register
 *  FPGA Scratch Register (REGMAP1_FPGA_SCRATCH0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH0_t
 *  \brief FPGA Scratch Register
 *  ======================================================================== */
typedef struct _ADI_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH0_t {
    union {
        struct {
             unsigned int SCRATCH0         : 16; /**< FPGA Scratch Register 0 */
        };
        uint16_t VALUE16;
    };
} ADI_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH0_t;

/*@}*/

/** @defgroup REGMAP1_FPGA_SCRATCH1 FPGA Scratch Register (REGMAP1_FPGA_SCRATCH1) Register
 *  FPGA Scratch Register (REGMAP1_FPGA_SCRATCH1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH1_t
 *  \brief FPGA Scratch Register
 *  ======================================================================== */
typedef struct _ADI_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH1_t {
    union {
        struct {
             unsigned int SCRATCH1         : 16; /**< FPGA Scratch Register 1 */
        };
        uint16_t VALUE16;
    };
} ADI_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH1_t;

/*@}*/

/** @defgroup REGMAP1_FPGA_SCRATCH2 FPGA Scratch Register (REGMAP1_FPGA_SCRATCH2) Register
 *  FPGA Scratch Register (REGMAP1_FPGA_SCRATCH2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH2_t
 *  \brief FPGA Scratch Register
 *  ======================================================================== */
typedef struct _ADI_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH2_t {
    union {
        struct {
             unsigned int SCRATCH2         : 16; /**< FPGA Scratch Register 2 */
        };
        uint16_t VALUE16;
    };
} ADI_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH2_t;

/*@}*/

/** @defgroup REGMAP1_FPGA_SCRATCH3 FPGA Scratch Register (REGMAP1_FPGA_SCRATCH3) Register
 *  FPGA Scratch Register (REGMAP1_FPGA_SCRATCH3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH3_t
 *  \brief FPGA Scratch Register
 *  ======================================================================== */
typedef struct _ADI_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH3_t {
    union {
        struct {
             unsigned int SCRATCH3         : 16; /**< FPGA Scratch Register 3 */
        };
        uint16_t VALUE16;
    };
} ADI_FPGA_BACKDOOR_REGMAP1_FPGA_SCRATCH3_t;

/*@}*/

/** @defgroup REGMAP1_HSP_BYPASS FPGA HSP Bypass Enable (REGMAP1_HSP_BYPASS) Register
 *  FPGA HSP Bypass Enable (REGMAP1_HSP_BYPASS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS_t
 *  \brief FPGA HSP Bypass Enable
 *  ======================================================================== */
typedef struct _ADI_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS_t {
    union {
        struct {
             unsigned int HSP_BYPASS_EN    : 1; /**< FPGA HSP Bypass Enable */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        uint16_t VALUE16;
    };
} ADI_FPGA_BACKDOOR_REGMAP1_HSP_BYPASS_t;

/*@}*/

#endif  /* end ifndef ADI_FPGA_BACKDOOR_TYPEDEFS_H_ */

#if defined (__CC_ARM)
#pragma pop
#endif

#endif /* FPGA_BACKDOOR_TYPEDEFS_H */

