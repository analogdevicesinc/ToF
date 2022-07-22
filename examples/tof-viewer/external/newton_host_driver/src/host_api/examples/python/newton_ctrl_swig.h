////
// Copyright (c) 2020 by Analog Devices, Inc. All Rights Reserved.
//
// This software is proprietary and confidential.  By possession or use of this
// software you agree to the terms of the associated Analog Devices License Agreement.
//
//-----------------------------------------------------------------------------
// File Name          : newton_control.h
// Description        : newton host API control implementation
//-----------------------------------------------------------------------------

#ifndef _NEWTON_CONTROL_H_
#define _NEWTON_CONTROL_H_

/**
 * @file newton_control.h
 *
 * @section newton_control_api_sec Newton Control API
 * @section api_intro_sec Newton Control API
 *
 * The purpose of the Newton Control API is to provide an API for
 * controlling the Newton from a host processor via the SPI Slave interface
 * of the Newton.
 * 
 * @section functions_sec Summary of Functionality
 *
 * The following is a summary of funcionality provided by the Newton Control API:
 * 
 * - Reset HSP
 * - SPI read
 * - SPI write
 * - register read
 * - register write
 * - Load fuses from file
 * - Load HSP ROM from file
 * - Load program into newton memory from file
 * - Unload fuses to file
 * - Unload HSP ROM to file
 * - Unload program from newton memory to file
 * - Verify fuses against file contents
 * - Verify HSP ROM against file contents
 * - Verify program in newton memory against file contents
 *
 * @section examples_sec Code examples
 *
 * \code{.c}
 * FIXME: need new code example
 * \endcode
 *
 * @section load_firmware_sec Load Firmware code example
 *
 * \code{.c}
 * FIXME: need new code example
 * \endcode
 *
 * See newton_control.h for detailed documentation.
 *
 */

//==============================================================================
// INCLUDES
//==============================================================================
#include <stdio.h>
#include <stdlib.h>
#include <stdint.h>
#include <stdbool.h>
#include <unistd.h>
#include <string.h>
#include <ctype.h>
#include <time.h>
#include <sys/ioctl.h>
#include <sys/stat.h>
#include <linux/types.h>
#include <linux/spi/spidev.h>
#include <fcntl.h>
#include "newton_addr_cdef.h"
#include "newton_typedefs.h"
#include "newton_memMap.h"
#include "fpga_backdoor_addr_cdef.h"
#include "fpga_backdoor_typedefs.h"
#include "hsp_regs_addr_cdef.h"
#include "hsp_regs_typedefs.h"
#ifdef WIRINGPI
    #include <wiringPi.h>
#endif

//==============================================================================
// DEFINES
//==============================================================================
#define MAX_REG_LIST 2048 / 4
#define HOST_API_VERSION 1.0

//      Newton Pin Name             Wiring Pin Number
#define RSTN                        3    

#define i2cScl_spisSclk             14
#define I2C_SCL                     14

#define i2cSda_spisMosi             12
#define I2C_SDA                     12

#define GPIO0_spisMiso_trstDft      13    
#define GPIO0                       13

#define GPIO1_spisScs               10
#define GPIO1                       10

#define GPIO2_uartSout              15
#define GPIO2                       15

#define GPIO3_uartSin               16
#define GPIO3                       16

#define GPIO4_mboxOut_tdoDft        0
#define GPIO4                       0

#define GPIO5_fsync_tmsDft          1
#define GPIO5                       1

#define GPIO6_lightEn_trstHsp       28
#define GPIO6                       28

#define GPIO7_spimSclk_tdiHsp       29
#define GPIO7                       29

#define GPIO8_spimMosi_tmsHsp       25
#define GPIO8                       25

#define GPIO9_spimScs_tck           27
#define GPIO9                       27

#define GPIO10_spimMiso_tdoHsp      24
#define GPIO10                      24

#define PIN_HIGH                    1
#define PIN_LOW                     0

//==============================================================================
// TYPEDEFS
//==============================================================================
%include "cstring.i"

%inline %{
    typedef unsigned long long u64;
    typedef unsigned int u32;
    typedef unsigned short u16;
    typedef unsigned char u08;
%}


#ifndef PYTHON
    typedef struct str_parms str_parms;
#endif

%inline %{
typedef struct _adi_ErrorLog_s {
    union {
        struct {
            u32 MSFTDebugData0[4];
            u32 errorStatus;
            u32 MSFTDebugData1[3];
        };
        u16 data16[16];
    };
} adi_ErrorLog_s;
%}

//==============================================================================
// ENUMS
//==============================================================================
/**   
 * @brief Newton WiringPi Pins.
 *
 *  Newton WiringPi pin numbers.
 *
 */
%inline %{
typedef enum {
    PIN_RSTN    = 3,
    PIN_I2C_SCL = 14,
    PIN_I2C_SDA = 12,
    PIN_GPIO0   = 13,
    PIN_GPIO1   = 10,
    PIN_GPIO2   = 16,
    PIN_GPIO3   = 15,
    PIN_GPIO4   = 24,
    PIN_GPIO5   = 21,
    PIN_GPIO6   = 28,
    PIN_GPIO7   = 29,
    PIN_GPIO8   = 25,
    PIN_GPIO9   = 27,
    PIN_GPIO10  = 24
} adi_pin_numbers_e;
%}

/**   
 * @brief Newton Pin Modes.
 *
 *  Newton Pin Modes.
 *
 */
%inline %{
typedef enum {
    PIN_MODE_FUNCTIONAL = 0,
    PIN_MODE_HSP_DEBUG,
    PIN_MODE_DFT_JTAG,
    PIN_MODE_PROD_SCAN,
    PIN_MODE_FIELD_RETURN_SCAN,
    PIN_MODE_KEEP_CURRENT_MODE
} adi_pin_modes_e;
%}

/**   
 * @brief HSP mailbox attributes.
 *
 *  Newton HSP mailbox attributes.
 *
 */
%inline %{
typedef enum {
    NO_ATTR                   = 0x00,
    WRITE_ATTR                = 0x01, // [0]
    SIGNED_ATTR               = 0x02, // [1]
    ENCRYPTED_ATTR            = 0x04, // [2]
    GROUPED_ATTR              = 0x08, // [3]
    MBX_UNSIGNED_SEQ_WFI      = 0x10, // [8:4], Maps to unsigned sequence with no 1SP; starts sequence after
                                      //        grouped packet and WFI
    MBX_UNSIGNED_SEQ_1SP      = 0x20, // [8:4], Maps to unsigned sequence with 1SP; immediately jumps to 1SP
    MBX_ADI_SEQ_WFI           = 0x30, // [8:4], Maps to ADI sequence with no 1SP; starts sequence after grouped packet,
                                      //        applies firewall, and WFI
    MBX_ADI_SEQ_1SP           = 0x40, // [8:4], Maps to ADI sequence with 1SP; immediately jumps to 1SP
    MBX_ADI_SEQ_PROD_1SP      = 0x60, // [8:4], Maps to the ADI production 1SP sequence; immediately jumps
                                      //        to ADI production 1SP
    MBX_MSFT_SEQ_PROD_1SP     = 0x70, // [8:4],
    MBX_MSFT_ADI_SEQ_PROD_1SP = 0x80  // [8:4], Maps to MSFT production 1SP sequencewhich sets up for ADI production 1SP
} adi_attribute_e;
%}

/**   
 * @brief HSP Mailbox Commands
 *
 *  HSP Mailbox Commands.
 *
 */
%inline %{
typedef enum adi_command {
    CMD_GROUPED_DATA        = 0x1001,
    CMD_SEQ_RAM             = 0x1002,
    CMD_WAVE_RAM            = 0x1003,
    CMD_MAP_RAM             = 0x1004,
    CMD_DATAPATH_RAM        = 0x1005,
    CMD_DUMP_ENGINE_RAM     = 0x1006,
    CMD_LPS1_RAM            = 0x1007,
    CMD_LPS2_RAM            = 0x1008,
    CMD_REGISTER_CFG        = 0x1009,
    CMD_1SP_IMAGE           = 0x100A,
    CMD_ERROR_LOG           = 0x100B,
    CMD_PUBLIC_KEY          = 0x100C,
    CMD_SIGNATURE           = 0x100D,
    CMD_OPERATING_MODE      = 0x100E
} adi_command_e;
%}

/**   
 * @brief Newton RAM Targets
 *
 *  Newton RAM targets.
 *
 */
%inline %{
typedef enum adi_loadTargets {
    USEQ_SEQ_RAM = 0, // Microsequencer Sequence RAM
    USEQ_MAP_RAM,     // Microsequencer MAP RAM
    USEQ_WAVE_RAM,    // Microsequencer Wave RAM
    DATAPATH_RAM,     // Gain Correction RAM
    DE_RAM,           // Dump Engine RAM
    LPS1_RAM,         // LPS1 
    LPS2_RAM,         // LPS2 
    CMD_FILE,         // Command File 
    HSP_ROM,          // HSP ROM (FPGA only)
    HSP_RAM,          // HSP RAM (FPGA only)
    EFUSE             // eFuse   (FPGA only)
} adi_loadTargets_e;
%}

/**   
 * @brief HSP memory bit widths
 *
 *  HSP memory bit widths.
 *
 */
%inline %{
typedef enum adi_hspMemBitWidths {
    HSP_ROM_WIDTH = 40,
    HSP_RAM_WIDTH = 40,
    EFUSE_WIDTH   = 32
} adi_hspMemBitWidths_e;
%}

/**   
 * @brief Newton memory bit widths
 *
 *  Newton memory bit widths.
 *
 */
%inline %{
typedef enum adi_newtonMemBitWidths {
    USEQ_SEQ_RAM_WIDTH = 16,
    USEQ_MAP_RAM_WIDTH = 11,
    USEQ_WAVE_RAM_WIDTH = 16,
    DATAPATH_RAM_WIDTH = 16,
    DE_RAM_WIDTH = 64,
    LPS1_RAM_WIDTH = 24,
    LPS2_RAM_WIDTH = 24
} adi_newtonMemBitWidths_e;
%}

/**   
 * @brief Newton memory bytte widths
 *
 *  Newton memory byte widths.
 *
 */
%inline %{
typedef enum adi_newtonMemByteWidths {
    USEQ_SEQ_RAM_WIDTH_BYTES = 2,
    USEQ_MAP_RAM_WIDTH_BYTES = 2,
    USEQ_WAVE_RAM_WIDTH_BYTES = 2,
    DATAPATH_RAM_WIDTH_BYTES = 2,
    DE_RAM_WIDTH_BYTES = 8,
    LPS1_RAM_WIDTH_BYTES = 4,
    LPS2_RAM_WIDTH_BYTES = 4
} adi_newtonMemByteWidths_e;
%}

/**   
 * @brief Newton memory depths
 *
 *  Newton memory depths.
 *
 */
%inline %{
typedef enum adi_newtonMemDepths {
    USEQ_SEQ_RAM_DEPTH = 4096,
    USEQ_MAP_RAM_DEPTH = 128,
    USEQ_WAVE_RAM_DEPTH = 2048,
    DATAPATH_RAM_DEPTH = 4096,
    DE_RAM_DEPTH = 1024,
    LPS1_RAM_DEPTH = 256,
    LPS2_RAM_DEPTH = 256
} adi_newtonMemDepths_e;
%}

/**   
 * @brief Newton memory masks
 *
 *  Newton memory masks.
 *
 */
%inline %{
typedef enum adi_newtonMemMasks {
    USEQ_SEQ_RAM_MASK = 0xffff,
    USEQ_MAP_RAM_MASK = 0x7ff,
    USEQ_WAVE_RAM_MASK = 0xffff,
    DATAPATH_RAM_MASK = 0xffff,
    DE_RAM_MASK = 0xffffffffffffffff,
    LPS1_RAM_MASK = 0xffffff,
    LPS2_RAM_MASK = 0xffffff
} adi_newtonMemMasks_e;
%}

/**   
 * @brief Command Read States
 *
 *  Command Read States.
 *
 */
%inline %{
typedef enum adi_commandLoadStates {
    CMD_LOAD_HEADER = 0,
    CMD_GROUPED_DATA_WRITE,
    CMD_LOAD_REG_WRITE,
    CMD_LOAD_REG_READ,
    CMD_LOAD_RAM_WRITE,
    CMD_LOAD_ERROR_LOG,
    CMD_LOAD_1SP_IMAGE,
    CMD_LOAD_SIGNATURE,
    CMD_LOAD_PUBLIC_KEY,
    CMD_LOAD_OPERATING_MODE,
    CMD_LOAD_UNKNOWN
} adi_commandLoadStates_e;
%}


/**   
 * @brief HSP Error Codes
 *
 *  HSP error codes.
 *
 */
%inline %{
typedef enum adi_hspErrorCodes {
    HSP_SUCCESS                              = 0x00, /**< Success */
    HSP_UNKNOWN_ERROR                        = 0x01, /**< Unknown error */
    HSP_INVALID_PARAMETER                    = 0x02, /**< Invalid parameter */
    HSP_NOT_IMPLEMENTED                      = 0x03, /**< Not implemented */
    HSP_COMMAND_ERROR                        = 0x04, /**< Command error */
    HSP_BUS_ERROR                            = 0x05, /**< Bus error */
    HSP_FAULT_ERROR                          = 0x06, /**< Fault error */
    HSP_NOT_OWNER_ERROR                      = 0x07, /**< Not owner error */
    HSP_BUSY_STATE                           = 0x08, /**< HSP is busy */
    HSP_SIGNATURE_MISMATCH                   = 0x09, /**< Signature does not match */
    HSP_AUTHENTICATION_FAILED                = 0x0A, /**< Authentication failed */
    HSP_IRQ_STATE                            = 0x0B, /**< Trapped in IRQ */
    HSP_FIQ_STATE                            = 0x0C, /**< Trapped in FIQ */
    HSP_KEY_NOT_ZERO_ERROR                   = 0x0D, /**< Key not zero error */
    HSP_BLANK_CHECK_FAIL                     = 0x0E, /**< Fuse word check is not blank; this might not be error */
    HSP_ECC_CORRECTION_FAIL                  = 0x0F, /**< Ecc correction fails, having more than 1 errors */
    HSP_SVN_VERSION_MISMATCH                 = 0x10, /**< SVN in 1SP header does not match the SVN in fuses */
    HSP_INVALID_OPERATING_MODE               = 0x11, /**< Invalid operating mode */
    HSP_INVALID_CEK_KEY                      = 0x12, /**< Invalid CEK decryption key */
    HSP_DECRYPTION_FAILED                    = 0x13, /**< Decrypted payload does not match expected hash */
    HSP_SW_FAULT                             = 0x14, /**< SW Exception */
    HSP_HW_FAULT                             = 0x15, /**< Unexpected HW fault */
    HSP_FIQ_UNHANDLED                        = 0x16, /**< Unhandled FIQ error */
    HSP_FIQ_CRYPTO_ERR                       = 0x17, /**< Critical Crypto error */
    HSP_FIQ_ACC_VIO                          = 0x18, /**< Access violation */
    HSP_FIQ_MEM_EDC_ERR                      = 0x19, /**< MEM EDC error */
    HSP_FIQ_MEM_ERASE_ERR                    = 0x1A, /**< Mem Erase error */
    HSP_FIQ_SP_BUS_ERR                       = 0x1B, /**< SP Bus error */
    HSP_FIQ_FATAL_UNHANDLED                  = 0x1C, /**< Fatal FIQ that are not handled */
    HSP_MBX_INVALID_CMD_ERROR                = 0x80, /**< Invalid Command Packet Command field */
    HSP_MBX_INVALID_SIZE_ERROR               = 0x81, /**< Invalid Command Packet Size field */
    HSP_MBX_INVALID_ADDR_ERROR               = 0x82, /**< Invalid Command Packet Addr field */
    HSP_MBX_INVALID_ATTRIBUTE_ERROR          = 0x83, /**< Invalid Command Packet Attribute field */
    HSP_MBX_REGISTER_ACCESS_ERROR            = 0x84, /**< Invalid register access */
    HSP_MBX_EXCEED_ENCRYPT_ENTRIES_ERROR     = 0x85, /**< Exceeded Number of Mailbox Encryption Entries */
    HSP_MBX_SIGN_VERF_FAIL_ERROR             = 0x86, /**< Signature Verification Failed */
    HSP_MBX_INVALID_AUTH_CERT                = 0x87, /**< Unsigned Authorization Certificate */
    HSP_MBX_GROUPED_DATA_SIZE_MISMATCH_ERROR = 0x88, /**< Mismatch of Grouped data size attribute and sum of subpacket data size */
    HSP_MBX_SEQ_ERROR                        = 0x89  /**< Error in the mailbox sequence */
} adi_hspErrorCodes_e;
%}

%inline %{
static const char* adi_hspErrorMessages[39] = {
    "Success",
    "Unknown error",
    "Invalid parameter",
    "Not implemented",
    "Command error",
    "Bus error",
    "Fault error",
    "Not owner error",
    "HSP is busy",
    "Signature does not match",
    "Authentication failed",
    "Trapped in IRQ",
    "Trapped in FIQ",
    "Key not zero error",
    "Fuse word check is not blank; this might not be error",
    "Ecc correction fails, having more than 1 errors",
    "SVN in 1SP header does not match the SVN in fuses",
    "Invalid operating mode",
    "Invalid CEK decryption key",
    "Decrypted payload does not match expected hash",
    "SW Exception",
    "Unexpected HW fault",
    "Unhandled FIQ error",
    "Critical Crypto error",
    "Access violation",
    "MEM EDC error",
    "Mem Erase error",
    "SP Bus error",
    "Fatal FIQ that are not handled",
    "Invalid Command Packet Command field",
    "Invalid Command Packet Size field",
    "Invalid Command Packet Addr field",
    "Invalid Command Packet Attribute field",
    "Invalid register access",
    "Exceeded Number of Mailbox Encryption Entries",
    "Signature Verification Failed",
    "Unsigned Authorization Certificate",
    "Mismatch of Grouped data size attribute and sum of subpacket data size",
    "Error in the mailbox sequence"
};
%}

/**   
 * @brief Newton memory depths
 *
 *  Newton memory depths.
 *
 */
%inline %{
typedef enum adi_newtonRegisterAddresses {
    USEQ_REGS_USEQRAMLOADADDR   = 0X000000C0,    /**< Useq Ram Load start Address Register */
    USEQ_REGS_USEQRAMRDSTADDR   = 0X000000C2,    /**< Useq Ram Read start Address Register */
    USEQ_REGS_USEQRAMLOADDATA   = 0X000000C4,    /**< Useq Ram Load Data Register */
    USEQ_REGS_USEQRAMRDDATA     = 0X000000C8,    /**< Useq Ram Read Data Register */
    LPS1_REGS_LPSRAMADDR        = 0X00000906,    /**< No description provided */
    LPS1_REGS_LPSRAMRDCMD       = 0X00000908,    /**< No description provided */
    LPS1_REGS_LPSRAMDATA        = 0X00000920,    /**< No description provided */
    LPS2_REGS_LPSRAMADDR        = 0X00000A06,    /**< No description provided */
    LPS2_REGS_LPSRAMRDCMD       = 0X00000A08,    /**< No description provided */
    LPS2_REGS_LPSRAMDATA        = 0X00000A20,    /**< No description provided */
    DATAPATH_REGS_IA_SELECT     = 0X00000D70,    /**< No description provided */
    DATAPATH_REGS_IA_ADDR_REG   = 0X00000D72,    /**< No description provided */
    DATAPATH_REGS_IA_WRDATA_REG = 0X00000D74,    /**< No description provided */
    DATAPATH_REGS_IA_RDDATA_REG = 0X00000D78,    /**< No description provided */
    DATAPATH_REGS_IA_BANK_TYPE  = 0X00000D7C,    /**< No description provided */
    DE_REGS_DE_IA_SELECT        = 0X00000E70,    /**< No description provided */
    DE_REGS_DE_IA_ADDR_REG      = 0X00000E72,    /**< No description provided */
    DE_REGS_DE_IA_WRDATA_REG    = 0X00000E74,    /**< No description provided */
    DE_REGS_DE_IA_RDDATA_REG    = 0X00000E78     /**< No description provided */
} adi_newtonRegisterAddresses_e;
%}

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_t
 *  \brief Useq Ram Load start Address Register
 *  ======================================================================== */
%inline %{
typedef struct _ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_s {
    union {
        struct {
             unsigned int LD_RAM_SEL       : 2; /**< No description provided */
             unsigned int LD_ADDR          : 12; /**< No description provided */
             unsigned int RESERVED14       : 2; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQRAMLOADADDR_s;
%}

/*@}*/

/** @defgroup USEQRAMRDSTADDR Useq Ram Read start Address Register (USEQRAMRDSTADDR) Register
 *  Useq Ram Read start Address Register (USEQRAMRDSTADDR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQRAMRDSTADDR_s
 *  \brief Useq Ram Read start Address Register
 *  ======================================================================== */
%inline %{
typedef struct _ADI_USEQ_REGS_MAP1_USEQRAMRDSTADDR_s {
    union {
        struct {
             unsigned int RD_RAM_SEL       : 2; /**< No description provided */
             unsigned int RD_ADDR          : 12; /**< No description provided */
             unsigned int RESERVED14       : 2; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQRAMRDSTADDR_s;
%}

/*@}*/

/** @defgroup USEQRAMLOADDATA Useq Ram Load Data Register (USEQRAMLOADDATA) Register
 *  Useq Ram Load Data Register (USEQRAMLOADDATA) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQRAMLOADDATA_s
 *  \brief Useq Ram Load Data Register
 *  ======================================================================== */
%inline %{
typedef struct _ADI_USEQ_REGS_MAP1_USEQRAMLOADDATA_s {
    union {
        struct {
             unsigned int LD_DATA          : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQRAMLOADDATA_s;
%}

/*@}*/

/** @defgroup USEQRAMLOADDATAALIAS Useq Ram Load Data Register (USEQRAMLOADDATAALIAS) Register
 *  Useq Ram Load Data Register (USEQRAMLOADDATAALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQRAMLOADDATAALIAS_s
 *  \brief Useq Ram Load Data Register
 *  ======================================================================== */
%inline %{
typedef struct _ADI_USEQ_REGS_MAP1_USEQRAMLOADDATAALIAS_s {
    union {
        struct {
             unsigned int LD_DATA_ALIAS    : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQRAMLOADDATAALIAS_s;
%}

/*@}*/

/** @defgroup USEQRAMRDDATA Useq Ram Read Data Register (USEQRAMRDDATA) Register
 *  Useq Ram Read Data Register (USEQRAMRDDATA) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQRAMRDDATA_s
 *  \brief Useq Ram Read Data Register
 *  ======================================================================== */
%inline %{
typedef struct _ADI_USEQ_REGS_MAP1_USEQRAMRDDATA_s {
    union {
        struct {
             unsigned int USEQ_RAM_RD_DATA : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQRAMRDDATA_s;
%}

/*@}*/

/** @defgroup USEQRAMRDDATAALIAS Useq Ram Read Data Register (USEQRAMRDDATAALIAS) Register
 *  Useq Ram Read Data Register (USEQRAMRDDATAALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_USEQ_REGS_MAP1_USEQRAMRDDATAALIAS_s
 *  \brief Useq Ram Read Data Register
 *  ======================================================================== */
%inline %{
typedef struct _ADI_USEQ_REGS_MAP1_USEQRAMRDDATAALIAS_s {
    union {
        struct {
             unsigned int USEQ_RAM_RD_DATA_ALIAS : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_USEQ_REGS_MAP1_USEQRAMRDDATAALIAS_s;
%}

/*@}*/


/** @defgroup LPSCTRL No description provided (LPSCTRL) Register
 *  No description provided (LPSCTRL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSCTRL_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_LPS_REGS_YODA_LPSCTRL_s {
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
        u16 VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSCTRL_s;
%}

/*@}*/

/** @defgroup LPSWAVEFREQ No description provided (LPSWAVEFREQ) Register
 *  No description provided (LPSWAVEFREQ) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSWAVEFREQ_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_LPS_REGS_YODA_LPSWAVEFREQ_s {
    union {
        struct {
             unsigned int LPS_WAVE_FREQ    : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSWAVEFREQ_s;
%}

/*@}*/

/** @defgroup LPSWAVEGENACC No description provided (LPSWAVEGENACC) Register
 *  No description provided (LPSWAVEGENACC) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSWAVEGENACC_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_LPS_REGS_YODA_LPSWAVEGENACC_s {
    union {
        struct {
             unsigned int LPS_WG_ACC       : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSWAVEGENACC_s;
%}

/*@}*/

/** @defgroup LPSRAMADDR No description provided (LPSRAMADDR) Register
 *  No description provided (LPSRAMADDR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSRAMADDR_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_LPS_REGS_YODA_LPSRAMADDR_s {
    union {
        struct {
             unsigned int LPS_RAM_ADDR     : 9; /**< No description provided */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSRAMADDR_s;
%}

/*@}*/

/** @defgroup LPSRAMRDCMD No description provided (LPSRAMRDCMD) Register
 *  No description provided (LPSRAMRDCMD) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSRAMRDCMD_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_LPS_REGS_YODA_LPSRAMRDCMD_s {
    union {
        struct {
             unsigned int LPS_RAM_READ_EN  : 1; /**< No description provided */
             unsigned int LPS_RAM_READ_RDY : 1; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSRAMRDCMD_s;
%}

/*@}*/

/** @defgroup LPSWAVEGENADDR No description provided (LPSWAVEGENADDR) Register
 *  No description provided (LPSWAVEGENADDR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSWAVEGENADDR_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_LPS_REGS_YODA_LPSWAVEGENADDR_s {
    union {
        struct {
             unsigned int LPS_RAM_START    : 8; /**< No description provided */
             unsigned int LPS_RAM_END      : 8; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSWAVEGENADDR_s;
%}

/*@}*/

/** @defgroup LPSMARGIN No description provided (LPSMARGIN) Register
 *  No description provided (LPSMARGIN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSMARGIN_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_LPS_REGS_YODA_LPSMARGIN_s {
    union {
        struct {
             unsigned int LPS_PORT0_MARGIN : 1; /**< No description provided */
             unsigned int LPS_PORT1_MARGIN : 1; /**< No description provided */
             unsigned int RESERVED2        : 13; /**< Reserved */
             unsigned int LPS_PARITY_ERR   : 1; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSMARGIN_s;
%}

/*@}*/

/** @defgroup LPSDBG No description provided (LPSDBG) Register
 *  No description provided (LPSDBG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSDBG_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_LPS_REGS_YODA_LPSDBG_s {
    union {
        struct {
             unsigned int LPSDBGSEL        : 3; /**< No description provided */
             unsigned int RESERVED3        : 5; /**< Reserved */
             unsigned int LPSDBGCOM        : 2; /**< No description provided */
             unsigned int RESERVED10       : 5; /**< Reserved */
             unsigned int LPSDBGEN         : 1; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSDBG_s;
%}

/*@}*/

/** @defgroup LPSRAMDATA No description provided (LPSRAMDATA) Register
 *  No description provided (LPSRAMDATA) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSRAMDATA_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_LPS_REGS_YODA_LPSRAMDATA_s {
    union {
        struct {
             unsigned int LPS_RAM_DATA     : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSRAMDATA_s;
%}

/*@}*/

/** @defgroup LPSRAMDATA_ALIAS No description provided (LPSRAMDATA_ALIAS) Register
 *  No description provided (LPSRAMDATA_ALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_LPS_REGS_YODA_LPSRAMDATA_ALIAS_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_LPS_REGS_YODA_LPSRAMDATA_ALIAS_s {
    union {
        struct {
             unsigned int LPS_RAM_DATA_ALIAS : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_LPS_REGS_YODA_LPSRAMDATA_ALIAS_s;
%}

/*@}*/

/*! ========================================================================
 *  \struct ADI_DATAPATH_CORRECTION_CONFIG_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_CORRECTION_CONFIG_s {
    union {
        struct {
             unsigned int BYPASS_PC_GAIN   : 1; /**< Bypass per-column gain correction */
             unsigned int BYPASS_PC_OFFSET : 1; /**< Bypass per-column offset correction */
             unsigned int BYPASS_SCALE     : 1; /**< Bypass gain scaling */
             unsigned int BYPASS_SATTAG    : 1; /**< Bypass saturation tag while pixel adjustment */
             unsigned int BINNING_AVG_EN   : 1; /**< 1'b1 = average two rows while digital binning; 1'b0 = add two rows while digital binning */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_CORRECTION_CONFIG_s;
%}

/*@}*/

/*! ========================================================================
 *  \struct ADI_DATAPATH_USE_CASE_FRAME_CONFIG_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_USE_CASE_FRAME_CONFIG_s {
    union {
        struct {
             unsigned int ADC_9B           : 1; /**< When 1, ADC valid data width is 9 bits */
             unsigned int ALTERNATE_AMP_MUX_POL : 1; /**< 1'b1 = Inverted gain scaling for even ADCs (needed for supply rejection work-around); 1'b0 = No inversion */
             unsigned int DIGITAL_BIN_EN   : 1; /**< Enable digital binning between adjacent rows, invalid to set if delta_comp_en is set */
             unsigned int DELTA_COMP_EN    : 1; /**< Enable delta compression */
             unsigned int FIX2FLT_EN       : 1; /**< enable fixed to floating point conversion */
             unsigned int RAW_MODE         : 2; /**< Enable RAW mode and select how the ADC clipping is handled */
             unsigned int OUTPUT_WIDTH     : 3; /**< Bit width of final output to MIPI */
             unsigned int DARK_ROW_VEC     : 2; /**< Each bit enables the readout of  8 dark rows on the top and the bottom of the array */
             unsigned int MIPI_OUT_8BIT    : 1; /**< 1'b1 = Send RAW14 or RAW16 as two 8 bit RAW pixels to MIPI; when 1'b0 = Send the pixel data as 14 bit or 16 bit value */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_USE_CASE_FRAME_CONFIG_s;
%}

/*@}*/

/** @defgroup USE_CASE_MIPI_PACKET_CONTROL No description provided (USE_CASE_MIPI_PACKET_CONTROL) Register
 *  No description provided (USE_CASE_MIPI_PACKET_CONTROL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_s {
    union {
        struct {
             unsigned int ROWS_PER_PACKET  : 7; /**< Number of rows to be packed in a MIPI  packet. Valid values are 1,2,4,8,16,32,64 */
             unsigned int AUTO_ROWS_PER_PACKET_EN : 1; /**< When 1, datapath calculates the rows to be packed in a MIPI packet; when 0 use the value programmed in rows per packet register */
             unsigned int MIPI_BUFF_RD_LIMIT : 2; /**< Number of buffers to be filled before sending packet request */
             unsigned int MIPI_BUFF_RD_CTRL_EN : 1; /**< When 1, reading of MIPI buffers is based on mipi_buff_rd_limit, else done in datapath */
             unsigned int RESERVED11       : 5; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_USE_CASE_MIPI_PACKET_CONTROL_s;
%}

/*@}*/

/** @defgroup GAIN0 No description provided (GAIN0) Register
 *  No description provided (GAIN0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_GAIN0_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_GAIN0_s {
    union {
        struct {
             unsigned int GLOBAL_GAIN_SCALE_P0 : 10; /**< Gain scale for GT0 */
             unsigned int GLOBAL_GAIN_SHIFT_P0 : 3; /**< Gain shift for GT0 */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_GAIN0_s;
%}

/*@}*/

/** @defgroup GAIN1 No description provided (GAIN1) Register
 *  No description provided (GAIN1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_GAIN1_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_GAIN1_s {
    union {
        struct {
             unsigned int GLOBAL_GAIN_SCALE_P1 : 10; /**< Gain scale for GT1 */
             unsigned int GLOBAL_GAIN_SHIFT_P1 : 3; /**< Gain shift for GT1 */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_GAIN1_s;
%}

/*@}*/

/** @defgroup GAIN2 No description provided (GAIN2) Register
 *  No description provided (GAIN2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_GAIN2_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_GAIN2_s {
    union {
        struct {
             unsigned int GLOBAL_GAIN_SCALE_P2 : 10; /**< Gain scale for GT2 */
             unsigned int GLOBAL_GAIN_SHIFT_P2 : 3; /**< Gain shift for GT2 */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_GAIN2_s;
%}

/*@}*/

/** @defgroup GAIN3 No description provided (GAIN3) Register
 *  No description provided (GAIN3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_GAIN3_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_GAIN3_s {
    union {
        struct {
             unsigned int GLOBAL_GAIN_SCALE_P3 : 10; /**< Gain scale for GT3 */
             unsigned int GLOBAL_GAIN_SHIFT_P3 : 3; /**< Gain shift for GT3 */
             unsigned int RESERVED13       : 3; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_GAIN3_s;
%}

/*@}*/

/** @defgroup PARITY_GAIN_MEM No description provided (PARITY_GAIN_MEM) Register
 *  No description provided (PARITY_GAIN_MEM) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PARITY_GAIN_MEM_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_PARITY_GAIN_MEM_s {
    union {
        struct {
             unsigned int GAIN_MEM_PARITY_ERR : 16; /**< Parity error bits from gain correction memories. One bit per bank */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_PARITY_GAIN_MEM_s;
%}

/*@}*/

/** @defgroup PARITY_LINE_MEM No description provided (PARITY_LINE_MEM) Register
 *  No description provided (PARITY_LINE_MEM) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PARITY_LINE_MEM_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_PARITY_LINE_MEM_s {
    union {
        struct {
             unsigned int LINE_MEM_PARITY_ERR : 2; /**< Parity error bits from line buffer memories. One bit per bank */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_PARITY_LINE_MEM_s;
%}

/*@}*/

/** @defgroup PP_LFSR No description provided (PP_LFSR) Register
 *  No description provided (PP_LFSR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_LFSR_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_PP_LFSR_s {
    union {
        struct {
             unsigned int LFSR_EN          : 1; /**< Indicates whether the Pixel Packer must work in LFSR mode */
             unsigned int LFSR_MODE        : 2; /**< Indicates the LFSR data generation mode */
             unsigned int LFSR_SEED        : 12; /**< The initial seed value for the data generation in LFSR mode */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_PP_LFSR_s;
%}

/*@}*/

/** @defgroup PP_DECODE_ST_1 No description provided (PP_DECODE_ST_1) Register
 *  No description provided (PP_DECODE_ST_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_DECODE_ST_1_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_PP_DECODE_ST_1_s {
    union {
        struct {
             unsigned int ST_DECODE_000    : 3; /**< Mux input for the ST selection */
             unsigned int ST_DECODE_001    : 3; /**< Mux input for the ST selection */
             unsigned int ST_DECODE_010    : 3; /**< Mux input for the ST selection */
             unsigned int ST_DECODE_011    : 3; /**< Mux input for the ST selection */
             unsigned int ST_DECODE_100    : 3; /**< Mux input for the ST selection */
             unsigned int RESERVED15       : 1; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_PP_DECODE_ST_1_s;
%}

/*@}*/

/** @defgroup PP_DECODE_ST_2 No description provided (PP_DECODE_ST_2) Register
 *  No description provided (PP_DECODE_ST_2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_DECODE_ST_2_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_PP_DECODE_ST_2_s {
    union {
        struct {
             unsigned int ST_DECODE_101    : 3; /**< Mux input for the ST selection */
             unsigned int ST_DECODE_110    : 3; /**< Mux input for the ST selection */
             unsigned int ST_DECODE_111    : 3; /**< Mux input for the ST selection */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_PP_DECODE_ST_2_s;
%}

/*@}*/

/** @defgroup PP_ENCODE_ST No description provided (PP_ENCODE_ST) Register
 *  No description provided (PP_ENCODE_ST) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_ENCODE_ST_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_PP_ENCODE_ST_s {
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
        u16 VALUE16;
    };
} ADI_DATAPATH_PP_ENCODE_ST_s;
%}

/*@}*/

/** @defgroup PP_ENCODE_GT No description provided (PP_ENCODE_GT) Register
 *  No description provided (PP_ENCODE_GT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_ENCODE_GT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_PP_ENCODE_GT_s {
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
        u16 VALUE16;
    };
} ADI_DATAPATH_PP_ENCODE_GT_s;
%}

/*@}*/

/** @defgroup DBG_MUX No description provided (DBG_MUX) Register
 *  No description provided (DBG_MUX) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_DBG_MUX_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_DBG_MUX_s {
    union {
        struct {
             unsigned int DBG_SEL          : 5; /**< Internal signal debug mux select signal. Refer to Datapath HRM for mux description */
             unsigned int RESERVED5        : 10; /**< Reserved */
             unsigned int DBG_EN           : 1; /**< Enable for the Debug mux */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_DBG_MUX_s;
%}

/*@}*/

/** @defgroup GAIN_MARGIN_CONTROL No description provided (GAIN_MARGIN_CONTROL) Register
 *  No description provided (GAIN_MARGIN_CONTROL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_GAIN_MARGIN_CONTROL_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_GAIN_MARGIN_CONTROL_s {
    union {
        struct {
             unsigned int GAIN_MEM_MARGIN  : 16; /**< Read and write margin control. One bit for each gain memory. */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_GAIN_MARGIN_CONTROL_s;
%}

/*@}*/

/** @defgroup LINE_MARGIN_CONTROL No description provided (LINE_MARGIN_CONTROL) Register
 *  No description provided (LINE_MARGIN_CONTROL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_LINE_MARGIN_CONTROL_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_LINE_MARGIN_CONTROL_s {
    union {
        struct {
             unsigned int LINE_MEM_MARGIN  : 2; /**< Read and write margin control. One bit for each line memory. */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_LINE_MARGIN_CONTROL_s;
%}

/*@}*/

/** @defgroup ROI_ROW_START No description provided (ROI_ROW_START) Register
 *  No description provided (ROI_ROW_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ROI_ROW_START_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_ROI_ROW_START_s {
    union {
        struct {
             unsigned int START_ROW        : 10; /**< Start row of ROI. Should be a multiple of 64. Range (0-639) */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_ROI_ROW_START_s;
%}

/*@}*/

/** @defgroup ROI_HEIGHT No description provided (ROI_HEIGHT) Register
 *  No description provided (ROI_HEIGHT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ROI_HEIGHT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_ROI_HEIGHT_s {
    union {
        struct {
             unsigned int ROI_HEIGHT       : 10; /**< Numbers of rows enabled in ROI */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_ROI_HEIGHT_s;
%}

/*@}*/

/** @defgroup ROI_COLUMN_START No description provided (ROI_COLUMN_START) Register
 *  No description provided (ROI_COLUMN_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ROI_COLUMN_START_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_ROI_COLUMN_START_s {
    union {
        struct {
             unsigned int START_COLUMN     : 9; /**< Start column of ROI. Should be a multiple of 16 */
             unsigned int RESERVED9        : 7; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_ROI_COLUMN_START_s;
%}

/*@}*/

/** @defgroup ROI_WIDTH No description provided (ROI_WIDTH) Register
 *  No description provided (ROI_WIDTH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ROI_WIDTH_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_ROI_WIDTH_s {
    union {
        struct {
             unsigned int ROI_WIDTH        : 10; /**< Number of columns enabled in ROI */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_ROI_WIDTH_s;
%}

/*@}*/

/** @defgroup PP_USEQ_WRITE No description provided (PP_USEQ_WRITE) Register
 *  No description provided (PP_USEQ_WRITE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_USEQ_WRITE_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_PP_USEQ_WRITE_s {
    union {
        struct {
             unsigned int DUMP_START       : 1; /**< Bit to be written by micro-sequencer */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_PP_USEQ_WRITE_s;
%}

/*@}*/

/** @defgroup PP_ADC_DELAY No description provided (PP_ADC_DELAY) Register
 *  No description provided (PP_ADC_DELAY) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PP_ADC_DELAY_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_PP_ADC_DELAY_s {
    union {
        struct {
             unsigned int ADC_DELAY        : 6; /**< Specifies the number of clock cycles after which the data from adc_bus can read and used on adc_convert positive transition */
             unsigned int RESERVED6        : 10; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_PP_ADC_DELAY_s;
%}

/*@}*/

/** @defgroup MIPI_BUFF_MARGIN_CONTROL No description provided (MIPI_BUFF_MARGIN_CONTROL) Register
 *  No description provided (MIPI_BUFF_MARGIN_CONTROL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_MIPI_BUFF_MARGIN_CONTROL_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_MIPI_BUFF_MARGIN_CONTROL_s {
    union {
        struct {
             unsigned int MIPI_BUFF_MARGIN : 4; /**< Read and write margin control. One bit for each MIPI buffer */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_MIPI_BUFF_MARGIN_CONTROL_s;
%}

/*@}*/

/** @defgroup MIPI_HEADER_WIDTH No description provided (MIPI_HEADER_WIDTH) Register
 *  No description provided (MIPI_HEADER_WIDTH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_MIPI_HEADER_WIDTH_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_MIPI_HEADER_WIDTH_s {
    union {
        struct {
             unsigned int METADATA_BYTES   : 8; /**< Number of bytes sent as Metadata */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_MIPI_HEADER_WIDTH_s;
%}

/*@}*/

/** @defgroup FRAME_NUMBER No description provided (FRAME_NUMBER) Register
 *  No description provided (FRAME_NUMBER) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_FRAME_NUMBER_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_FRAME_NUMBER_s {
    union {
        struct {
             unsigned int FRAME_COUNT      : 16; /**< Frame counter to be incremented by micro-sequencer */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_FRAME_NUMBER_s;
%}

/*@}*/

/** @defgroup MICRO_SEQUENCER_FW_VERSION_LSB No description provided (MICRO_SEQUENCER_FW_VERSION_LSB) Register
 *  No description provided (MICRO_SEQUENCER_FW_VERSION_LSB) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_MICRO_SEQUENCER_FW_VERSION_LSB_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_MICRO_SEQUENCER_FW_VERSION_LSB_s {
    union {
        struct {
             unsigned int USEQ_FW_VERSION_LSB : 16; /**< LSB bits of micro-sequencer FW version */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_MICRO_SEQUENCER_FW_VERSION_LSB_s;
%}

/*@}*/

/** @defgroup MICRO_SEQUENCER_FW_VERSION_MSB No description provided (MICRO_SEQUENCER_FW_VERSION_MSB) Register
 *  No description provided (MICRO_SEQUENCER_FW_VERSION_MSB) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_MICRO_SEQUENCER_FW_VERSION_MSB_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_MICRO_SEQUENCER_FW_VERSION_MSB_s {
    union {
        struct {
             unsigned int USEQ_FW_VERSION_MSB : 16; /**< MSB bits of micro-sequencer FW version */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_MICRO_SEQUENCER_FW_VERSION_MSB_s;
%}

/*@}*/

/** @defgroup TS_CAL_VER No description provided (TS_CAL_VER) Register
 *  No description provided (TS_CAL_VER) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_sS_CAL_VER_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_TS_CAL_VER_s {
    union {
        struct {
             unsigned int TS_CAL_VER       : 16; /**< Temperature sensor calibration version */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_sS_CAL_VER_s;
%}

/*@}*/

/** @defgroup ADC_CAL_VER No description provided (ADC_CAL_VER) Register
 *  No description provided (ADC_CAL_VER) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ADC_CAL_VER_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_ADC_CAL_VER_s {
    union {
        struct {
             unsigned int ADC_CAL_VER      : 16; /**< ADC and gain calibration version */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_ADC_CAL_VER_s;
%}

/*@}*/

/** @defgroup REG_0 No description provided (REG_0) Register
 *  No description provided (REG_0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_0_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_REG_0_s {
    union {
        struct {
             unsigned int REG_0            : 16; /**< Reserved for MIPI header */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_REG_0_s;
%}

/*@}*/

/** @defgroup REG_1 No description provided (REG_1) Register
 *  No description provided (REG_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_1_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_REG_1_s {
    union {
        struct {
             unsigned int REG_1            : 16; /**< Reserved for MIPI header */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_REG_1_s;
%}

/*@}*/

/** @defgroup REG_2 No description provided (REG_2) Register
 *  No description provided (REG_2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_2_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_REG_2_s {
    union {
        struct {
             unsigned int REG_2            : 16; /**< Reserved for MIPI header */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_REG_2_s;
%}

/*@}*/

/** @defgroup REG_3 No description provided (REG_3) Register
 *  No description provided (REG_3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_3_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_REG_3_s {
    union {
        struct {
             unsigned int REG_3            : 16; /**< Reserved for MIPI header */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_REG_3_s;
%}

/*@}*/

/** @defgroup REG_4 No description provided (REG_4) Register
 *  No description provided (REG_4) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_4_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_REG_4_s {
    union {
        struct {
             unsigned int REG_4            : 16; /**< Reserved for MIPI header */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_REG_4_s;
%}

/*@}*/

/** @defgroup REG_5 No description provided (REG_5) Register
 *  No description provided (REG_5) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_5_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_REG_5_s {
    union {
        struct {
             unsigned int REG_5            : 16; /**< Reserved for MIPI header */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_REG_5_s;
%}

/*@}*/

/** @defgroup REG_6 No description provided (REG_6) Register
 *  No description provided (REG_6) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_6_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_REG_6_s {
    union {
        struct {
             unsigned int REG_6            : 16; /**< Reserved for MIPI header */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_REG_6_s;
%}

/*@}*/

/** @defgroup REG_7 No description provided (REG_7) Register
 *  No description provided (REG_7) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_REG_7_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_REG_7_s {
    union {
        struct {
             unsigned int REG_7            : 16; /**< Reserved for MIPI header */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_REG_7_s;
%}

/*@}*/

/** @defgroup PARITY_MIPI_BUFFER No description provided (PARITY_MIPI_BUFFER) Register
 *  No description provided (PARITY_MIPI_BUFFER) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PARITY_MIPI_BUFFER_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_PARITY_MIPI_BUFFER_s {
    union {
        struct {
             unsigned int MIPI_BUFFER_PARITY_ERROR : 4; /**< parity error bits from MIPI buffer memory. 1 per line memory, memory not divided into banks. 4 memories of size 65WX128D */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_PARITY_MIPI_BUFFER_s;
%}

/*@}*/

/** @defgroup PACKET_COUNT No description provided (PACKET_COUNT) Register
 *  No description provided (PACKET_COUNT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PACKET_COUNT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_PACKET_COUNT_s {
    union {
        struct {
             unsigned int PACKET_COUNT     : 10; /**< Indicates the packet number being sent to MIPI. */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_PACKET_COUNT_s;
%}

/*@}*/

/** @defgroup PACKETS_PER_FRAME No description provided (PACKETS_PER_FRAME) Register
 *  No description provided (PACKETS_PER_FRAME) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_PACKETS_PER_FRAME_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_PACKETS_PER_FRAME_s {
    union {
        struct {
             unsigned int TOTAL_PACKETS_PER_FRAME : 10; /**< Indicates the total number of data packets in a MIPI frame */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_PACKETS_PER_FRAME_s;
%}

/*@}*/

/** @defgroup ROW_VECTOR No description provided (ROW_VECTOR) Register
 *  No description provided (ROW_VECTOR) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ROW_VECTOR_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_ROW_VECTOR_s {
    union {
        struct {
             unsigned int ROW_VECTOR       : 10; /**< Each bit is used to indicate that a group of 64 rows of pixel array are enabled. LSB bit used to indicate rows 0-63 */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_ROW_VECTOR_s;
%}

/*@}*/

/** @defgroup ROWS_PER_PACKET_OUT No description provided (ROWS_PER_PACKET_OUT) Register
 *  No description provided (ROWS_PER_PACKET_OUT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ROWS_PER_PACKET_OUT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_ROWS_PER_PACKET_OUT_s {
    union {
        struct {
             unsigned int ROWS_PER_PACKET_OUT : 7; /**< When auto_rows_per_packet_en = 1; readback this value calculated by the datapath */
             unsigned int RESERVED7        : 9; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_ROWS_PER_PACKET_OUT_s;
%}

/*@}*/

/** @defgroup MIPI_RD_EN_MAX No description provided (MIPI_RD_EN_MAX) Register
 *  No description provided (MIPI_RD_EN_MAX) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_MIPI_RD_EN_MAX_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_MIPI_RD_EN_MAX_s {
    union {
        struct {
             unsigned int MIPI_BUFF_READ_ENABLE_COUNT_MAX : 8; /**< Indicates the number of reads done per mipi buffer. This value will be equal to (number of pixels per dump written to the MIPI buffer)/4 */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_MIPI_RD_EN_MAX_s;
%}

/*@}*/

/** @defgroup ANALOG_SS No description provided (ANALOG_SS) Register
 *  No description provided (ANALOG_SS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_ANALOG_SS_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_ANALOG_SS_s {
    union {
        struct {
             unsigned int ANALOG_SS        : 4; /**< Indicates the sub-sampling factor. Input from DE */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_ANALOG_SS_s;
%}

/*@}*/

/** @defgroup MIPI_BUFF_PARITY_ERR_CNT No description provided (MIPI_BUFF_PARITY_ERR_CNT) Register
 *  No description provided (MIPI_BUFF_PARITY_ERR_CNT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_MIPI_BUFF_PARITY_ERR_CNT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_MIPI_BUFF_PARITY_ERR_CNT_s {
    union {
        struct {
             unsigned int MIPI_BUFF_PARITY_ERR_COUNT : 16; /**< Count of parity errors indicated for every MIPI buffer read */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_MIPI_BUFF_PARITY_ERR_CNT_s;
%}

/*@}*/

/** @defgroup LINE_MEM_PARITY_ERR_CNT No description provided (LINE_MEM_PARITY_ERR_CNT) Register
 *  No description provided (LINE_MEM_PARITY_ERR_CNT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_LINE_MEM_PARITY_ERR_CNT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_LINE_MEM_PARITY_ERR_CNT_s {
    union {
        struct {
             unsigned int LINE_MEM_PARITY_ERR_COUNT : 16; /**< Count of parity errors indicated for every line memory read */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_LINE_MEM_PARITY_ERR_CNT_s;
%}

/*@}*/

/** @defgroup GAIN_MEM_PARITY_ERR_CNT No description provided (GAIN_MEM_PARITY_ERR_CNT) Register
 *  No description provided (GAIN_MEM_PARITY_ERR_CNT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_GAIN_MEM_PARITY_ERR_CNT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_GAIN_MEM_PARITY_ERR_CNT_s {
    union {
        struct {
             unsigned int GAIN_MEM_PARITY_ERR_COUNT : 16; /**< Count of parity errors indicated for every gain memory read */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_GAIN_MEM_PARITY_ERR_CNT_s;
%}

/*@}*/

/** @defgroup IA_SELECT No description provided (IA_SELECT) Register
 *  No description provided (IA_SELECT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_SELECT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_IA_SELECT_s {
    union {
        struct {
             unsigned int IA_ENA           : 1; /**< Indirect access enable for column correction memory */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_IA_SELECT_s;
%}

/*@}*/

/** @defgroup IA_ADDR_REG No description provided (IA_ADDR_REG) Register
 *  No description provided (IA_ADDR_REG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_ADDR_REG_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_IA_ADDR_REG_s {
    union {
        struct {
             unsigned int IA_START_ADDR    : 12; /**< Indirect access start address */
             unsigned int RESERVED12       : 4; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_IA_ADDR_REG_s;
%}

/*@}*/

/** @defgroup IA_WRDATA_REG No description provided (IA_WRDATA_REG) Register
 *  No description provided (IA_WRDATA_REG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_WRDATA_REG_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_IA_WRDATA_REG_s {
    union {
        struct {
             unsigned int IA_WRDATA        : 16; /**< Indirect access write data */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_IA_WRDATA_REG_s;
%}

/*@}*/

/** @defgroup IA_WRDATA_REG_ALIAS No description provided (IA_WRDATA_REG_ALIAS) Register
 *  No description provided (IA_WRDATA_REG_ALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_WRDATA_REG_ALIAS_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_IA_WRDATA_REG_ALIAS_s {
    union {
        struct {
             unsigned int IA_WRDATA_ALIAS  : 16; /**< Indirect access write data (alias for ia_wrdata) */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_IA_WRDATA_REG_ALIAS_s;
%}

/*@}*/

/** @defgroup IA_RDDATA_REG No description provided (IA_RDDATA_REG) Register
 *  No description provided (IA_RDDATA_REG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_RDDATA_REG_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_IA_RDDATA_REG_s {
    union {
        struct {
             unsigned int IA_RDDATA        : 16; /**< Indirect access read data */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_IA_RDDATA_REG_s;
%}

/*@}*/

/** @defgroup IA_RDDATA_REG_ALIAS No description provided (IA_RDDATA_REG_ALIAS) Register
 *  No description provided (IA_RDDATA_REG_ALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_RDDATA_REG_ALIAS_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_IA_RDDATA_REG_ALIAS_s {
    union {
        struct {
             unsigned int IA_RDDATA_ALIAS  : 16; /**< Indirect access read data (alias for ia_rdata) */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_IA_RDDATA_REG_ALIAS_s;
%}

/*@}*/

/** @defgroup IA_BANK_sYPE No description provided (IA_BANK_sYPE) Register
 *  No description provided (IA_BANK_TYPE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DATAPATH_IA_BANK_TYPE_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DATAPATH_IA_BANK_TYPE_s {
    union {
        struct {
             unsigned int IA_BANK_TYPE     : 1; /**< 1'b1 = LSB bits of address used to select bank in reg mem access module. 1'b0 = have another input which specifies the bank */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DATAPATH_IA_BANK_TYPE_s;
%}

/*@}*/

/** @defgroup DE_CONTROL No description provided (DE_CONTROL) Register
 *  No description provided (DE_CONTROL) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_CONTROL_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DE_CONTROL_s {
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
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DE_CONTROL_s;
%}

/*@}*/

/** @defgroup BINNED1X2_REPEAT_COUNT No description provided (BINNED1X2_REPEAT_COUNT) Register
 *  No description provided (BINNED1X2_REPEAT_COUNT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_BINNED1X2_REPEAT_COUNT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_BINNED1X2_REPEAT_COUNT_s {
    union {
        struct {
             unsigned int REPEAT_COUNT     : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_BINNED1X2_REPEAT_COUNT_s;
%}

/*@}*/

/** @defgroup OVERRIDE_DATA_REG1 No description provided (OVERRIDE_DATA_REG1) Register
 *  No description provided (OVERRIDE_DATA_REG1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_OVERRIDE_DATA_REG1_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_OVERRIDE_DATA_REG1_s {
    union {
        struct {
             unsigned int OVR_VAL          : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_OVERRIDE_DATA_REG1_s;
%}

/*@}*/

/** @defgroup OVERRIDE_DATA_REG2 No description provided (OVERRIDE_DATA_REG2) Register
 *  No description provided (OVERRIDE_DATA_REG2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_OVERRIDE_DATA_REG2_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_OVERRIDE_DATA_REG2_s {
    union {
        struct {
             unsigned int OVR_VAL          : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_OVERRIDE_DATA_REG2_s;
%}

/*@}*/

/** @defgroup OVERRIDE_DATA_REG3 No description provided (OVERRIDE_DATA_REG3) Register
 *  No description provided (OVERRIDE_DATA_REG3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_OVERRIDE_DATA_REG3_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_OVERRIDE_DATA_REG3_s {
    union {
        struct {
             unsigned int OVR_VAL          : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_OVERRIDE_DATA_REG3_s;
%}

/*@}*/

/** @defgroup BINNED1X2_END No description provided (BINNED1X2_END) Register
 *  No description provided (BINNED1X2_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_BINNED1X2_END_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_BINNED1X2_END_s {
    union {
        struct {
             unsigned int END_ADDRESS      : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_BINNED1X2_END_s;
%}

/*@}*/

/** @defgroup OVERRIDE_SEL_REG1 No description provided (OVERRIDE_SEL_REG1) Register
 *  No description provided (OVERRIDE_SEL_REG1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_OVERRIDE_SEL_REG1_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_OVERRIDE_SEL_REG1_s {
    union {
        struct {
             unsigned int OVR_VAL_SEL      : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_OVERRIDE_SEL_REG1_s;
%}

/*@}*/

/** @defgroup OVERRIDE_SEL_REG2 No description provided (OVERRIDE_SEL_REG2) Register
 *  No description provided (OVERRIDE_SEL_REG2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_OVERRIDE_SEL_REG2_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_OVERRIDE_SEL_REG2_s {
    union {
        struct {
             unsigned int OVR_VAL_SEL      : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_OVERRIDE_SEL_REG2_s;
%}

/*@}*/

/** @defgroup OVERRIDE_SEL_REG3 No description provided (OVERRIDE_SEL_REG3) Register
 *  No description provided (OVERRIDE_SEL_REG3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_OVERRIDE_SEL_REG3_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_OVERRIDE_SEL_REG3_s {
    union {
        struct {
             unsigned int OVR_VAL_SEL      : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_OVERRIDE_SEL_REG3_s;
%}

/*@}*/

/** @defgroup BINNED1X2_START No description provided (BINNED1X2_START) Register
 *  No description provided (BINNED1X2_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_BINNED1X2_START_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_BINNED1X2_START_s {
    union {
        struct {
             unsigned int START_ADDRESS    : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_BINNED1X2_START_s;
%}

/*@}*/

/** @defgroup AMP_MUX_SEL_EE_LOW No description provided (AMP_MUX_SEL_EE_LOW) Register
 *  No description provided (AMP_MUX_SEL_EE_LOW) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_EE_LOW_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_EE_LOW_s {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_EE_LOW_s;
%}

/*@}*/

/** @defgroup AMP_MUX_SEL_EE_HIGH No description provided (AMP_MUX_SEL_EE_HIGH) Register
 *  No description provided (AMP_MUX_SEL_EE_HIGH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_EE_HIGH_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_EE_HIGH_s {
    union {
        struct {
             unsigned int BITS20_16        : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_EE_HIGH_s;
%}

/*@}*/

/** @defgroup AMP_MUX_SEL_EO_LOW No description provided (AMP_MUX_SEL_EO_LOW) Register
 *  No description provided (AMP_MUX_SEL_EO_LOW) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_EO_LOW_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_EO_LOW_s {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_EO_LOW_s;
%}

/*@}*/

/** @defgroup AMP_MUX_SEL_EO_HIGH No description provided (AMP_MUX_SEL_EO_HIGH) Register
 *  No description provided (AMP_MUX_SEL_EO_HIGH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_EO_HIGH_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_EO_HIGH_s {
    union {
        struct {
             unsigned int BITS20_16        : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_EO_HIGH_s;
%}

/*@}*/

/** @defgroup AMP_MUX_SEL_OE_LOW No description provided (AMP_MUX_SEL_OE_LOW) Register
 *  No description provided (AMP_MUX_SEL_OE_LOW) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_OE_LOW_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_OE_LOW_s {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_OE_LOW_s;
%}

/*@}*/

/** @defgroup AMP_MUX_SEL_OE_HIGH No description provided (AMP_MUX_SEL_OE_HIGH) Register
 *  No description provided (AMP_MUX_SEL_OE_HIGH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_OE_HIGH_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_OE_HIGH_s {
    union {
        struct {
             unsigned int BITS20_16        : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_OE_HIGH_s;
%}

/*@}*/

/** @defgroup AMP_MUX_SEL_OO_LOW No description provided (AMP_MUX_SEL_OO_LOW) Register
 *  No description provided (AMP_MUX_SEL_OO_LOW) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_OO_LOW_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_OO_LOW_s {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_OO_LOW_s;
%}

/*@}*/

/** @defgroup AMP_MUX_SEL_OO_HIGH No description provided (AMP_MUX_SEL_OO_HIGH) Register
 *  No description provided (AMP_MUX_SEL_OO_HIGH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_OO_HIGH_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_OO_HIGH_s {
    union {
        struct {
             unsigned int BITS20_16        : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_OO_HIGH_s;
%}

/*@}*/

/** @defgroup AMP_MUX_SEL_SELB_LOW No description provided (AMP_MUX_SEL_SELB_LOW) Register
 *  No description provided (AMP_MUX_SEL_SELB_LOW) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_SELB_LOW_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_SELB_LOW_s {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_SELB_LOW_s;
%}

/*@}*/

/** @defgroup AMP_MUX_SEL_SELB_HIGH No description provided (AMP_MUX_SEL_SELB_HIGH) Register
 *  No description provided (AMP_MUX_SEL_SELB_HIGH) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_AMP_MUX_SEL_SELB_HIGH_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_AMP_MUX_SEL_SELB_HIGH_s {
    union {
        struct {
             unsigned int BITS20_16        : 5; /**< No description provided */
             unsigned int RESERVED5        : 11; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_AMP_MUX_SEL_SELB_HIGH_s;
%}

/*@}*/

/** @defgroup NATIVE_RESOLUTION_START No description provided (NATIVE_RESOLUTION_START) Register
 *  No description provided (NATIVE_RESOLUTION_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_NATIVE_RESOLUTION_START_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_NATIVE_RESOLUTION_START_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_NATIVE_RESOLUTION_START_s;
%}

/*@}*/

/** @defgroup NATIVE_RESOLUTION_END No description provided (NATIVE_RESOLUTION_END) Register
 *  No description provided (NATIVE_RESOLUTION_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_NATIVE_RESOLUTION_END_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_NATIVE_RESOLUTION_END_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_NATIVE_RESOLUTION_END_s;
%}

/*@}*/

/** @defgroup NATIVE_RESOLUTION_REPEAT No description provided (NATIVE_RESOLUTION_REPEAT) Register
 *  No description provided (NATIVE_RESOLUTION_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_NATIVE_RESOLUTION_REPEAT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_NATIVE_RESOLUTION_REPEAT_s {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_NATIVE_RESOLUTION_REPEAT_s;
%}

/*@}*/

/** @defgroup SUB_SAMPLED_2X_START No description provided (SUB_SAMPLED_2X_START) Register
 *  No description provided (SUB_SAMPLED_2X_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_SUB_SAMPLED_2X_START_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_SUB_SAMPLED_2X_START_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_SUB_SAMPLED_2X_START_s;
%}

/*@}*/

/** @defgroup SUB_SAMPLED_2X_END No description provided (SUB_SAMPLED_2X_END) Register
 *  No description provided (SUB_SAMPLED_2X_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_SUB_SAMPLED_2X_END_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_SUB_SAMPLED_2X_END_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_SUB_SAMPLED_2X_END_s;
%}

/*@}*/

/** @defgroup SUB_SAMPLED_2X_REPEAT No description provided (SUB_SAMPLED_2X_REPEAT) Register
 *  No description provided (SUB_SAMPLED_2X_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_SUB_SAMPLED_2X_REPEAT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_SUB_SAMPLED_2X_REPEAT_s {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_SUB_SAMPLED_2X_REPEAT_s;
%}

/*@}*/

/** @defgroup SUB_SAMPLED_4X_START No description provided (SUB_SAMPLED_4X_START) Register
 *  No description provided (SUB_SAMPLED_4X_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_SUB_SAMPLED_4X_START_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_SUB_SAMPLED_4X_START_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_SUB_SAMPLED_4X_START_s;
%}

/*@}*/

/** @defgroup SUB_SAMPLED_4X_END No description provided (SUB_SAMPLED_4X_END) Register
 *  No description provided (SUB_SAMPLED_4X_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_SUB_SAMPLED_4X_END_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_SUB_SAMPLED_4X_END_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_SUB_SAMPLED_4X_END_s;
%}

/*@}*/

/** @defgroup SUB_SAMPLED_4X_REPEAT No description provided (SUB_SAMPLED_4X_REPEAT) Register
 *  No description provided (SUB_SAMPLED_4X_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_SUB_SAMPLED_4X_REPEAT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_SUB_SAMPLED_4X_REPEAT_s {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_SUB_SAMPLED_4X_REPEAT_s;
%}

/*@}*/

/** @defgroup BINNED_START No description provided (BINNED_START) Register
 *  No description provided (BINNED_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_BINNED_START_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_BINNED_START_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_BINNED_START_s;
%}

/*@}*/

/** @defgroup BINNED_END No description provided (BINNED_END) Register
 *  No description provided (BINNED_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_BINNED_END_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_BINNED_END_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_BINNED_END_s;
%}

/*@}*/

/** @defgroup BINNED_REPEAT No description provided (BINNED_REPEAT) Register
 *  No description provided (BINNED_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_BINNED_REPEAT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_BINNED_REPEAT_s {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_BINNED_REPEAT_s;
%}

/*@}*/

/** @defgroup DARK_START No description provided (DARK_START) Register
 *  No description provided (DARK_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DARK_START_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DARK_START_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DARK_START_s;
%}

/*@}*/

/** @defgroup DARK_END No description provided (DARK_END) Register
 *  No description provided (DARK_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DARK_END_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DARK_END_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DARK_END_s;
%}

/*@}*/

/** @defgroup DARK_REPEAT No description provided (DARK_REPEAT) Register
 *  No description provided (DARK_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DARK_REPEAT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DARK_REPEAT_s {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DARK_REPEAT_s;
%}

/*@}*/

/** @defgroup PREAMBLE_START No description provided (PREAMBLE_START) Register
 *  No description provided (PREAMBLE_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_PREAMBLE_START_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_PREAMBLE_START_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_PREAMBLE_START_s;
%}

/*@}*/

/** @defgroup PREAMBLE_END No description provided (PREAMBLE_END) Register
 *  No description provided (PREAMBLE_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_PREAMBLE_END_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_PREAMBLE_END_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_PREAMBLE_END_s;
%}

/*@}*/

/** @defgroup PREAMBLE_REPEAT No description provided (PREAMBLE_REPEAT) Register
 *  No description provided (PREAMBLE_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_PREAMBLE_REPEAT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_PREAMBLE_REPEAT_s {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_PREAMBLE_REPEAT_s;
%}

/*@}*/

/** @defgroup POSTAMBLE_START No description provided (POSTAMBLE_START) Register
 *  No description provided (POSTAMBLE_START) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_POSTAMBLE_START_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_POSTAMBLE_START_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_POSTAMBLE_START_s;
%}

/*@}*/

/** @defgroup POSTAMBLE_END No description provided (POSTAMBLE_END) Register
 *  No description provided (POSTAMBLE_END) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_POSTAMBLE_END_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_POSTAMBLE_END_s {
    union {
        struct {
             unsigned int ADDRESS          : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_POSTAMBLE_END_s;
%}

/*@}*/

/** @defgroup POSTAMBLE_REPEAT No description provided (POSTAMBLE_REPEAT) Register
 *  No description provided (POSTAMBLE_REPEAT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_POSTAMBLE_REPEAT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_POSTAMBLE_REPEAT_s {
    union {
        struct {
             unsigned int COUNT            : 8; /**< No description provided */
             unsigned int RESERVED8        : 8; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_POSTAMBLE_REPEAT_s;
%}

/*@}*/

/** @defgroup ARRAY_INIT_VEC_DARK No description provided (ARRAY_INIT_VEC_DARK) Register
 *  No description provided (ARRAY_INIT_VEC_DARK) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_ARRAY_INIT_VEC_DARK_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_ARRAY_INIT_VEC_DARK_s {
    union {
        struct {
             unsigned int DARK_BITS        : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_ARRAY_INIT_VEC_DARK_s;
%}

/*@}*/

/** @defgroup ARRAY_INIT_VEC No description provided (ARRAY_INIT_VEC) Register
 *  No description provided (ARRAY_INIT_VEC) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_ARRAY_INIT_VEC_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_ARRAY_INIT_VEC_s {
    union {
        struct {
             unsigned int ARRAY_BITS       : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_ARRAY_INIT_VEC_s;
%}

/*@}*/

/** @defgroup TYPE_OVERRIDE No description provided (TYPE_OVERRIDE) Register
 *  No description provided (TYPE_OVERRIDE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_sYPE_OVERRIDE_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_TYPE_OVERRIDE_s {
    union {
        struct {
             unsigned int ROI_0            : 4; /**< No description provided */
             unsigned int ROI_1            : 4; /**< No description provided */
             unsigned int ROI_2            : 4; /**< No description provided */
             unsigned int ANALOG           : 4; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_TYPE_OVERRIDE_s;
%}

/*@}*/

/** @defgroup MEM_DFT No description provided (MEM_DFT) Register
 *  No description provided (MEM_DFT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_MEM_DFT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_MEM_DFT_s {
    union {
        struct {
             unsigned int MARGIN           : 1; /**< No description provided */
             unsigned int RESERVED1        : 14; /**< Reserved */
             unsigned int PARITY_ERR       : 1; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_MEM_DFT_s;
%}

/*@}*/

/** @defgroup DBG_MUX_CONTROL_0 No description provided (DBG_MUX_CONTROL_0) Register
 *  No description provided (DBG_MUX_CONTROL_0) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DBG_MUX_CONTROL_0_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DBG_MUX_CONTROL_0_s {
    union {
        struct {
             unsigned int CNTRL_0          : 8; /**< No description provided */
             unsigned int CNTRL_1          : 8; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DBG_MUX_CONTROL_0_s;
%}

/*@}*/

/** @defgroup DBG_MUX_CONTROL_1 No description provided (DBG_MUX_CONTROL_1) Register
 *  No description provided (DBG_MUX_CONTROL_1) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DBG_MUX_CONTROL_1_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DBG_MUX_CONTROL_1_s {
    union {
        struct {
             unsigned int CNTRL_2          : 8; /**< No description provided */
             unsigned int CNTRL_3          : 8; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DBG_MUX_CONTROL_1_s;
%}

/*@}*/

/** @defgroup DBG_MUX_CONTROL_2 No description provided (DBG_MUX_CONTROL_2) Register
 *  No description provided (DBG_MUX_CONTROL_2) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DBG_MUX_CONTROL_2_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DBG_MUX_CONTROL_2_s {
    union {
        struct {
             unsigned int CNTRL_4          : 8; /**< No description provided */
             unsigned int CNTRL_5          : 8; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DBG_MUX_CONTROL_2_s;
%}

/*@}*/

/** @defgroup DBG_MUX_CONTROL_3 No description provided (DBG_MUX_CONTROL_3) Register
 *  No description provided (DBG_MUX_CONTROL_3) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DBG_MUX_CONTROL_3_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DBG_MUX_CONTROL_3_s {
    union {
        struct {
             unsigned int CNTRL_6          : 8; /**< No description provided */
             unsigned int CNTRL_7          : 8; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DBG_MUX_CONTROL_3_s;
%}

/*@}*/

/** @defgroup DBG_MUX_CONTROL_4 No description provided (DBG_MUX_CONTROL_4) Register
 *  No description provided (DBG_MUX_CONTROL_4) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DBG_MUX_CONTROL_4_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DBG_MUX_CONTROL_4_s {
    union {
        struct {
             unsigned int CNTRL_8          : 8; /**< No description provided */
             unsigned int CNTRL_9          : 8; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DBG_MUX_CONTROL_4_s;
%}

/*@}*/

/** @defgroup DE_IA_SELECT No description provided (DE_IA_SELECT) Register
 *  No description provided (DE_IA_SELECT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_IA_SELECT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DE_IA_SELECT_s {
    union {
        struct {
             unsigned int RAM              : 1; /**< No description provided */
             unsigned int RESERVED1        : 15; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DE_IA_SELECT_s;
%}

/*@}*/

/** @defgroup DE_IA_ADDR_REG No description provided (DE_IA_ADDR_REG) Register
 *  No description provided (DE_IA_ADDR_REG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_IA_ADDR_REG_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DE_IA_ADDR_REG_s {
    union {
        struct {
             unsigned int RAM_ADDR         : 10; /**< No description provided */
             unsigned int RESERVED10       : 6; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DE_IA_ADDR_REG_s;
%}

/*@}*/

/** @defgroup DE_IA_WRDATA_REG No description provided (DE_IA_WRDATA_REG) Register
 *  No description provided (DE_IA_WRDATA_REG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_IA_WRDATA_REG_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DE_IA_WRDATA_REG_s {
    union {
        struct {
             unsigned int RAM_WRDATA       : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DE_IA_WRDATA_REG_s;
%}

/*@}*/

/** @defgroup DE_IA_WRDATA_REG_ALIAS No description provided (DE_IA_WRDATA_REG_ALIAS) Register
 *  No description provided (DE_IA_WRDATA_REG_ALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_IA_WRDATA_REG_ALIAS_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DE_IA_WRDATA_REG_ALIAS_s {
    union {
        struct {
             unsigned int RAM_WRDATA_ALIAS : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DE_IA_WRDATA_REG_ALIAS_s;
%}

/*@}*/

/** @defgroup DE_IA_RDDATA_REG No description provided (DE_IA_RDDATA_REG) Register
 *  No description provided (DE_IA_RDDATA_REG) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_IA_RDDATA_REG_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DE_IA_RDDATA_REG_s {
    union {
        struct {
             unsigned int RAM_RDDATA       : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DE_IA_RDDATA_REG_s;
%}

/*@}*/

/** @defgroup DE_IA_RDDATA_REG_ALIAS No description provided (DE_IA_RDDATA_REG_ALIAS) Register
 *  No description provided (DE_IA_RDDATA_REG_ALIAS) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_DE_IA_RDDATA_REG_ALIAS_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_DE_IA_RDDATA_REG_ALIAS_s {
    union {
        struct {
             unsigned int RAM_RDDATA_ALIAS : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_DE_IA_RDDATA_REG_ALIAS_s;
%}

/*@}*/

/** @defgroup USE_CASE_0_ROI_0_ROW_VEC_sOPBOT No description provided (USE_CASE_0_ROI_0_ROW_VEC_sOPBOT) Register
 *  No description provided (USE_CASE_0_ROI_0_ROW_VEC_TOPBOT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_TOPBOT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_TOPBOT_s {
    union {
        struct {
             unsigned int BITS17_AND_0     : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_TOPBOT_s;
%}

/*@}*/

/** @defgroup USE_CASE_0_ROI_0_ROW_VEC_MAIN No description provided (USE_CASE_0_ROI_0_ROW_VEC_MAIN) Register
 *  No description provided (USE_CASE_0_ROI_0_ROW_VEC_MAIN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_MAIN_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_MAIN_s {
    union {
        struct {
             unsigned int BITS16_1         : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROW_VEC_MAIN_s;
%}

/*@}*/

/** @defgroup USE_CASE_0_ROI_0_COLUMN_VEC No description provided (USE_CASE_0_ROI_0_COLUMN_VEC) Register
 *  No description provided (USE_CASE_0_ROI_0_COLUMN_VEC) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_COLUMN_VEC_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_COLUMN_VEC_s {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_COLUMN_VEC_s;
%}

/*@}*/

/** @defgroup USE_CASE_0_ROI_0_ROI_TYPE No description provided (USE_CASE_0_ROI_0_ROI_TYPE) Register
 *  No description provided (USE_CASE_0_ROI_0_ROI_TYPE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROI_TYPE_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROI_TYPE_s {
    union {
        struct {
             unsigned int BINSS            : 4; /**< No description provided */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_0_ROI_TYPE_s;
%}

/*@}*/

/** @defgroup USE_CASE_0_ROI_1_ROW_VEC_TOPBOT No description provided (USE_CASE_0_ROI_1_ROW_VEC_TOPBOT) Register
 *  No description provided (USE_CASE_0_ROI_1_ROW_VEC_TOPBOT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_TOPBOT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_TOPBOT_s {
    union {
        struct {
             unsigned int BITS17_AND_0     : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_TOPBOT_s;
%}

/*@}*/

/** @defgroup USE_CASE_0_ROI_1_ROW_VEC_MAIN No description provided (USE_CASE_0_ROI_1_ROW_VEC_MAIN) Register
 *  No description provided (USE_CASE_0_ROI_1_ROW_VEC_MAIN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_MAIN_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_MAIN_s {
    union {
        struct {
             unsigned int BITS16_1         : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROW_VEC_MAIN_s;
%}

/*@}*/

/** @defgroup USE_CASE_0_ROI_1_COLUMN_VEC No description provided (USE_CASE_0_ROI_1_COLUMN_VEC) Register
 *  No description provided (USE_CASE_0_ROI_1_COLUMN_VEC) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_COLUMN_VEC_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_COLUMN_VEC_s {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_COLUMN_VEC_s;
%}

/*@}*/

/** @defgroup USE_CASE_0_ROI_1_ROI_TYPE No description provided (USE_CASE_0_ROI_1_ROI_TYPE) Register
 *  No description provided (USE_CASE_0_ROI_1_ROI_TYPE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROI_TYPE_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROI_TYPE_s {
    union {
        struct {
             unsigned int BINSS            : 4; /**< No description provided */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_1_ROI_TYPE_s;
%}

/*@}*/

/** @defgroup USE_CASE_0_ROI_2_ROW_VEC_TOPBOT No description provided (USE_CASE_0_ROI_2_ROW_VEC_TOPBOT) Register
 *  No description provided (USE_CASE_0_ROI_2_ROW_VEC_TOPBOT) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_TOPBOT_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_TOPBOT_s {
    union {
        struct {
             unsigned int BITS17_AND_0     : 2; /**< No description provided */
             unsigned int RESERVED2        : 14; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_TOPBOT_s;
%}

/*@}*/

/** @defgroup USE_CASE_0_ROI_2_ROW_VEC_MAIN No description provided (USE_CASE_0_ROI_2_ROW_VEC_MAIN) Register
 *  No description provided (USE_CASE_0_ROI_2_ROW_VEC_MAIN) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_MAIN_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_MAIN_s {
    union {
        struct {
             unsigned int BITS16_1         : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROW_VEC_MAIN_s;
%}

/*@}*/

/** @defgroup USE_CASE_0_ROI_2_COLUMN_VEC No description provided (USE_CASE_0_ROI_2_COLUMN_VEC) Register
 *  No description provided (USE_CASE_0_ROI_2_COLUMN_VEC) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_COLUMN_VEC_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_COLUMN_VEC_s {
    union {
        struct {
             unsigned int BITS15_0         : 16; /**< No description provided */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_COLUMN_VEC_s;
%}

/*@}*/

/** @defgroup USE_CASE_0_ROI_2_ROI_TYPE No description provided (USE_CASE_0_ROI_2_ROI_TYPE) Register
 *  No description provided (USE_CASE_0_ROI_2_ROI_TYPE) Register
 *  @{
 */

/*! ========================================================================
 *  \struct ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROI_TYPE_s
 *  \brief No description provided
 *  ======================================================================== */
%inline %{
typedef struct _ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROI_TYPE_s {
    union {
        struct {
             unsigned int BINSS            : 4; /**< No description provided */
             unsigned int RESERVED4        : 12; /**< Reserved */
        };
        u16 VALUE16;
    };
} ADI_DE_REGS_YODA_USE_CASE_0_ROI_2_ROI_TYPE_s;
%}

/**   
 * @brief Error code definitions.
 *
 *  These error codes are used in function return values.
 *
 */
%inline %{
typedef enum {
    ADI_NO_ERROR = 0,                           /**< No error. */
    ADI_JSON_FILE_NOT_FOUND  = 0x0ad10001,      /**< JSON file not found error. */
    ADI_JSON_FILE_OPEN_ERROR  = 0x0ad10002,     /**< JSON file open error. */
    ADI_JSON_PARSE_ERROR = 0x0ad10003,          /**< JSON parsing error. */
    ADI_JSON_UNEXPECTED_KEY = 0x0ad10004,       /**< Unexpected JSON Key. */
    ADI_UNEXPECTED_SPI_BYTE_COUNT = 0x0ad10005, /**< Unexpected SPI count. */ 
    ADI_FILE_NOT_FOUND = 0x0ad10006,            /**< File not found. */ 
    ADI_SPI_DRIVER_ERROR = 0x0ad10007,          /**< SPI Driver Error. */ 
    ADI_SPI_XFER_ERROR = 0x0ad10008,            /**< SPI Transfer Error. */ 
    ADI_SPI_BIT_RATE_ERROR = 0x0ad10009,        /**< Unexpected SPI Bit Rate. */ 
    ADI_ERROR_CODE_MISSING = 0x0ad1000a,        /**< Missing error code. */  
    ADI_UNEXPECTED_ARGS = 0x0ad1000b,           /**< Unexpected arguments. */  
    ADI_H2S_VALID_TIMEOUT = 0x0ad1000c,         /**< H2s valid timeout. */
    ADI_S2H_NOT_VALID_TIMEOUT = 0x0ad1000d,     /**< S2H not valid timeout. */
    ADI_H2S_ERROR = 0x0ad1000e,                 /**< H2S_MBX_INSTS.H2S_ERR_BIT. */
    ADI_MISCOMPARE_ERROR = 0x0ad1000f,          /**< Data miscompare. */
    ADI_FILE_FORMAT_ERROR = 0x0ad10010,         /**< File format error. */
    ADI_UNEXPECTED_PIN_MODE = 0x0ad10011,       /**< Unexpected pin_mode specified in call to adi_reset_newton. */
    ADI_WIRINGPI_ERROR = 0x0ad10012             /**< Error return from . */
} adi_errorCodes_e;
%}

#define ERROR_CODE_BASE 0x0ad10000
%inline %{
static const char* adi_errorMessages[28] = {
    "No error.",
    "JSON file not found error.",
    "JSON file open error.",
    "JSON parsing error.",
    "JSON unexpected Key.",
    "Unexpected SPI count.",
    "File not found.",
    "SPI Driver Error.",
    "SPI Transfer Error.",
    "Unexpected SPI Bit Rate.",
    "Missing error code.",
    "Unexpected arguments.",
    "H2S valid timeout.",
    "S2H not valid timeout.",
    "H2S error.",
    "Data miscompare.",
    "File format error.",
    "Unexpected Pin Mode.",
    "WiringPi setup error.",
};
%}

/**
 * @brief Enums for specifying the data type codes
 *
 * Set / Get Parameter data type codes.
 *
 */
%inline %{
typedef enum {
    DTYPE_INT32 = 0,
    DTYPE_UINT32,
    DTYPE_FLOAT
} adi_data_type_codes_e;
%}

// These are the valid SPI Clock Rates
/**
 * @brief Enums for specifying valid SPI Clock Rates
 *
 * Valid SPI Clock Rates.
 *
 */
%inline %{
enum spiBitRates {
    SPI_BIT_RATE_1M  =  1000, // 1 MHz
    SPI_BIT_RATE_2M  =  2000, // 2 MHz
    SPI_BIT_RATE_4M  =  4000, // 4 MHz
    SPI_BIT_RATE_8M  =  8000, // 8 MHz
    SPI_BIT_RATE_12M = 12000, // 12 MHz
    SPI_BIT_RATE_16M = 16000  // 16 MHz
} spiBitRates_e;
%}
#define BIT_RATE_COUNT 6


//==============================================================================
// STRUCTS
//==============================================================================

#ifndef PYTHON
    static str_parms* newton_parms;
#endif

%apply u32 *OUTPUT {u32 *dataReadPtr32};
%apply u32 *OUTPUT {u32 *param_value};
%apply u32 *OUTPUT {u32 *param_address};

// typemap for an incoming data buffer from Newton
%typemap(in) (u08 *dataReadPtr08, int dataLength) {
   if (!PyInt_Check($input)) {
       PyErr_SetString(PyExc_ValueError, "Expecting an integer");
       return NULL;
   }
   $2 = PyInt_AsLong($input);
   if ($2 < 0) {
       PyErr_SetString(PyExc_ValueError, "Positive integer expected");
       return NULL;
   }
   $1 = (u08 *) malloc($2*4+6);
}

// Return the buffer.  Discarding any previous return result
%typemap(argout) (u08 *dataReadPtr08,  int dataLength) {
   int i;
   Py_XDECREF($result);   /* Blow away any previous result */
   if (result < 0) {      /* Check for I/O error */
       free($1);
       PyErr_SetFromErrno(PyExc_IOError);
       return NULL;
   }
   $result = PyList_New($2);
   for (i = 0; i < $2; i++) {
       PyList_SetItem($result,i, PyInt_FromLong($1[i]));
   }
   free($1);
}

//==============================================================================
// FUNCTIONS
//==============================================================================

/**
 * @brief Check the 1SP completion code.
 * 
 * This task checks the 1SP completion code.
 *
 * @return status value. Indicates success or failure of the function.
 *
 */
int adi_check_done_code( );

/**
 * @brief Read a 16-bit word from the Newton over the SPI Interface
 *
 * @param address the address of the 32-bit word.
 * @return read data.
 *
 */
u16 adi_spi_read_word_py( u16 address );

/**
 * @brief Write a 16-bit word to the Newton over the SPI Interface
 *
 * @param bytes_out the number of bytes to be sent over the SPI interface to the Newton.
 * @param data_out the data to be sent over the SPI interface to the Newton.
 * @param bytes_in the number of bytes to be recevied from the Newton over the SPI interface.
 * @param data_in the data to be recevied from the Newton over the SPI interface.
 * @return status value. Indicates success or failure of the function.
 *
 */
int adi_spi_write_word( u16 address, u16 data );

/**
 * @brief Dump Error Log.
 * 
 * This task reads and displays the HSP error log..
 *
 * @return status value. Indicates success or failure of the function.
 *
 */
int adi_dump_error_log( );

/**
 * @brief Perform register write through HSP.
 * 
 * This task performs register write through HSP.
 *
 * @param addr Write address.
 * @param wr_data Write Data.
 * @return status value. Indicates success or failure of the function.
 *
 */
int adi_write_register( u16 addr, u16 wr_data );

/**
 * @brief Perform register write bypassing HSP.
 * 
 * This task performs register write bypassing HSP.
 *
 * @param addr Write address.
 * @param wr_data Write Data.
 * @return status value. Indicates success or failure of the function.
 *
 */
int adi_write_register_backdoor( u16 addr, u16 data );

/**
 * @brief Perform register read through HSP.
 * 
 * This task performs register read through HSP.
 *
 * @param addr Read address.
 * @return read data
 *
 */
u16 adi_read_register_py( u16 addr );

/**
 * @brief Perform register read through HSP.
 * 
 * This task performs register read through HSP.
 *
 * @param addr Read address.
 * @param data Expected Data.
 * @return read data
 *
 */
u16 adi_check_register_py( u16 addr, u16 data );

/**
 * @brief Perform register write bypassing the HSP.
 * 
 * This task performs register write bypassing HSP.
 *
 * @param addr Write address.
 * @return read data
 *
 */
u16 adi_read_register_backdoor_py( u16 addr );

/**
 * @brief Load command file.
 *
 * @param fileName name of the file containing the Newton RAM to be loaded. 
 * @return status value. Indicates success or failure of the function.
 *
 */
int adi_load_command_file( const char *fileName );

/**
 * @brief Load register file.
 *
 * This function performs the following:
 * 
 * - Write the registers listed in the specified file with the write data
 *   contained in the file.
 * 
 * @param fileName name of the file containing register address / data pairs. 
 * @return status value. Indicates success or failure of the function.
 *
 */
int adi_load_register_file( const char *fileName );

/**
 * @brief erifies that the RAMs match the memory images contained in the soecifide file.
 *
 * This function performs the following:
 * 
 * - Verifies that the RAMs match the memory images contained in the specified file.
 *
 * Operation:
 *    - Read the file into an array.
 *    - Backdoor reads the contents of the specified RAM.
 *
 * @param fileName name of the file containing the mail box command.
 * @return status value. Indicates success or failure of the function.
 *
 */
int adi_verify_command_file( const char *fileName );
int adi_verify_command_file_hsp( const char *fileName );

/**
 * @brief Load the memory image contained in the specified file into the specified HSP memory.
 *
 * This function performs the following:
 * 
 * - Loads the memory image contained in the specified file into specified HSP memory.
 *   This function is only valid for the newton FPGA.
 *
 * @param loadTarget HSP memory to load. 
 * @param fileName name of the file containing the HSP ROM to be loaded. 
 * @return status value. Indicates success or failure of the function.
 *
 */
int adi_load_hsp( adi_loadTargets_e loadTarget, char *fileName );

/**
 * @brief Verify that the memory image contained in the specified file matches the containt of the specified HSP memory.
 *
 * This function performs the following:
 * 
 * - Verifies that the memory image contained in the specified file matches the contents of the specified HSP memory.
 *   This function is only valid for the newton FPGA.
 *
 * @param verifyTarget HSP memory to verify. 
 * @param fileName name of the file containing the HSP ROM to be verified. 
 * @return status value. Indicates success or failure of the function.
 *
 */
int adi_verify_hsp( adi_loadTargets_e verifyTarget, const char *fileName );

/**
 * @brief Unload the memory image to the specified file from the specified HSP memory.
 *
 * This function performs the following:
 * 
 * - Unloads the memory to the specified file from specified HSP memory.
 *   This function is only valid for the newton FPGA.
 *
 * @param unloadTarget HSP memory to unload. 
 * @param fileName name of the file containing the HSP ROM to be loaded. 
 * @return status value. Indicates success or failure of the function.
 *
 */
int adi_unload_hsp( adi_loadTargets_e unloadTarget, const char *fileName );

/**
 * @brief Issue a soft reset to the newton.
 *
 * @param channel_id the channel ID of the to be written.
 * @return status value. Indicates success or failure of the function.
 *
 */
int  adi_soft_reset( );

/**
 * @brief Issue a soft reset to the HSP hardware
 *
 * @return status value. Indicates success or failure of the function.
 *
 */
int  adi_reset_hsp( );

/**
 * @brief Issue a hard reset to newton
 *
 * @param pin_mode Newton Pin Mode. 
 * @return status value. Indicates success or failure of the function.
 *
 */
int  adi_reset_newton( adi_pin_modes_e pin_mode );

/**
 * @brief Test the micro-sequencer sequence memory.
 *
 * @return status value. Indicates success or failure of the function.
 *
 */
int adi_test_useq_ram ( );

/**
 * @brief Configure the newton control program.
 *
 * @param bitRateOverride use this SPI bit rate instead of the default.
 * @return status value. Indicates success or failure of the function.
 *
 */
int  adi_newton_config( int bitRateOverride );

/**
 * @brief Return error message string for given error code.
 *
 * @param errorCode error code value.
 * @return error message string.
 *
 */
char* adi_error_msg( int returnCode );

/**
 * @brief Configure (input or output) the specified Newton GPIO.
 *
 * @param gpio the number of the Newton GPIO to be set
 * @param direction the direction (0 = input, 1 = output) that the Newton GPIO is to be configured for.
 * @return status value. Indicates success or failure of the function.
 *
 */
int  adi_configure_newton_gpio( int gpio, int direction );

/**
 * @brief Set Newton GPIO to the specified value.
 *
 * @param gpio the number of the Newton GPIO to be set
 * @param value the value that the Newton GPIO is to be set to.
 * @return status value. Indicates success or failure of the function.
 *
 */
int  adi_set_newton_gpio( int gpio, int value );

/**
 * @brief Get Newton GPIO value.
 *
 * @param gpio the number of the Newton GPIO to be set
 * @return value. Value of the selected GPIO pin.
 *
 */
int  adi_get_newton_gpio( int gpio );

/**
 * @brief Toggle (change state of) Newton GPIO
 *
 * @param gpio the number of the Newton GPIO to be toggled
 * @return status value. Indicates success or failure of the function.
 *
 */
int  adi_toggle_newton_gpio( int gpio );

/**
 * @brief Pulse Newton GPIO for the specified pulse width.
 *
 * @param gpio the number of the Newton GPIO to be set
 * @param width the pulse width in microseconds.
 * @return status value. Indicates success or failure of the function.
 *
 */
int  adi_pulse_newton_gpio( int gpio, int width );

#endif // _NEWTON_CONTROL_H_
