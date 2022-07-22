/**
 * @file platform_postcodes.h
 * @brief Post codes - Analog Devices platform specific
 */

#ifndef __PLATFORM_POSTCODES_H
#define __PLATFORM_POSTCODES_H

#include "utils/generic/generic.h"

#define BOOT_STAGE_OFFSET   28
#define PLAT_OFFSET         24
#define HW_BLOCK_OFFSET     16

#define HSP_BOOT_STAGE_0SP (0 << BOOT_STAGE_OFFSET)
#define HSP_BOOT_STAGE_1SP (1 << BOOT_STAGE_OFFSET)
#define HSP_BOOT_STAGE_2SP (2 << BOOT_STAGE_OFFSET)

#define ANALOG_DEVICES_PLAT (1 << PLAT_OFFSET)

#define HSP_PLATFORM ANALOG_DEVICES_PLAT

#define HW_GENERIC      (1 << HW_BLOCK_OFFSET)
#define HW_FUSE_CTRL    (2 << HW_BLOCK_OFFSET)
#define HW_CRYPTO       (3 << HW_BLOCK_OFFSET)
#define HW_MBOX         (4 << HW_BLOCK_OFFSET)     
#define HW_PMP          (5 << HW_BLOCK_OFFSET)
#define HW_VENDOR_HW    (6 << HW_BLOCK_OFFSET)


/**
 * @brief Post code values
 */
typedef enum
{
    // System Status
    HW_INIT_DONE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_GENERIC) | 0x0 ),                       /**< HW initializing */
    CRITICAL_ERR = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_GENERIC) | 0x1 ),                       /**< Critical Error*/
    HALT = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_GENERIC) | 0x2 ),                               /**< System halt */
    RUN_RESET = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_GENERIC) | 0x3 ),                          /**< Resetting system */
    RUN_TEST_STATE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_GENERIC) | 0x4 ),                     /**< HSP in Test state flow */
    RUN_RECOVERY = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_GENERIC) | 0x5 ),                       /**< Running recovery */
    NO_RECOVERY = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_GENERIC) | 0x6 ),                        /**< Unable to recover */
    JUMP_TO_1SP = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_GENERIC) | 0x7 ),                        /**< Jumping to 1SP */
    RUN_PROD_STATE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_GENERIC) | 0x8 ),                     /**< HSP in Production state flow */
    RUN_SECURED_STATE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_GENERIC) | 0x9 ),                  /**< HSP in Secured state flow */
    SET_OPERATING_MODE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_GENERIC) | 0xA ),                 /**< Setting operating mode of HSP */
    PLL_BIT_SET = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_GENERIC) | 0xB ),                        /**< PLL register bit[10] is set */

    // Fuse CTRL post codes         
    WAIT_FCTRL_DONE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_FUSE_CTRL) | 0x0 ),                  /**< Waiting for FCTRL to be done */
    SECURITY_STATE_READ = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_FUSE_CTRL) | 0x1 ),              /**< Read Security State */
    
    // Crypto
    CRYPTO_HW_INIT_DONE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_CRYPTO) | 0x0 ),                 /**< Done initializing crypto hardware */
    CRYPTO_INITIALIZE_CRYPTO_KEYS_DONE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_CRYPTO) | 0x1 ),  /**< Initializing Crypto Keys complete */
    CRYPTO_RNG_WAIT_START = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_CRYPTO) | 0x2 ),               /**< Starting wait for RNG initialization to complete */
    CRYPTO_RNG_WAIT_DONE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_CRYPTO) | 0x3 ),                /**< Done waiting for RNG initialization to complete */

    // Mailbox Status
    MBX_LOOP_ENTERED = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x0 ),                      /**< Mailbox loop entered */
    MBX_H2S_SET_ERR_BIT = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x1 ),                   /**< Mailbox set h2s error bit */
    MBX_WFI_MODE_ENTERED = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x2 ),                  /**< Mailbox in WFI mode */
    MBX_GET_MBX_CMD = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x3 ),                       /**< Getting Mailbox Cmd Header */
    MBX_GET_MBX_ERROR_LOG_CMD = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x4 ),             /**< Getting Mailbox Cmd Header inside error loop */
    MBX_PROCCESS_SEQ_LOOKUP = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x5 ),               /**< Processing Mailbox Lookup Sequence */
    MBX_CHECK_MBX_PKT = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x6 ),                     /**< Validating Mailbox Cmd Header */
    MBX_VALIDATE_MBX_ATTR = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x7 ),                 /**< Validate Mailbox Cmd Attributes */    
    MBX_PROCESS_CMD = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x8 ),                       /**< Processing Mailbox Cmd; Entering Cmd handling switch case*/
    MBX_CMD_OPERATING_MODE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x9 ),                /**< Processing Mailbox Operating Mode Cmd */
    MBX_CMD_GROUPED_DATA = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0xA ),                  /**< Processing Mailbox Grouped Data Cmd */
    MBX_CMD_RAM_DATA = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0xB ),                      /**< Processing Mailbox Ram Data Cmd */
    MBX_CMD_REG_CFG_DATA = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0xC ),                  /**< Processing Mailbox Reg Cfg Cmd */
    MBX_CMD_1SP_IMAGE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0xD ),                     /**< Processing Mailbox 1SP Image Cmd */
    MBX_CMD_SIGNATURE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0xE ),                     /**< Processing Mailbox Signature Cmd */
    MBX_CMD_PUB_KEY = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0xF ),                       /**< Processing Mailbox Public Key Cmd */
    MBX_CMD_INVALID = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x10 ),                      /**< Mailbox Cmd Invalid*/
    MBX_AUTH_PUB_KEY = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x11 ),                     /**< Mailbox Authenticating Public Keys */
    MBX_GROUPED_PAYLOAD_POST_PROCESS = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x12 ),     /**< Grouped Payload Post Processing*/
    MBX_INDIVIDUAL_PAYLOAD_POST_PROCESS = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x13 ),  /**< Individual Payload Post Processing*/
    MBX_SIGN_VERIF_FAIL = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x14 ),                  /**< Mailbox Signature Verification Fail */
    MBX_SIGN_VERIF_PASS = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x15 ),                  /**< Mailbox Signature Verification Pass */
    MBX_RECEIVED_SIG = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x16 ),                     /**< Mailbox Received Signature */
    MBX_RECEIVED_UNSIGNED_1SP = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x17 ),            /**< Mailbox Received Unsigned 1SP */    
    MBX_RECEIVED_GROUPED_UNSIGNED_1SP = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x18 ),    /**< Mailbox Received Grouped Unsigned 1SP */    
    MBX_RECEIVED_SIGNED_1SP = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x19 ),              /**< Mailbox Received Signed 1SP */
    MBX_POST_PROCESSING_COMPLETE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x1A ),         /**< Mailbox Post Processing Complete */
    MBX_SHA_GROUPED_START_NEW = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_CRYPTO) | 0x1B ),          /**< Mailbox SHA start new for grouped payload */
    MBX_SHA_GROUPED_WAIT_COMPLETE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x1C ),        /**< Mailbox SHA wait complete for grouped payload */
    MBX_SHA_INDIV_START_NEW = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_CRYPTO) | 0x1D ),            /**< Mailbox SHA start new for individual payload */
    MBX_SHA_INDIV_WAIT_COMPLETE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x1E ),          /**< Mailbox SHA wait complete for individual payload */
    MBX_ERROR_LOOP = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x1F ),                       /**< Entered Mailbox Error loop */

    // 1SP related post codes
    MBX_DECRYPTING_GENERIC_1SP = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x20 ),           /**< Decrypting Generic 1SP */
    MBX_DECRYPTING_MSFT_1SP = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x21 ),              /**< Decrypting MSFT 1SP */
    MBX_MSFT_1SP_GOT_CEK = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x22 ),                 /**< Received CEK for MSFT 1SP */
    MBX_MSFT_1SP_DECRYPT_HEADER = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x23 ),          /**< Decrypt header for MSFT 1SP */
    MBX_MSFT_1SP_DECRYPT_BODY = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x24 ),            /**< Decrypt body for MSFT 1SP */
    MBX_MSFT_1SP_DECRYPT_DONE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x25 ),            /**< Finished decrypting MSFT 1SP */
    
    MBX_ADI_1SP_DECRYPT_BODY = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x26 ),             /**< Decrypt body for ADI 1SP */
    MBX_ADI_1SP_DECRYPT_DONE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x27 ),             /**< Finished decrypting ADI 1SP */

    MBX_PRE_JUMP_1SP_AEB_CONFIG_DONE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x28 ),             /**< Finished Configuring AEBs before jumping to 1SP */
    MBX_PRE_JUMP_1SP_WIPE_CRYPTO_SECRETS_DONE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_MBOX) | 0x29 ),    /**< Finished wiping crypto secrets before jumping to 1SP */

    // Vendor Hardware related post codes
    VENDOR_HW_READ_DECRYPTION_KEYS = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_VENDOR_HW) | 0x1 ),       /**< Vendor HW read decryption keys from fuses */
    VENDOR_HW_INIT_DONE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_VENDOR_HW) | 0x2 ),                  /**< Vendor HW init done */
    VENDOR_AUTH_PUB_KEYS_SHA_COMPLETE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_VENDOR_HW) | 0x3 ),    /**< Vendor Authorize Public Keys SHA wait completed */
    VENDOR_AUTH_PUB_KEYS_DONE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_VENDOR_HW) | 0x4 ),            /**< Vendor Authorize Public Keys done */

    // PMP related post codes
    PMP_0SP_CONFIG_DONE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_PMP) | 0x0 ),    /**< Configured PMP in 0SP */
    PMP_1SP_CONFIG_DONE = ((HSP_BOOT_STAGE_0SP) | (HSP_PLATFORM) | (HW_PMP) | 0x1 ),    /**< Configured PMP in 1SP */

} Hsp_post_code_t;

#endif //__PLATFORM_POSTCODES_H
