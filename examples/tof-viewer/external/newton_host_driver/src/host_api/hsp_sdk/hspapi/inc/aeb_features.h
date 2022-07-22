/**
 * @file aeb_features.h
 * @brief Header for Analog Devices platform specific AEB definitions
 */

#ifndef __AEB_FEATURES_H
#define __AEB_FEATURES_H

/**
 * @brief Feature of Access Enable Block that can be toggled
 */
typedef enum
{
    AEB_000_SP_ROM_PARTITION_SEL = 0,   /**< ROM: SPROM secure (1) vs. unsecure (0) partition selection */
    AEB_001_ALLOW_EXEC_AXI_TRANS = 1,   /**< Intercepts all executable AXI transactions from SP to HSP bus (and the rest of SOCs) */
    AEB_002_SP_DBGTAP_EN = 2,           /**< JTAG: SW Debug for HSP's SP cores */
    AEB_003_RNG_DAS_EN = 3,             /**< RNG: Allow RNG DAS mode for raw TRBG collection */
    AEB_004_BLK_HWCHKPT_LOCKS = 4,      /**< HWCHKPT: Turn-off HSP HW checkpointing (hashing) feature locks */
    AEB_005_HSP_DFT_EN = 5,             /**< En/Dis JTAG/Scan chain of the HSP (include MBIST, Mdump, todo: list out all DFT tests) */
    AEB_006_TOP_DFT_EN = 6,             /**< En/dis Master TAP controller for SCANDUMP/DEBUG/IOCONTROL */
    AEB_007_FUSE_DFT_EN = 7,            /**< JTAG: Enable Fuse Macro DFT Commands from TDR */
    AEB_008_SS128_TDR_EN = 8,           /**< JTAG: TDR read access to 128-bit security state register */
    AEB_009_MEM_CHK_DISABLE = 9,        /**< MEM: 1) Disable EDC checks on RAMs.   2) Disable PARITY checks on TCM */
    AEB_010_ALLOW_EDC_ERR_INJECT = 10,  /**< MEM: Allow EDC error injection */
    AEB_011_AEBFUSE_PROG_EN = 11,       /**< FCTRL: Program enable for AEB  Fuse */
    AEB_012_SOCID_PROG_EN = 12,         /**< FCTRL: Program enable for the N_SOC_ID  */
    AEB_013_KEY_PROG_EN = 13,           /**< FCTRL: Program enable for the entire key region  */
    AEB_014_RSVD0_PROG_EN = 14,         /**< FCTRL: Program enable for RESERVED0 region */
    AEB_015_UART_EN = 15,               /**< UART: ability to send and receive UART messages */
    AEB_016_RSVD1_PROG_EN = 16,         /**< FCTRL: Program enable for all RESERVED1 region */
    AEB_017_KEY_SENSE_EN = 17,          /**< FCTRL: Sense enable for the Key fuses */
    AEB_018_OBS_EN = 18,                /**< FCTRL: Obs enable */
    AEB_019_CPU_BOOT_EN = 19,           /**< Enable CPU Boot Control bit CPUBOOTEN */
    AEB_020_SP_EN = 20,                 /**< allow SP to run */
    AEB_021_ALLOW_TDR_RESET = 21,       /**< Allow HSP to be Reset via the tdr_hsp_reset register */
    AEB_022_FATAL_RSTMSK_WEN = 22,      /**< HSP Fatal Error Mask Register (Sticky/Non-Sticky) Write Enable */
    AEB_023_MPU_PROTECTION_DIS = 23,    /**< Disable MPU protection */
    AEB_024_MST_DBG = 24,               /**< Enable DBGU : through config reg IF gating */
    AEB_025_UNUSED = 25,                /**< SPARE: HSP Only */
    AEB_026_UNUSED = 26,                /**< SPARE: HSP Only */
    AEB_027_ADI_DBG = 27,               /**< ADI Debug */
    AEB_028_UNUSED = 28,                /**< SPARE: HSP Only */
    AEB_029_UNUSED = 29,                /**< SPARE: HSP Only */
    AEB_030_UNUSED = 30,                /**< SPARE: HSP Only */
    AEB_031_A0BYPASS = 31,              /**< A0_BYPASS Status Output */

    AEB_FEATURE_FIRST = AEB_000_SP_ROM_PARTITION_SEL,   /**< First AEB bit, used for parameter validation */
    AEB_FEATURE_LAST = AEB_031_A0BYPASS                 /**< Last AEB bit, used for parameter validation */
} Aeb_Feature_t;

#endif //__AEB_FEATURES_H
