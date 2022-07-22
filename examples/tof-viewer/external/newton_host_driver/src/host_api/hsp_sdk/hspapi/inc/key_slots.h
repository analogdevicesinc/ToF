/**
 * @file key_slots.h
 * @brief Header for Analog Devices platform specific key slot definitions
 */

#ifndef __KEY_SLOTS_H
#define __KEY_SLOTS_H

/**
 * @brief KSU key slots
 */
typedef enum
{
    KSU_KEY_SLOT_DEVICE_KEY             = 0,    /**< Device key */
    KSU_KEY_SLOT_GLOBAL_KEY             = 1,    /**< Global key */
    KSU_KEY_SLOT_DERIVED_KEY            = 2,    /**< Temporary key derived from other keys (e.g. for 1SP decryption) */
    KSU_KEY_SLOT_DECRYPTION_KEY         = 3,    /**< ADI decryption key */
    KSU_KEY_SLOT_ECC_SIGN               = 4,    /**< ECC private key for Attestation */
    KSU_KEY_SLOT_ECDH                   = 5,    /**< ECC private key for ECDH */
    KSU_KEY_SLOT_VER_INDEPENDENT_DEVICE = 6,    /**< Version independent device key */
    KSU_KEY_SLOT_VER_INDEPENDENT_GLOBAL = 7,    /**< Version independent global key */

    KSU_KEY_SLOT_FIRST = KSU_KEY_SLOT_DEVICE_KEY,              /**< First valid key slot, used for parameter validation */
    KSU_KEY_SLOT_LAST = KSU_KEY_SLOT_VER_INDEPENDENT_GLOBAL    /**< Last valid key slot, used for parameter validation */
} Ksu_Key_Slot_t;

/**
 * @brief KSU PCR slots
 */
typedef enum
{
    KSU_PCR_SLOT_0  = 0,   /**< Platform Configuration Register 0 */
    KSU_PCR_SLOT_1  = 1,   /**< Platform Configuration Register 1 */

    KSU_PCR_SLOT_FIRST = KSU_PCR_SLOT_0,    /**< First valid PCR slot, used for parameter validation */
    KSU_PCR_SLOT_LAST = KSU_PCR_SLOT_1      /**< Last valid PCR slot, used for parameter validation */
} Ksu_Pcr_Slot_t;

#endif //__KEY_SLOTS_H
