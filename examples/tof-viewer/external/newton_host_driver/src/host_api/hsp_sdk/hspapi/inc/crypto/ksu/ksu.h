/**
 * @file ksu.h
 * @brief Header file for KSU engine block
 */

#ifndef __KSU_H
#define __KSU_H

#include "utils/generic/generic.h"
#include "key_slots.h"   // Platform specific, determined by build

// NOTE: Ksu_Key_Slot_t and Ksu_Pcr_Slot_t enums are platform specific and therefore not included here

/**
 * @brief KSU key attributes
 */
typedef enum
{
    IsDeviceSecret          = 0x000001,    /**< This attribute indicates this key is unique and secret to an individual SHACK instance. There should be no way for any party other than the one SHACK instance to have knowledge of this key. */
    AesEncryptAllowed       = 0x000002,    /**< AES Encrypt operations are allowed on this key if this attribute bit is set */
    AesDecryptAllowed       = 0x000004,    /**< AES Decrypt operations are allowed on this key if this attribute bit is set */
    Aes128bitAllowed        = 0x000008,    /**< Each key slot always holds 256 bits of key material. When this attribute is not set to 1, the full 256 bits must be used as key to perform AES operations. When this bit is set to 1, first 128 bits of the key can be used as the key for AES 128-bit operations. This attribute is only meaningful when either AESEncryptAllowed or AESDecryptAllowed is set. */
    AesXtsOnly              = 0x000010,    /**< If this bit is set, the key can only be used to perform AES operations in AES-XTS mode. No other AES modes are allowed. If this bit is not set, then all AES modes are allowed. */
    SendKeyAllowed          = 0x000020,    /**< The SendKey operation is allowed on this key if this attribute bit is set */
    LoadKeyAllowed          = 0x000040,    /**< The LoadKey operation is allowed on this key if this attribute bit is set */
    DecryptLegacyKeyAllowed = 0x000080,    /**< The DecryptLegacyKey operation is allowed on this key if this attribute bit is set */
    StoreKeyAllowed         = 0x000100,    /**< The StoreKey operation is allowed on this key if this attribute bit is set */
    SaveKeyAllowed          = 0x000200,    /**< The SaveKey operation is allowed on this key if this attribute bit is set */
    KdfKeyAllowed           = 0x000400,    /**< A KDFKey operation is allowed on this key if this attribute bit is set */
    KdfPcrAllowed           = 0x000800,    /**< A KDFPCR or KDFAsPCR operation is allowed on this key with any PCR slot if this attribute bit is set */
    EccSignAllowed          = 0x001000,    /**< This key can be used as an ECC Private key to perform the ECCSign operation */
    EcdhAllowed             = 0x002000,    /**< This key can be used as an ECC Private key to perform the ECDHKeyExchange, or ECDHPCRKeyExchange operations */
    MustAppendPcr           = 0x004000     /**< When this attribute bit is set, the ECCSign operation must include a PCR into the calculation of the digest that is signed. Which PCR is used is specified as an input parameter to ECCSignxx. */
} Ksu_Attribute_t;

/**
 * @brief Initialize the KSU block
 * 
 * \b Description:
 *      Initializes the KSU block.
 *      This basically just involves waiting for the hardware to come out of reset.
 * 
 * @param void
 * 
 * @return void
 */
void KsuInit(void);

//
// Helper functions

static inline bool KsuIsValidKeySlot(Ksu_Key_Slot_t slot)
{
    return (slot >= KSU_KEY_SLOT_FIRST && slot <= KSU_KEY_SLOT_LAST);
}

static inline bool KsuIsValidPcrSlot(Ksu_Pcr_Slot_t slot)
{
    return (slot >= KSU_PCR_SLOT_FIRST && slot <= KSU_PCR_SLOT_LAST);
}

static inline bool KsuIsValidAttributes(uint32_t attributes)
{
    // Attributes are 24-bits
    return (attributes <= 0x00FFFFFF);
}

#endif /* __KSU_H */
