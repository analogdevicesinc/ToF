/**
 * @file shack.h
 * @brief Header file for SHACK functions
 */

#ifndef __SHACK_H
#define __SHACK_H

#include "utils/generic/generic.h"
#include "crypto/ccs/ccs.h"
#include "crypto/ksu/ksu.h"
#include "crypto/pka/pka.h"

/**
 * @brief Set a key value into a key slot along with supplied attributes
 * 
 * \b Description:
 *      Set a key value into a key slot along with supplied attributes. The CCS expects IsDeviceSecret to be zero on
 *      input attributes, if it is not, it will silently clear it to zero before attributes are written to the KSU.
 *      (Note that a nonzero input value for the IsDeviceSecret attribute is not an error.)
 * 
 * @param target Destination key slot
 * 
 * @param keyData Address of 256-bit key
 * 
 * @param attributes Attributes
 * 
 * @return void
 */
void ShackSetKey(Ksu_Key_Slot_t target, ConstPtr_Msg256_t keyData, uint32_t attributes);

/**
 * @brief Clear the data in a key slot
 * 
 * \b Description:
 *      Clears the data in a key slot by setting it to all zero, including attributes.
 * 
 * @param target Destination key slot
 * 
 * @return void
 */
void ShackClearKey(Ksu_Key_Slot_t target);

/**
 * @brief Generate a random key and place it in a key slot along with supplied attributes
 * 
 * \b Description:
 *      Generate a random key and place it in a key slot along with supplied attributes.
 * 
 * @param target Destination key slot
 * 
 * @param attributes Attributes
 * 
 * @return void
 */
void ShackGenRandomKey(Ksu_Key_Slot_t target, uint32_t attributes);

/**
 * @brief Send a key through a private hardware channel to another silicon based crypto accelerator on the same die 
 * 
 * \b Description:
 *      Sends a key through a private hardware channel to another silicon based crypto accelerator on the same die
 *      (e.g. SCP). Both the 256-bit key, and the 24 bits of attribute are sent over the private key interface.
 * 
 * @param target Destination key address
 * 
 * @param source Source key slot
 * 
 * @return void
 */
void ShackSendKey(void* target, Ksu_Key_Slot_t source);

/**
 * @brief Load a key into a key slot
 * 
 * \b Description:
 *      Takes a key blob which contains a 256-bit key, associated attributes, and 64-bit Initial Value (IV) value;
 *      decrypts and verifies the IV value. If the integrity check passes, the decrypted key and its attributes are
 *      loaded into the destination key slot.
 * 
 * @param target Destination key slot
 * 
 * @param decryptor Decryption key slot
 * 
 * @param keyBlob Key blob address
 * 
 * @return void
 */
void ShackLoadKey(Ksu_Key_Slot_t target, Ksu_Key_Slot_t decryptor, ConstPtr_Msg384_t keyBlob);

/**
 * @brief Decrypts and loads a legacy key into a key slot
 * 
 * \b Description:
 *      This operation mimics the KR_D_Decrypt operation on HSP 1.0 and is needed to support legacy applications.
 *      It takes an AES encrypted 256-bit key blob, decrypts it using the specified key register and loads the
 *      resulting key and into a key register. The attributes on the destination key will default to zeros except
 *      the following five attributes that will be copied from the input attribute parameter: SendKeyAllowed,
 *      AESEncryptAllowed, AESDecryptAllowed, AES128bitAllowed, and AESXTSOnly.
 * 
 * @param target Destination key slot
 * 
 * @param decryptor Decryption key slot
 * 
 * @param keyBlob Legacy key blob address
 * 
 * @param attributes Attributes
 * 
 * @return void
 */
void DecryptLegacyKey(Ksu_Key_Slot_t target, Ksu_Key_Slot_t decryptor, ConstPtr_Msg384_t keyBlob, uint32_t attributes);

/**
 * @brief Stores a key into a key blob
 * 
 * \b Description:
 *      Takes a 256-bit key value, and associated 24-bit attribute as input, and uses the specified encryption key slot
 *      to encrypt and add an integrity tag value to a key blob that can be used by LoadKey in the future. The future
 *      LoadKey will only succeed if the key in the decryption register is the same as the value in the encryption key
 *      register at StoreKey time.
 * 
 * @param keyData Address of 256-bit key
 * 
 * @param attributes Attributes
 * 
 * @param encryptor Encryption key slot
 * 
 * @param keyBlob Output key blob address
 * 
 * @return void
 */
void ShackStoreKey(ConstPtr_Msg256_t keyData, uint32_t attributes, Ksu_Key_Slot_t encryptor, Ptr_Msg384_t keyBlob);

/**
 * @brief Saves a key into a key blob
 * 
 * \b Description:
 *      Takes a key slot to save and uses the key value in the encryption key slot to encrypt and integrity tag the
 *      saved key value and attributes into a key blob that can be used by LoadKey in the future. The future LoadKey
 *      will only succeed if the key in the source key slot is the same as the value in the source key at SaveKey time.
 *      Note that this SaveKey operation will only be performed if the encryption key slot has the IsDeviceSecret
 *      attribute set.
 * 
 * @param source Source key slot
 * 
 * @param encryptor Encryption key slot
 * 
 * @param keyBlob Output key blob address
 * 
 * @return void
 */
void ShackSaveKey(Ksu_Key_Slot_t source, Ksu_Key_Slot_t encryptor, Ptr_Msg384_t keyBlob);

/**
 * @brief Apply KDF function to source key, attributes, and 256-bit input to derive a new key
 * 
 * \b Description:
 *      A SHA-256 based KDF function is applied to the key in the source key slot, the supplied attributes, and the
 *      256-bit input to derive a new key. The new key along with the new attributes are stored in the destination key
 *      slot. If the ECCSignAllowed or ECDHAllowed attribute bit is set in the input attribute field, the resulting key
 *      from the KDFKey operation must conform to the requirements of a private ECC-P256 key (automatically satisfied
 *      by MS PKA engine). After the command completes, key in destination address = SHA256_AUTOPAD(input bits ||
 *      source key || new attributes). “new attributes” means 24-bits of attributes for the newly derived key after
 *      enforcement of all attribute requirements.
 * 
 * @param target Destination key slot
 * 
 * @param source Source key slot
 * 
 * @param data Address of 256-bit input
 * 
 * @param attributes Attributes
 * 
 * @return void
 */
void ShackKdfKey(Ksu_Key_Slot_t target, Ksu_Key_Slot_t source, ConstPtr_Msg256_t data, uint32_t attributes);

/**
 * @brief Apply KDF function to source key, attributes, and arbitrarily sized input to derive a new key
 * 
 * \b Description:
 *      KDF a key with a value. Similar to \c ShackExtendPcr but takes any sized buffer (up to 256 bits).
 * 
 * @param target Destination key slot
 * 
 * @param source Source key slot
 * 
 * @param data Address of input data
 * 
 * @param size Size of input data
 * 
 * @param attributes Attributes
 * 
 * @return void
 */
void ShackKdfKeyEx(Ksu_Key_Slot_t target, Ksu_Key_Slot_t source, const void* data, uint32_t size, uint32_t attributes);

/**
 * @brief Apply KDF function to source key, PCR value, and attributes to derive a new key
 * 
 * \b Description:
 *      A SHA-256 based KDF function is applied to the source key, and the PCR value, and the supplied attributes to
 *      derive a new key. The order of arguments for the SHA-256 is intentionally different from the KDFKey command.
 *      The new key along with the new attributes is stored in the destination key slot. Note: If the ECDSAAllowed
 *      or ECDHAllowed attribute bit is set in the input attribute field, the resulting key from the KDFPCR operation
 *      must conform to the requirements of a private ECC-P256 key (automatically satisfied by MS PKA engine). After
 *      the command completes, the key in destination address = SHA256_AUTOPAD(source key || PCR || new attributes).
 *      The “new attributes” argument means 24-bits of attributes for the newly-derived key after enforcement of all
 *      attribute requirements.
 * 
 * @param target Destination key slot
 * 
 * @param source Source key slot
 * 
 * @param pcrSlot PCR slot
 * 
 * @param attributes Attributes
 * 
 * @return void
 */
void ShackKdfPcr(Ksu_Key_Slot_t target, Ksu_Key_Slot_t source, Ksu_Pcr_Slot_t pcrSlot, uint32_t attributes);

/**
 * @brief Apply KDF function to source key, target PCR value, and attributes to derive a new key
 * 
 * \b Description:
 *      A SHA-256 based KDF function (must match one used by KDFPCR) is applied to the source key, the supplied target PCR value,
 *      and the supplied attributes to derive a new key. The target PCR is expected value of a PCR register that would allow
 *      deriving this key in the future using a KDFPCR command. The new key along with the attributes is stored in the destination
 *      key slot. However, the attributes in the destination key slot are modified by clearing the LoadKeyAllowed attribute and
 *      setting the StoreKeyAllowed attribute. This enables the destination key slot to be used for storing keys that can be loaded
 *      in the future when the exact same key is generated into a key slot using KDFPCR. After the command completes, key in the
 *      destination address = SHA256_AUTOPAD(source key || Target PCR || new attributes). This complex command is different from all
 *      others in that the attributes used in the KDF function are not the same as the ones written out at the destination key slot.
 * 
 * @param target Destination key slot
 * 
 * @param source Source key slot
 * 
 * @param pcrSlot Target PCR value
 * 
 * @param attributes Attributes
 * 
 * @return void
 */
void ShackKdfAsPcr(Ksu_Key_Slot_t target, Ksu_Key_Slot_t source, Ksu_Pcr_Slot_t pcrSlot, uint32_t attributes);

/**
 * @brief Generate the ECC public key using the specified ECC Private key
 * 
 * \b Description:
 *      Apply the appropriate ECC operation on the ECC Private key in the specified key slot to generate the
 *      corresponding ECC public key and return the ECC public key.
 * 
 * @param source Source key slot
 * 
 * @param publicKey Address to receive 512-bit output
 * 
 * @return void
 */
void ShackDeriveEccPublicKey(Ksu_Key_Slot_t source, Ptr_Msg512_t publicKey);

/**
 * @brief Use the specified signing ECC private key to sign the hash of the concatenation of the specified PCR value and the 256-bit input
 * 
 * \b Description:
 *      Use the specified signing ECC private key to sign the SHA256_AUTOPAD hash of the concatenation of the specified
 *      PCR value, and the 256-bit input. The operations are:
 *      Temp = SHA256_AUTOPAD(input value || PCR value)
 *      ECDSA signature = ECC256_sign(Temp, ECC Signing key)
 * 
 * @param source Source key slot
 * 
 * @param pcrReg PCR slot
 * 
 * @param data Address of 256-bit input
 * 
 * @param signature Output signature
 * 
 * @return void
 */
void ShackEccPcrSign(Ksu_Key_Slot_t source, Ksu_Pcr_Slot_t pcrReg, ConstPtr_Msg256_t data, Ptr_Msg512_t signature);

/**
 * @brief Use the specified signing ECC private key to sign the 256-bit input value
 * 
 * \b Description:
 *      Use the specified signing ECC private key to sign the 256-bit input value.
 * 
 * @param source Source key slot
 * 
 * @param data Address of 256-bit input
 * 
 * @param signature Output signature
 * 
 * @return void
 */
void ShackEccSign(Ksu_Key_Slot_t source, ConstPtr_Msg256_t data, Ptr_Msg512_t signature);

/**
 * @brief Use the provided ECC public key to verify a given message digest against a given signature
 * 
 * \b Description:
 *      Use the provided 512-bit ECC public key to verify a given message digest (256-bit hash) against a
 *      given 512-bit signature.  \c HSP_SUCCESS is returned if verification succeeds, or \c HSP_SIGNATURE_MISMATCH
 *      if it fails.
 * 
 * @param publicKey Address of 512-bit ECC public key.  Will be copied into shared memory (i.e. HSP accessible) if necessary.
 * 
 * @param digest Address of 256-bit SHA digest.  Must be HSP accessible (i.e. in shared memory).
 * 
 * @param signature Address of 512-bit signature.  Must be HSP accessible (i.e. in shared memory).
 * 
 * @return \c HSP_SUCCESS if successful, or \c HSP_SIGNATURE_MISMATCH if not.
 */
Hsp_Status_t ShackEccVerifySignature(ConstPtr_Msg512_t publicKey, ConstPtr_Msg256_t digest, ConstPtr_Msg512_t signature);

/**
 * @brief Use the specified ECC private key to perform ECDH with the supplied public key
 * 
 * \b Description:
 *      Use the specified ECC private key to perform ECDH with the supplied public key. The resulting EC point’s X
 *      coordinate is hashed with the Destination Attributes and the PCR value in the supplied PCR slot number to form
 *      the 256-bit key which is placed into the destination key slot. The Destination Attributes are placed in the
 *      destination key slot. The operations are:
 *      Shared secret = ECC_PT_MULTIPLICATION_256(Private ECC key, partner public key)
 *      Destination key = SHA256_AUTOPAD(Shared secret X coordinate || PCR value || Destination attributes)
 * 
 * @param target Destination key slot
 * 
 * @param eccKey Private ECC key slot
 * 
 * @param pcrReg PCR slot
 * 
 * @param publicKey Address of partner public key
 * 
 * @param attributes Destination attributes
 * 
 * @return void
 */
void ShackEcdhPcrKeyExchange(Ksu_Key_Slot_t target, Ksu_Key_Slot_t eccKey, Ksu_Pcr_Slot_t pcrReg, ConstPtr_Msg512_t publicKey, uint32_t attributes);

/**
 * @brief Use the specified ECC private key to perform ECDH with the supplied public key
 * 
 * \b Description:
 *      Use the specified ECC private key to perform ECDH with the provided public key. The resulting EC point’s X
 *      coordinate is hashed with the Destination Attributes to form the 256-bit key placed in the destination key
 *      slot. The Destination Attributes are placed in the destination key slot. The operations are:
 *      Shared secret = ECC_PT_MULTIPLICATION_256(ECC private key, partner public key)
 *      Destination key = SHA256_AUTOPAD(Shared secret X coordinate || Destination attributes)
 * 
 * @param target Destination key slot
 * 
 * @param eccKey Private ECC key slot
 * 
 * @param publicKey Address of partner public key
 * 
 * @param attributes Destination attributes
 * 
 * @return void
 */
void ShackEcdhKeyExchange(Ksu_Key_Slot_t target, Ksu_Key_Slot_t eccKey, ConstPtr_Msg512_t publicKey, uint32_t attributes);

/**
 * @brief Extend a value into a PCR register
 * 
 * \b Description:
 *      Extend a value into a PCR register. The new PCR value is the SHA-256 of the concatenation of the PCR value with
 *      the input 256-bit value added to the end of the SHA message. New PCR value = SHA256_AUTOPAD(PCR_value ||
 *      256-bit input value).
 * 
 * @param target PCR slot
 * 
 * @param data Address of 256-bit input
 * 
 * @return void
 */
void ShackExtendPcr(Ksu_Pcr_Slot_t target, ConstPtr_Msg256_t data);

/**
 * @brief Extend an arbitrarily sized value into a PCR register
 * 
 * \b Description:
 *      Extend a value into a PCR register. Similar to \c ShackExtendPcr but takes any sized buffer (up to 256 bits).
 * 
 * @param target PCR slot
 * 
 * @param data Address of input data
 * 
 * @param size Size of input data
 * 
 * @return void
 */
void ShackExtendPcrEx(Ksu_Pcr_Slot_t target, const void* data, uint32_t size);

/**
 * @brief Burn the key and attributes in a key slot into the corresponding fuse location
 * 
 * \b Description:
 *      Copy the key and attributes in the specified key slot to the fuse location corresponding to this key slot.
 *      This command first makes sure all fuses associated with the specified key slot are unprogrammed and then programs
 *      the key value and attributes in fuses using fuse controller commands. Only the first N (currently N=4) key slots
 *      have corresponding fuses. The fuse controller sends these values to KSU through the KSU import interface at
 *      initialization. If an improper key slot is specified, the command will fail with no changes to keys, or fuses.
 * 
 * @param keySlot Key slot
 * 
 * @return void
 */
void ShackBurnKey(Ksu_Key_Slot_t keySlot);

/**
 * @brief Decrypt payload using CBC mode
 * 
 * \b Description:
 *      Decrypt payload with provided with key and initial vector under AES CBC mode.
 * 
 * @param key 256 bit AES CBC key slot id.
 * 
 * @param initialVec 128 bit initial input vector.
 *                   In case of chaining multiple AES operations, initialVec 
 *                   needs to be provided in first call to the API, later calls
 *                   it has to be null.
 * 
 * @param dataIn pointer to where input data is stored, has to be in sharedram
 * 
 * @param dataLen size of data in bytes
 * 
 * @param dataOut pointer to where decrypted data will be stored, 
 *                has to be in sharedram
 * 
 * @param waitCompletion wait on completion or defered check
 * 
 * @return void
 */
void ShackAesCbc256Decrypt(Ksu_Key_Slot_t key, ConstPtr_Msg128_t initialVec, 
                           void* dataIn,  uint32_t dataLen, 
                           void* dataOut, bool waitCompletion);

/**
 * @brief Encrypt payload using CBC mode
 * 
 * \b Description:
 *      Encrypt payload with provided with key and initial vector under AES CBC mode.
 * 
 * @param key 256 bit AES CBC key slot id.
 * 
 * @param initialVec 128 bit initial input vector.
 *                   In case of chaining multiple AES operations, initialVec 
 *                   needs to be provided in first call to the API, later calls
 *                   it has to be null.
 * 
 * @param dataIn pointer to where input data is stored, has to be in sharedram
 * 
 * @param dataLen size of data in bytes
 * 
 * @param dataOut pointer to where encrypted data will be stored, 
 *                has to be in sharedram 
 * 
 * @param waitCompletion wait on completion or defered check
 * 
 * @return void
 */
void ShackAesCbc256Encrypt(Ksu_Key_Slot_t key, ConstPtr_Msg128_t initialVec, 
                           void* dataIn,  uint32_t dataLen, 
                           void* dataOut, bool waitCompletion);

#endif /* __SHACK_H */
