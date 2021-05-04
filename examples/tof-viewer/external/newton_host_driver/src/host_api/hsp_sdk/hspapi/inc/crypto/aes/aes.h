/**
 * @file aes.h
 * @brief Header file for AES engine block
 */

#ifndef __AES_H
#define __AES_H

#include "utils/generic/generic.h"

/**
 * @brief Size of a single AES block
 */
#define AES_BLOCK_SIZE 16

/**
 * @brief AES Action Encrypt/Decrypt
*/
typedef enum
{
    AES_DECRYPT = 0,                /**< Value of 0*/
    AES_ENCRYPT,                    /**< Value of 1*/
    AES_ACTION_MAX_TYPE = 0xF       /**< Value of 15*/
} Aes_Action_t;

/**
 * @brief AES Key Length
 */
typedef enum
{
    AES_128 = 0x1,                  /**< Value of 1*/
    AES_192,                        /**< Value of 2*/
    AES_256,                        /**< Value of 3*/
    AES_KEY_LENGTH_MAX_TYPE = 0xF   /**< Value of 15*/
} Aes_Key_Length_t;

/**
 * @brief AES Mode types
 */
typedef enum
{
    AES_ECB = 0x1,                  /**< Value of 1*/
    AES_CBC,                        /**< Value of 2*/
    AES_CTR,                        /**< Value of 3*/
    AES_CFB,                        /**< Value of 4*/
    AES_OFB,                        /**< Value of 5*/
    AES_XTS,                        /**< Value of 6*/
    AES_MODE_MAX_TYPE = 0xF         /**< Value of 15*/
} Aes_Mode_t;

/**
 * @brief AES Unit size, to detemine unit size in XTS mode.
 */
typedef enum
{
    XTS_UNUSED = 0x0,               /**< Value of 0*/
    XTS_16B,                        /**< Value of 1*/
    XTS_512B,                       /**< Value of 2*/
    XTS_1024B,                      /**< Value of 3*/
    XTS_2048B,                      /**< Value of 4*/
    XTS_4096B,                      /**< Value of 5*/
    XTS_UNIT_SIZE_MAX_TYPE = 0xF    /**< Value of 15*/
} Xts_Unit_Size_t;

/**
 * @brief Initialize the AES block
 * 
 * \b Description:
 *      Initializes the AES block so that it is ready to receive commands.
 *      This basically just involves waiting for the hardware to come out of reset.
 * 
 * @param void
 * 
 * @return void
 */
void AesInit(void);

/**
 * @brief Prepare an AES command
 * 
 * \b Description:
 *      Prepares an AES command (in shared memory) using the given parameters so that it is ready for execution.
 * 
 *      Before writing command structure address to the command address
 *      register, it is necessary to check the status register.
 *      Poll busy bit in status register.
 *      Then
 * 
 *      Below is the format of the command structure
 *          1. Command code
 *              a. mode - encrypt/decrypt mode
 *              b. e/d  - decrypt=0, encrypt=1
 *              c. wiv  - when set, writes out initial vector.
 *              d. unit size - smallest unit operated on by AES engine
 *              e. key length - length of cipher key
 *          2. result_ptr - AES result written to this address.
 *          3. byte_count - number of bytes to process.
 *          4. message_ptr - input data to be processed.
 *          5. key_ptr - location of cipher key.
 *          6. init_vector_ptr - location of initial vector.
 * 
 * @param mode - encrypt/decrypt mode
 * @param encryptDecrypt  - decrypt=0, encrypt=1
 * @param wiv  - when set, writes out initial vector.
 * @param unitSize - AES_16B, AES_512B, AES_1024B, AES_2048B, AES_4096B
 * @param keyLength - AES_128, AES_192, AES_256
 * @param byteCount - number of bytes to process.
 * @param messagePtr - input data to be processed.
 * @param keyPtr - location of cipher key.
 * @param initVectorPtr - location of initial vector.
 * @param resultPtr - AES result written to this address.
 * 
 * @return void
 */
void AesPrepareCmd(Aes_Mode_t mode, Aes_Action_t encryptDecrypt, bool wiv, 
                   Xts_Unit_Size_t unitSize, Aes_Key_Length_t keyLength, 
                   uint32_t byteCount, void* messagePtr, void* keyPtr, 
                   void* initVectorPtr, void* resultPtr);

/**
 * @brief Execute an AES command
 * 
 * \b Description:
 *      Executes an AES command (prepared by calling \c AesPrepareCmd), optionally waiting for it to complete.
 * 
 * @return void
 */
void AesExecuteCmd(bool waitCompletion);

/**
 * @brief Wait for an AES command to complete
 * 
 * \b Description:
 *      Waits for an AES command (from calling \c AesExecuteCmd with \c waitCompletion set to \c false) to complete before returning.
 * 
 * @return void
 */
void AesWaitCmdComplete(void);

#endif /* __AES_H */
