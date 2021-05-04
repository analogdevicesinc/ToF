/**
 * @file sha.h
 * @brief Header file for SHA engine block
 */

#ifndef __SHA_H
#define __SHA_H

#include "utils/generic/generic.h"

/**
 * @brief Size of a single SHA block
 */
#define SHA_BLOCK_SIZE 64

/**
 * @brief Initialize the SHA block
 * 
 * \b Description:
 *      Initializes the SHA block so that it is ready to receive commands.
 *      This basically just involves waiting for the hardware to come out of reset.
 * 
 * @param void
 * 
 * @return void
 */
void ShaInit(void);

/**
 * @brief Prepare a SHA command
 * 
 * \b Description:
 *      Prepares a SHA command (in shared memory) using the given parameters so that it is ready for execution.
 * 
 * @param shaCommand SHA command code as defined in hspcmd.h.
 * 
 * @param digest Pointer to where the first byte of final digest should be written.
 *               Caller is responsible for allocating memory.
 *               No address alignment requirements.
 * 
 * @param byteCount Used as 'message length' for padding. Only valid for AUTOPAD commands, ignore otherwise.
 * 
 * @param messageBytes Size of message to be hashed. 
 *                     Must be a non-zero multiple of \c SHA_BLOCK_SIZE bytes (64 bytes) for non-AUTOPAD commands. 
 *                     Any byte count, including zero, for AUTOPAD commands.
 * 
 * @param message Pointer to the first byte of message for engine to hash.
 *                Message must be in contiguous memory. No address alignment requirements.
 * 
 * @param initialDigest Pointer to the first byte of initial digest. Initial digest
 *                      must be in contiguous memory. Only valid for LOAD commands, ignore otherwise.
 *                      No address alignment requirements.
 * 
 * @return void
 */
void ShaPrepareCmd(uint32_t shaCommand, Ptr_Msg256_t digest, uint32_t byteCount, uint32_t messageBytes, const void* message, ConstPtr_Msg256_t initialDigest);

/**
 * @brief Execute a SHA command
 * 
 * \b Description:
 *      Executes a SHA command (prepared by calling \c ShaPrepareCmd), optionally waiting for it to complete.
 * 
 * @return void
 */
void ShaExecuteCmd(bool waitCompletion);

/**
 * @brief Wait for a SHA command to complete
 * 
 * \b Description:
 *      Waits for a SHA command (from calling \c ShaExecuteCmd with \c waitCompletion set to \c false) to complete before returning.
 * 
 * @return void
 */
void ShaWaitCmdComplete(void);

/**
 * @brief SHA option flags
 */
typedef enum
{
    SHA_NO_FLAGS            = 0x00000000,   /**< No flags. */
    SHA_REVERSE_OUTPUT_FLAG = 0x00000001,   /**< Flag indicating that the output digest bytes should be reversed after the SHA operation compeltes. This is to accomodate different endianness requirements. */
    SHA_ASYNC_MODE_FLAG     = 0x00000002    /**< Flag indicating that SHA operations be performed in asynchronous mode. This causes the API to return while the current SHA operation is still in progress. */
} Sha_Flags_t;

/**
 * @brief Run SHA 256 to generate 256-bit digest
 *
 * \b Description:
 *      Exercise the SHA engine to output a 256 bit hash.
 * 
 * @param message Pointer to the message to be hashed.  Will be copied into shared memory (i.e. HSP accessible) if necessary.
 * 
 * @param length Length of the message.
 * 
 * @param digest Pointer to output buffer to receive the digest output.  (Does not need to be in shared memory.)
 * 
 * @param flags Combination of \c Sha_Flags_t values to further control the behavior of the SHA operation.
 *              NOTE: Asynchronous mode is not available with this API.
 * 
 * @return void
 */
void Sha256(const void* message, size_t length, Ptr_Msg256_t digest, uint32_t flags);


/**
 * @brief Begin a new SHA 256 procedure.
 *
 * \b Description:
 *      Indicates the start of a new SHA256 procedure (asynchronously chained SHA256 operations).  It is expected
 *      that \c Sha256ProvideData will be called repeatedly until \c messageLength bytes have been received.
 * 
 * @param messageLength Total length of data to be hashed, in bytes.
 * 
 * @param flags Combination of \c Sha_Flags_t values to further control the behavior of the SHA procedure.
 * 
 * @return void
 */

void Sha256StartNew(size_t messageLength, uint32_t flags);

/**
 * @brief Provide a buffer of data for a SHA 256 procedure.
 *
 * \b Description:
 *      Provides a buffer of data for a currently in-progress SHA256 procedure, which will have beeen started with a
 *      call to \c Sha256StartNew.  This API can work asynchronously.  It will wait for any previous HSP operation to
 *      complete before starting processing of this buffer, and will return before the processing is complete.  Thus
 *      it will be necessary to call \c Sha256WaitComplete before accessing the result.
 * 
 * @param data Pointer to data to hash.  Must be in shared memory (i.e. HSP accessible).  Must also be a mutliple
 *             of \c SHA_BLOCK_SIZE bytes (64 bytes) unless it is the final buffer to be hashed.
 * 
 * @param length Length of buffer, in bytes.
 * 
 * @return void
 */
void Sha256ProvideData(const void* data, size_t length);

/**
 * @brief Wait for a SHA 256 procedure to complete.
 *
 * \b Description:
 *      Waits for any in-progress SHA256 procedure to complete.  Once this API returns, the final resulting hash
 *      value will be available in shared memory (see \c SHARED_SHA256_DIGEST_PTR) and ready for functions such
 *      as \c ShackEccVerifySignature.
 * 
 * @return void
 */
void Sha256WaitComplete(void);

#endif /* __SHA_H */
