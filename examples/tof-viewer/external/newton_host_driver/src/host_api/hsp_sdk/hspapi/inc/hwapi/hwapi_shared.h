/**
 * @file hwapi_shared.h
 * @brief Structure in shared memory that is used by all HSP related APIs.
 */

#ifndef __HWAPI_SHARED_H
#define __HWAPI_SHARED_H

#include "include/sharedram_struct.h"
#include "include/aes_cmd.h"
#include "include/ccs_cmd.h"
#include "include/pka_cmd.h"
#include "include/sha_cmd.h"
#include "utils/generic/generic.h"
#include "crypto/sha/sha.h"

/**
 * @brief Structure in shared memory that is used by all HSP related APIs.
 *        This structure will be placed at the end of shared memory.
 */
typedef struct Hwapi_Shared {
    Aes_Cmd_Aes_Cmd_t aesCmdStruct;                 /**< AES command structure */
    Ccs_Cmd_Ccs_Cmd_t ccsCmdStruct;                 /**< CCS command structure */
    Pka_Cmd_Pka_Cmd_t pkaCmdStruct;                 /**< PKA command structure */
    Sha_Cmd_Sha_Cmd_t shaCmdStruct;                 /**< SHA command structure */
    uint32_t          hspExecFlags;                 /**< Flags indicating states of execution (HSP blocks and critical error handler) (See \c Hsp_Exec_Flags_t) */
    Msg256_t          sha256CalcDigest;             /**< SHA256 calculated digest */
    uint32_t          shaBytesRemaining;                 /**< For Sha256ProvideData(): Number of bytes remaining that have not been sent to hardware */
    uint32_t          shaFlags;                          /**< For Sha256ProvideData(): SHA flags associated with the SHA procedure */
    uint8_t           shaResidualBuffer[SHA_BLOCK_SIZE]; /**< For Sha256ProvideData(): Residual buffer for handling non-aligned SHA buffers */
    uint32_t          shaResidualBytes;                  /**< For Sha256ProvideData(): Number of bytes in SHA residual buffer */
    Msg128_t          aesInitialVector;             /**< Initial vector used by AES module for chaining */
    uint32_t          pkaResult;                    /**< 32-bit result from PKA operations */
    Msg512_t          receivedSignature;            /**< 512-bit signature received with message*/
    Msg128_t          scratch128;                   /**< General 128-bit scratch space */
    Msg256_t          scratch256;                   /**< General 256-bit scratch space */
    Msg512_t          scratch512;                   /**< General 512-bit scratch space */
} Hwapi_Shared_t;

/**
 * @brief Version number of Hwapi_1Sp_Shared_t structure.
 *        NOTE: Must be bumped anytime the structure is updated in a non-backwards compatible way.
 */
#define HWAPI_1SP_SHARED_STRUCT_VERSION     0x101

/**
 * @brief Structure that is shared with 1SP. 
 *        Before jumping to 1SP, this structure will be locked to read-only
 */
typedef struct Hwapi_1Sp_Shared {
    uint32_t          structVersion;                /**< Version number of this structure */
    Operating_Mode_t  operatingMode;                /**< Operating mode (Microsoft, Vendor, etc) */
    Operating_Mode_t  targetOperatingMode;          /**< Target operating mode (Microsoft, Vendor, etc), used for Production 1SP */
    Msg512_t          publicKeys[PUBLIC_KEY_COUNT]; /**< Public keys used for signature verification, certificate authorization, etc */
    uint32_t          reserved;                     /**< Reserved (was ROM functions structure) */
} Hwapi_1Sp_Shared_t;

#define HWAPI_SHARED_STRUCT_ADDRESS         (SHAREDRAM_ADDRESS + sizeof(Sharedram_Sharedram_t) - sizeof(Hwapi_Shared_t))
#define HWAPI_SHARED_STRUCT_PTR             ((Hwapi_Shared_t*)HWAPI_SHARED_STRUCT_ADDRESS)

#define SHARED_AES_CMD_STRUCT_PTR           (&(HWAPI_SHARED_STRUCT_PTR->aesCmdStruct))
#define SHARED_CCS_CMD_STRUCT_PTR           (&(HWAPI_SHARED_STRUCT_PTR->ccsCmdStruct))
#define SHARED_PKA_CMD_STRUCT_PTR           (&(HWAPI_SHARED_STRUCT_PTR->pkaCmdStruct))
#define SHARED_SHA_CMD_STRUCT_PTR           (&(HWAPI_SHARED_STRUCT_PTR->shaCmdStruct))
 
#define SHARED_SHA256_DIGEST_PTR            (&(HWAPI_SHARED_STRUCT_PTR->sha256CalcDigest))
 
#define SHARED_AES_INITIAL_VECTOR_PTR       (&(HWAPI_SHARED_STRUCT_PTR->aesInitialVector))
 
#define SHARED_RECEIVED_SIGNATURE_PTR       (&(HWAPI_SHARED_STRUCT_PTR->receivedSignature))

#define SHARED_SCRATCH128_PTR               (&(HWAPI_SHARED_STRUCT_PTR->scratch128))
#define SHARED_SCRATCH256_PTR               (&(HWAPI_SHARED_STRUCT_PTR->scratch256))
#define SHARED_SCRATCH512_PTR               (&(HWAPI_SHARED_STRUCT_PTR->scratch512))

extern Hwapi_1Sp_Shared_t _hob_addr; // from linker script
#define HWAPI_1SP_SHARED_STRUCT_PTR         (&_hob_addr)

#define SHARED_PUBLIC_KEY_PTR(n)            (&(HWAPI_1SP_SHARED_STRUCT_PTR->publicKeys[n]))

typedef enum
{
    HSP_EXEC_AES_CMD  = 0x00000001,     /**< AES command executing */
    HSP_EXEC_CCS_CMD  = 0x00000002,     /**< CCS command executing */
    HSP_EXEC_PKA_CMD  = 0x00000004,     /**< PKA command executing */
    HSP_EXEC_SHA_CMD  = 0x00000008,     /**< SHA command executing */
    HSP_EXEC_RNG_INIT = 0x00000010,     /**< RNG initialization executing */
    HSP_EXEC_CRIT_ERR = 0x00000020      /**< Critical error handler executing */
} Hsp_Exec_Flags_t;

#endif //__HWAPI_SHARED_H
