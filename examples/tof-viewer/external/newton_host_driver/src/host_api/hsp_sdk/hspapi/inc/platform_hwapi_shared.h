/**
 * @file platform_hwapi_shared.h
 * @brief Structure in shared memory that is used by platform specific APIs.
 */

#ifndef __PLATFORM_HWAPI_SHARED_H
#define __PLATFORM_HWAPI_SHARED_H

#include "include/sharedram_struct.h"
#include "include/aes_cmd.h"
#include "include/ccs_cmd.h"
#include "include/pka_cmd.h"
#include "include/sha_cmd.h"
#include "utils/generic/generic.h"
#include "hwapi/hwapi_shared.h"

#include "apbmailbox.h"
#include "debug/postcode/postcode.h"

/**
 * @brief Structure in shared memory that is used by all HSP related APIs.
 *        This structure will be placed at the end of shared memory.
 */
typedef struct Platform_Hwapi_Shared {
    Hsp_post_code_t         postCode;                                   /**< Post Code */
    Mailbox_state_t         mailboxState;                               /**< Mailbox State */
    uint16_t                encryptedDataFlag;                          /**< Flag to indicate that encrypted data is stored in the RAMs */
    Mbx_cmd_pkt_t           encryptedCmdPktList[CMD_PKT_BUFF_SIZE];     /**< List of all Command Packets corresponding to encrypted RAM data packets */
    uint16_t                encryptedCmdPktCnt;                         /**< Count of encrypted payloads received from mailbox (excludes encrypted 1SP)*/ 
} Platform_Hwapi_Shared_t;
typedef volatile Platform_Hwapi_Shared_t* Ptr_platform_Hwapi_Shared_t;

#define PLATFORM_HWAPI_SHARED_STRUCT_ADDRESS            (HWAPI_SHARED_STRUCT_ADDRESS - sizeof(Platform_Hwapi_Shared_t))
#define PLATFORM_HWAPI_SHARED_STRUCT_PTR                ((Ptr_platform_Hwapi_Shared_t)PLATFORM_HWAPI_SHARED_STRUCT_ADDRESS)

#define PLATFORM_SHARED_ENC_CMD_PKT_STRUCT_PTR(n)       (&(PLATFORM_HWAPI_SHARED_STRUCT_PTR->encryptedCmdPktList[n]))
#define PLATFORM_SHARED_EN_CMD_PKT_CNT_STRUCT_PTR       (&(PLATFORM_HWAPI_SHARED_STRUCT_PTR->encryptedCmdPktCnt))

#endif //__PLATFORM_HWAPI_SHARED_H
