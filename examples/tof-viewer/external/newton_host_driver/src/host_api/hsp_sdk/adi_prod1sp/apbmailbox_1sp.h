/**
 * @file apbmailbox_1sp.h
 * @brief 1SP APB mailbox header
 */

#ifndef __APB_MAILBOX_1SP_H
#define __APB_MAILBOX_1SP_H

#include "utils/generic/generic.h"
#include "utils/generic/hsp_status.h"
#include "mbox_hal.h"

#define CMD_PKT_BUFF_SIZE   16 // Number of encrypted data packets mailbox can handle according to ROM requirements doc
#define MBX_CMD_PKT_SIZE    8
#define MAX_FIFO_DEPTH      4

/**
 * =======================================================
 * Enums
 * =======================================================
 */


typedef enum MBX_CMD
{
    CMD_1SP_START           = 0x2000,
    CMD_1SP_RD_CALIB_DATA   = 0x2001,
    CMD_1SP_RD_SOCID_DATA   = 0x2002,
    CMD_1SP_LAST,
} Mbx_cmd_t;

typedef enum
{
    MBX_STATUS_PASS = 0,
    MBX_STATUS_FAIL,
} Mbx_ret_status_t;

/**
 * =======================================================
 * Structs
 * =======================================================
 */

typedef union _Mbx_attr_t 
{
    struct 
    {
        uint16_t writeAttr          : 1;
        uint16_t signedAttr         : 1;
        uint16_t encryptedAttr      : 1;
        uint16_t groupedAttr        : 1;
        uint16_t reserved           : 12;
    };
    uint16_t u;
}Mbx_attr_t;
typedef volatile Mbx_attr_t* Ptr_mbx_attr_t;

/**
 * @brief Mailbox Command Packet Structure
 * 
 * Structure of the mailbox command packet.
 * Sequence to send data through mailbox: 
 *  1) Command 
 *  2) Address
 *  3) Size
 *  4) Attribute
 */
typedef struct _Mbx_cmd_pkt_t
{
    volatile uint16_t cmd;
    volatile uint16_t addr;
    volatile uint16_t size;
    volatile Mbx_attr_t attr;
}Mbx_cmd_pkt_t;
typedef volatile Mbx_cmd_pkt_t* Ptr_mbx_cmd_pkt_t;

/**
 * =======================================================
 * Functions
 * =======================================================
 */

/**
 * @brief Get Mailbox Command from S2H Mailbox FIFO
 * 
 * @param ptrMbxCmd Pointer to be updated with incoming mailbox header data
 * 
 * @return MBX_STATUS_PASS
 */
Mbx_ret_status_t Get1SPMbxCmdPacket(Ptr_mbx_cmd_pkt_t ptrMbxCmd);

/**
 * @brief Read Calibration Data from Fuses
 * 
 * @param data \c uint32_t pointer to buffer where data is to be stored
 * 
 * @param wordSize Size of data (in DWords) to be stored into buffer
 * 
 * @return void
 */
void ReadCalibrationData(uint32_t *data, uint32_t wordSize);

/**
 * @brief Read Soc ID from register
 * 
 * @param data \c uint32_t pointer to buffer where data is to be stored
 * 
 * @param wordSize Number of DWords of data to be stored into buffer
 * 
 * @return void
 */
void ReadSocID(uint32_t *data, uint32_t wordSize);

/**
 * @brief Send data stored in buffer through H2S mailbox FIFO
 * 
 * @param payloadAddr \c uint32_t pointer to the data buffer
 * 
 * @param payloadWordSize Number of DWords of data to be sent through H2S mailbox FIFO
 * 
 * @return MBX_STATUS_PASS
 */
Mbx_ret_status_t SendH2SPayload(uint32_t *payloadAddr, uint32_t payloadWordSize);

/**
 * @brief Signal 1SP is complete
 * 
 * @param void
 * 
 * @return void
 */
void SignalComplete1sp(void);

#endif /* __APB_MAILBOX_1SP_H */