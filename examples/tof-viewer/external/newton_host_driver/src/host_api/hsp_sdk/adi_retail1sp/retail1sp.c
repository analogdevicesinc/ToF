/**
 * @file retail1sp.c
 * @brief Sample Retail 1SP flow
 */
#include "utils/uart/uart.h"
#include "hwapi/hwapi_shared.h"
#include "utils/memory/memutils.h"

#include "fuses/fuses_public.h"
#include "fuses/fuses_utils.h"

#include "common_1sp.h"
#include "apbmailbox_1sp.h"

#define FUSE_CAL_DATA_WORD_SIZE FUSE_PUB_KEY_HASH_WORD_SIZE
#define DATA_BUFF_SIZE 8

void Start1Sp(void)
{
    // Lock all AEB bits except for what we need
    LockdownHardware(LOCK_EXEMPT_UART | LOCK_EXEMPT_FUSES);

    UartInit();

    UartWriteLine("Starting Retail 1SP!");

    Mbx_cmd_pkt_t mbxCmd;
    Ptr_mbx_cmd_pkt_t ptrMbxCmd = (Ptr_mbx_cmd_pkt_t) &mbxCmd;
    uint32_t data[DATA_BUFF_SIZE] = {0};

    // Mailbox loop should be in while loop.
    // For the purpose of testing this sample code, while loop is disabled
    // while(1)
    // {
        UartWriteLine("Get Mailbox Cmd");

        S2hMbxWaitValid();

        Get1SPMbxCmdPacket(ptrMbxCmd);

        S2hMbxClearValid();

        switch(ptrMbxCmd->cmd)
        {
            case CMD_1SP_RD_CALIB_DATA:
                UartWriteLine("CMD_1SP_RD_CALIB_DATA");

                ReadCalibrationData(data, FUSE_CAL_DATA_WORD_SIZE);
                SendH2SPayload(data, FUSE_CAL_DATA_WORD_SIZE);
                break;
            case CMD_1SP_RD_SOCID_DATA:
                UartWriteLine("CMD_1SP_RD_SOCID_DATA");

                ReadSocID(data, FUSE_SOCID_WORD_SIZE);
                SendH2SPayload(data, FUSE_SOCID_WORD_SIZE);
                break;
            default:
                // Handle invalid command
                UartWriteLine("Invalid Command");
                break;
        }
    // }

    UartWriteLine("Completed Retail 1SP!");

    SignalComplete1sp();

    // Lock all remaining AEB bits
    LockdownHardware(LOCK_EXEMPT_NONE);

    // Don't fall out of our flow
    while (true) {};
}
