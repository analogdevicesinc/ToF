#include "utils/uart/uart.h"
#include "crypto/sha/sha.h"
#include "hwapi/hwapi_shared.h"
#include "utils/memory/memutils.h"
#include "fuses/fuses_public.h"
#include "fuses/fuses_utils.h"
#include "shack/shack.h"
#include "creg/aeb/aeb.h"

#include "apbmailbox_1sp.h"
#include "postcode_1sp.h"

Mbx_ret_status_t Get1SPMbxCmdPacket(Ptr_mbx_cmd_pkt_t ptrMbxCmd)
{
    uint32_t data1 = 0, data2 = 0;

    data1 = S2hMbxFifoPop();
    data2 = S2hMbxFifoPop();

    // store mbx commands to be processed later
    ptrMbxCmd->cmd = (uint16_t) (data1 >> 16);
    ptrMbxCmd->addr = (uint16_t) data1;
    ptrMbxCmd->size = (uint16_t) (data2 >> 16);
    ptrMbxCmd->attr.u = (uint16_t) data2;

    return MBX_STATUS_PASS;
}

void ReadCalibrationData(uint32_t *data, uint32_t wordSize)
{
    Hsp_Status_t hspStatus;

    AebEnableFeature(AEB_014_RSVD0_PROG_EN);
    AebEnableFeature(AEB_016_RSVD1_PROG_EN);

    // TODO: replace with the actual calibration address (to be given by ADI)
    #define FUSE_CAL_DATA_WORD_BEGIN_ADDR   FUSE_PUB_KEY_HASH_WORD_BEGIN_ADDR

    hspStatus = GetSoftwareFuseFromFuse(FUSE_CAL_DATA_WORD_BEGIN_ADDR, data, wordSize);
    PrintNumberWithMessage((hspStatus == HSP_SUCCESS) ? "GetSoftwareFuseFromFuse succeeded, hsp status = "
                                                        : "GetSoftwareFuseFromFuse failed, hsp status = ",
                            hspStatus);

    // For debug prints
    // UartWriteString("calData Read Back = \n");
    // for (int i = 0; i < wordSize; i++)
    // {
    //     UartWriteHex32(data[i]);
    //     UartWriteString(" ");
    // }
    // UartWriteLine(" ");

    AebRestoreDefaultFeature(AEB_014_RSVD0_PROG_EN);
    AebRestoreDefaultFeature(AEB_016_RSVD1_PROG_EN);
}

void ReadSocID(uint32_t *data, uint32_t wordSize)
{
    Hsp_Status_t hspStatus;
       
    hspStatus = GetSocIdFromRegs(data);

    PrintNumberWithMessage((hspStatus == HSP_SUCCESS) ? "GetSocIdFromRegs succeeded, hsp status = "
                                                        : "GetSocIdFromRegs failed, hsp status = ",
                            hspStatus);

    // For debug prints
    // UartWriteString("socId Read Back = \n");
    // for (int i = 0; i < wordSize; i++)
    // {
    //     UartWriteHex32(data[i]);
    //     UartWriteString(" ");
    // }
    // UartWriteLine(" ");
}

Mbx_ret_status_t SendH2SPayload(uint32_t *payloadAddr, uint32_t payloadWordSize)
{
    uint32_t wordsInFifo;
    uint32_t payloadIdx = 0;
    uint32_t mbxPushData = 0;

    while(payloadWordSize > 0)
    {
        H2sMbxWaitValid();
;
        wordsInFifo = (payloadWordSize / MAX_FIFO_DEPTH) == 0 ? payloadWordSize % MAX_FIFO_DEPTH : MAX_FIFO_DEPTH;

        for(uint32_t i = 0; i < wordsInFifo; i++)
        {
            mbxPushData = payloadAddr[payloadIdx++];

            H2sMbxFifoPush(mbxPushData);

            payloadWordSize--;
        }

        // Set H2S valid bit to signal data is ready
        H2sMbxSetValid();
    }

    return MBX_STATUS_PASS;
}

void SignalComplete1sp(void)
{
    // Setup command message
    uint32_t completeMsg = 0x444f4e45; // DONE

    // Wait for H2S fifo to be valid
    H2sMbxWaitValid();

    // Push data into H2S fifo
    H2sMbxFifoPush(completeMsg);

    // Set H2S valid
    H2sMbxSetValid();

    // Post HW_1SP_COMPLETE 
    Post1sp(HW_1SP_COMPLETE);
}