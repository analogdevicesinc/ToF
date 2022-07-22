/**
 * @file release1spToRetest.c
 * @brief Sample Prod 1SP to retest flow
 */
#include "utils/uart/uart.h"
#include "crypto/sha/sha.h"
#include "hwapi/hwapi_shared.h"
#include "utils/memory/memutils.h"
#include "fuses/fuses_public.h"
#include "fuses/fuses_utils.h"
#include "shack/shack.h"
#include "creg/aeb/aeb.h"

#include "common_1sp.h"
#include "apbmailbox_1sp.h"

void Start1Sp(void)
{
    // Lock all AEB bits except for what we need
    LockdownHardware(LOCK_EXEMPT_UART | LOCK_EXEMPT_FUSES | LOCK_EXEMPT_RETEST);

    AebEnableFeature(AEB_011_AEBFUSE_PROG_EN);
    AebEnableFeature(AEB_015_UART_EN);
    AebEnableFeature(AEB_021_ALLOW_TDR_RESET);
    UartInit();

    UartWriteLine("Starting Prod 1SP to enable secure to retest");
    Hsp_Status_t hspStatus;

    Security_State_t secState = ReadSecurityState();

    UartWriteString("Security State: ");
    UartWriteHex32(secState);
    UartWriteLine("");

    UartWriteString("Operating Mode: ");
    UartWriteHex32(HWAPI_1SP_SHARED_STRUCT_PTR->targetOperatingMode);
    UartWriteLine("");


    if (secState == HSP_SECURITY_STATE_SECURED)
    {
        uint32_t socId[FUSE_SOCID_WORD_SIZE];
UartWriteLine("Before calling GetSocIdFromRegs");
        hspStatus = GetSocIdFromRegs(socId);
UartWriteLine("After  calling GetSocIdFromRegs");
        PrintNumberWithMessage((hspStatus == HSP_SUCCESS) ? "GetSocIdFromRegs succeeded, hsp status = "
                                                          : "GetSocIdFromRegs failed, hsp status = ",
                               hspStatus);

        UartWriteLine("SocId is: ");
        for (int i = 0; i < FUSE_SOCID_WORD_SIZE; i++)
        {
            UartWriteHex32(socId[i]);
            UartWriteString(" ");
        }
        UartWriteLine("");

        //in secure state, need to program fuse aeb word 1, on bit 30 to turn on uart for retest
        // FUSE_AEB_WORD_BEGIN_ADDR + 1 
        uint32_t data = (1 << 30);
        hspStatus =  FctrlProgramWords(FUSE_AEB_WORD_BEGIN_ADDR + 1, &data, 1, FUSE_AEB_WORD_ACCESSIBLE);
        if(hspStatus != HSP_SUCCESS){
            UartWriteLine("Not able to program aeb fuse bit 62, FuseB of enabling uart (aeb 15)");
        }
        else{
            UartWriteLine("Programmed aeb fuse bit 62, FuseB of enabling uart (aeb 15)");
        }

        if (RetestTransitionAllowed() == 0)
        {
            UartWriteLine("ERROR: Never saw allow chss retest bit\n");
        }
        else
        {
            UartWriteLine("Saw allow chss retest bit\n");
        }

        UartWriteLine("Changing security state to retest");

        hspStatus = ChangeSecurityState(HSP_SECURITY_STATE_RETEST);
        PrintNumberWithMessage((hspStatus == HSP_SUCCESS) ? "ChangeSecurityState succeeded, hsp status = "
                                                          : "ChangeSecurityState failed, hsp status = ",
                               hspStatus);


        SignalComplete1sp();

        UartWriteLine("Completing Prod 1SP on secure state to retest state");

        // Lock all remaining AEB bits
        LockdownHardware(LOCK_EXEMPT_NONE);

        while (1)
        {
        }
    }

    UartWriteLine("Not in secure state");

    // Lock all remaining AEB bits
    LockdownHardware(LOCK_EXEMPT_NONE);

    // Don't fall out of our flow
    while (true)
    {
    };
}
