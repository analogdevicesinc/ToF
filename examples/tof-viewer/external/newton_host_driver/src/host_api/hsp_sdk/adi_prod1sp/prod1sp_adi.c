/**
 * @file prod1sp_adi.c
 * @brief Sample Prod 1SP ADI flow
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

#define OPERATING_MODE_ADI  OPERATING_MODE_VENDOR   // alias

static struct {
    uint32_t svn[FUSE_SVN_WORD_SIZE];
    uint32_t pubKeyHash[FUSE_PUB_KEY_HASH_WORD_SIZE];
    uint32_t encryptionKey[FUSE_ENCRYPTION_KEY_WORD_SIZE];
    uint32_t globalPrivateKey[FUSE_PRIVATE_KEY_WORD_SIZE];
    uint32_t blockedOperationModes;
} g_fuseData __attribute__((section(".data.fuse")));

void Start1Sp(void)
{
    // Lock all AEB bits except for what we need
    LockdownHardware(LOCK_EXEMPT_UART | LOCK_EXEMPT_FUSES);

    AebEnableFeature(AEB_015_UART_EN);
    UartInit();

    if (HWAPI_1SP_SHARED_STRUCT_PTR->targetOperatingMode != OPERATING_MODE_ADI)
    {
        UartWriteLine("Error: Hsp is not adi target operation mode.  Stopping..."); 

        // Lock all remaining AEB bits
        LockdownHardware(LOCK_EXEMPT_NONE);

        while (true);   
    }

    UartWriteLine("Starting Prod 1SP");
    Hsp_Status_t hspStatus;

    Security_State_t secState = ReadSecurityState();

    if (secState == HSP_SECURITY_STATE_SECURED)
    {
        uint32_t socId[FUSE_SOCID_WORD_SIZE];

        hspStatus = GetSocIdFromRegs(socId);

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

        SignalComplete1sp();

        UartWriteLine("Completing Prod 1SP on secure state");

        // Lock all remaining AEB bits
        LockdownHardware(LOCK_EXEMPT_NONE);

        while (1)
        {
        }
    }

    hspStatus = BurnSoftwareFuse(FUSE_SVN_WORD_BEGIN_ADDR, g_fuseData.svn, FUSE_SVN_WORD_SIZE);
    PrintNumberWithMessage((hspStatus == HSP_SUCCESS) ? "Burn svn succeeded, hsp status = "
                                                      : "Burn svn failed, hsp status = ",
                           hspStatus);

    uint32_t SVNRead[FUSE_SVN_WORD_SIZE] = {0};
    hspStatus = GetSoftwareFuseFromFuse(FUSE_SVN_WORD_BEGIN_ADDR, SVNRead, FUSE_SVN_WORD_SIZE);
    PrintNumberWithMessage((hspStatus == HSP_SUCCESS) ? "GetSoftwareFuseFromFuse succeeded, hsp status = "
                                                        : "GetSoftwareFuseFromFuse failed, hsp status = ",
                            hspStatus);
    UartWriteString("SVN Read Back = \n");
    for (int i = 0; i < FUSE_SVN_WORD_SIZE; i++)
    {
        UartWriteHex32(SVNRead[i]);
        UartWriteString(" ");
    }
    UartWriteLine(" ");

    UartWriteString("Resulting SVN = \n");
    ReverseBytes32(SVNRead, sizeof(uint64_t));    // Stored in fuses as big endian, so must reverse bytes for little endian
    UartWriteHex32(SVNRead[1]);
    UartWriteString(" ");
    UartWriteHex32(SVNRead[0]);
    UartWriteLine(" ");

    hspStatus = BurnSoftwareFuseEcc(FUSE_ENCRYPTION_KEY_WORD_BEGIN_ADDR, g_fuseData.encryptionKey, FUSE_ENCRYPTION_KEY_WORD_SIZE);
    PrintNumberWithMessage((hspStatus == HSP_SUCCESS) ? "Burn encryption key ecc succeeded, hsp status = "
                                                      : "Burn  encryption key hash ecc failed, hsp status = ",
                           hspStatus);

    uint32_t encryptionKeyRead[FUSE_ENCRYPTION_KEY_WORD_SIZE] = {0}; 
    hspStatus = GetSoftwareFuseFromFuseEcc(FUSE_ENCRYPTION_KEY_WORD_BEGIN_ADDR, encryptionKeyRead, FUSE_ENCRYPTION_KEY_WORD_SIZE);
    PrintNumberWithMessage((hspStatus == HSP_SUCCESS) ? "GetSoftwareFuseFromFuseEcc succeeded, hsp status = "
                                                        : "GetSoftwareFuseFromFuseEcc failed, hsp status = ",
                            hspStatus);
    UartWriteString("Encryption key Read Back = \n");
    for (int i = 0; i < FUSE_ENCRYPTION_KEY_WORD_SIZE; i++)
    {
        UartWriteHex32(encryptionKeyRead[i]);
        UartWriteString(" ");
    }
    UartWriteLine(" ");


    hspStatus = ChangeSecurityState(HSP_SECURITY_STATE_SECURED);
    PrintNumberWithMessage((hspStatus == HSP_SUCCESS) ? "ChangeSecurityState succeeded, hsp status = "
                                                      : "ChangeSecurityState failed, hsp status = ",
                           hspStatus);

    SignalComplete1sp();

    UartWriteLine("Completed Prod 1SP");

    // Lock all remaining AEB bits
    LockdownHardware(LOCK_EXEMPT_NONE);

    // Don't fall out of our flow
    while (true)
    {
    };
}
