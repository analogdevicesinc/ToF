/**
 * @file common_1sp.c
 * @brief Common 1SP helper functions
 */

#include "common_1sp.h"
#include "creg/aeb/aeb.h"
#include "utils/uart/uart.h"

void LockdownHardware(uint32_t flags)
{
    static const uint32_t exemptUart   = (1 << AEB_015_UART_EN);
    static const uint32_t exemptFuses  = (1 << AEB_011_AEBFUSE_PROG_EN) | (1 << AEB_012_SOCID_PROG_EN) | (1 << AEB_013_KEY_PROG_EN) |
                                         (1 << AEB_014_RSVD0_PROG_EN) | (1 << AEB_016_RSVD1_PROG_EN);
    static const uint32_t exemptRetest = (1 << AEB_011_AEBFUSE_PROG_EN) | (1 << AEB_021_ALLOW_TDR_RESET);

    // If we are disabling UART, make sure UART data has been flushed
    if ((flags & LOCK_EXEMPT_UART) == 0)
    {
        UartFlush();
    }

    // Build AEB lock mask
    uint32_t lockMask = 0xFFFFFFFF;
    if (flags & LOCK_EXEMPT_UART)
    {
        lockMask &= ~exemptUart;
    }
    if (flags & LOCK_EXEMPT_FUSES)
    {
        lockMask &= ~exemptFuses;
    }
    if (flags & LOCK_EXEMPT_RETEST)
    {
        lockMask &= ~exemptRetest;
    }

    // Lock all AEB bits as specified
    AebLock(lockMask);
}
