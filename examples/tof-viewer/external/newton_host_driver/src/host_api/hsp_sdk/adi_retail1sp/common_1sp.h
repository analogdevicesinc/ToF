/**
 * @file common_1sp.h
 * @brief Common 1SP helper functions
 */

#ifndef __COMMON_1SP_H
#define __COMMON_1SP_H

#include "utils/generic/generic.h"

/**
 * @brief Lockdown exemption flags
 */
typedef enum
{
    LOCK_EXEMPT_NONE    = 0x00000000,   /**< Exempt nothing (lock everything) */
    LOCK_EXEMPT_UART    = 0x00000001,   /**< Exempt UART */
    LOCK_EXEMPT_FUSES   = 0x00000002,   /**< Exempt fuses */
    LOCK_EXEMPT_RETEST  = 0x00000004    /**< Exempt retest transition */
} Lock_Exempt_Flags_t;

/**
 * @brief Lockdown hardware
 * 
 * \b Description:
 *        If 1SP is signed, the ROM will leave hardware access open and it will be 1SP's
 *        responsbility to lock it down.  Use this function to perform any necessary
 *        security operations, such as locking AEB bits.
 * 
 *        Note it is safe to always call this function, so it is not necessary to check
 *        whether 1SP was signed or not.
 * 
 * @param flags Combination of \c Lock_Exempt_Flags_t values that specify any excemptions.
 * 
 * @return void
 */
void LockdownHardware(uint32_t flags);

#endif /* __COMMON_1SP_H */
