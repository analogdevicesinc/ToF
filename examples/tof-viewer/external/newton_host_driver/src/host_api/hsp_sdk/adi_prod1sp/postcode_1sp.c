/**
 * @file postcode_1sp.c
 * @brief 1SP Post code function implementation - Analog Devices platform specific
 */

#include "postcode_1sp.h"

void Post1sp(Hsp_1sp_post_code_t postCode)
{
    UartWriteString("Postcode: ");
    UartWriteHex32(postCode);
    UartWriteLine("");
}
