/**
 * @file postcode_1sp.h
 * @brief Post code function declarations, must be implemented independently by each platform
 */

#ifndef __POSTCODE_1SP_H
#define __POSTCODE_1SP_H

#include "utils/generic/generic.h"
#include "utils/uart/uart.h"

#define HSP_BOOT_STAGE_1SP (1 << 30)
#define HSP_BOOT_STAGE_2SP (2 << 30)

#define ANALOG_DEVICES_PLAT (1 << 27)
#define ATHENA_PLAT (2 << 27)

#define HSP_PLATFORM ANALOG_DEVICES_PLAT

#define HW_GENERIC      (1 << 22)

/**
 * @brief Post code values
 */
typedef enum
{
    // System Status
    HW_1SP_COMPLETE = ((HSP_BOOT_STAGE_1SP) | (HSP_PLATFORM) | (HW_GENERIC) | 0x0 ),    /**< 1SP Complete */
} Hsp_1sp_post_code_t;

/**
 * @brief Write post code to available Debug channel
 * 
 * \b Description:
 *      Write a post code to either UART/shared memory/other debug options     
 * 
 * @param   postCode Post code to write to debug channel
 * 
 * @return  void
 * 
 */
void Post1sp(Hsp_1sp_post_code_t postCode);

#endif //__POSTCODE_H