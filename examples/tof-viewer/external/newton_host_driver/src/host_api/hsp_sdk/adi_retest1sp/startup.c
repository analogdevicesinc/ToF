/**
 * @file startup.c
 * @brief HSP Firmware: Startup code
 * 
 * \b Description:
 * Startup code to set up C runtime environment.
 * Initializes:
 *  - stack pointer
 *  - trap vector
 * 
 * Also,
 *  - Copies initialized data from ROM to RAM
 *  - Initializes BSS section to zeros
 *  - Runs main rom flow
 */

#include <stdint.h>
#include "register_utils.h"
#include "uart.h"

// Defined in linkderscript
extern uint32_t _start_bss;
extern uint32_t _end_bss;

extern void Start1Sp(void);

void __attribute__ ((interrupt, aligned(64))) _1sp_trap_vector(void) {
    unsigned long mcause, mepc;

    __asm__ volatile ("csrr %0, mcause ": "=r"(mcause));
    UartWriteString("1SP: Got exception/interrupt MCAUSE=");
    UartWriteHex32(mcause);

    UartWriteString(" MEPC=");
    __asm__ volatile ("csrr %0, mepc ": "=r"(mepc));
    UartWriteHex32(mepc);
    UartWriteLine("");

    // TODO: For fatal error, send POST CODE and reset.
    //       Other case, identify & save interrupt source and return

    while(1) { };
}

/**
 * @brief Startup code
 * 
 * \b Description:
 *      1. Setup stack pointer
 *      2. Setup initial trap vector
 *      4. Initialize BSS section to zeros
 *      5. Runs main rom flow function
 * 
 * @param void
 * 
 * @return void
 */
void __attribute__((section(".text.init"), naked)) _start(void) {
    register uint32_t *dst;

    // Setup stack pointer
    asm volatile("la sp, _sp");

    //can we wipe out stack by 0x400 size and then Initialize BSS section

    // Don't need to copy data section. They are already in RAM
    // Initialize bss section to zero
    dst = (uint32_t *) &_start_bss;
    while (dst < (uint32_t *) &_end_bss) {
        *dst = 0U;
        dst++;
    }

    // TODO: Setup interrupt

    WriteMtvec((uint32_t) _1sp_trap_vector | CLIC_DIRECT_MODE);

    Start1Sp();
}
