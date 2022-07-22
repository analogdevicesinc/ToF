/**
 * @file rng.h
 * @brief Header file for RNG engine block
 */

#ifndef __RNG_H
#define __RNG_H

#include "utils/generic/generic.h"

/**
 * @brief Initialize Random Number Generator
 * 
 * \b Description:
 *      The function does the following:
 *      1. Write to RNG control register to enable RNG block
 *          a. Write clk_div value if necessary
 *             @see RNG_architecture_v1.45.docx Software Guidance section for more details
 *      2. Poll busy bit to determine when initial rn_data is ready. 
 *         Once rn_data is ready then polling is not necessary anymore.
 *  
 * @param waitCompletion Flag indicating whether we wait for initialization to complete
 * 
 * @return void
 */
void RngInit(bool waitCompletion);

/**
 * @brief Wait for RNG initialization to complete, if necessary
 * 
 * \b Description:
 *      Waits for RNG initialization (from calling \c RngInit with \c waitCompletion set to \c false) to complete before returning.
 *      Safe to call if RNG initialization has already completed.
 * 
 * @return void
 */
void RngWaitInitComplete(void);

/**
 * @brief Generates Random Data of requested size
 * 
 * \b Description:
 *      Generates and random data of size \c size and stores at the pointer
 *      \c outputBuf
 *  
 * @param size Size in bytes of amount of random data to generate. 
 *             Size needs to be multiple of 4 bytes.
 * @param outputBuf Pointer to buffer, callee ensure memory of correct size is allocated
 * 
 * @return void
 */
void RngGetData(uint32_t* outputBuf, uint32_t size);

/**
 * @brief Generates 4 bytes of Random Data
 * 
 * \b Description:
 *      Generates and random data of 4 bytes \c size and stores at the pointer
 *      \c outputBuf
 *  
 * @param outputBuf Pointer to buffer, callee ensure memory of correct size is allocated
 * 
 * @return 32-bit word of random data
 */
uint32_t RngGetData32(void);

#endif //__RNG_H
