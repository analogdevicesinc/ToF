/**
 * @file ccs.h
 * @brief Header file for CCS block
 */

#ifndef __CCS_H
#define __CCS_H

#include "utils/generic/generic.h"

/**
 * @brief Initialize the CCS block
 * 
 * \b Description:
 *      Initializes the CCS block so that it is ready to receive commands.
 *      This basically just involves waiting for the hardware to come out of reset.
 * 
 * @param void
 * 
 * @return void
 */
void CcsInit(void);

/**
 * @brief Prepare a CCS command
 * 
 * \b Description:
 *      Prepares a CCS command (in shared memory) using the given parameters so that it is ready for execution.
 * 
 * @param commandCode CCS command code as defined in hspcmd.h.
 * 
 * @param arg0 Argument 0
 * 
 * @param arg1 Argument 1
 * 
 * @param arg2 Argument 2
 * 
 * @param arg3 Argument 3
 * 
 * @param attributes Attributes
 * 
 * @return void
 */
void CcsPrepareCmd(uint32_t commandCode, uint32_t arg0, uint32_t arg1, uint32_t arg2, uint32_t arg3, uint32_t attributes);

/**
 * @brief Execute a CCS command
 * 
 * \b Description:
 *      Executes a CCS command (prepared by calling \c CcsPrepareCmd), optionally waiting for it to complete.
 * 
 * @return void
 */
void CcsExecuteCmd(bool waitCompletion);

/**
 * @brief Wait for a CCS command to complete
 * 
 * \b Description:
 *      Waits for a CCS command (from calling \c CcsExecuteCmd with \c waitCompletion set to \c false) to complete before returning.
 * 
 * @return void
 */
void CcsWaitCmdComplete(void);

#endif /* __CCS_H */
