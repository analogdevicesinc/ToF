/**
 * @file error_handler.h
 * @brief Runtime error handler function declaration, must be implemented independently by each platform
 */

#ifndef __ERROR_HANDLER_H
#define __ERROR_HANDLER_H

#include "utils/generic/generic.h"
#include "utils/generic/hsp_status.h"
#include "platformconfig.h"

// In debug builds we pass the faulting file and line number to the critical error handler
#if CONFIG_INCLUDE_VERBOSE_ERROR_INFO
    #define __DECL_DEBUG_PARAMETERS     , const char* file, int line
    #define __DEBUG_PARAMETERS          , __FILE__, __LINE__
#else
    #define __DECL_DEBUG_PARAMETERS
    #define __DEBUG_PARAMETERS
#endif

/**
 * @brief Critical error handler
 * 
 * \b Description:
 *      Critical error handler
 *      - Called by main code when a non-recoverable error occurs.
 *      - Must be implemented indepdently for each supported platform.
 *      - Function must NOT return.
 * 
 * @param status HSP status code representing the error that has occurred.
 * 
 * @param context1 First context value.  This can be any user-supplied value relating to the error.
 * 
 * @param context2 Second context value.  This can be any user-supplied value relating to the error.
 * 
 * @param function DEBUG ONLY - Pointer to name of function that caused the error.
 * 
 * @param file DEBUG ONLY - Pointer to source filename where error occurred.
 * 
 * @param line DEBUG ONLY - Source line number where error occurred.
 * 
 * @return void
 */
void CriticalErrorHandler(Hsp_Status_t status, uint32_t context1, uint32_t context2 __DECL_DEBUG_PARAMETERS);

/**
 * @brief Critical error handler function pointer
 */
typedef void (*CritErr_FuncPtr_t) (Hsp_Status_t status, uint32_t context1, uint32_t context2 __DECL_DEBUG_PARAMETERS);

#endif //__ERROR_HANDLER_H
