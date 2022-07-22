/**
 * @file fuses_utils.h
 * @brief Header for utility functions of fuse controller and fuses
 */

#ifndef __FUSES_UTILS_H
#define __FUSES_UTILS_H
#include "utils/generic/generic.h"
#include "ecc.h"

#ifdef DEBUG    // UART print is only present in DEBUG builds

#ifdef VERBOSE_FUSES
void HandleFuseError(Hsp_Status_t hspStatus, const char *callingFunction);
void PrintEccCorrectStatus(STATUS_ECC_CHECK_ACCUM_t correctStatus, Ecc_Check_Count_t eccCheckCount);
void PrintComputeSyndromeResult(int32_t syndrome,	uint32_t bitsRequiredOut, uint32_t eccIn, uint32_t eccCal);
void ResetHspInSimulation(void);
#else
static inline void HandleFuseError(Hsp_Status_t hspStatus, const char *callingFunction){}
static inline void PrintEccCorrectStatus(STATUS_ECC_CHECK_ACCUM_t correctStatus, Ecc_Check_Count_t eccCheckCount){}
static inline void PrintComputeSyndromeResult(int32_t syndrome,	uint32_t bitsRequiredOut, uint32_t eccIn, uint32_t eccCal){}
static inline void ResetHspInSimulation(void){}
#endif // VERBOSE_FUSES
void PrintNumberWithMessage(char *msg, int32_t number);
void UartPrintSSChangeMessage(Hsp_Status_t hspStatus, Security_State_t mySsState);

#else

static inline void HandleFuseError(Hsp_Status_t hspStatus, const char *callingFunction){}
static inline void PrintEccCorrectStatus(STATUS_ECC_CHECK_ACCUM_t correctStatus, Ecc_Check_Count_t eccCheckCount){}
static inline void PrintComputeSyndromeResult(int32_t syndrome,	uint32_t bitsRequiredOut, uint32_t eccIn, uint32_t eccCal){}
static inline void ResetHspInSimulation(void){}
static inline void PrintNumberWithMessage(char *msg, int32_t number){}
static inline void UartPrintSSChangeMessage(Hsp_Status_t hspStatus, Security_State_t mySsState){}

#endif //DEBUG

#endif //__FUSES_UTILS_H