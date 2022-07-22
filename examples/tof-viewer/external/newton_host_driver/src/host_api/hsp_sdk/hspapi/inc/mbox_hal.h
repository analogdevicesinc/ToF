/**
 * @file mbox_hal.h
 * @brief Mailbox header for accessing Mbox Cregs.
 */

#ifndef __MBOX_HAL_H
#define __MBOX_HAL_H

#include "utils/generic/generic.h"

void S2hMbxWaitValid(void);

bool S2hMbxCheckValid(void);

void S2hMbxClearValid(void);

uint32_t S2hMbxFifoPop(void);

uint32_t S2hMbxGetFifoCnt(void);

uint32_t S2hMbxGetErrBit(void);

uint32_t S2hMbxCheckNotEmpty(void);

void H2sMbxWaitValid(void);

void H2sMbxSetValid(void);

void H2sMbxFifoPush(uint32_t data);

void H2sMbxSetErrBit(void);

#endif // __MBOX_HAL_H