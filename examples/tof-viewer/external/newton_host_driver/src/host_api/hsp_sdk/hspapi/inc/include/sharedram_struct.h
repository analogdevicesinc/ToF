//
// Copyright (c) Microsoft Corp. All rights reserved.
// Auto generated using reggen.exe utility
//            -- DO NOT MODIFY --
//

#pragma once

// namespace Hsp {
// namespace Sharedram {

typedef union _Sharedram_SHAREDRAM_t
{
    volatile uint32_t u;
} Sharedram_SHAREDRAM_t;
typedef Sharedram_SHAREDRAM_t volatile *Ptr_Sharedram_SHAREDRAM_t;

typedef struct _Sharedram_Sharedram_t
{
    Sharedram_SHAREDRAM_t SHAREDRAM[1024];
} Sharedram_Sharedram_t;
typedef Sharedram_Sharedram_t volatile *Ptr_Sharedram_Sharedram_t;

//
// The offset of the registers
//
#define SHAREDRAM_ADDRESS 0x2f020000
#define SHAREDRAM_OFFSET_SHAREDRAM  0x2f020000

//
// The macros for registers
//
#define SHAREDRAM_REG(_reg)                           *(uint32_t)(SHAREDRAM_OFFSET_ ## _reg)

// } // namespace Sharedram
// } // namespace Hsp
