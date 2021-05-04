//
// Copyright (c) Microsoft Corp. All rights reserved.
// Auto generated using reggen.exe utility
//            -- DO NOT MODIFY --
//

#pragma once

// namespace Hsp {
// namespace Ccs_Cmd {

typedef union _Ccs_Cmd_CCS_CMD_CODE_t
{
    struct {
        uint32_t ccs_mode  :  5; // [4:0]
        uint32_t rsvd_27_5 : 23; // [27:5]
        uint32_t id        :  4; // [31:28]
    };
    volatile uint32_t u;
} Ccs_Cmd_CCS_CMD_CODE_t;
typedef Ccs_Cmd_CCS_CMD_CODE_t volatile *Ptr_Ccs_Cmd_CCS_CMD_CODE_t;

typedef union _Ccs_Cmd_CCS_CMD_ADDR_1_t
{
    struct {
        volatile uint32_t CMD_ADDR_1;
    };
    volatile uint32_t u;
} Ccs_Cmd_CCS_CMD_ADDR_1_t;
typedef Ccs_Cmd_CCS_CMD_ADDR_1_t volatile *Ptr_Ccs_Cmd_CCS_CMD_ADDR_1_t;

typedef union _Ccs_Cmd_CCS_CMD_ADDR_2_t
{
    struct {
        volatile uint32_t CMD_ADDR_2;
    };
    volatile uint32_t u;
} Ccs_Cmd_CCS_CMD_ADDR_2_t;
typedef Ccs_Cmd_CCS_CMD_ADDR_2_t volatile *Ptr_Ccs_Cmd_CCS_CMD_ADDR_2_t;

typedef union _Ccs_Cmd_CCS_CMD_ADDR_3_t
{
    struct {
        volatile uint32_t CMD_ADDR_3;
    };
    volatile uint32_t u;
} Ccs_Cmd_CCS_CMD_ADDR_3_t;
typedef Ccs_Cmd_CCS_CMD_ADDR_3_t volatile *Ptr_Ccs_Cmd_CCS_CMD_ADDR_3_t;

typedef union _Ccs_Cmd_CCS_CMD_ADDR_4_t
{
    struct {
        volatile uint32_t CMD_ADDR_4;
    };
    volatile uint32_t u;
} Ccs_Cmd_CCS_CMD_ADDR_4_t;
typedef Ccs_Cmd_CCS_CMD_ADDR_4_t volatile *Ptr_Ccs_Cmd_CCS_CMD_ADDR_4_t;

typedef union _Ccs_Cmd_CCS_CMD_ATTR_t
{
    struct {
        volatile uint32_t CMD_ATTR;
    };
    volatile uint32_t u;
} Ccs_Cmd_CCS_CMD_ATTR_t;
typedef Ccs_Cmd_CCS_CMD_ATTR_t volatile *Ptr_Ccs_Cmd_CCS_CMD_ATTR_t;

typedef struct _Ccs_Cmd_Ccs_Cmd_t
{
    Ccs_Cmd_CCS_CMD_CODE_t CCS_CMD_CODE;
    Ccs_Cmd_CCS_CMD_ADDR_1_t CCS_CMD_ADDR_1;
    Ccs_Cmd_CCS_CMD_ADDR_2_t CCS_CMD_ADDR_2;
    Ccs_Cmd_CCS_CMD_ADDR_3_t CCS_CMD_ADDR_3;
    Ccs_Cmd_CCS_CMD_ADDR_4_t CCS_CMD_ADDR_4;
    Ccs_Cmd_CCS_CMD_ATTR_t CCS_CMD_ATTR;
} Ccs_Cmd_Ccs_Cmd_t;
typedef Ccs_Cmd_Ccs_Cmd_t volatile *Ptr_Ccs_Cmd_Ccs_Cmd_t;

//
// The offset of the registers
//
#define CCS_CMD_ADDRESS 0x300
#define CCS_CMD_OFFSET_CCS_CMD_CODE  0x300
#define CCS_CMD_OFFSET_CCS_CMD_ADDR_1  0x304
#define CCS_CMD_OFFSET_CCS_CMD_ADDR_2  0x308
#define CCS_CMD_OFFSET_CCS_CMD_ADDR_3  0x30c
#define CCS_CMD_OFFSET_CCS_CMD_ADDR_4  0x310
#define CCS_CMD_OFFSET_CCS_CMD_ATTR  0x314

//
// The macros for registers
//
#define CCS_CMD_REG(_reg)                             *(uint32_t)(CCS_CMD_OFFSET_ ## _reg)

// } // namespace Ccs_Cmd
// } // namespace Hsp
