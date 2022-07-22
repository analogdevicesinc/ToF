//
// Copyright (c) Microsoft Corp. All rights reserved.
// Auto generated using reggen.exe utility
//            -- DO NOT MODIFY --
//

#pragma once

// namespace Hsp {
// namespace Pka_Cmd {

typedef union _Pka_Cmd_PKA_CMD_CODE_MOD_MONT_t
{
    struct {
        uint32_t size_mode  :  4; // [3:0]
        uint32_t rsvd_15_4  : 12; // [15:4]
        uint32_t op_mode    :  4; // [19:16]
        uint32_t rsvd_27_20 :  8; // [27:20]
        uint32_t id         :  4; // [31:28]
    };
    volatile uint32_t u;
} Pka_Cmd_PKA_CMD_CODE_MOD_MONT_t;
typedef Pka_Cmd_PKA_CMD_CODE_MOD_MONT_t volatile *Ptr_Pka_Cmd_PKA_CMD_CODE_MOD_MONT_t;

typedef union _Pka_Cmd_PKA_CMD_CODE_ECC_t
{
    struct {
        uint32_t size_mode  :  4; // [3:0]
        uint32_t rsvd_15_4  : 12; // [15:4]
        uint32_t op_mode    :  4; // [19:16]
        uint32_t rsvd_27_20 :  8; // [27:20]
        uint32_t id         :  4; // [31:28]
    };
    volatile uint32_t u;
} Pka_Cmd_PKA_CMD_CODE_ECC_t;
typedef Pka_Cmd_PKA_CMD_CODE_ECC_t volatile *Ptr_Pka_Cmd_PKA_CMD_CODE_ECC_t;

typedef union _Pka_Cmd_PKA_CMD_RSLT_t
{
    struct {
        volatile uint32_t CMD_RSLT;
    };
    volatile uint32_t u;
} Pka_Cmd_PKA_CMD_RSLT_t;
typedef Pka_Cmd_PKA_CMD_RSLT_t volatile *Ptr_Pka_Cmd_PKA_CMD_RSLT_t;

typedef union _Pka_Cmd_PKA_CMD_ARG_1_t
{
    struct {
        volatile uint32_t CMD_ARG_1;
    };
    volatile uint32_t u;
} Pka_Cmd_PKA_CMD_ARG_1_t;
typedef Pka_Cmd_PKA_CMD_ARG_1_t volatile *Ptr_Pka_Cmd_PKA_CMD_ARG_1_t;

typedef union _Pka_Cmd_PKA_CMD_ARG_2_t
{
    struct {
        volatile uint32_t CMD_ARG_2;
    };
    volatile uint32_t u;
} Pka_Cmd_PKA_CMD_ARG_2_t;
typedef Pka_Cmd_PKA_CMD_ARG_2_t volatile *Ptr_Pka_Cmd_PKA_CMD_ARG_2_t;

typedef union _Pka_Cmd_PKA_CMD_SIG_t
{
    struct {
        volatile uint32_t CMD_SIG;
    };
    volatile uint32_t u;
} Pka_Cmd_PKA_CMD_SIG_t;
typedef Pka_Cmd_PKA_CMD_SIG_t volatile *Ptr_Pka_Cmd_PKA_CMD_SIG_t;

typedef struct _Pka_Cmd_Pka_Cmd_t
{
    Pka_Cmd_PKA_CMD_CODE_MOD_MONT_t PKA_CMD_CODE_MOD_MONT;
    Pka_Cmd_PKA_CMD_RSLT_t PKA_CMD_RSLT;
    Pka_Cmd_PKA_CMD_ARG_1_t PKA_CMD_ARG_1;
    Pka_Cmd_PKA_CMD_ARG_2_t PKA_CMD_ARG_2;
    Pka_Cmd_PKA_CMD_SIG_t PKA_CMD_SIG;
} Pka_Cmd_Pka_Cmd_t;
typedef Pka_Cmd_Pka_Cmd_t volatile *Ptr_Pka_Cmd_Pka_Cmd_t;

//
// The offset of the registers
//
#define PKA_CMD_ADDRESS 0x200
#define PKA_CMD_OFFSET_PKA_CMD_CODE_MOD_MONT  0x200
#define PKA_CMD_OFFSET_PKA_CMD_CODE_ECC  0x200
#define PKA_CMD_OFFSET_PKA_CMD_RSLT  0x204
#define PKA_CMD_OFFSET_PKA_CMD_ARG_1  0x208
#define PKA_CMD_OFFSET_PKA_CMD_ARG_2  0x20c
#define PKA_CMD_OFFSET_PKA_CMD_SIG  0x210

//
// The macros for registers
//
#define PKA_CMD_REG(_reg)                             *(uint32_t)(PKA_CMD_OFFSET_ ## _reg)

// } // namespace Pka_Cmd
// } // namespace Hsp
