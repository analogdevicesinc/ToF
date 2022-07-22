//
// Copyright (c) Microsoft Corp. All rights reserved.
// Auto generated using reggen.exe utility
//            -- DO NOT MODIFY --
//

#pragma once

// namespace Hsp {
// namespace Sha_Cmd {

typedef union _Sha_Cmd_SHA_CMD_CODE_t
{
    struct {
        uint32_t load_digest       : 1; // [0]
        uint32_t rsvd_7_1          : 7; // [7:1]
        uint32_t auto_pad          : 1; // [8]
        uint32_t read_message_mode : 2; // [10:9]
        uint32_t rsvd_15_11        : 5; // [15:11]
        uint32_t sha_mode          : 4; // [19:16]
        uint32_t endian            : 1; // [20]
        uint32_t check_digest      : 1; // [21]
        uint32_t pass_message_mode : 2; // [23:22]
        uint32_t rsvd_27_24        : 4; // [27:24]
        uint32_t id                : 4; // [31:28]
    };
    volatile uint32_t u;
} Sha_Cmd_SHA_CMD_CODE_t;
typedef Sha_Cmd_SHA_CMD_CODE_t volatile *Ptr_Sha_Cmd_SHA_CMD_CODE_t;

typedef union _Sha_Cmd_SHA_CMD_DGST_t
{
    struct {
        volatile uint32_t CMD_DGST;
    };
    volatile uint32_t u;
} Sha_Cmd_SHA_CMD_DGST_t;
typedef Sha_Cmd_SHA_CMD_DGST_t volatile *Ptr_Sha_Cmd_SHA_CMD_DGST_t;

typedef union _Sha_Cmd_SHA_CMD_BYTE_COUNT_t
{
    struct {
        volatile uint32_t CMD_BYTE_COUNT;
    };
    volatile uint32_t u;
} Sha_Cmd_SHA_CMD_BYTE_COUNT_t;
typedef Sha_Cmd_SHA_CMD_BYTE_COUNT_t volatile *Ptr_Sha_Cmd_SHA_CMD_BYTE_COUNT_t;

typedef union _Sha_Cmd_SHA_CMD_MSG_BYTES_t
{
    struct {
        volatile uint32_t CMD_MSG_BYTES;
    };
    volatile uint32_t u;
} Sha_Cmd_SHA_CMD_MSG_BYTES_t;
typedef Sha_Cmd_SHA_CMD_MSG_BYTES_t volatile *Ptr_Sha_Cmd_SHA_CMD_MSG_BYTES_t;

typedef union _Sha_Cmd_SHA_CMD_MSG_t
{
    struct {
        volatile uint32_t CMD_MSG;
    };
    volatile uint32_t u;
} Sha_Cmd_SHA_CMD_MSG_t;
typedef Sha_Cmd_SHA_CMD_MSG_t volatile *Ptr_Sha_Cmd_SHA_CMD_MSG_t;

typedef union _Sha_Cmd_SHA_CMD_INIT_t
{
    struct {
        volatile uint32_t CMD_INIT;
    };
    volatile uint32_t u;
} Sha_Cmd_SHA_CMD_INIT_t;
typedef Sha_Cmd_SHA_CMD_INIT_t volatile *Ptr_Sha_Cmd_SHA_CMD_INIT_t;

typedef union _Sha_Cmd_SHA_CMD_PASS_t
{
    struct {
        volatile uint32_t CMD_PASS;
    };
    volatile uint32_t u;
} Sha_Cmd_SHA_CMD_PASS_t;
typedef Sha_Cmd_SHA_CMD_PASS_t volatile *Ptr_Sha_Cmd_SHA_CMD_PASS_t;

typedef union _Sha_Cmd_SHA_CMD_REF_t
{
    struct {
        volatile uint32_t CMD_REF;
    };
    volatile uint32_t u;
} Sha_Cmd_SHA_CMD_REF_t;
typedef Sha_Cmd_SHA_CMD_REF_t volatile *Ptr_Sha_Cmd_SHA_CMD_REF_t;

typedef struct _Sha_Cmd_Sha_Cmd_t
{
    Sha_Cmd_SHA_CMD_CODE_t SHA_CMD_CODE;
    Sha_Cmd_SHA_CMD_DGST_t SHA_CMD_DGST;
    Sha_Cmd_SHA_CMD_BYTE_COUNT_t SHA_CMD_BYTE_COUNT;
    Sha_Cmd_SHA_CMD_MSG_BYTES_t SHA_CMD_MSG_BYTES;
    Sha_Cmd_SHA_CMD_MSG_t SHA_CMD_MSG;
    Sha_Cmd_SHA_CMD_INIT_t SHA_CMD_INIT;
    Sha_Cmd_SHA_CMD_PASS_t SHA_CMD_PASS;
    Sha_Cmd_SHA_CMD_REF_t SHA_CMD_REF;
} Sha_Cmd_Sha_Cmd_t;
typedef Sha_Cmd_Sha_Cmd_t volatile *Ptr_Sha_Cmd_Sha_Cmd_t;

//
// The offset of the registers
//
#define SHA_CMD_ADDRESS 0x100
#define SHA_CMD_OFFSET_SHA_CMD_CODE  0x100
#define SHA_CMD_OFFSET_SHA_CMD_DGST  0x104
#define SHA_CMD_OFFSET_SHA_CMD_BYTE_COUNT  0x108
#define SHA_CMD_OFFSET_SHA_CMD_MSG_BYTES  0x10c
#define SHA_CMD_OFFSET_SHA_CMD_MSG  0x110
#define SHA_CMD_OFFSET_SHA_CMD_INIT  0x114
#define SHA_CMD_OFFSET_SHA_CMD_PASS  0x118
#define SHA_CMD_OFFSET_SHA_CMD_REF  0x11c

//
// The macros for registers
//
#define SHA_CMD_REG(_reg)                             *(uint32_t)(SHA_CMD_OFFSET_ ## _reg)

// } // namespace Sha_Cmd
// } // namespace Hsp
