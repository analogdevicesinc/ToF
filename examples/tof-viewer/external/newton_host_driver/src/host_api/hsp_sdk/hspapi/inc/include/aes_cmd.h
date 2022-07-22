//
// Copyright (c) Microsoft Corp. All rights reserved.
// Auto generated using reggen.exe utility
//            -- DO NOT MODIFY --
//

#pragma once

// namespace Hsp {
// namespace Aes_Cmd {

typedef union _Aes_Cmd_AES_CMD_CODE_t
{
    struct {
        uint32_t key_len          : 4; // [3:0]
        uint32_t xts_size         : 4; // [7:4]
        uint32_t rsvd_11_8        : 4; // [11:8]
        uint32_t wiv              : 1; // [12]
        uint32_t rsvd_15_13       : 3; // [15:13]
        uint32_t ed_mode          : 1; // [16]
        uint32_t rsvd_19_17       : 3; // [19:17]
        uint32_t msgout_addr_mode : 2; // [21:20]
        uint32_t msgin_addr_mode  : 2; // [23:22]
        uint32_t cipher_mode      : 4; // [27:24]
        uint32_t id               : 4; // [31:28]
    };
    volatile uint32_t u;
} Aes_Cmd_AES_CMD_CODE_t;
typedef Aes_Cmd_AES_CMD_CODE_t volatile *Ptr_Aes_Cmd_AES_CMD_CODE_t;

typedef union _Aes_Cmd_AES_CMD_RSLT_t
{
    struct {
        volatile uint32_t CMD_RSLT;
    };
    volatile uint32_t u;
} Aes_Cmd_AES_CMD_RSLT_t;
typedef Aes_Cmd_AES_CMD_RSLT_t volatile *Ptr_Aes_Cmd_AES_CMD_RSLT_t;

typedef union _Aes_Cmd_AES_CMD_BYTE_COUNT_t
{
    struct {
        volatile uint32_t CMD_BYTE_COUNT;
    };
    volatile uint32_t u;
} Aes_Cmd_AES_CMD_BYTE_COUNT_t;
typedef Aes_Cmd_AES_CMD_BYTE_COUNT_t volatile *Ptr_Aes_Cmd_AES_CMD_BYTE_COUNT_t;

typedef union _Aes_Cmd_AES_CMD_MSG_t
{
    struct {
        volatile uint32_t CMD_MSG;
    };
    volatile uint32_t u;
} Aes_Cmd_AES_CMD_MSG_t;
typedef Aes_Cmd_AES_CMD_MSG_t volatile *Ptr_Aes_Cmd_AES_CMD_MSG_t;

typedef union _Aes_Cmd_AES_CMD_KEY_t
{
    struct {
        volatile uint32_t CMD_KEY;
    };
    volatile uint32_t u;
} Aes_Cmd_AES_CMD_KEY_t;
typedef Aes_Cmd_AES_CMD_KEY_t volatile *Ptr_Aes_Cmd_AES_CMD_KEY_t;

typedef union _Aes_Cmd_AES_CMD_IV_t
{
    struct {
        volatile uint32_t CMD_IV;
    };
    volatile uint32_t u;
} Aes_Cmd_AES_CMD_IV_t;
typedef Aes_Cmd_AES_CMD_IV_t volatile *Ptr_Aes_Cmd_AES_CMD_IV_t;

typedef struct _Aes_Cmd_Aes_Cmd_t
{
    Aes_Cmd_AES_CMD_CODE_t AES_CMD_CODE;
    Aes_Cmd_AES_CMD_RSLT_t AES_CMD_RSLT;
    Aes_Cmd_AES_CMD_BYTE_COUNT_t AES_CMD_BYTE_COUNT;
    Aes_Cmd_AES_CMD_MSG_t AES_CMD_MSG;
    Aes_Cmd_AES_CMD_KEY_t AES_CMD_KEY;
    Aes_Cmd_AES_CMD_IV_t AES_CMD_IV;
} Aes_Cmd_Aes_Cmd_t;
typedef Aes_Cmd_Aes_Cmd_t volatile *Ptr_Aes_Cmd_Aes_Cmd_t;

//
// The offset of the registers
//
#define AES_CMD_ADDRESS 0x0
#define AES_CMD_OFFSET_AES_CMD_CODE  0x0
#define AES_CMD_OFFSET_AES_CMD_RSLT  0x4
#define AES_CMD_OFFSET_AES_CMD_BYTE_COUNT  0x8
#define AES_CMD_OFFSET_AES_CMD_MSG  0xc
#define AES_CMD_OFFSET_AES_CMD_KEY  0x10
#define AES_CMD_OFFSET_AES_CMD_IV  0x14

//
// The macros for registers
//
#define AES_CMD_REG(_reg)                             *(uint32_t)(AES_CMD_OFFSET_ ## _reg)

// } // namespace Aes_Cmd
// } // namespace Hsp
