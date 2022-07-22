/**
 * @file generic.h
 * @brief Generic typedefs and enums for all HSP platforms
 */

#ifndef __GENERIC_H
#define __GENERIC_H

#include <stdint.h>
#include <stddef.h>
#include "hsp_status.h"

/**
 * @brief Location of Image
 */
typedef enum 
{
    IMAGE_LOC_SRAM = 0,   /**< Value of 0 */
    IMAGE_LOC_TCM,        /**< Value of 1 */
    IMAGE_LOC_XIP         /**< Value of 2 */
} Image_Location_t;

/**
 * @brief HSP Security State
 */
typedef enum
{
    HSP_SECURITY_STATE_UNKNOWN    = 0,          /**< Value of 0*/
    HSP_SECURITY_STATE_BLANK,                   /**< Value of 1*/
    HSP_SECURITY_STATE_TEST,                    /**< Value of 2*/
    HSP_SECURITY_STATE_PROD,                    /**< Value of 3*/
    HSP_SECURITY_STATE_SECURED,                 /**< Value of 4*/
    HSP_SECURITY_STATE_RETEST,                  /**< Value of 5*/
} Security_State_t;

/**
 * @brief Operating mode typedef
 */
typedef enum
{  
    OPERATING_MODE_UNKNOWN      = 0,    /**< Unknown mode */
    OPERATING_MODE_UNSIGNED     = 1,    /**< Unsigned mode */
    OPERATING_MODE_VENDOR       = 2,    /**< Vendor mode */
    OPERATING_MODE_MICROSOFT    = 3,    /**< Microsoft mode */
    OPERATING_MODE_COUNT,               /**< Number of operating modes */
} Operating_Mode_t;

/**
 * @brief Types of public keys
 */
typedef enum
{
   PUBLIC_KEY_SIGNATURE     = 0,    /**< Public key to be used for signature verification */
   PUBLIC_KEY_AUTHORIZATION = 1,    /**< Public key to be used for certificate authorization */
   PUBLIC_KEY_COUNT         = 2     /**< Number of public keys */
} Public_Key_t;

#ifndef __cplusplus
  #define bool      _Bool
  #define true      (1)
  #define false     (0)
#endif

#define SET     (1)
#define RESET   (0)

#define BYTES_PER_DWORD                  4

#define IN_DWORDS(x)                     ((x)/4)
#define IN_BYTES(x)                      (x)
#define IN_BITS(x)                       ((x)*8)

#define SP_MSG_128_SIZE                  0x10
#define SP_MSG_192_SIZE                  0x18
#define SP_MSG_256_SIZE                  0x20
#define SP_MSG_320_SIZE                  0x28
#define SP_MSG_384_SIZE                  0x30
#define SP_MSG_512_SIZE                  0x40
#define SP_MSG_768_SIZE                  0x60

static inline uint32_t min(uint32_t a, uint32_t b)
{
    return (a < b ? a : b);
}

/**
 * @brief 128-bit message
 */
typedef union _Msg128_t
{
    unsigned long   Dwords[IN_DWORDS(SP_MSG_128_SIZE)];
    unsigned char   Bytes[IN_BYTES(SP_MSG_128_SIZE)];
} Msg128_t, *Ptr_Msg128_t;

typedef const Msg128_t* ConstPtr_Msg128_t;

/**
 * @brief 192-bit message
 */
typedef union _Msg192_t
{
    unsigned long   Dwords[IN_DWORDS(SP_MSG_192_SIZE)];
    unsigned char   Bytes[IN_BYTES(SP_MSG_192_SIZE)];
} Msg192_t, *Ptr_Msg192_t;

typedef const Msg192_t* ConstPtr_Msg192_t;

/**
 * @brief 256-bit message
 */
typedef union _Msg256_t
{
    unsigned long   Dwords[IN_DWORDS(SP_MSG_256_SIZE)];
    unsigned char   Bytes[IN_BYTES(SP_MSG_256_SIZE)];
} Msg256_t, *Ptr_Msg256_t;

typedef const Msg256_t* ConstPtr_Msg256_t;

/**
 * @brief 320-bit message
 */
typedef union _Msg320_t
{
    unsigned long   Dwords[IN_DWORDS(SP_MSG_320_SIZE)];
    unsigned char   Bytes[IN_BYTES(SP_MSG_320_SIZE)];
} Msg320_t, *Ptr_Msg320_t;

typedef const Msg320_t* ConstPtr_Msg320_t;

/**
 * @brief 384-bit message
 */
typedef union _Msg384_t
{
    unsigned long   Dwords[IN_DWORDS(SP_MSG_384_SIZE)];
    unsigned char   Bytes[IN_BYTES(SP_MSG_384_SIZE)];
} Msg384_t, *Ptr_Msg384_t;

typedef const Msg384_t* ConstPtr_Msg384_t;

/**
 * @brief 512-bit message
 */
typedef union _Msg512_t
{
    unsigned long   Dwords[IN_DWORDS(SP_MSG_512_SIZE)];
    unsigned char   Bytes[IN_BYTES(SP_MSG_512_SIZE)];
} Msg512_t, *Ptr_Msg512_t;

typedef const Msg512_t* ConstPtr_Msg512_t;

/**
 * @brief 768-bit message
 */
typedef union _Msg768_t
{
    unsigned long   Dwords[IN_DWORDS(SP_MSG_768_SIZE)];
    unsigned char   Bytes[IN_BYTES(SP_MSG_768_SIZE)];
} Msg768_t, *Ptr_Msg768_t;

typedef const Msg768_t* ConstPtr_Msg768_t;

#endif //__GENERIC_H
