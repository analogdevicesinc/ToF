/**
 * @file hsp_status.h
 * @brief HSP status codes
 */

#ifndef __HSP_STATUS_H
#define __HSP_STATUS_H

/**
 * @brief HSP status codes
 */
typedef enum
{
    HSP_SUCCESS,                /**< 0x0 = Success */
    HSP_UNKNOWN_ERROR,          /**< 0x1 = Unknown error */
    HSP_INVALID_PARAMETER,      /**< 0x2 = Invalid parameter */
    HSP_NOT_IMPLEMENTED,        /**< 0x3 = Not implemented */
    HSP_COMMAND_ERROR,          /**< 0x4 = Command error */
    HSP_BUS_ERROR,              /**< 0x5 = Bus error */
    HSP_FAULT_ERROR,            /**< 0x6 = Fault error */
    HSP_NOT_OWNER_ERROR,        /**< 0x7 = Not owner error */
    HSP_BUSY_STATE,             /**< 0x8 = HSP is busy */
    HSP_SIGNATURE_MISMATCH,     /**< 0x9 = Signature does not match */
    HSP_AUTHENTICATION_FAILED,  /**< 0xA = Authentication failure */
    HSP_IRQ_STATE,              /**< 0xB = trapped in IRQ */
    HSP_FIQ_STATE,              /**< 0xC = trapped in FIQ */

    //When program command is issued to a fuse slot (often key slot) that has ECC, and any data or 
    //ECC bit in the slot is already programmed (not zero)
    HSP_KEY_NOT_ZERO_ERROR,     /**< 0xD = Key not zero error */
    HSP_BLANK_CHECK_FAIL,       /**< 0xE = Fuse word check is not blank; this might not be error */
    HSP_ECC_CORRECTION_FAIL,    /**< 0xF = Ecc correction fails, having more than 1 errors */

    HSP_SVN_VERSION_MISMATCH,   /**< 0x10 = SVN in 1SP header does not match the SVN in fuses */
    HSP_INVALID_OPERATING_MODE, /**< 0x11 = Invalid operating mode */
    HSP_INVALID_CEK_KEY,        /**< 0x12 = Invalid CEK decryption key */
    HSP_DECRYPTION_FAILED,      /**< 0x13 = Decrypted payload does not match expected hash */

    HSP_SW_FAULT,               /**< 0x14 = SW Exception */
    HSP_HW_FAULT,               /**< 0x15 = Unexpected HW fault */

    HSP_FIQ_UNHANDLED,          /**< 0x16 = Unhandled FIQ error */
    HSP_FIQ_CRYPTO_ERR,         /**< 0x17 = Critical Crypt error */
    HSP_FIQ_ACC_VIO,            /**< 0x18 = Access violation */
    HSP_FIQ_MEM_EDC_ERR,        /**< 0x19 = MEM EDC error */
    HSP_FIQ_MEM_ERASE_ERR,      /**< 0x1a = Mem Erase error */
    HSP_FIQ_SP_BUS_ERR,         /**< 0x1b = SP Bus error */
    HSP_FIQ_FATAL_UNHANDLED,    /**< 0x1c = Fatal FIQ that are not handled */
    
    HSP_STATUS_USER = 0x80      /**< User-defined status codes can start at this value */
} Hsp_Status_t;

//
// Helper functions
// TODO: Add docs

//
// Get text string representation of an HSP status code

static inline const char* GetHspResultAsString(Hsp_Status_t status)
{
    switch (status)
    {
        case HSP_SUCCESS:                           return "HSP_SUCCESS";
        case HSP_UNKNOWN_ERROR:                     return "HSP_UNKNOWN_ERROR";
        case HSP_INVALID_PARAMETER:                 return "HSP_INVALID_PARAMETER";
        case HSP_NOT_IMPLEMENTED:                   return "HSP_NOT_IMPLEMENTED";
        case HSP_COMMAND_ERROR:                     return "HSP_COMMAND_ERROR";
        case HSP_BUS_ERROR:                         return "HSP_BUS_ERROR";
        case HSP_FAULT_ERROR:                       return "HSP_FAULT_ERROR";
        case HSP_NOT_OWNER_ERROR:                   return "HSP_NOT_OWNER_ERROR";
        case HSP_BUSY_STATE:                        return "HSP_BUSY_STATE";
        case HSP_SIGNATURE_MISMATCH:                return "HSP_SIGNATURE_MISMATCH";
        case HSP_AUTHENTICATION_FAILED:             return "HSP_AUTHENTICATION_FAILED";
        case HSP_IRQ_STATE:                         return "HSP_IRQ_STATE";
        case HSP_FIQ_STATE:                         return "HSP_FIQ_STATE";
		case HSP_KEY_NOT_ZERO_ERROR:                return "HSP_KEY_NOT_ZERO_ERROR";
		case HSP_BLANK_CHECK_FAIL:		            return "HSP_BLANK_CHECK_FAIL";
		case HSP_ECC_CORRECTION_FAIL:               return "HSP_ECC_CORRECTION_FAIL";
        case HSP_SVN_VERSION_MISMATCH:              return "HSP_SVN_VERSION_MISMATCH";
        case HSP_INVALID_OPERATING_MODE:            return "HSP_INVALID_OPERATING_MODE";
        case HSP_INVALID_CEK_KEY:                   return "HSP_INVALID_CEK_KEY";
        case HSP_DECRYPTION_FAILED:                 return "HSP_DECRYPTION_FAILED";
        case HSP_SW_FAULT:                          return "HSP_SW_FAULT";
        case HSP_HW_FAULT:                          return "HSP_HW_FAULT";
        default:                                    return "????";
	}
}

#endif //__HSP_STATUS_H
