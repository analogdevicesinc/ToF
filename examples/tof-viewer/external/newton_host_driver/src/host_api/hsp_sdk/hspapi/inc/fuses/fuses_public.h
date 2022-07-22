/**
 * @file fuses_public.h
 * @brief Header for fuse controller and fuses
 */

#ifndef __FUSES_PUBLIC_H
#define __FUSES_PUBLIC_H
#include "utils/generic/generic.h"
#include "utils/generic/hsp_status.h"
//#include "include/gfc_struct.h"
#include "utils/memory/mem_access.h"
#include "crypto/rng/rng.h"

// Constants for SocId Generation 
#define PRODUCT_IDENTIFIER_NUM_BITS         (8)
#define SOCID_HAMMING_WEIGHT                (64)
#define PRODUCT_IDENTIFIER_HSP         	    (0x0F)
#define NUM_MASK_BITS_FOR_SOCID_SIZE        (7)
#define NUM_BITS_HSP_SOCID		            (128)
#define NUM_BITS_IN_WORD		            (32)
#define MAX_ECC_BUF_WORDS_SIZE              (36)

/**
 * An enum for Security state
 */
typedef enum {
    ONE_HOT_SS_BLANK=(1<<0),     /**< enum value Blank */
    ONE_HOT_SS_TEST=(1<<1),      /**< enum value Test */
    ONE_HOT_SS_PROD=(1<<2),      /**< enum value Prod */
    ONE_HOT_SS_SECURE=(1<<3),    /**< enum value Secure */
    ONE_HOT_SS_RETEST=(1<<4),    /**< enum value Retest */
    ONE_HOT_SS_EOL=(0),          /**< enum value EOL */
} ONE_HOT_SS_VAL_t;              /**< enum variable */

#define GFC_FIRST_REG_ADDRESS             GFC_OFFSET_COMMAND 
#define GFC_LAST_REG_ADDRESS              GFC_OFFSET_ECC_CORRECTED_COUNT

#define GFC_DATA_REGS_COUNT              (9)
#define GFC_SOCID_WORD_LEN               (4)
#define GFC_PUB_KEY_WORD_LEN             (8)

#define GFC_SOFTWARE_FUSE_WORD_LEN       (4)

#define FUSE_AEB_WORD_BEGIN_ADDR       (0x4)
#define FUSE_SOCID_WORD_BEGIN_ADDR     (0x8)
#define FUSE_RSVD_WORD_BEGIN_ADDR      (0x25)
#define FUSE_RSVD0_WORD_BEGIN_ADDR      (0x25)
#define FUSE_RSVD1_WORD_BEGIN_ADDR      (0x40)
#define FUSE_RSVD_WORD_END_ADDR        (0x7F)
#define FUSE_WORD_END_ADDR             (0x7F)

#define FUSE_AEB_WORD_SIZE              (4)
#define FUSE_SOCID_WORD_SIZE            (4)
#define FUSE_SVN_WORD_SIZE              (2)  //no ecc
#define FUSE_PUB_KEY_HASH_WORD_SIZE     (8)  //+1 ecc word
#define FUSE_ENCRYPTION_KEY_WORD_SIZE  (8)  //+1 ecc word
#define FUSE_PUB_KEY_WORD_SIZE          (16) //+2 ecc word
#define FUSE_PRIVATE_KEY_WORD_SIZE      (8) //+2 ecc word, +1 attrib, +1 attrib ecc

#define FUSE_SVN_WORD_BEGIN_ADDR      FUSE_RSVD0_WORD_BEGIN_ADDR
#define FUSE_PUB_KEY_HASH_WORD_BEGIN_ADDR     (FUSE_SVN_WORD_BEGIN_ADDR + FUSE_SVN_WORD_SIZE)  
#define FUSE_ENCRYPTION_KEY_WORD_BEGIN_ADDR     (FUSE_PUB_KEY_HASH_WORD_BEGIN_ADDR + FUSE_PUB_KEY_HASH_WORD_SIZE + 1) //1 ecc word for hash 
#define FUSE_BLOCKED_MODES_WORD_BEGIN_ADDR     (FUSE_ENCRYPTION_KEY_WORD_BEGIN_ADDR + FUSE_ENCRYPTION_KEY_WORD_SIZE + 1) //1 ecc word for encryption key 


#define FUSE_RSVD_WORD_SIZE      (91)

#define FUSE_AEB_WORD_ACCESSIBLE   (1)
#define FUSE_SOCID_WORD_ACCESSIBLE (0)
#define FUSE_RSVD_WORD_ACCESSIBLE  (1)

#define GFC_COMMAND_PROG_DATA           (0x01)
#define GFC_COMMAND_READ_DATA           (0x02)
#define GFC_COMMAND_BLANK_CHK           (0x03)

#define GFC_CHANGE_SECURITY_STATE_TEST   0x10
#define GFC_CHANGE_SECURITY_STATE_PROD   0x20
#define GFC_CHANGE_SECURITY_STATE_SECURE 0x30
#define GFC_CHANGE_SECURITY_STATE_RETEST 0x40

#define GFC_STATUS_READY                   0x00
#define GFC_STATUS_BUSY                    0x01
#define GFC_STATUS_COMPLETION_SUCCESS      0x02
#define GFC_STATUS_COMMAND_INVALID         0x04
#define GFC_STATUS_KEY_NOT_ZERO_ERR        0x40
#define GFC_STATUS_BLANK_CHECK_FAIL        0x80

#define ECC_BITS_PER_WORD                   (8)
#define NUM_OF_BITS_IN_WORD                 (32)
#define NUM_UNCODED_WORDS_IN_ECC_BLOCK      (4)


typedef union _Blocked_Operation_Modes_t
{
    struct {
        uint32_t rsvd_0             :  1; // [0], always 0
        uint32_t disableUnsigned    :  1; // [1]
        uint32_t disableVendor      :  1; // [2]
        uint32_t disableMsft        :  1; // [3]
        uint32_t rsvd_31_4          : 28; // [31:4]
    };
    uint32_t u;
} Blocked_Operation_Modes_t;
typedef Blocked_Operation_Modes_t *Ptr_Blocked_Operation_Modes_t;

typedef struct _Hsp_Generic_Register_t
{
    volatile uint32_t u;
} Hsp_Generic_Register_t;
typedef Hsp_Generic_Register_t volatile *Ptr_Hsp_Generic_Register_t;

/**
 * @brief Status for fuse blank check
 */
typedef enum
{
    FUSE_CHECK_BLANK    = 0,          /**< Value of 0*/
    FUSE_CHECK_NOT_BLANK,             /**< Value of 1*/
    FUSE_CHECK_ERROR,                 /**< Value of 2*/
} FUSE_CHECK_BLANK_STATUS_t;

bool RetestTransitionAllowed(void);
void GenerateSocIdForHsp(uint32_t *socId);
Hsp_Status_t WaitFctrlDone(void);
Security_State_t ReadSecurityState(void);
Hsp_Status_t ChangeSecurityState(Security_State_t state);
Hsp_Status_t GetSocIdFromRegs(uint32_t* socId);
Hsp_Status_t GetSocIdFromFuse(uint32_t* socId);
Hsp_Status_t BurnSocIdFuse(uint32_t* socId);
Hsp_Status_t GetPublicKeyFromFuse(uint32_t pubKeyAddr, uint32_t* pubKey, uint32_t numOfWords, bool wordAccessible);
Hsp_Status_t GetPublicKeyFromRegs(uint32_t pubKeyRegAddr, uint32_t* pubKey, uint32_t numOfWords);
Hsp_Status_t BurnPublicKey(uint32_t pubKeyAddr, uint32_t* pubKey, uint32_t numOfWords, bool wordAccessible);
Hsp_Status_t GetSoftwareFuseFromFuse(uint32_t swFuseAddr, uint32_t* softwareFuse, uint32_t numOfWords);
Hsp_Status_t GetSoftwareFuseFromFuseEcc(uint32_t swFuseAddr, uint32_t* softwareFuseUncoded, uint32_t numOfWordsUncoded);
Hsp_Status_t GetSoftwareFuseFromRegs(uint32_t swFuseRegAddr, uint32_t* softwareFuse, uint32_t numOfWords);
Hsp_Status_t BurnSoftwareFuse(uint32_t swFuseAddr, uint32_t* softwareFuse, uint32_t numOfWords);
Hsp_Status_t BurnSoftwareFuseEcc(uint32_t swFuseAddr, uint32_t* softwareFuseUncoded, uint32_t numOfWordsUncoded);
Hsp_Status_t FctrlProgramWords(uint32_t fuseWordAddr, uint32_t* data, uint32_t wordLen, bool wordAccessible);
Hsp_Status_t FctrlReadFuseWords(uint32_t fuseWordAddr, uint32_t* data, uint32_t wordLen, bool wordAccessible);
Hsp_Status_t CheckFuseLoadStatus(void);
FUSE_CHECK_BLANK_STATUS_t FuseBlankCheck(uint32_t addr,uint32_t size);

/**
 * @brief Generate SOC ID for HSP
 * \b Description:
 *      1. Generate SOC ID for HSP, with 50% Hamming weight, to be burned to the fuse.
 * 
 * @param uint32_t* socId, pointer to a storage large enough to hold the SOC ID
 *
 * * @return void 
 * 
 * @see CreateSocId
 * @see BurnSocIdFuse
 * @see GetSocIdFromFuse
 */
void GenerateSocIdForHsp(uint32_t *socId);

/**
 * @brief Poll for fctrl status register available
 * 
 * \b Description:
 *      1. Read FCTRL Status register
 *      2. Poll while Busy is set.
 *      3. If any other error bits in Status are set handle Failure - trap()
 * 
 * @param void
 * 
 * @return status for the waiting result
 * 
 */
Hsp_Status_t WaitFctrlDone(void);

/**
 * @brief Read Security State of HSP
 * \b Description:
 *      1. Read FCTRL Security State register.
 * 
 * @param void
 * 
 * @return Security_State_t enum value
 */
Security_State_t ReadSecurityState(void);

/**
 * @brief Change Security State of HSP
 * \b Description:
 *      1. Change security state of HSP to a new state.  Please refer to documentation for the transition rules
 * 
 * @param Security_State_t state, security state that we want to change to
 * 
 * @return status for the state change execution
 */
Hsp_Status_t ChangeSecurityState(Security_State_t state);

/**
 * @brief Get SOC ID of HSP
 * \b Description:
 *      1. Read SOC ID register, with data loaded from fuse during hardware initialization.
 * 
 * @param uint32_t* socId, pointer to a storage large enough to hold the SOC ID
 *
 * * @return register read execution status 
 * 
 * @see CreateSocId
 * @see BurnSocIdFuse
 */
Hsp_Status_t GetSocIdFromRegs(uint32_t* socId);

/**
 * @brief Read SOC ID of HSP from fuse
 * \b Description:
 *      1. Read SOC ID from fuse.
 * 
 * @param uint32_t* socId, pointer to a storage large enough to hold the SOC ID
 *
 * * @return fuse read execution status 
 * 
 * @see CreateSocId
 * @see BurnSocIdFuse
 */
Hsp_Status_t GetSocIdFromFuse(uint32_t* socId);


/**
 * @brief Burn SOC ID fuse
 * \b Description:
 *      1. Write a SOC ID to fuse permanently.
 * 
 * @param uint32_t* socId, pointer to a storage containing the SOC ID
 * 
 * @return fuse burn execution status 
 * 
 * @see createSOCid
 */
Hsp_Status_t BurnSocIdFuse(uint32_t* socId);

/**
 * @brief Get public key of HSP from fuse
 * \b Description:
 *      1. Read public key from fuse.  The function caller needs to know about ecc support scenarios for the key.  
 *          a) If there is no ecc (software/hardware) support for the key, reading result will be ecc uncoded
 *          b) If there is ecc hardware support for the key, reading result will be ecc uncoded since the hardware 
 *              will decode the encoded fuse content, and put the result in gfc data registers
 *          c) If there is ecc software support (not hardware) for the key, reading result will be ecc encoded since the hardware 
 *              will not decode the encoded fuse content, and put the encoded result in gfc data registers.  It is the caller method's
 *              software responsibility to decode the encoded content
 * @param uint32_t pubKeyAddr,  the public key fuse address
 * @param uint32_t* pubKey, pointer to a storage large enough to hold the public key
 * @param uint32_t numOfWords, public key length in number of words
 * @param bool wordAccessible, indicate if this is burning word by word, or by a block
 * 
 * @return register read execution status
 * 
 * @see VerifySignature
 */
Hsp_Status_t GetPublicKeyFromFuse(uint32_t pubKeyAddr, uint32_t* pubKey, uint32_t numOfWords, bool wordAccessible);

/**
 * @brief Get public key of HSP from gfc registers
 * \b Description:
 *      1. Read public key registers.  These registers will load public keys from fuse on startup. If there is hardware ecc support, 
 *          the register content is ecc uncoded, since hardware will do the decoding when loading the content.
 * 
 * @param uint32_t pubKeyRegAddr,  the public key gfc register address
 * @param uint32_t* pubKey, pointer to a storage large enough to hold the public key
 * @param uint32_t numOfWords, public key length in number of words
 * 
 * @return register read execution status
 * 
 * @see VerifySignature
 */
Hsp_Status_t GetPublicKeyFromRegs(uint32_t pubKeyRegAddr, uint32_t* pubKey, uint32_t numOfWords);


/**
 * @brief Burn fuse for public key of HSP
 * \b Description:
 *      1. Write public key to fuse permanently. The function caller needs to know about ecc support scenarios for the key.  
 *          a) If there is no ecc (software/hardware) support for the key, writing content will be ecc uncoded
 *          b) If there is ecc hardware support for the key, writing content will be ecc uncoded since the hardware 
 *              will encode the content, and program the encoded content to fuse
 *          c) If there is ecc software support (not hardware) for the key, caller function need to make sure writing content ecc encoded before 
 *              calling this method, since the hardware will not encode the content.  It is the caller method's
 *              software responsibility to encode the writing content
 * 
 * @param uint32_t pubKeyAddr,  the public key fuse address
 * @param uint32_t* pubKey, pointer to a storage large enough to hold the public key
 * @param uint32_t numOfWords, public key length in number of words
 * @param bool wordAccessible, indicate if this is burning word by word, or by a block
 * 
 * @return fuse burn execution status
 * 
 * @see VerifySignature
 */
Hsp_Status_t BurnPublicKey(uint32_t pubKeyAddr, uint32_t* pubKey, uint32_t numOfWords, bool wordAccessible);

/**
 * @brief Get software fuse of HSP
 * \b Description:
 *      1. Read software fuse from fuse, which can contain product config and info, such as revocation
 *          Currently there is no hardware ecc support for software fuse.
 *          If there is ecc software support (not hardware) for the data, the resulting read content will be ecc encoded.
 *          Caller function need to decode the encoded content ecc encoded  It is the caller method's
 *          software responsibility to ecc encode and decode the content
 * 
 * @param uint32_t swFuseAddr,  the software fuse address
 * @param uint32_t* softwareFuse, pointer to a storage large enough to hold the software fuse data
 * @param uint32_t numOfWords, software fuse data length in number of words
 * 
 * @return Fuse read execution status
 * 
 * @see ReadSVN
 */
Hsp_Status_t GetSoftwareFuseFromFuse(uint32_t swFuseAddr, uint32_t* softwareFuse, uint32_t numOfWords);

/**
 * @brief Get software fuse of HSP with ecc
 * \b Description:
 *      1. Read ecc coded software fuse data from fuse; perform ecc check and correction on the data 
 *   
 * @param uint32_t swFuseAddr,  the software fuse address
 * @param uint32_t* softwareFuseUncoded, pointer to storage to hold  the corrected and decoded software fuse data
 * @param uint32_t numOfWordsUncoded, uncoded software fuse data length in number of words
 * 
 * @return Fuse read and ecc check and correct execution status
 * 
 * @see ReadSVN
 */
Hsp_Status_t GetSoftwareFuseFromFuseEcc(uint32_t swFuseAddr, uint32_t* softwareFuseUncoded, uint32_t numOfWordsUncoded);

/**
 * @brief Get software fuse of HSP
 * \b Description:
 *      1. Read software fuse register, which can contain product config and info, such as revocation.  
 *          These registers will load software fuse from fuse on startup. Currently there is no hardware ecc support, 
 *          the register content should be ecc uncoded.

 * @param uint32_t swFuseAddr,  the software fuse gfc register address
 * @param uint32_t* softwareFuse, pointer to a storage large enough to hold the software fuse data
 * @param uint32_t numOfWords, software fuse data length in number of words
 * 
 * @return register read execution status
 * 
 * @see ReadSVN
 */
Hsp_Status_t GetSoftwareFuseFromRegs(uint32_t swFuseRegAddr, uint32_t* softwareFuse, uint32_t numOfWords);

/**
 * @brief Burn software fuse of HSP
 * \b Description:
 *      1. Write software fuse data permanently, which can contain product config and info, such as revocation. 
 *          Currently there is no hardware ecc support for software fuse.
 *          If there is ecc software support (not hardware) for the data, caller function need to make sure writing content ecc encoded before 
 *          calling this method, since the hardware will not encode the content.  It is the caller method's
 *          software responsibility to encode the writing content
 * 
 * @param uint32_t swFuseAddr,  the software fuse address
 * @param uint32_t* softwareFuse, pointer to a storage large enough to hold the software fuse data
 * @param uint32_t numOfWords, software fuse data length in number of words
 * 
 * @return fuse burn execution status
 * 
 * @see ReadSVN
 */
Hsp_Status_t BurnSoftwareFuse(uint32_t swFuseAddr, uint32_t* softwareFuse, uint32_t numOfWords);

/**
 * @brief Burn software fuse of HSP with ECC
 * \b Description:
 *      1. Ecc encode words, then burn them to software fuse lot of HSP.
 * 
 * @param uint32_t swFuseAddr,  the software fuse address
 * @param uint32_t* softwareFuseUncoded, pointer to a storage ecc uncoded software fuse data
 * @param uint32_t numOfWords, software fuse data length in number of words
 * 
 * @return fuse burn execution status
 * 
 * @see ReadSVN, BurnSoftwareFuse
 */
Hsp_Status_t BurnSoftwareFuseEcc(uint32_t swFuseAddr, uint32_t* softwareFuseUncoded, uint32_t numOfWordsUncoded);

/**
 * @brief Fuse controller program word
 * \b Description:
 *      1. Fuse controller writing a word data permanently to the fuse.  It should be used during provisionning
 *         for programming device data, such as AEB, SocId, public key, software fuse, etc...
 * 
 * @param uint32_t fuseWordAddr,  the fuse word address.  Notice this is not bus address.  It comes from the fuse map 
 *                          document. E.g., Security State address is 0 in the fuse map.
 * @param uint32_t data, the data word to be burned to the fuse
 * @param uint32_t wordLen, the number of data words to be retrieved from the fuse
 * @param uint32_t wordAccessible, indicate if fuse access is word by word, or by a block.   If it is by a block, 
 *                 the address needs to be the block's start address; else, an error will occur.
 * 
 * @return word program execution status
 * 
 * @see SetAEB
 * @see UpdateAEB
 * @see BurnSocIdFuse
 * @see DeriveECCPublic
 * @see WriteDeviceKey
 */
Hsp_Status_t FctrlProgramWords(uint32_t fuseWordAddr, uint32_t* data, uint32_t wordLen, bool wordAccessible);

/**
 * @brief Fuse controller read programed words
 * \b Description:
 *      1. Fuse controller fetches word data from the fuse. 
 * 
 * @param uint32_t addres, the fuse word address.  Notice this is not bus address.  It comes from the fuse map 
 *                          document. E.g., Security State address is 0 in the fuse map.
 * @param uint32_t data, the storage pointer for data word to be retrieved from the fuse
 * @param uint32_t wordLen, the number of data words to be retrieved from the fuse
 * @param uint32_t wordAccessible, indicate if fuse access is word by word, or by a block.   If it is by a block, 
 *                 the address needs to be the block's start address; else, an error will occur.
 * 
 * @return fuse read execution status
 * 
 * @see SetAEB
 * @see UpdateAEB
 * @see BurnSocIdFuse
 * @see DeriveECCPublic
 * @see WriteDeviceKey
 */
Hsp_Status_t FctrlReadFuseWords(uint32_t fuseWordAddr, uint32_t* data, uint32_t wordLen, bool wordAccessible);

/**
 * @brief Check fuse load status
 * \b Description:
 *      1. Check if fuse loads correctly upon reset 
 * 
 * @param void
 * 
 * @return fuse load status register read; return HSP_SUCCESS if all fuse loads pass
 * 
 */
Hsp_Status_t CheckFuseLoadStatus(void);

/**
 * @brief Check if a Fuse is blank
 * \b Description:
 *      1. Function to check if a Fuse is blank
 * 
 * @param Address of the fuse is the first argument
 * @param Size of the Fuse is the second argument
 * 
 * @return  return status fuse blank,  non-blank, or error
 * 
 */
 FUSE_CHECK_BLANK_STATUS_t FuseBlankCheck(uint32_t addr,uint32_t size);


#endif //__FUSES_PUBLIC_H
