
#include "utils/generic/generic.h"

#ifndef __ECC_H
#define __ECC_H

#define NUM_BITS_IN_WORD				    (32)

/**
 * An enum for status of ECC encoded index mapping to uncoded index
 */
typedef enum {
	VALID_INDEX_FOR_ENCODED_TO_UNCODED,
	PARITY_BIT_ENCODED_POSITION,
	INVALID_INDEX_FOR_ENCODED_TO_UNCODED,
} STATUS_ECC_ENCODED_INDEX_MAPPING_TO_UNCODED_t;              /**< enum variable */

#define MAX_ECC_PARITY_BITS			(7) //Not including the extra parity bit

typedef union
{
	struct {
		uint8_t ecc[4];  //Each 8 bits can be used for encoding up to 2 words (8 bytes)
	};
	uint32_t u;
} Ecc_Parity_Bits_t;
typedef Ecc_Parity_Bits_t *Ptr_Ecc_Parity_Bits_t;

/**
 * An enum for status of ECC Check for a one/two word unit
 */
typedef enum {
	ECC_CHECK_OK,
	ECC_CHECK_ONE_BIT_ERROR,
	ECC_CHECK_TWO_BIT_ERROR,
	ECC_CHECK_UNKNOWN_ERROR,
} STATUS_ECC_CHECK_t;              

/**
 * An enum for status of ECC accumulative check for word array
 */
typedef enum {
	ECC_CHECK_ACCUM_OK = 0,
	ECC_CHECK_ACCUM_ONE_BIT_ERROR,
	ECC_CHECK_ACCUM_TWO_BIT_ERROR,
	ECC_CHECK_ACCUM_UNKNOWN_ERROR,
} STATUS_ECC_CHECK_ACCUM_t;              

typedef union
{
	struct {
		uint8_t eccOKsCount;
		uint8_t eccOneBitErrorsCount;
		uint8_t eccTwoBitErrorsCount;
		uint8_t eccUnknownErrorsCount;
	};
	uint32_t u;
} Ecc_Check_Count_t;
typedef Ecc_Check_Count_t *Ptr_Ecc_Check_Count_t;

typedef struct
{
	Ecc_Parity_Bits_t parityArray;
	uint32_t wordsArray[8];
} Ecc_Encoding_Words_t;
typedef Ecc_Encoding_Words_t *Ptr_Ecc_Encoding_Words_t;

/**
 * @brief Map from ecc encoded index to uncoded index 
 * \b Description:
 *      1. Map from ecc encoded index to uncoded index 
 * 
 * @param encodedIndex, the input encoded ecc index
 * @param uncodedIndex, the output pointer for the uncoded ecc index. If the mapping status is
 * 						parity position or invalid, uncodedIndex is not a valid result, and should not be used
 * 
 * @return  status of mapping: valid, parity position, invalid
 */
STATUS_ECC_ENCODED_INDEX_MAPPING_TO_UNCODED_t MapEccEncodedIndexToUncodedIndex(uint32_t encodedIndex, uint32_t* uncodedIndex);

/**
 * @brief Compute number of bits need to do Hamming Ecc encoding
 * \b Description:
 *      1. Compute number of bits need to do Hamming Ecc encoding
 * 
 * @param numOfBits, number of input uncoded bits
 * 
 * @return  number of Ecc bits required to encode the input bits
 */
int32_t RequiredNumOfEccBits(uint32_t numOfBits);

/**
 * @brief Hamming Ecc General method
 * \b Description:
 *      1. General form of Hamming Ecc encoding scheme, Hamming (n, k), where k = n-m,
 * 		   n is total num of coded bits, k is num of uncoded bits, m is num of parity bits.  
 * 
 * @param words, pointer to input words
 * @param n, number of total encoded bits == numOfUncodedBit + m
 * @param m, number of parity bits
 * @param eccOut, pointer to output ecc coded parity bits
 * 
 * @return  status of encoding. 0 for success;  -1 for error.
 */
int32_t HammingEccGeneral(uint32_t *words, uint32_t n, uint32_t m, uint32_t *eccOut) ;

/**
 * @brief Encode words into ecc coded words
 * \b Description:
 *      1. Encode words into ecc coded words. We use SECDED Hamming Code (72, 64).
 * 		   For every 2 words(64 bits), we use 7 ecc bits + 1 extra parity bit. 
 * 
 * @param inWords, pointer to input words
 * @param inNumOfWords, number of input words
 * @param outWords, pointer to output ecc coded words
 * 
 * @return  status of encoding. 0 for success;  -1 for error.
 */
int32_t EncodeEccForWords(uint32_t *inWords, uint32_t inNumOfWords, uint32_t *outWords) ;

/**
 * @brief Check and correct ecc coded words 
 * \b Description:
 *      1. Check and correct ecc coded words, putting correction to the output. We also put out
 * 			checking status.
 * 
 * @param inCodedWords, pointer to input ecc coded words
 * @param inNumOfWords, number of input ecc coded words
 * @param outcodedWords, pointer to output corrrected coded words
 * @param ptrEccCheckCount, counts the check results of 0, 1, 2, or unknown errors.
 * 
 * @return  accumulative status of ecc check: no error, at most 1's errors, at most 2's errors, containing unknown errors.
 */
STATUS_ECC_CHECK_ACCUM_t EccCheckForCodedWords(uint32_t *inCodedWords, uint32_t inNumOfWords, uint32_t *outCodedWords, Ptr_Ecc_Check_Count_t ptrEccCheckCount) ;

/**
 * @brief Convert ecc coded words into uncoded words
 * \b Description:
 *      1. Convert ecc coded words into uncoded words, without doing ecc check or correction.
 * 			This assumes EccCheckForCodedWords has done the check and correction 
 * 
 * @param inCodedWords, pointer to input ecc coded words
 * @param inNumOfWords, number of input ecc coded words
 * @param outUncodedWords, pointer to output uncoded words
 * 
 * @return  number of converted ecc unit, blocks of two words.  -1 indicates an error.
 */
int32_t EccConvertCodedWordsToUncoded(uint32_t *inCodedWords, uint32_t inNumOfWords, uint32_t *outUncodedWords) ;

#endif //__ECC_H