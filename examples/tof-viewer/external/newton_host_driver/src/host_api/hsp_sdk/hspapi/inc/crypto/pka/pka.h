/**
 * @file pka.h
 * @brief Public Key Accelerator Hardware Block Header
 */ 
#ifndef __PKA_H
#define __PKA_H

#include "utils/generic/generic.h"
#include "crypto/hspcmd.h"

/**
 * @brief Point represented by 192-bit x and y coordinates
 */
typedef struct
{
  Msg192_t x;
  Msg192_t y;
} Point192_t;

/**
 * @brief Point represented by 256-bit x and y coordinates
 */
typedef struct
{
  Msg256_t x;
  Msg256_t y;
} Point256_t;

/**
 * @brief Point represented by 384-bit x and y coordinates
 */
typedef struct
{
  Msg384_t x;
  Msg384_t y;
} Point384_t;

/**
 * @brief Initialize the PKA block
 * 
 * \b Description:
 *      Initializes the PKA block so that it is ready to receive commands.
 *      This basically just involves waiting for the hardware to come out of reset.
 * 
 * @param void
 * 
 * @return void
 */
void PkaInit(void);

/**
 * @brief Prepare a PKA command
 * 
 * \b Description:
 *      Prepares a PKA command (in shared memory) using the given parameters so that it is ready for execution.
 * 
 * @param pkaCommand PKA command code as defined in hspcmd.h.
 * 
 * @param result Pointer to the location where the result is stored.
 * 
 * @param arg1 Pointer to argument 1
 * 
 * @param arg2 Pointer to argument 2
 * 
 * @param arg3 Pointer to argument 3
 * 
 * @return void
 */
void PkaPrepareCmd(uint32_t pkaCommand, void* result, const void* arg1, const void* arg2, const void* arg3);

/**
 * @brief Execute a PKA command
 * 
 * \b Description:
 *      Executes a PKA command (prepared by calling \c PkaPrepareCmd), optionally waiting for it to complete.
 * 
 * @return void
 */
void PkaExecuteCmd(bool waitCompletion);

/**
 * @brief Wait for a PKA command to complete
 * 
 * \b Description:
 *      Waits for a PKA command (from calling \c PkaExecuteCmd with \c waitCompletion set to \c false) to complete before returning.
 * 
 * @return void
 */
void PkaWaitCmdComplete(void);


/**
 * @brief Eliptical Curve Digital Signature Generation (192 bits)
 * 
 * \b Description:
 *      Execute PKA command to generate a 384-bit signature from a 192-bit digest.
 * 
 * @param digest Pointer to digest of the data to be signed
 * @param privateKey Pointer to the private key to sign with
 * @param signature Pointer to buffer to receive output signature
 * 
 * @return void
 */
void EcdsaGenerateSignature192(ConstPtr_Msg192_t digest, ConstPtr_Msg192_t privateKey, Ptr_Msg384_t signature);

/**
 * @brief Eliptical Curve Digital Signature Generation (256 bits)
 * 
 * \b Description:
 *      Execute PKA command to generate a 512-bit signature from a 256-bit digest.
 * 
 * @param digest Pointer to digest of the data to be signed
 * @param privateKey Pointer to the private key to sign with
 * @param signature Pointer to buffer to receive output signature
 * 
 * @return void
 */
void EcdsaGenerateSignature256(ConstPtr_Msg256_t digest, ConstPtr_Msg256_t privateKey, Ptr_Msg512_t signature);

/**
 * @brief Eliptical Curve Digital Signature Generation (384 bits)
 * 
 * \b Description:
 *      Execute PKA command to generate a 768-bit signature from a 384-bit digest.
 * 
 * @param digest Pointer to digest of the data to be signed
 * @param privateKey Pointer to the private key to sign with
 * @param signature Pointer to buffer to receive output signature
 * 
 * @return void
 */
void EcdsaGenerateSignature384(ConstPtr_Msg384_t digest, ConstPtr_Msg384_t privateKey, Ptr_Msg768_t signature);


/**
 * @brief Eliptical Curve Digital Signature Verification (192 bits)
 * 
 * \b Description:
 *      Execute PKA command to verify the signature of a 192-bit digest.
 * 
 * @param digest Pointer to digest of the data that is signed
 * @param publicKey Pointer to the public key used to verify signature
 * @param signature Pointer to signature to verify
 * 
 * @return bool \c true if verification succeeded, or \c false if it failed.
 */
bool EcdsaVerifySignature192(ConstPtr_Msg192_t digest, ConstPtr_Msg384_t publicKey, ConstPtr_Msg384_t signature);

/**
 * @brief Eliptical Curve Digital Signature Verification (256 bits)
 * 
 * \b Description:
 *      Execute PKA command to verify the signature of a 256-bit digest.
 * 
 * @param digest Pointer to digest of the data that is signed
 * @param publicKey Pointer to the public key used to verify signature
 * @param signature Pointer to signature to verify
 * 
 * @return bool \c true if verification succeeded, or \c false if it failed.
 */
bool EcdsaVerifySignature256(ConstPtr_Msg256_t digest, ConstPtr_Msg512_t publicKey, ConstPtr_Msg512_t signature);

/**
 * @brief Eliptical Curve Digital Signature Verification (384 bits)
 * 
 * \b Description:
 *      Execute PKA command to verify the signature of a 384-bit digest.
 * 
 * @param digest Pointer to digest of the data that is signed
 * @param publicKey Pointer to the public key used to verify signature
 * @param signature Pointer to signature to verify
 * 
 * @return bool \c true if verification succeeded, or \c false if it failed.
 */
bool EcdsaVerifySignature384(ConstPtr_Msg384_t digest, ConstPtr_Msg768_t publicKey, ConstPtr_Msg768_t signature);


/**
 * @brief ECC Key Pair Generation (192 bits)
 * 
 * \b Description:
 *      Execute PKA command to generate a pair of private/public keys.
 * 
 * @param privateKey Pointer to buffer to hold the private key
 * @param publicKey Pointer to buffer to hold the public key
 * 
 * @return void
 */
void EccGenerateKeyPair192(Ptr_Msg192_t privateKey, Ptr_Msg384_t publicKey);

/**
 * @brief ECC Key Pair Generation (256 bits)
 * 
 * \b Description:
 *      Execute PKA command to generate a pair of private/public keys.
 * 
 * @param privateKey Pointer to buffer to hold the private key
 * @param publicKey Pointer to buffer to hold the public key
 * 
 * @return void
 */
void EccGenerateKeyPair256(Ptr_Msg256_t privateKey, Ptr_Msg512_t publicKey);

/**
 * @brief ECC Key Pair Generation (384 bits)
 * 
 * \b Description:
 *      Execute PKA command to generate a pair of private/public keys.
 * 
 * @param privateKey Pointer to buffer to hold the private key
 * @param publicKey Pointer to buffer to hold the public key
 * 
 * @return void
 */
void EccGenerateKeyPair384(Ptr_Msg384_t privateKey, Ptr_Msg768_t publicKey);


/**
 * @brief ECC Point Multiplication (192 bits)
 * 
 * \b Description:
 *      Execute the ECC point multiplication functionality of the PKA engine.
 * \b Operation:
 *      *result = *arg1 (point) * *arg2 (scalar)
 * 
 * @param point Pointer to the point data to be multiplied
 * @param scaler Pointer to scalar data to be multiplied
 * @param result Pointer to buffer to hold result of the point multiplication
 * 
 * @return void
 */
void EccPtMultiplication192(const Point192_t* point, ConstPtr_Msg192_t scaler, Point192_t* result);

/**
 * @brief ECC Point Multiplication (256 bits)
 * 
 * \b Description:
 *      Execute the ECC point multiplication functionality of the PKA engine.
 * \b Operation:
 *      *result = *arg1 (point) * *arg2 (scalar)
 * 
 * @param point Pointer to the point data to be multiplied
 * @param scaler Pointer to scalar data to be multiplied
 * @param result Pointer to buffer to hold result of the point multiplication
 * 
 * @return void
 */
void EccPtMultiplication256(const Point256_t* point, ConstPtr_Msg256_t scaler, Point256_t* result);

/**
 * @brief ECC Point Multiplication (384 bits)
 * 
 * \b Description:
 *      Execute the ECC point multiplication functionality of the PKA engine.
 * \b Operation:
 *      *result = *arg1 (point) * *arg2 (scalar)
 * 
 * @param point Pointer to the point data to be multiplied
 * @param scaler Pointer to scalar data to be multiplied
 * @param result Pointer to buffer to hold result of the point multiplication
 * 
 * @return void
 */
void EccPtMultiplication384(const Point384_t* point, ConstPtr_Msg384_t scaler, Point384_t* result);


/**
 * @brief ECC Point Addition (192 bits)
 * 
 * \b Description:
 *      Execute the ECC point addition functionality of the PKA engine.
 * \b Operation:
 *      *result = *arg1 + *arg2 
 * 
 * @param point1 Pointer to the first value to be added
 * @param point2 Pointer to the second value to be added
 * @param result Pointer to buffer to hold the result of the point addition
 * 
 * @return void
 */
void EccPtAddition192(const Point192_t* point1, const Point192_t* point2, Point192_t* result);

/**
 * @brief ECC Point Addition (256 bits)
 * 
 * \b Description:
 *      Execute the ECC point addition functionality of the PKA engine.
 * \b Operation:
 *      *result = *arg1 + *arg2 
 * 
 * @param point1 Pointer to the first value to be added
 * @param point2 Pointer to the second value to be added
 * @param result Pointer to buffer to hold the result of the point addition
 * 
 * @return void
 */
void EccPtAddition256(const Point256_t* point1, const Point256_t* point2, Point256_t* result);

/**
 * @brief ECC Point Addition (384 bits)
 * 
 * \b Description:
 *      Execute the ECC point addition functionality of the PKA engine.
 * \b Operation:
 *      *result = *arg1 + *arg2 
 * 
 * @param point1 Pointer to the first value to be added
 * @param point2 Pointer to the second value to be added
 * @param result Pointer to buffer to hold the result of the point addition
 * 
 * @return void
 */
void EccPtAddition384(const Point384_t* point1, const Point384_t* point2, Point384_t* result);


/**
 * @brief ECC Point Negation (192 bits)
 * 
 * \b Description:
 *      Execute the ECC point negation functionality of the PKA engine.
 * \b Operation:
 *      *result = - *arg1
 * 
 * @param point Pointer to the value to be negated
 * @param result Pointer to buffer to hold the result of the point negation
 * 
 * @return void
 */
void EccPtNegation192(const Point192_t* point, Point192_t* result);

/**
 * @brief ECC Point Negation (256 bits)
 * 
 * \b Description:
 *      Execute the ECC point negation functionality of the PKA engine.
 * \b Operation:
 *      *result = - *arg1
 * 
 * @param point Pointer to the value to be negated
 * @param result Pointer to buffer to hold the result of the point negation
 * 
 * @return void
 */
void EccPtNegation256(const Point256_t* point, Point256_t* result);

/**
 * @brief ECC Point Negation (384 bits)
 * 
 * \b Description:
 *      Execute the ECC point negation functionality of the PKA engine.
 * \b Operation:
 *      *result = - *arg1
 * 
 * @param point Pointer to the value to be negated
 * @param result Pointer to buffer to hold the result of the point negation
 * 
 * @return void
 */
void EccPtNegation384(const Point384_t* point, Point384_t* result);


/**
 * @brief ECC Point Validation (192 bits)
 * 
 * \b Description:
 *      Execute the ECC point validation functionality of the PKA engine.
 * 
 * \b Operation:
 *      *result = is_valid(*arg1) ? 32'h0 : 32’h1
 * 
 * @param point Pointer to the value to be validated
 * 
 * @return bool \c true if validation succeeded, or \c false if it failed.
 */
bool EccPtValidation192(const Point192_t* point);

/**
 * @brief ECC Point Validation (256 bits)
 * 
 * \b Description:
 *      Execute the ECC point validation functionality of the PKA engine.
 * 
 * \b Operation:
 *      *result = is_valid(*arg1) ? 32'h0 : 32’h1
 * 
 * @param point Pointer to the value to be validated
 * 
 * @return bool \c true if validation succeeded, or \c false if it failed.
 */
bool EccPtValidation256(const Point256_t* point);

/**
 * @brief ECC Point Validation (384 bits)
 * 
 * \b Description:
 *      Execute the ECC point validation functionality of the PKA engine.
 * 
 * \b Operation:
 *      *result = is_valid(*arg1) ? 32'h0 : 32’h1
 * 
 * @param point Pointer to the value to be validated
 * 
 * @return bool \c true if validation succeeded, or \c false if it failed.
 */
bool EccPtValidation384(const Point384_t* point);

#endif /* __PKA_H */
