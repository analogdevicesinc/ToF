/**
 * @file hspcmd.h
 * @brief Header for crypto command definitions
 */

#ifndef __HSP_CMD_H
#define __HSP_CMD_H

// Command_Name                                                Command_Code                     // arg0                      arg1                     arg2           arg3        arg4
#define CMD_PKA_MOD_EXPONENTIATION_HL_256                      0x50000000                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_HL_256                0x50010000                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_256                         0x50020000                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_256                   0x50030000                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_MULTIPLICATION_256                         0x50040000                       // *result                   *a                       *b
#define CMD_PKA_MOD_ADDITION_256                               0x50050000                       // *result                   *a                       *b
#define CMD_PKA_MOD_SUBTRACTION_256                            0x50060000                       // *result                   *a                       *b
#define CMD_PKA_MOD_INVERSE_256                                0x50070000                       // *result                   *a
#define CMD_PKA_MOD_NEGATION_256                               0x50080000                       // *result                   *a
#define CMD_PKA_MOD_REDUCTION_256                              0x50090000                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_OUT_256                    0x500A0000                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_IN_256                     0x500B0000                       // *result                   *a
#define CMD_PKA_MONT_CONSTANT_CALC_256                         0x500C0000                       // *result                   *mod_prime
#define CMD_PKA_MOD_EXPONENTIATION_HL_384                      0x50000001                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_HL_384                0x50010001                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_384                         0x50020001                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_384                   0x50030001                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_MULTIPLICATION_384                         0x50040001                       // *result                   *a                       *b
#define CMD_PKA_MOD_ADDITION_384                               0x50050001                       // *result                   *a                       *b
#define CMD_PKA_MOD_SUBTRACTION_384                            0x50060001                       // *result                   *a                       *b
#define CMD_PKA_MOD_INVERSE_384                                0x50070001                       // *result                   *a
#define CMD_PKA_MOD_NEGATION_384                               0x50080001                       // *result                   *a
#define CMD_PKA_MOD_REDUCTION_384                              0x50090001                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_OUT_384                    0x500A0001                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_IN_384                     0x500B0001                       // *result                   *a
#define CMD_PKA_MONT_CONSTANT_CALC_384                         0x500C0001                       // *result                   *mod_prime
#define CMD_PKA_MOD_EXPONENTIATION_HL_1024                     0x50000002                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_HL_1024               0x50010002                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_1024                        0x50020002                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_1024                  0x50030002                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_MULTIPLICATION_1024                        0x50040002                       // *result                   *a                       *b
#define CMD_PKA_MOD_ADDITION_1024                              0x50050002                       // *result                   *a                       *b
#define CMD_PKA_MOD_SUBTRACTION_1024                           0x50060002                       // *result                   *a                       *b
#define CMD_PKA_MOD_INVERSE_1024                               0x50070002                       // *result                   *a
#define CMD_PKA_MOD_NEGATION_1024                              0x50080002                       // *result                   *a
#define CMD_PKA_MOD_REDUCTION_1024                             0x50090002                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_OUT_1024                   0x500A0002                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_IN_1024                    0x500B0002                       // *result                   *a
#define CMD_PKA_MONT_CONSTANT_CALC_1024                        0x500C0002                       // *result                   *mod_prime
#define CMD_PKA_MOD_EXPONENTIATION_HL_2048                     0x50000003                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_HL_2048               0x50010003                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_2048                        0x50020003                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_2048                  0x50030003                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_MULTIPLICATION_2048                        0x50040003                       // *result                   *a                       *b
#define CMD_PKA_MOD_ADDITION_2048                              0x50050003                       // *result                   *a                       *b
#define CMD_PKA_MOD_SUBTRACTION_2048                           0x50060003                       // *result                   *a                       *b
#define CMD_PKA_MOD_INVERSE_2048                               0x50070003                       // *result                   *a
#define CMD_PKA_MOD_NEGATION_2048                              0x50080003                       // *result                   *a
#define CMD_PKA_MOD_REDUCTION_2048                             0x50090003                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_OUT_2048                   0x500A0003                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_IN_2048                    0x500B0003                       // *result                   *a
#define CMD_PKA_MONT_CONSTANT_CALC_2048                        0x500C0003                       // *result                   *mod_prime
#define CMD_PKA_MOD_EXPONENTIATION_HL_3072                     0x50000004                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_HL_3072               0x50010004                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_3072                        0x50020004                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_3072                  0x50030004                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_MULTIPLICATION_3072                        0x50040004                       // *result                   *a                       *b
#define CMD_PKA_MOD_ADDITION_3072                              0x50050004                       // *result                   *a                       *b
#define CMD_PKA_MOD_SUBTRACTION_3072                           0x50060004                       // *result                   *a                       *b
#define CMD_PKA_MOD_INVERSE_3072                               0x50070004                       // *result                   *a
#define CMD_PKA_MOD_NEGATION_3072                              0x50080004                       // *result                   *a
#define CMD_PKA_MOD_REDUCTION_3072                             0x50090004                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_OUT_3072                   0x500A0004                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_IN_3072                    0x500B0004                       // *result                   *a
#define CMD_PKA_MONT_CONSTANT_CALC_3072                        0x500C0004                       // *result                   *mod_prime
#define CMD_PKA_MOD_EXPONENTIATION_HL_4096                     0x50000005                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_HL_4096               0x50010005                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_4096                        0x50020005                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_4096                  0x50030005                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_MULTIPLICATION_4096                        0x50040005                       // *result                   *a                       *b
#define CMD_PKA_MOD_ADDITION_4096                              0x50050005                       // *result                   *a                       *b
#define CMD_PKA_MOD_SUBTRACTION_4096                           0x50060005                       // *result                   *a                       *b
#define CMD_PKA_MOD_INVERSE_4096                               0x50070005                       // *result                   *a
#define CMD_PKA_MOD_NEGATION_4096                              0x50080005                       // *result                   *a
#define CMD_PKA_MOD_REDUCTION_4096                             0x50090005                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_OUT_4096                   0x500A0005                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_IN_4096                    0x500B0005                       // *result                   *a
#define CMD_PKA_MONT_CONSTANT_CALC_4096                        0x500C0005                       // *result                   *mod_prime
#define CMD_PKA_MOD_EXPONENTIATION_HL_192                      0x50000006                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_HL_192                0x50010006                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_192                         0x50020006                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_192                   0x50030006                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_MULTIPLICATION_192                         0x50040006                       // *result                   *a                       *b
#define CMD_PKA_MOD_ADDITION_192                               0x50050006                       // *result                   *a                       *b
#define CMD_PKA_MOD_SUBTRACTION_192                            0x50060006                       // *result                   *a                       *b
#define CMD_PKA_MOD_INVERSE_192                                0x50070006                       // *result                   *a
#define CMD_PKA_MOD_NEGATION_192                               0x50080006                       // *result                   *a
#define CMD_PKA_MOD_REDUCTION_192                              0x50090006                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_OUT_192                    0x500A0006                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_IN_192                     0x500B0006                       // *result                   *a
#define CMD_PKA_MONT_CONSTANT_CALC_192                         0x500C0006                       // *result                   *mod_prime
#define CMD_PKA_MOD_EXPONENTIATION_HL_224                      0x50000007                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_HL_224                0x50010007                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_224                         0x50020007                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_224                   0x50030007                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_MULTIPLICATION_224                         0x50040007                       // *result                   *a                       *b
#define CMD_PKA_MOD_ADDITION_224                               0x50050007                       // *result                   *a                       *b
#define CMD_PKA_MOD_SUBTRACTION_224                            0x50060007                       // *result                   *a                       *b
#define CMD_PKA_MOD_INVERSE_224                                0x50070007                       // *result                   *a
#define CMD_PKA_MOD_NEGATION_224                               0x50080007                       // *result                   *a
#define CMD_PKA_MOD_REDUCTION_224                              0x50090007                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_OUT_224                    0x500A0007                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_IN_224                     0x500B0007                       // *result                   *a
#define CMD_PKA_MONT_CONSTANT_CALC_224                         0x500C0007                       // *result                   *mod_prime
#define CMD_PKA_MOD_EXPONENTIATION_HL_521                      0x50000008                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_HL_521                0x50010008                       // *result                   *data                    key
#define CMD_PKA_MOD_EXPONENTIATION_521                         0x50020008                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_EXPONENTIATION_SMALL_521                   0x50030008                       // *result                   *base                    *exponent
#define CMD_PKA_MOD_MULTIPLICATION_521                         0x50040008                       // *result                   *a                       *b
#define CMD_PKA_MOD_ADDITION_521                               0x50050008                       // *result                   *a                       *b
#define CMD_PKA_MOD_SUBTRACTION_521                            0x50060008                       // *result                   *a                       *b
#define CMD_PKA_MOD_INVERSE_521                                0x50070008                       // *result                   *a
#define CMD_PKA_MOD_NEGATION_521                               0x50080008                       // *result                   *a
#define CMD_PKA_MOD_REDUCTION_521                              0x50090008                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_OUT_521                    0x500A0008                       // *result                   *a
#define CMD_PKA_MONT_REPRESENTATION_IN_521                     0x500B0008                       // *result                   *a
#define CMD_PKA_MONT_CONSTANT_CALC_521                         0x500C0008                       // *result                   *mod_prime
#define CMD_PKA_ECC_VERIFY_256                                 0x10000000                       // *result                   *data                    key            *signature
#define CMD_PKA_ECC_SIGN_256                                   0x10010000                       // *result                   *data                    key
#define CMD_PKA_ECC_PT_MULTIPLICATION_256                      0x10020000                       // *result                   *a                       key
#define CMD_PKA_ECC_PT_ADDITION_256                            0x10030000                       // *result                   *a                       *b
#define CMD_PKA_ECC_PT_NEGATION_256                            0x10040000                       // *result                   *a
#define CMD_PKA_ECC_PT_VALIDATION_256                          0x10050000                       // *result                   *a
#define CMD_PKA_ECC_KEY_GENERATE_256                           0x10060000                       // *result
#define CMD_PKA_ECC_VERIFY_384                                 0x10000001                       // *result                   *data                    key            *signature
#define CMD_PKA_ECC_SIGN_384                                   0x10010001                       // *result                   *data                    key
#define CMD_PKA_ECC_PT_MULTIPLICATION_384                      0x10020001                       // *result                   *a                       key
#define CMD_PKA_ECC_PT_ADDITION_384                            0x10030001                       // *result                   *a                       *b
#define CMD_PKA_ECC_PT_NEGATION_384                            0x10040001                       // *result                   *a
#define CMD_PKA_ECC_PT_VALIDATION_384                          0x10050001                       // *result                   *a
#define CMD_PKA_ECC_KEY_GENERATE_384                           0x10060001                       // *result
#define CMD_PKA_ECC_VERIFY_192                                 0x10000006                       // *result                   *data                    key            *signature
#define CMD_PKA_ECC_SIGN_192                                   0x10010006                       // *result                   *data                    key
#define CMD_PKA_ECC_PT_MULTIPLICATION_192                      0x10020006                       // *result                   *a                       key
#define CMD_PKA_ECC_PT_ADDITION_192                            0x10030006                       // *result                   *a                       *b
#define CMD_PKA_ECC_PT_NEGATION_192                            0x10040006                       // *result                   *a
#define CMD_PKA_ECC_PT_VALIDATION_192                          0x10050006                       // *result                   *a
#define CMD_PKA_ECC_KEY_GENERATE_192                           0x10060006                       // *result
#define CMD_PKA_ECC_VERIFY_224                                 0x10000007                       // *result                   *data                    key            *signature
#define CMD_PKA_ECC_SIGN_224                                   0x10010007                       // *result                   *data                    key
#define CMD_PKA_ECC_PT_MULTIPLICATION_224                      0x10020007                       // *result                   *a                       key
#define CMD_PKA_ECC_PT_ADDITION_224                            0x10030007                       // *result                   *a                       *b
#define CMD_PKA_ECC_PT_NEGATION_224                            0x10040007                       // *result                   *a
#define CMD_PKA_ECC_PT_VALIDATION_224                          0x10050007                       // *result                   *a
#define CMD_PKA_ECC_KEY_GENERATE_224                           0x10060007                       // *result
#define CMD_PKA_ECC_VERIFY_521                                 0x10000008                       // *result                   *data                    key            *signature
#define CMD_PKA_ECC_SIGN_521                                   0x10010008                       // *result                   *data                    key
#define CMD_PKA_ECC_PT_MULTIPLICATION_521                      0x10020008                       // *result                   *a                       key
#define CMD_PKA_ECC_PT_ADDITION_521                            0x10030008                       // *result                   *a                       *b
#define CMD_PKA_ECC_PT_NEGATION_521                            0x10040008                       // *result                   *a
#define CMD_PKA_ECC_PT_VALIDATION_521                          0x10050008                       // *result                   *a
#define CMD_PKA_ECC_KEY_GENERATE_521                           0x10060008                       // *result
#define CMD_AES_ECB_DECRYPT_128                                0x21000001                       // *result                   byte_count               *message       key         reserved
#define CMD_AES_CBC_DECRYPT_128                                0x22000001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CBC_DECRYPT_128_UPDATEIV                       0x22001001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CTR_DECRYPT_128                                0x23000001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CTR_DECRYPT_128_UPDATEIV                       0x23001001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CFB_DECRYPT_128                                0x24000001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CFB_DECRYPT_128_UPDATEIV                       0x24001001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_OFB_DECRYPT_128                                0x25000001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_OFB_DECRYPT_128_UPDATEIV                       0x25001001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS16_DECRYPT_128                              0x26000011                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS16_DECRYPT_128_UPDATEIV                     0x26001011                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS512_DECRYPT_128                             0x26000021                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS512_DECRYPT_128_UPDATEIV                    0x26001021                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS1K_DECRYPT_128                              0x26000031                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS1K_DECRYPT_128_UPDATEIV                     0x26001031                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS2K_DECRYPT_128                              0x26000041                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS2K_DECRYPT_128_UPDATEIV                     0x26001041                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS4K_DECRYPT_128                              0x26000051                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS4K_DECRYPT_128_UPDATEIV                     0x26001051                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_ECB_ENCRYPT_128                                0x21010001                       // *result                   byte_count               *message       key         reserved
#define CMD_AES_CBC_ENCRYPT_128                                0x22010001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CBC_ENCRYPT_12_UPDATEIV                        0x22011001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CTR_ENCRYPT_128                                0x23010001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CTR_ENCRYPT_128_UPDATEIV                       0x23011001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CFB_ENCRYPT_128                                0x24010001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CFB_ENCRYPT_128_UPDATEIV                       0x24011001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_OFB_ENCRYPT_128                                0x25010001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_OFB_ENCRYPT_128_UPDATEIV                       0x25011001                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS16_ENCRYPT_128                              0x26010011                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS16_ENCRYPT_128_UPDATEIV                     0x26011011                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS512_ENCRYPT_128                             0x26010021                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS512_ENCRYPT_128_UPDATEIV                    0x26011021                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS1K_ENCRYPT_128                              0x26010031                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS1K_ENCRYPT_128_UPDATEIV                     0x26011031                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS2K_ENCRYPT_128                              0x26010041                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS2K_ENCRYPT_128_UPDATEIV                     0x26011041                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS4K_ENCRYPT_128                              0x26010051                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_XTS4K_ENCRYPT_128_UPDATEIV                     0x26011051                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_ECB_DECRYPT_192                                0x21000002                       // *result                   byte_count               *message       key         reserved
#define CMD_AES_CBC_DECRYPT_192                                0x22000002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CBC_DECRYPT_192_UPDATEIV                       0x22001002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CTR_DECRYPT_192                                0x23000002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CTR_DECRYPT_192_UPDATEIV                       0x23001002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CFB_DECRYPT_192                                0x24000002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CFB_DECRYPT_192_UPDATEIV                       0x24001002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_OFB_DECRYPT_192                                0x25000002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_OFB_DECRYPT_192_UPDATEIV                       0x25001002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_ECB_ENCRYPT_192                                0x21010002                       // *result                   byte_count               *message       key         reserved
#define CMD_AES_CBC_ENCRYPT_192                                0x22010002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CBC_ENCRYPT_192_UPDATEIV                       0x22011002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CTR_ENCRYPT_192                                0x23010002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CTR_ENCRYPT_192_UPDATEIV                       0x23011002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CFB_ENCRYPT_192                                0x24010002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CFB_ENCRYPT_192_UPDATEIV                       0x24011002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_OFB_ENCRYPT_192                                0x25010002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_OFB_ENCRYPT_192_UPDATEIV                       0x25011002                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_ECB_DECRYPT_256                                0x21000003                       // *result                   byte_count               *message       key         reserved
#define CMD_AES_CBC_DECRYPT_256                                0x22000003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CBC_DECRYPT_256_UPDATEIV                       0x22001003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CTR_DECRYPT_256                                0x23000003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CTR_DECRYPT_256_UPDATEIV                       0x23001003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CFB_DECRYPT_256                                0x24000003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CFB_DECRYPT_256_UPDATEIV                       0x24001003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_OFB_DECRYPT_256                                0x25000003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_OFB_DECRYPT_256_UPDATEIV                       0x25001003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_ECB_ENCRYPT_256                                0x21010003                       // *result                   byte_count               *message       key         reserved
#define CMD_AES_ECB_ENCRYPT_256_UPDATEIV                       0x21011003                       // *result                   byte_count               *message       key         reserved
#define CMD_AES_CBC_ENCRYPT_256                                0x22010003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CBC_ENCRYPT_256_UPDATEIV                       0x22011003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CTR_ENCRYPT_256                                0x23010003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CTR_ENCRYPT_256_UPDATEIV                       0x23011003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CFB_ENCRYPT_256                                0x24010003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_CFB_ENCRYPT_256_UPDATEIV                       0x24011003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_OFB_ENCRYPT_256                                0x25010003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_AES_OFB_ENCRYPT_256_UPDATEIV                       0x25011003                       // *result                   byte_count               *message       key         *init_vector
#define CMD_SHA1                                               0x30010000                       // *digest                   byte_count               message_bytes  *message
#define CMD_SHA2_224                                           0x30020000                       // *digest                   byte_count               message_bytes  *message
#define CMD_SHA2_256                                           0x30030000                       // *digest                   byte_count               message_bytes  *message
#define CMD_SHA1_AUTOPAD                                       0x30010100                       // *digest                   byte_count               message_bytes  *message
#define CMD_SHA2_224_AUTOPAD                                   0x30020100                       // *digest                   byte_count               message_bytes  *message
#define CMD_SHA2_256_AUTOPAD                                   0x30030100                       // *digest                   byte_count               message_bytes  *message
#define CMD_SHA1_LOAD                                          0x30010001                       // *digest                   byte_count               message_bytes  *message    *initial_digest
#define CMD_SHA2_224_LOAD                                      0x30020001                       // *digest                   byte_count               message_bytes  *message    *initial_digest
#define CMD_SHA2_256_LOAD                                      0x30030001                       // *digest                   byte_count               message_bytes  *message    *initial_digest
#define CMD_SHA1_LOAD_AUTOPAD                                  0x30010101                       // *digest                   byte_count               message_bytes  *message    *initial_digest
#define CMD_SHA2_224_LOAD_AUTOPAD                              0x30020101                       // *digest                   byte_count               message_bytes  *message    *initial_digest
#define CMD_SHA2_256_LOAD_AUTOPAD                              0x30030101                       // *digest                   byte_count               message_bytes  *message    *initial_digest
#define CMD_CCS_SET_KEY                                        0x60000000                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_GEN_RANDOM_KEY                                 0x60000001                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_SEND_KEY                                       0x60000002                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_LOAD_KEY                                       0x60000003                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_DECRYPT_LEGACY_KEY                             0x60000004                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_STORE_KEY                                      0x60000005                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_SAVE_KEY                                       0x60000006                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_KDF_KEY                                        0x60000007                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_KDF_PCR                                        0x60000008                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_KDF_AS_PCR                                     0x60000009                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_DERIVE_ECC_PUBLIC                              0x6000000a                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_ECC_PCR_SIGN                                   0x6000000b                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_ECC_SIGN                                       0x6000000c                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_ECDH_PCR_KEY_EXCHANGE                          0x6000000d                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_ECDH_KEY_EXCHANGE                              0x6000000e                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_EXTEND_PCR                                     0x6000000f                       // *address                  *address                 *address       *address    *address _or_attribute
#define CMD_CCS_BURN_KEY                                       0x60000010                       // *address                  *address                 *address       *address    *address _or_attribute
  
#endif //__HSP_CMD_H
