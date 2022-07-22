/**
 * @file memutils.h
 * @brief Utility functions for memory manipulation
 */

#ifndef __MEMUTILS_H
#define __MEMUTILS_H

#include "utils/generic/generic.h"

/**
 * @brief Copy memory from source in memory to destination
 * 
 * \b Description:
 *      Copies \c size bytes from \c src memory area to \c dst memory area.
 *      NOTE: Addresses and size must be 32-bit aligned.
 * 
 * @param dst Pointer to destination memory area
 * @param src Pointer to source memory area
 * @param size Size of data in bytes
 * 
 * @return Void pointer to the destination memory
 */
void* memcpy32(void* dst, const void* src, uint32_t size);

// NOTE: Do not use traditional memcmp in secure environments due to it being timing variant
#if 0
/**
 * @brief Compares memory areas
 * 
 * \b Description:
 *      Compares first \c n bytes of memory areas \c str1 and \c str2
 * 
 * @param str1 Pointer to memory area 1
 * @param str2 Pointer to memory area 2
 * @param n Size in Bytes of memory to compare
 * 
 * @return Integer less than, equal to, or greater than zero.
 *         Non zero sign is determined by the difference of the first byte of 
 *         each memory region ( \c str1 and \c str2) that is different.
 *         \n If \c n is zero, then return value is zero.
 */
int memcmp(const void* str1, const void* str2, size_t n);
#endif

/**
 * @brief Fills a block of memory with a constant value
 * 
 * \b Description:
 *      Sets \c size bytes of memory starting at \c ptr with value \c value
 *      NOTE: Address and size must be 32-bit aligned.
 * 
 * @param ptr Pointer to memory
 * @param value Value to set
 * @param size Size of memory in bytes
 * 
 * @return Void pointer to the memory
 */
void* memset32(void* ptr, int value, size_t size);

/**
 * @brief Copy memory from source to destination, reversing bytes
 * 
 * \b Description:
 *      Copies \c size bytes from \c src memory area to \c dst memory area, reversing the
 *      bytes in the process so that the last byte in \c dst is the first byte in \c src.
 *      This is typically used to deal with endianness. Note that the buffers CANNOT overlap.
 *      NOTE: Addresses and size must be 32-bit aligned.
 * 
 * @param dst Pointer to destination memory area
 * @param src Pointer to source memory area
 * @param size Size of data in bytes
 * 
 * @return void
 */
void ReverseMemCpy32(void* dst, const void* src, uint32_t size);

/**
 * @brief Reverses bytes in memory
 * 
 * \b Description:
 *      Reverses in-place the \c size bytes in \c ptr memory area, so that the last byte in
 *      the buffer becomes the first byte. This is typically used to deal with endianness.
 *      NOTE: Address and size must be 32-bit aligned.
 * 
 * @param ptr Pointer to memory
 * @param size Size of memory in bytes
 * 
 * @return void
 */
void ReverseBytes32(void* ptr, uint32_t size);

/**
 * @brief Result of memory comparison, where a success requires many bits to be set (more secure)
 */
typedef enum
{
    MEMCMP_SUCCESS = 0xFFFFFFFF,    /**< Success */
    MEMCMP_FAIL    = 0x00000000     /**< Failure */
} MemCmp_Result_t;

/**
 * @brief Compares memory in a secure manner
 * 
 * \b Description:
 *      Compares first \c n bytes of memory areas \c ptr1 and \c ptr2.  The comparison is done in
 *      secure way, where the execution time will always be the same (timing invariant), and a
 *      successful return value consists of more than a single bit being set.
 *      NOTE: Addresses and size must be 32-bit aligned.
 * 
 * @param ptr1 Pointer to memory area 1
 * @param ptr2 Pointer to memory area 2
 * @param n Size in bytes of memory to compare
 * 
 * @return \c MemCmp_Result_t indicating the result of the comparison.
 */
MemCmp_Result_t SecureMemCmp32(const void* ptr1, const void* ptr2, size_t n);

//
// Helper functions
// TODO: Add docs

// Check if the given memory address is accessible by the SP
static inline bool IsValidMemoryAddress(const void* address, size_t size)
{
    // TODO: Need better checking here
    return (address != NULL && size < 0x80000000);
}

// We expect all HSP accessible addresses to begin with 0x2F0xxxxx
#define VALID_HSP_MEMORY_ADDRESS_BASE   0x2F000000
#define VALID_HSP_MEMORY_ADDRESS_MASK   0xFFF00000
// Maximum address range size an HSP crypto block can handle  (Arbitrarily set to 64K since this is just used for sanity checking)
#define MAX_HSP_ADDRESS_RANGE_SIZE      0x10000

// Check if the given memory address is accessible by the HSP
static inline bool IsValidHspMemoryAddress(const void* address, size_t size)
{
    return (((uint32_t)address & VALID_HSP_MEMORY_ADDRESS_MASK) == VALID_HSP_MEMORY_ADDRESS_BASE && size < MAX_HSP_ADDRESS_RANGE_SIZE);
}

#endif //__MEMUTILS_H
