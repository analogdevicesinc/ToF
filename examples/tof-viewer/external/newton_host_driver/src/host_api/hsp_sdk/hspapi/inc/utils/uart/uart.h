#ifndef __UART_H_INC__ 
#define __UART_H_INC__

#include "utils/uart/uart_driver.h"
#include "platformconfig.h"

#if CONFIG_USE_UART

#define MAX_UART_PRINT_BUFFER_SIZE      (40)

/**
 * @brief UART function to print buffer in hex
 * 
 * @param buf - pointer to buffer
 * @param size - size in bytes
 * 
 * @return void
 */
void UartWriteBufferHex(const void *buf, uint32_t size);


/**
 * @brief UART function to write string
 * 
 * @param buf - pointer to buffer
 * 
 * @return void
 */
void UartWriteString(const char *buf);

/**
 * @brief UART function to write buffer and add new line
 *        at the end.
 * 
 * @param buf - pointer to buffer
 * 
 * @return void
 */
void UartWriteLine(const char *buf);

/**
 * @brief UART function to output a value as a 16-bit hex value
 * 
 * @param value - value to output
 * 
 * @return void
 */
void UartWriteHex16(uint16_t value);

/**
 * @brief UART function to output a value as a 32-bit hex value
 * 
 * @param value - value to output
 * 
 * @return void
 */
void UartWriteHex32(uint32_t value);

/**
 * @brief UART function to output a value as a decimal value
 * 
 * @param value - value to output
 * 
 * @return void
 */
void UartWriteDecimal(uint32_t value);

/**
 * @brief UART function to output a value as a signed decimal value
 * 
 * @param value - value to output
 * 
 * @return void
 */
void UartWriteSignedDecimal(int32_t value);

/**
 * @brief UART function to output a value as a binary value
 * 
 * @param value - value to output
 * 
 * @return void
 */
void UartWriteBinary(uint32_t value);

/**
 * @brief UART function to output memory as formatted hex data, 16 bytes per line
 * 
 * @param address - pointer to memory
 * @param size - number of bytes to output
 * 
 * @return void
 */
void UartDumpMemory(const void *address, uint32_t size);

/**
 * @brief UART function to output unsigned number array as binary, least significant bit first
 * 
 * @param info - point to info string about the number array
 * @param A - pointer to the unsigned number array
 * @param len - number of bits to output
 * 
 * @return void
 */
void UartDumpBits(const char *info, uint32_t *A, uint32_t len);

/**
 * @brief Flush the UART buffer
 * 
 * @param void
 * 
 * @return void
 */
void UartFlush(void);

#else

// Stub out functions in RELEASE builds
static inline void UartWriteBufferHex(const void *buf, uint32_t size)   {}
static inline void UartWriteString(const char *buf)                     {}
static inline void UartWriteLine(const char *buf)                       {}
static inline void UartWriteHex16(uint16_t value)                       {}
static inline void UartWriteHex32(uint32_t value)                       {}
static inline void UartWriteDecimal(uint32_t value)                     {}
static inline void UartWriteSignedDecimal(int32_t value)                {}
static inline void UartWriteBinary(uint32_t value)                      {}
static inline void UartDumpMemory(const void *address, uint32_t size)   {}
static inline void UartDumpBits(const char *info, uint32_t *A, uint32_t len) {}
static inline void UartFlush(void)                                      {}

#endif // CONFIG_USE_UART

#endif //#define __UART_H_INC__
