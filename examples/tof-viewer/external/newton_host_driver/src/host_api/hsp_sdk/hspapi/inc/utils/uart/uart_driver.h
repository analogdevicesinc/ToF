
//EXTERN_C_BEGIN

#ifndef __UART_SYNOPSYS_H_INC__ 
#define __UART_SYNOPSYS_H_INC__

#include "utils/generic/generic.h"
#include "platformconfig.h"

#if CONFIG_USE_UART

#define UART_RBR			        (0x00)	/* Read only */
#define UART_THR			        (0x00)	/* Write only */
#define UART_IER			        (0x04)
#define UART_IIR			        (0x08)	/* Read only */
#define UART_FCR			        (0x08)	/* Write only */
#define UART_LCR			        (0x0c)
#define UART_MCR			        (0x10)
#define UART_LSR			        (0x14)
#define UART_MSR			        (0x18)
#define UART_SCR			        (0x1c)
#define UART_DLL			        (0x00)	/* Only when LCR.DLAB = 1 */
#define UART_DLH			        (0x04)	/* Only when LCR.DLAB = 1 */

#define UART_FAR			        (0x70)	
#define UART_TFR 			        (0x74)	/* Only when LCR = 0xbf */
#define UART_RFW 			        (0x78)	/* Only when LCR = 0xbf */
#define UART_USR  			        (0x7c)	/* Only when LCR = 0xbf */
#define UART_TFL                    (0x80)	/* Only when LCR = 0xbf */
#define UART_RFL                    (0x84)
#define UART_HTX                    (0xa4)
#define UART_DMASA                  (0xa8)
#define UART_CPR                    (0xf4)
#define UART_UCV                    (0xf8)
#define UART_CTR                    (0xfc)

/* IER */
#define UART_IER_ERBFI              (1<<0)
#define UART_IER_ETBEI              (1<<1)
#define UART_IER_ELSI               (1<<2)

/* IIR */
#define UART_IIR_ID_SHIFT           (0)
#define UART_IIR_ID_MASK            (0x3F)
#define UART_IIR_LSR                (0x06)
#define UART_IIR_RXDATA_TMO         (0x0C)
#define UART_IIR_RXDATA_RECEIVED    (0x04)
#define UART_IIR_TX_EMPTY           (0x02)
#define UART_IIR_NO_INTERRUPT       (0x01)

/* FCR */
#define UART_FCR_FIFOE              (1 << 0)
#define UART_FCR_CLRR               (1 << 1)
#define UART_FCR_CLRT               (1 << 2)
#define UART_FCR_DMA1               (1 << 3)
#define UART_FCR_RXFIFO_1B_TRI      (0 << 6)
#define UART_FCR_RXFIFO_6B_TRI      (1 << 6)
#define UART_FCR_RXFIFO_12B_TRI     (2 << 6)
#define UART_FCR_RXFIFO_RX_TRI      (3 << 6)
#define UART_FCR_TXFIFO_1B_TRI      (0 << 4)
#define UART_FCR_TXFIFO_4B_TRI      (1 << 4)
#define UART_FCR_TXFIFO_8B_TRI      (2 << 4)
#define UART_FCR_TXFIFO_14B_TRI     (3 << 4)

#define UART_FCR_FIFO_INIT          (UART_FCR_FIFOE|UART_FCR_CLRR|UART_FCR_CLRT)
#define UART_FCR_NORMAL             (UART_FCR_FIFO_INIT | \
                                     UART_FCR_TXFIFO_4B_TRI| \
                                     UART_FCR_RXFIFO_12B_TRI)
/*---------------------------------------------------------------------------*/
/* LCR */
#define UART_LCR_BREAK              (1 << 6)
#define UART_LCR_DLAB               (1 << 7)

#define UART_WLS_5                  (0 << 0)
#define UART_WLS_6                  (1 << 0)
#define UART_WLS_7                  (2 << 0)
#define UART_WLS_8                  (3 << 0)
#define UART_WLS_MASK               (3 << 0)

#define UART_1_STOP                 (0 << 2)
#define UART_2_STOP                 (1 << 2)
#define UART_1_5_STOP               (1 << 2)    /* Only when WLS=5 */
#define UART_STOP_MASK              (1 << 2)

#define UART_NONE_PARITY            (0 << 3)
#define UART_ODD_PARITY             (0x1 << 3)
#define UART_EVEN_PARITY            (0x3 << 3)
#define UART_MARK_PARITY            (0x5 << 3)
#define UART_SPACE_PARITY           (0x7 << 3)
#define UART_PARITY_MASK            (0x7 << 3)
/*---------------------------------------------------------------------------*/
/* MCR */
#define UART_MCR_DTR                (1 << 0)
#define UART_MCR_RTS                (1 << 1)
#define UART_MCR_OUT1               (1 << 2)
#define UART_MCR_OUT2               (1 << 3)
#define UART_MCR_LOOP               (1 << 4)
#define UART_MCR_DCM_EN             (1 << 5)    /* MT6589 move to bit5 */
#define UART_MCR_XOFF               (1 << 7)    /* read only */
#define UART_MCR_NORMAL	            (UART_MCR_DTR|UART_MCR_RTS)
/*---------------------------------------------------------------------------*/
/* LSR */
#define UART_LSR_DR                 (1 << 0)
#define UART_LSR_OE                 (1 << 1)
#define UART_LSR_PE                 (1 << 2)
#define UART_LSR_FE                 (1 << 3)
#define UART_LSR_BI                 (1 << 4)
#define UART_LSR_THRE               (1 << 5)
#define UART_LSR_TEMT               (1 << 6)
#define UART_LSR_FIFOERR            (1 << 7)

/*---------------------------------------------------------------------------*/
/* MSR */
#define UART_MSR_DCTS               (1 << 0)
#define UART_MSR_DDSR               (1 << 1)
#define UART_MSR_TERI               (1 << 2)
#define UART_MSR_DDCD               (1 << 3)
#define UART_MSR_CTS                (1 << 4)
#define UART_MSR_DSR                (1 << 5)
#define UART_MSR_RI                 (1 << 6)
#define UART_MSR_DCD                (1 << 7)

/*---------------------------------------------------------------------------*/
/* EFR */
#define UART_EFR_RTS                (1 << 6)
#define UART_EFR_CTS                (1 << 7)

//! Defaults
#ifdef BUILD_FOR_FPGA
#define UART_DEFAULT_XTAL_FREQ      (24000000)  // 24 MHz Clock on FPGA
#else
#define UART_DEFAULT_XTAL_FREQ      (136000000) // 136 MHz PLL
#endif

#define UART_DEFAULT_BAUD_RATE 		(115200)
#define UART_DEFAULT_DATA_BIT		(UART_WLS_8)
#define UART_DEFAULT_PARITY			(UART_NONE_PARITY)
#define UART_DEFAULT_STOP_BIT		(UART_1_STOP)

#define FRAC_BRG_BITS       6
#define FRAC_BRG_VALUE      (2^FRAC_BRG_BITS)

/**
 * @brief Uart init function 
 * 
 * @return void
 */
void UartInit(void);

/**
 * @brief Write 
 * 
 * @param data - input data buffer
 * 
 * @return void
 */
void WriteChar(char data);

/**
 * @brief Read 
 * 
 * @param data - input data buffer
 * 
 * @return void
 */
void ReadChar(char* data);

/**
 * @brief Flush UART buffer
 * 
 * @param void
 * 
 * @return void
 */
void FlushUart(void);

#else

// Stub out functions in configurations that don't have UART
static inline void UartInit(void)           {}
static inline void WriteChar(char data)     {}
static inline void ReadChar(char* data)     {}
static inline void FlushUart(void)          {}

#endif // CONFIG_USE_UART

#endif // __UART_SYNOPSYS_H_INC__
