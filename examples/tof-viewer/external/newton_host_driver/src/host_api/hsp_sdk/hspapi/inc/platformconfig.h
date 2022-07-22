/**
 * @file platformconfig.h
 * @brief Configuration file for specific platforms
 */

/** Processor architecture config defines */
#define CONFIG_PROC_RISCV 1
#define CONFIG_ARM_CM4 0

/** Crypto HW config defines */
#define CONFIG_USE_CHKPT 0

/** Platform Flash devices defines */
#define CONFIG_USE_FLASH 0

/** Platform HW defines */
#ifdef DEBUG
  #define CONFIG_USE_UART 1
#endif

/** Include verbose error info, such as faulting file and line number, when calling critical error handler (Typically only for debug builds) */
#ifdef DEBUG
  #define CONFIG_INCLUDE_VERBOSE_ERROR_INFO 0
#endif

/** Platform straps defines */
#define CONFIG_USE_STRAPS 0

/** Platform development platform defines */
#define CONFIG_FPGA_EMULATION 1