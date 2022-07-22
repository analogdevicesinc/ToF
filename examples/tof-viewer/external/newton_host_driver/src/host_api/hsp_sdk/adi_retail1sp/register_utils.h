/**
 * @file register_utils.h
 * @brief Helper functions to read/write to riscv registers
 */

#ifndef __REG_UTILS_H
#define __REG_UTILS_H

#define RISCV_MCAUSE_INTERRUPT      (0x80000000)
#define RISCV_MCAUSE_CAUSE          (0x000003ff)

#define RISCV_MACHINE_SOFTWARE_INTERRUPT        (3)
#define RISCV_MACHINE_TIMER_INTERRUPT           (7)
#define RISCV_MACHINE_EXTERNAL_INTERRUPT        (11)
#define RISCV_MACHINE_CLIC_SOFTWARE_INTERRUPT   (12)
#define RISCV_LOCAL_INTERRUPT_0                 (16)
#define RISCV_LOCAL_INTERRUPT_1                 (17)

#define CLIC_DIRECT_MODE            (0x2)
#define CLIC_BASE_ADDRESS           (0x02000000)
#define CLIC_HART0_OFFSET           (0x800000)
#define CLIC_IE_OFFSET              (0x400)
#define CLIC_IP_OFFSET              (0)


typedef union _Riscv_MSTATUS_t
{
    struct {
        uint32_t reserved_2_0         :  3; // [2:0]
        uint32_t mie                  :  1; // [3]
        uint32_t reserved_6_4         :  3; // [6:4]
        uint32_t mpie                 :  1; // [7]
        uint32_t reserved_10_8        :  3; // [10:8]
        uint32_t mpp                  :  2; // [12:11]
        uint32_t reserved_31_13       :  19; // [31_13]
    };
    volatile uint32_t u;
} Riscv_MSTATUS_t;

// TODO: add docs

static inline void WriteMtvec (uint32_t wr_val) {
  asm volatile ("csrw mtvec, %0" : : "r"(wr_val));
}
static inline void WriteMtvt (uint32_t wr_val) {
  asm volatile ("csrw mtvt, %0" : : "r"(wr_val));
}

static inline uint32_t ReadMtvec (void) {
  uint32_t mtvec;
  asm volatile ("csrr %0, mtvec" : "=r"(mtvec));
  return  mtvec;
}

static inline void WriteStatus (uint32_t wr_val) {
  asm volatile ("csrw mstatus, %0" : : "r"(wr_val));
}

static inline uint32_t ReadStatus(void) {
  uint32_t mstatus;
  asm volatile ("csrr %0, mstatus" : "=r"(mstatus));
  return mstatus;
}

static inline uint32_t ReadSp (void) {
  uint32_t regVal;
  asm volatile ("add  %0, x0, x2" : "=r"(regVal));
  return  regVal;
}

static inline uint32_t ReadMepc (void) {
  uint32_t mepc;
  asm volatile ("csrr %0, mepc" : "=r"(mepc));
  return  mepc;
}

static inline uint32_t ReadMcause (void) {
  uint32_t mcause;
  asm volatile ("csrr %0, mcause" : "=r"(mcause));
  return  mcause;
}


#endif //#define __REG_UTILS_H
