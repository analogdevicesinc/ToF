#ifndef MEM_ACCESS_H
#define MEM_ACCESS_H

#include <stdint.h>

static inline void hsp_write32(volatile uint32_t* addr, uint32_t data) { *addr = data; }
static inline uint32_t hsp_read32(volatile uint32_t* addr) { return *addr; }

#endif /* MEM_ACCESS_H */