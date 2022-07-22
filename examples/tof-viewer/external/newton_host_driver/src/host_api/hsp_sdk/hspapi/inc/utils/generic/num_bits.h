/**
 * @file num_bits.h
 * @brief Utility for manipulating numbers and bits
 */

#ifndef __NUM_BITS_H
#define __NUM_BITS_H

#define SetBit(A,k)        ( (A)[( (k)/32)] |= (1 << ( (k)%32)) ) 
#define ToggleBit(A,k)     ( (A)[( (k)/32)] ^= (1 << ( (k)%32)) ) 
#define ClearBit(A,k)      ( (A)[( (k)/32)] &= ~(1 << ( (k)%32)) ) 
#define TestBit(A,k)       ( (A)[( (k)/32)] & (1 << ( (k)%32)) ) 
#define GetBit(A,k)        ( TestBit((A), (k) ) >> ( (k)%32) )
#define XorBit(A,k,b)      ( (A)[( (k)/32)] ^= ( (b) << ( (k)%32)) ) 
#define PowerOfTwo(x)		(x) && !((x) & ((x)-1))

#endif //__NUM_BITS_H
