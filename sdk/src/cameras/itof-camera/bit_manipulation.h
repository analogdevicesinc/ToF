/*
 * libcrc - Library which calculates CRC checksums
 *
 * Copyright (C) 2016 Marco Guerri <marco.guerri.dev@fastmail.com>
 *
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 * 
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 * 
 * You should have received a copy of the GNU General Public License
 * along with this program.  If not, see <http://www.gnu.org/licenses/>
 */
#ifndef __BIT_MANIPULATION_H__
#define __BIT_MANIPULATION_H__

#include <stdint.h> 
void invert(void *payload, uint8_t bit_size);
void reverse(void *payload, uint8_t bit_size);
void shift_left(void *payload, uint8_t bit_size, uint32_t shift_len);
void xor(void *payload, void *mask, uint8_t bit_size);
uint8_t get_bit(void *payload, uint8_t bit_size, uint8_t bitno);

uint32_t reflect32(uint32_t in_byte);
uint8_t reflect8(uint8_t in_byte);

#endif
