/*
 * pbcd.h
 *
 * Copyright 2018 Thomas Gollmer <th_goso@freenet.de>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 3 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
 * MA 02110-1301, USA.
 *
 *
 */

#ifndef MY_PBCD
#define MY_PBCD

// ---------------------------------------------------------------------------------------------------------------------
// Inkrementiert eine Packed-BCD-Variable um eins
// pbcd_var = uint8 Variable gefüllt mit PackedBCD Wert
// Min/Max Unter/Obergrenze (Format EBENFALLS PackedBCD)
#define pbcd_inc(pbcd_var, min, max) \
  __asm__ __volatile__ (            \
  "cpi  %0,%2"            "\n\t"    \
  "brne .PBCDINCA%="      "\n\t"    \
  "ldi  %0,%1"            "\n\t"    \
  "rjmp .PBCDINCB%="      "\n\t"    \
  ".PBCDINCA%=:"          "\n\t"    \
  "subi %0,-7"            "\n\t"    \
  "brhc .PBCDINCB%="      "\n\t"    \
  "subi %0,6"             "\n\t"    \
  ".PBCDINCB%=:"          "\n\t"    \
  : "+d" (pbcd_var) : "i" (min), "i" (max))
// ---------------------------------------------------------------------------------------------------------------------
// Dekrementiert eine Packed-BCD-Variable um eins
// pbcd_var = uint8 Variable gefüllt mit PackedBCD Wert
// Min/Max Unter/Obergrenze (Format EBENFALLS PackedBCD)
#define pbcd_dec(pbcd_var, min, max) \
  __asm__ __volatile__ (            \
  "cpi  %0,%1"            "\n\t"    \
  "brne .PBCDDECA%="      "\n\t"    \
  "ldi  %0,%2"            "\n\t"    \
  "rjmp .PBCDDECB%="      "\n\t"    \
  ".PBCDDECA%=:"          "\n\t"    \
  "subi %0,1"             "\n\t"    \
  "brhc .PBCDDECB%="      "\n\t"    \
  "subi %0,6"             "\n\t"    \
  ".PBCDDECB%=:"          "\n\t"    \
  : "+d" (pbcd_var) : "i" (min), "i" (max))
// ---------------------------------------------------------------------------------------------------------------------
// Wandelt einen Packed-BCD-Wert ins normale Binärformat
#define pbcd_to_bin(pbcd_var)       \
{                                   \
  uint8_t tmp = pbcd_var >>4;       \
  pbcd_var &= 0b00001111;           \
  tmp <<=1;                         \
  pbcd_var += tmp;                  \
  tmp <<=2;                         \
  pbcd_var += tmp;                  \
}
// ---------------------------------------------------------------------------------------------------------------------

#endif
