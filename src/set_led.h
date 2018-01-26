/*
 * set_led.h
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

#ifndef MY_SET_LED
#define MY_SET_LED

// ---------------------------------------------------------------------------------------------------------------------
// Endlose Statusblinkanzeige, Übergabe LED-Byte zur Steuerung
void leds_blink_endless (uint8_t led_byte) __attribute__((noreturn));
// ---------------------------------------------------------------------------------------------------------------------
// Läßt LEDs n mal aufleuchten/ausgehen, Übergabe Anzahl, LED-Byte zur Steuerung
void leds_blink_n_times (uint8_t n, uint8_t led_byte);
// ---------------------------------------------------------------------------------------------------------------------
// Schaltet LEDs 100ms an, 900ms aus, Bricht ab (mit LEDs aus) wenn zwischendurch Taster gedrückt wurde
// Übergabe: LED-Byte zur Steuerung
// Rückgabe: KEY_UNPRESSED wenn 1 Sekunde ohne Tasterdruck durch
//           KEY_PRESSED wenn Tasterdruck erfolgte

#define KEY_PRESSED    0
#define KEY_UNPRESSED  1

uint8_t leds_flash_100_900 (uint8_t led_byte);
// ---------------------------------------------------------------------------------------------------------------------

#endif
