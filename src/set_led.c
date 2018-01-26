/*
 * set_led.c
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

#include <avr/io.h>
#include <util/delay.h>
#include "config.h"
#include "set_led.h"

// LEDs am LED_KEY_PORT setzten, am Tasteranschluss jedoch Pull-Up an lassen
#define SET_LEDS(ledbyte)     (LED_KEY_PORT = (ledbyte | (1<<KEY)))
#define CLEAR_LEDS            (LED_KEY_PORT = (0b00000000 | (1<<KEY)))

/***********************************************************************************************************************
* Endlosschleife mit schnell blinkenden LEDs zur Status/Fehleranzeige
* Übergabe: LED-Byte zum steuern von LED_KEY_PORT
***********************************************************************************************************************/
void leds_blink_endless (uint8_t led_byte)
{
  while (1) {
    CLEAR_LEDS;
    _delay_ms(233);
    SET_LEDS(led_byte);
    _delay_ms(100);
  }
}
/***********************************************************************************************************************
* LEDs werden so oft an/aus geschaltet wie n angibt, danach eine Sekunde Pause
* Übergabe: Anzahl An/Aus in n
*           LED-Byte zum steuern von LED_KEY_PORT
***********************************************************************************************************************/
void leds_blink_n_times (uint8_t n, uint8_t led_byte)
{
  for ( ;n>0; n--) {
    SET_LEDS(led_byte);
    _delay_ms(100);
    CLEAR_LEDS;
    _delay_ms(500);
  }
  _delay_ms(1000);
}
/***********************************************************************************************************************
* LEDs werden 100ms angeschaltet, dann 900ms aus gelassen, danach geht's zurück
* Sollte zwischendurch der Taster gedrückt werden: LEDs aus, Rücksprung
* Übergabe: LED-Byte zum steuern von LED_KEY_PORT
* Rückgabe: KEY_UNPRESSED wenn 1 Sekunde ohne Tasterdruck durch
*           KEY_PRESSED   wenn der Taster gedrückt wurde
***********************************************************************************************************************/
uint8_t leds_flash_100_900 (uint8_t led_byte)
{
  uint8_t cnt;

  SET_LEDS(led_byte);
  _delay_ms(100);
#if LED_MODE == MODE_BLINK
  CLEAR_LEDS;
#endif
  for (cnt=18; cnt>0; cnt--) {
    if ((LED_KEY_PIN & (1<<KEY)) == 0) return KEY_PRESSED;
    _delay_ms(50);
  }
  return KEY_UNPRESSED;
}
