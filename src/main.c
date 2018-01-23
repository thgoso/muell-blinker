/*
 * main.c
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
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "config.h"
#include "io_func.h"

/***********************************************************************************************************************
* Tonnen-Blinker Batterieversion mit ATTiny24
* max. 7 LEDs können angesteuert werden
*
* CPU Clock:               8 MHz (interner Oszi)
* Interne Hardware:        INT0 IRQ zum aufwecken des AVR durch den DS3231
* Abfuhr-, LED-Bytes im: - EEPROM (24c32) des DS-Boards
*                        - oder im Flash des AVR
*                        - oder im internen EEPROM des AVR (für andere AVR-Typen mit größerem EEPROM)
*                          in "config.h" wählbar
***********************************************************************************************************************/
int main (void)
{
  uint8_t  led_byte;

  // Init IO-Ports, I2C-Bus
  init_ioports();

  // Wenn Taste beim einschalten 5 Sek gedrückt --> Start Uhrzeit aus ext.EEPROM/int.EEPROM/Flash laden und RTC stellen
  // In Dauerschleife StatusLED für "Zeit gesetzt" schnell blinken lassen
  if (check_key_pressed_5s() == TRUE) {
    set_rtc();
    status_blink_endless(LED_TIME_SET);
  }

  // Wenn nach einschalten KEIN Tasterdruck, wird aktuelles Datum/Uhrzeit angezeigt
  // Dazu blinken die LEDs so oft, wie das Datums/Zeit - Teilstück angibt
  // LED0 = 5xblink, LED1 = 2xblink, LED2 = 15xblink, LED3 = 1xblink --> 5. Februar 15:01 Uhr
  // Welche LED was anzeigt, kann in config.h angepaßt werden
  delay_ms(3000);
  show_date_and_time();
  delay_ms(3000);

  // Normalbetrieb, Hauptschleife
  // Byte zur LED Steuerung für aktuellen Tag laden
  // Falls keine Daten für Heute 1,5 Sekunden warten, AVR schlafen legen
  // Falls Daten, LEDs 10 Minuten ansteuern, bei Abbruch durch Tastendruck, AVR schlafen legen
  while (1) {
    while (1) {
      if (get_ledbyte_today(&led_byte) == FALSE) {
        delay_ms(1500);
        break;
      }
      else {
        if (led_blink_10min(led_byte) == FALSE) break;
      }
    }

    set_daily_alarm();                            // RTC Alarm einschalten (nächster Tag Weckzeit)
    GIMSK = 0b01000000;                           // INT0 IRQ bei Low Pegel an INT0
    sei();                                        // IRQ Freigabe (zum aufwachen durch Low an INT0)
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);          // AVR Tiefschlaf, wecken durch IRQ INT0 (generiert vom DS3231)
    sleep_mode();                                 // Gute Nacht AVR!
    cli();                                        // Guten Morgen AVR! --> Statt Zähne putzen, weitere IRQs verbieten,
  }                                               // da der DS weiterhin INT0 auf Low zieht
}
/***********************************************************************************************************************
* Einsprung nach Aufwachen durch INT0 IRQ
* Da nix weiter gemacht wird, muß kein SREG / Register gepusht, gepopt werden
***********************************************************************************************************************/
ISR (INT0_vect, ISR_NAKED)
{
  asm ("reti");
}
/**********************************************************************************************************************/

