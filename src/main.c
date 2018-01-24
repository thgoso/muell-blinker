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






/***********************************************************************************************************************
* INT0 IRQ wird nur für's aufwachen benötigt
***********************************************************************************************************************/
EMPTY_INTERRUPT (INT0_vect)
/**********************************************************************************************************************/
int main (void)
{
  pbcd_date_time_t date_time;
  uint8_t          led_byte;

  // Init IO-Ports, I2C-Bus
  init_ioports();

  // Wenn beim beim einschalten Taste 5 Sek gedrückt
  // Start Uhrzeit aus ext.EEPROM/int.EEPROM/Flash laden und RTC stellen
  // In Dauerschleife StatusLED für "Zeit gesetzt" schnell blinken lassen
  set_rtc_on_keypress();

  // Wenn nach einschalten KEIN Tasterdruck, wird aktuelles Datum/Uhrzeit angezeigt
  // Dazu blinken die LEDs so oft, wie das Datums/Zeit - Teilstück angibt
  // LED0 = 5xblink, LED1 = 2xblink, LED2 = 15xblink, LED3 = 1xblink --> 5. Februar 15:01 Uhr
  // Welche LED was anzeigt, kann in config.h angepaßt werden
  // INT0 IRQ bei Low Pegel an INT0
  get_date_and_time(&date_time);
  show_date_and_time(&date_time);
  GIMSK = 0b01000000;

  // Normalbetrieb, Hauptschleife
  // Byte zur LED Steuerung für aktuellen Tag laden
  // Falls Daten für Heute vorhanden:
  //   LEDs 10 Minuten ansteuern, bei Abbruch durch Tastendruck, AVR bis 01:30 schlafen legen
  //   Falls es nach 23:59 Uhr ist (Stunde == 0), AVR bis 01:30 schlafen legen
  // Falls keine Daten für Heute, AVR bis 01:30 schlafen legen
  while (1) {
    while (1) {
      get_date_and_time(&date_time);                        // Aktuelles Datum/Uhrzeit laden
      if (date_time.time.hour==0x00) break;                 // schlafen bis 01:30 wenn Stunde == 0
      led_byte = get_ledbyte_for_date(&date_time.date);     // LED-Byte für aktuelles Datum laden
      if (led_byte == 0) break;                             // schlafen bis 01:30 wenn für heute keine LED-Daten
      if (led_blink_10min(led_byte) == KEY_PRESSED) break;  // LEDs 10 Minuten ansteuern
    }                                                       // schlafen bis 01:30 falls zwischendurch Tasterdruck

    // schlafen bis 01:30
    date_time.time.hour = 0x01;                             // Weckzeit stellen
    date_time.time.minute = 0x30;                           // Format PBCD
    set_alarm_time(&date_time.time);                        // Aufwachen um 01:30
    sei();                                                  // IRQ Freigabe (zum aufwachen durch Low an INT0)
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);                    // AVR Tiefschlaf, wecken durch IRQ generiert vom DS3231
    sleep_mode();                                           // Gute Nacht AVR!
    // Weckzeit 01:30 erreicht
    cli();                                                  // IRQs verbieten, da der DS weiterhin INT0 auf Low zieht
    get_date_and_time(&date_time);                          // Aktuelles Datum/Uhrzeit laden
    switch_time_on_switchdate_1_30(&date_time.date);        // bei Bedarf Stunde umstellen ME(S)Z auf 0 oder 2
    get_daily_wakeup_time(&date_time.time);                 // Tägliche Weckzeit aus EEPROM/Flash laden
    set_alarm_time(&date_time.time);                        // Aufwachen zur täglichen Weckzeit
    sei();                                                  // IRQ Freigabe (zum aufwachen durch Low an INT0)
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);                    // AVR Tiefschlaf, wecken durch IRQ generiert vom DS3231
    sleep_mode();                                           // Gute Nacht AVR!
    // Tägliche Weckzeit erreicht
    cli();                                                  // IRQs verbieten, da der DS weiterhin INT0 auf Low zieht
  }
}
