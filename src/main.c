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
************************************************************************************************************************
* INT0 IRQ wird nur für's aufwachen benötigt
***********************************************************************************************************************/
EMPTY_INTERRUPT (INT0_vect)
/***********************************************************************************************************************
* Läßt AVR bis zur übergebenen Uhrzeit schlafen
* Nach Aufwachen wird geprüft ob die Zeit auch tatsächlich erreicht ist
* Falls der AVR "Schlafstörungen" hat, wird er wieder schlafen gelegt
***********************************************************************************************************************/
static void sleep_until (const pbcd_time_t *wakeuptime)
{
  pbcd_date_time_t cur_date_time;

  while (1) {
    set_alarm_time(wakeuptime);                             // Alarmfunktion DS3231 einschalten
    GIMSK = 0b01000000;                                     // INT0 IRQ bei Low Pegel an INT0
    sei();                                                  // IRQ Freigabe (zum aufwachen durch Low an INT0)
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);                    // AVR Tiefschlaf, wecken durch IRQ generiert vom DS3231
    sleep_mode();                                           // Gute Nacht AVR!
    cli();                                                  // Nach Aufwachen IRQs verbieten, da INT0 weiter Low ist
    get_date_and_time(&cur_date_time);                      // Aktuelle Zeit abholen, Vergleichen mit pbcd_wakeuptime
    if ((cur_date_time.time.hour == wakeuptime->hour) && \
        (cur_date_time.time.minute == wakeuptime->minute)) break;
  }
}
/***********************************************************************************************************************
* Hauptprogramm
***********************************************************************************************************************/
int main (void)
{
  pbcd_date_time_t cur_date_time;
  pbcd_time_t      wakeup_time;
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
  get_date_and_time(&cur_date_time);
  show_date_and_time(&cur_date_time);

  // Normalbetrieb, Hauptschleife
  // Byte zur LED Steuerung für aktuellen Tag laden
  // Falls Daten für Heute vorhanden:
  //   LEDs 10 Minuten ansteuern, bei Abbruch durch Tastendruck, AVR bis 01:30 schlafen legen
  //   Falls es nach 23:59 Uhr ist (Stunde == 0), AVR bis 01:30 schlafen legen
  // Falls keine Daten für Heute, AVR bis 01:30 schlafen legen
  // Nach aufwachen um 01:30 wird nur geprüft ob Datum für Wechsel ME(S)Z erreicht und die RTC ggf. umgestellt
  // Danach geht's wieder schlafen bis zur normalen täglichen Weckzeit
  while (1) {
    while (1) {
      get_date_and_time(&cur_date_time);
      if (cur_date_time.time.hour == 0x00) break;
      led_byte = get_ledbyte_for_date(&cur_date_time.date);
      if (led_byte == 0) break;
      if (led_blink_10min(led_byte) == KEY_PRESSED) break;
    }

    wakeup_time.hour = 0x01;
    wakeup_time.minute = 0x30;
    sleep_until(&wakeup_time);
    get_date_and_time(&cur_date_time);
    switch_time_on_switchdate_1_30(&cur_date_time.date);
    get_daily_wakeup_time(&wakeup_time);
    sleep_until(&wakeup_time);
  }
}
