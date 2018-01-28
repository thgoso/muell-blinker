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
#include <util/delay.h>
#include <avr/sleep.h>
#include <avr/interrupt.h>
#include "config.h"
#include "load_data.h"
#include "ds3231.h"
#include "set_led.h"

/***********************************************************************************************************************
* Tonnen-Blinker Batterieversion mit ATTiny24
* max. 7 LEDs können angesteuert werden
*
* CPU Clock:               1 MHz (interner Oszi)
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
/***********************************************************************************************************************
* Läßt AVR bis zur übergebenen Uhrzeit schlafen
* Nach Aufwachen wird geprüft ob die Zeit auch tatsächlich erreicht ist
* Falls der AVR "Schlafstörungen" hat, wird er wieder schlafen gelegt
***********************************************************************************************************************/
static void sleep_until (uint8_t pbcd_hour, uint8_t pbcd_minute)
{
  uint8_t cur_hour, cur_minute;

  while (1) {
    ds_set_alarm_time(pbcd_hour, pbcd_minute);    // Alarmfunktion DS3231 einschalten (JETZT ist INT0 = High)
    sei();                                        // IRQ Freigabe (zum aufwachen durch Low an INT0)
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);          // AVR Tiefschlaf, wecken durch IRQ generiert vom DS3231
    sleep_mode();                                 // Gute Nacht AVR!
    cli();                                        // Nach Aufwachen IRQs verbieten, da INT0 weiter Low ist
    cur_hour = ds_read_reg(DS_HOURS);             // Aktuelle Zeit abholen, Vergleichen mit pbcd_wakeuptime
    cur_minute = ds_read_reg(DS_MINUTES);         // Ende wenn gleich
    if ((cur_hour == pbcd_hour) && (cur_minute == pbcd_minute)) break;
  }
}
/***********************************************************************************************************************
* Wenn Taste 5 Sekunden durchgehend gedrückt:
* Initialwerte Startzeit/Datum aus EEPROM/Flash laden und RTC stellen (Startzeit/Datum)
* Endlosschleife Statusblinken
* Wenn nicht gedrückt, Rückkehr ohne Änderungen
***********************************************************************************************************************/
static void set_rtc_on_keypress (void)
{
  uint8_t tmp;

  for (tmp=50; tmp>0; tmp--) {
    _delay_ms(100);
    if ((LED_KEY_PIN & (1<<KEY)) != 0) return;
  }

  // Startsekunde auf "05" stellen, da beim Einschalten Taste 5 Sekunden gedrückt wurde
  ds_write_reg(DS_SECONDS, 0x05);
  tmp = load_pbcd_byte(ADR_RTC_START_MINUTE); ds_write_reg(DS_MINUTES, tmp);
  tmp = load_pbcd_byte(ADR_RTC_START_HOUR);   ds_write_reg(DS_HOURS, tmp);
  tmp = load_pbcd_byte(ADR_RTC_START_DATE);   ds_write_reg(DS_DATE, tmp);
  tmp = load_pbcd_byte(ADR_RTC_START_MONTH);  ds_write_reg(DS_MONTH, tmp);
  tmp = load_pbcd_byte(ADR_RTC_START_YEAR);   ds_write_reg(DS_YEAR, tmp);

  // Endlosschleife Statusblinken
  leds_blink_endless(LED_TIME_SET);
}
/***********************************************************************************************************************
* Läßt LEDs so oft blinken wie die Zeitbytes dies angeben
* Keine Tastenabfrage während blinken
*
* z.B. es ist der 10. Febraur 15:10 Uhr
* LED_DATE blinkt   10 mal
* LED_MONTH blinkt   2 mal
* LED_HOUR blink    15 mal
* LED_MINUTE blinkt 10 mal
***********************************************************************************************************************/
static void show_date_and_time (void)
{
  uint8_t tmp;

  tmp = ds_read_reg(DS_DATE);    leds_blink_pbcd_times(tmp, LED_DATE);
  tmp = ds_read_reg(DS_MONTH);   leds_blink_pbcd_times(tmp, LED_MONTH);
  tmp = ds_read_reg(DS_HOURS);   leds_blink_pbcd_times(tmp, LED_HOUR);
  tmp = ds_read_reg(DS_MINUTES); leds_blink_pbcd_times(tmp, LED_MINUTE);
}
/***********************************************************************************************************************
* Läd zum aktuellen Datum das LED-Byte (Abfuhrdaten) und steuert LEDs passend dazu an
* Kehrt zurück sobald der Taster gedrückt wurde, die reguläre Schlafenszeit erreicht ist oder keine Daten für Heute
***********************************************************************************************************************/
static void work_until_sleeptime (void)
{
  uint8_t cur_minute, cur_hour;
  uint8_t sleep_hour, sleep_minute;
  uint8_t led_byte;

  while (1) {
    // Schlafenszeit aus EEPROM, Flash laden
    // DS3231 Alarm auf diese Zeit setzten, DS zieht INT0 auf Low wenn Alarmzeit erreicht/überschritten
    sleep_hour = load_pbcd_byte(ADR_SLEEP_HOUR);
    sleep_minute = load_pbcd_byte(ADR_SLEEP_MINUTE);
    ds_set_alarm_time(sleep_hour, sleep_minute);

    // LED-Byte zum aktuellen Datum holen, gleich raus, wenn zu heute keine Daten vorliegen
    led_byte = load_led_byte_for_date((ds_read_reg(DS_DATE)), (ds_read_reg(DS_MONTH)));
    if (led_byte == 0) return;

    // LEDs steuern bis Tasterdruck, oder Schlafenszeit erreicht
    while (1) {
      if (leds_flash_100_900(led_byte) == KEY_PRESSED) return;
      // DS gibt Alarm... nachprüfen ob's auch stimmt
      if (READ_INT0PIN == 0) {
        cur_minute = ds_read_reg(DS_MINUTES);
        cur_hour = ds_read_reg(DS_HOURS);
        // Es ist Schlafenszeit, dann raus
        if ((cur_hour == sleep_hour) && (cur_minute == sleep_minute)) return;
        // DS hat Fehlalarm ausgelöst
        else break;
      }
    }
  }
}
/***********************************************************************************************************************
* Prüft ob Datum zur Zeitumstellung ME(S)Z ist
* Falls ja, wird das Stundenregister des DS3231 angepaßt
* Falls nein, geht's ohne Änderungen zurück
*
* Funktion MUSS IMMER Nachts um 01:30 aufgerufen werden
***********************************************************************************************************************/
static void work_on_1_30 (void)
{
  uint8_t tmp = ds_read_reg(DS_MONTH);

  switch (tmp) {
    case 0x03:
      tmp = ds_read_reg(DS_DATE);
      if (load_pbcd_byte(ADR_MESZ_MAR) == tmp) ds_write_reg(DS_HOURS, 0x02);
      break;
    case 0x10:
      tmp = ds_read_reg(DS_DATE);
      if (load_pbcd_byte(ADR_MESZ_OCT) == tmp) ds_write_reg(DS_HOURS, 0x00);
      break;
  }
}
/**********************************************************************************************************************/
int main (void)
{
  // Init IO-Ports, I2C-Bus
  INIT_IO;

  // Wenn beim beim einschalten Taste 5 Sek gedrückt
  // Start Uhrzeit aus ext.EEPROM/int.EEPROM/Flash laden und RTC stellen
  // In Dauerschleife StatusLED für "Zeit gesetzt" schnell blinken lassen
  set_rtc_on_keypress();

  // Wenn nach einschalten KEIN Tasterdruck, wird aktuelles Datum/Uhrzeit angezeigt
  // Dazu blinken die LEDs so oft, wie das Datums/Zeit - Teilstück angibt
  // LED0 = 5xblink, LED1 = 2xblink, LED2 = 15xblink, LED3 = 1xblink --> 5. Februar 15:01 Uhr
  // Welche LED was anzeigt, kann in config.h angepaßt werden
  show_date_and_time();

  // Normalbetrieb, Hauptschleife
  while (1) {
    // LEDs werden passend zum Datum gesteuert bis Tasterdruck, Schlafenszeit erreicht, gar keine Daten für heute
    work_until_sleeptime();
    // Schlafen bis 01:30
    sleep_until(0x01, 0x30);
    // ggf. Zeit umstellen ME(S)Z
    work_on_1_30();
    // schlafen bis reguläre Weckzeit
    sleep_until((load_pbcd_byte(ADR_WAKEUP_HOUR)), (load_pbcd_byte(ADR_WAKEUP_MINUTE)));
  }
}
