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

// ---------------------------------------------------------------------------------------------------------------------
// Hilfsfunktionen
/***********************************************************************************************************************
* Konvertiert ein übergebenes Byte vom Packed-BCD nach normal binär
* Übergabe: Byte im PBCD-Format
* Rückgabe: Wert gewandelt
***********************************************************************************************************************/
static inline void __attribute__((always_inline)) pbcd_to_bin (uint8_t *byte)
{
  uint8_t tmp = *byte >>4;                        // tmp = 10er-Stelle
  *byte &= 0b00001111;                            // ret = 1er
  tmp <<=1;                                       // 10er-Stelle * 2
  *byte += tmp;                                   // ret = 1er + 10er*2
  tmp <<=2;                                       // 10er-Stelle * 8
  *byte += tmp;                                   // ret = 1er + 10er*2 + 10er*8
}
/***********************************************************************************************************************
* Liest die Abfuhrdaten für übergebenes Datum aus dem int.EEPROM/ext.EEPROM/Flash
* Gibt das Byte zur LED Ansteuerung zurück
* Wenn heute KEINE Abholung, Rückgabe = 0
***********************************************************************************************************************/
static uint8_t get_ledbyte_for_date (uint8_t pbcd_date, uint8_t pbcd_month)
{
  uint16_t adr;
  uint8_t  retval;

  pbcd_to_bin(&pbcd_date);                        // Beide wandeln PBCD --> Binär
  pbcd_to_bin(&pbcd_month);                       // Adresse (EEPROM/Array) für Datum berechnen: ((Monat-1)*32)+(Tag-1)
  pbcd_date--;                                    // 0...30
  pbcd_month--;                                   // 0...11
  adr = pbcd_month;                               // adr (16Bit) = 0...11
  adr <<=5;                                       // * 32 = 0,32,64,96,128,160,192,224,256,288,320,352
  adr += pbcd_date;                               // + Tag 0...30
  retval = load_data_byte(adr);                   // LED-Byte (Abfuhrdaten) aus int.EEPROM/ext.EEPROM/Flash laden
  retval &= ~(1<<KEY);                            // KEY-PIN ausmaskieren/löschen
  return retval;
}
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
    GIMSK = 0b01000000;                           // INT0 IRQ bei Low Pegel an INT0
    sei();                                        // IRQ Freigabe (zum aufwachen durch Low an INT0)
    set_sleep_mode(SLEEP_MODE_PWR_DOWN);          // AVR Tiefschlaf, wecken durch IRQ generiert vom DS3231
    sleep_mode();                                 // Gute Nacht AVR!
    cli();                                        // Nach Aufwachen IRQs verbieten, da INT0 weiter Low ist
    cur_hour = ds_read_reg(DS_HOURS);             // Aktuelle Zeit abholen, Vergleichen mit pbcd_wakeuptime
    cur_minute = ds_read_reg(DS_MINUTES);         // Ende wenn gleich
    if ((cur_hour == pbcd_hour) && (cur_minute == pbcd_minute)) break;
  }
}
// ---------------------------------------------------------------------------------------------------------------------
// Hauptfunktionen
/***********************************************************************************************************************
* Wenn Taste 5 Sekunden durchgehend gedrückt:
* Initialwerte Startzeit/Datum aus EEPROM/Flash laden und RTC stellen (Startzeit/Datum)
* Endlosschleife Statusblinken
* Wenn nicht gedrückt, Rückkehr ohne Änderungen
***********************************************************************************************************************/
static void Set_RTC_On_Keypress (void)
{
  uint8_t tmp;

  for (tmp=50; tmp>0; tmp--) {
    _delay_ms(100);
    if ((LED_KEY_PIN & (1<<KEY)) != 0) return;
  }

  // Startsekunde auf "05" stellen, da beim Einschalten Taste 5 Sekunden gedrückt wurde
  ds_write_reg(DS_SECONDS, 0x05);
  tmp = load_data_byte(ADR_RTC_START_MINUTE); ds_write_reg(DS_MINUTES, tmp);
  tmp = load_data_byte(ADR_RTC_START_HOUR);   ds_write_reg(DS_HOURS, tmp);
  tmp = load_data_byte(ADR_RTC_START_DATE);   ds_write_reg(DS_DATE, tmp);
  tmp = load_data_byte(ADR_RTC_START_MONTH);  ds_write_reg(DS_MONTH, tmp);
  tmp = load_data_byte(ADR_RTC_START_YEAR);   ds_write_reg(DS_YEAR, tmp);

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
static void Show_Date_And_Time (void)
{
  uint8_t tmp;

  tmp = ds_read_reg(DS_DATE);    pbcd_to_bin(&tmp); leds_blink_n_times(tmp, LED_DATE);
  tmp = ds_read_reg(DS_MONTH);   pbcd_to_bin(&tmp); leds_blink_n_times(tmp, LED_MONTH);
  tmp = ds_read_reg(DS_HOURS);   pbcd_to_bin(&tmp); leds_blink_n_times(tmp, LED_HOUR);
  tmp = ds_read_reg(DS_MINUTES); pbcd_to_bin(&tmp); leds_blink_n_times(tmp, LED_MINUTE);
}
/***********************************************************************************************************************
* Läd zum aktuellen Datum das LED-Byte (Abfuhrdaten) und steuert LEDs passend dazu an
* Kehrt zurück sobald der Taster gedrückt wurde, die reguläre Schlafenszeit erreicht ist oder keine Daten für Heute
***********************************************************************************************************************/
static void Work_Until_Sleeptime (void)
{
  uint8_t cur_minute, cur_hour;
  uint8_t sleep_hour, sleep_minute;
  uint8_t led_byte;

  while (1) {
    // Schlafenszeit aus EEPROM, Flash laden
    // DS3231 Alarm auf diese Zeit setzten, DS zieht INT0 auf High wenn Alarmzeit erreicht/überschritten
    sleep_hour = load_data_byte(ADR_SLEEP_HOUR);
    sleep_minute = load_data_byte(ADR_SLEEP_MINUTE);
    ds_set_alarm_time(sleep_hour, sleep_minute);

    // LED-Byte zum aktuellen Datum holen, gleich raus, wenn zu heute keine Daten vorliegen
    led_byte = get_ledbyte_for_date((ds_read_reg(DS_DATE)), (ds_read_reg(DS_MONTH)));
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
* Läßt AVR bis 01:30 Uhr schlafen
***********************************************************************************************************************/
static void Sleep_Until_1_30 (void)
{
  sleep_until(0x01, 0x30);
}
/***********************************************************************************************************************
* Prüft ob Datum zur Zeitumstellung ME(S)Z ist
* Falls ja, wird das Stundenregister des DS3231 angepaßt
* Falls nein, geht's ohne Änderungen zurück
*
* Funktion MUSS IMMER Nachts um 01:30 aufgerufen werden
***********************************************************************************************************************/
static void Work_On_1_30 (void)
{
  uint8_t cur_date = ds_read_reg(DS_DATE);
  uint8_t cur_month = ds_read_reg(DS_MONTH);

  switch (cur_month) {
    case 0x03:
      // Heute Zeitumstellung "??.03" eine Stunde vor von 1 Uhr auf 2 Uhr
      if (load_data_byte(ADR_MESZ_MAR) == cur_date) ds_write_reg(DS_HOURS, 0x02);
      break;
    case 0x10:
      // Heute Zeitumstellung "??.10" eine Stunde zurück von 1 Uhr auf 0 Uhr
      if (load_data_byte(ADR_MESZ_OCT) == cur_date) ds_write_reg(DS_HOURS, 0x00);
      break;
  }
}
/***********************************************************************************************************************
* Läßt AVR bis zur normalen Weckzeit schlafen
***********************************************************************************************************************/
static void Sleep_Until_Wakeuptime (void)
{
  uint8_t wakeup_hour = load_data_byte(ADR_WAKEUP_HOUR);
  uint8_t wakeup_minute = load_data_byte(ADR_WAKEUP_MINUTE);

  sleep_until(wakeup_hour, wakeup_minute);
}
// ---------------------------------------------------------------------------------------------------------------------
// Hauptprogramm
/**********************************************************************************************************************/
int main (void)
{
  // Init IO-Ports, I2C-Bus
  INIT_IO;

  // Wenn beim beim einschalten Taste 5 Sek gedrückt
  // Start Uhrzeit aus ext.EEPROM/int.EEPROM/Flash laden und RTC stellen
  // In Dauerschleife StatusLED für "Zeit gesetzt" schnell blinken lassen
  Set_RTC_On_Keypress();

  // Wenn nach einschalten KEIN Tasterdruck, wird aktuelles Datum/Uhrzeit angezeigt
  // Dazu blinken die LEDs so oft, wie das Datums/Zeit - Teilstück angibt
  // LED0 = 5xblink, LED1 = 2xblink, LED2 = 15xblink, LED3 = 1xblink --> 5. Februar 15:01 Uhr
  // Welche LED was anzeigt, kann in config.h angepaßt werden
  Show_Date_And_Time();

  // Normalbetrieb, Hauptschleife
  while (1) {
    // LEDs werden passend zum Datum gesteuert bis Tasterdruck, Schlafenszeit erreicht, gar keine Daten für heute
    Work_Until_Sleeptime();
    // Schlafen bis 01:30
    Sleep_Until_1_30();
    // ggf. Zeit umstellen ME(S)Z
    Work_On_1_30();
    // schlafen bis reguläre Weckzeit
    Sleep_Until_Wakeuptime();
  }
}
