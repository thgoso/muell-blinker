/*
 * io_func.h
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
#ifndef MY_IOFUNC
#define MY_IOFUNC

// Für Funktionsrückgaben, -übergaben

#define KEY_PRESSED   0                           // Taste wurde gedrückt
#define KEY_UNPRESSED 1                           // Kein Tasterdruck erfolgt

typedef struct {                                  // Datum TT.MM in Packed BCD Form
  uint8_t day;                                    // Tag
  uint8_t month;                                  // Monat
} pbcd_date_t;

typedef struct {                                  // Uhrzeit in Packed BCD Form
  uint8_t hour;                                   // Sunde
  uint8_t minute;                                 // Minute
} pbcd_time_t;

typedef struct {                                  // Datum + Uhrzeit in Packed BCD Form
  pbcd_date_t date;                               // Datum
  pbcd_time_t time;                               // Uhrzeit
} pbcd_date_time_t;

// ---------------------------------------------------------------------------------------------------------------------
// Initialisiert IOs des AVR, einmalig nach AVR Start aufrufen
void init_ioports (void);
// ---------------------------------------------------------------------------------------------------------------------
// Prüft ob Taste 5 Sek. durchgehend gedrückt,
// JA:   RTC auf Initialwerte aus Flash/EEPROM setzten und in Dauerschleife Statusblinken anzeigen (Uhr gestellt)
// NEIN: Funktion kehrt einfach zurück
void set_rtc_on_keypress (void);
// ---------------------------------------------------------------------------------------------------------------------
// Liest aktuelles Datum und Uhrzeit vom DS3231 und gibt Daten in *pbcd_date_time zurück
void get_date_and_time (pbcd_date_time_t *pbcd_date_time);
// ---------------------------------------------------------------------------------------------------------------------
// Läßt die LEDs so oft blinken wie übergebenes Datum/Uhrzeit in *pbcd_date_time angeben
// Keine Tasterabfrage zwischendurch
void show_date_and_time (const pbcd_date_time_t *pbcd_date_time);
// ---------------------------------------------------------------------------------------------------------------------
// Holt Byte zur LED-Steuerung aus 24C32-EEPROM/AVR-EEPROM/Flash-Variable
// Übergabe: Datum zu welchem die Abfuhrdaten geladen werden sollen in *pbcd_date
// Rückgabe: LED-Byte zur Steuerung, wenn am Datum KEINE LEDs gesteuert werden müssen, Rückgabe = 0
uint8_t get_ledbyte_for_date (const pbcd_date_t *pbcd_date);
// ---------------------------------------------------------------------------------------------------------------------
// Steuert 10 Minuten die LEDs an
// Übergabe: LED-Byte in led_byte
// Rückgabe: KEY_UNPRESSED wenn 10 Minuten ohne Tasterdruck um
//           falls der Nutzer auf den Taster gedrückt hat, wird abgebrochen und KEY_PRESSED zurückgegeben
uint8_t led_blink_10min (uint8_t led_byte);
// ---------------------------------------------------------------------------------------------------------------------
// Holt tägliche Weckzeit aus EEPROM/Flash und gibt diese in *pbcd_time zurück
void get_daily_wakeup_time (pbcd_time_t *pbcd_time);
// ---------------------------------------------------------------------------------------------------------------------
// Setzt Alarmfunktion DS3231
// Ist die Alarmzeit erreicht weckt der DS den AVR mittels Low-Signal an INT0 auf
// INT0 bleibt SO LANGE Low, bis der Alarm erneut gesetzt wird
// Nach Funktionsaufruf ist INT0 High und der AVR kann schlafen gelegt werden
// Übergabe: Alarmzeit im Packed-BCD Format in *pbcd_time
void set_alarm_time (const pbcd_time_t *pbcd_time);
// ---------------------------------------------------------------------------------------------------------------------
// Prüft ob übergebenes Datum einem Datum entspricht an dem Zeitumstellung ME(S)Z erfolgt
// Falls ja, wird die RTC eine Stunde vor- oder zurückgestellt
// Funktion MUSS IMMER Nachts um 01:30 aufgerufen werden
// Falls die Zeit umgestellt wird ist es danach 00:30 oder 02:30
// Es wird im DS3231 NUR das Register für Stunde korrigiert... der Rest läuft so weiter wie er ist
// Falls am übergebenem Datum keine Zeitumstellung stattfindet, geht's ohne Änderungen zurück
void switch_time_on_switchdate_1_30 (const pbcd_date_t *pbcd_date);
// ---------------------------------------------------------------------------------------------------------------------

#endif

