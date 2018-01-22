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

// Für Funktionsrückgaben
#define TRUE          0
#define FALSE         1

// ---------------------------------------------------------------------------------------------------------------------
// Initialisiert IOs des AVR
void init_ioports (void);
// ---------------------------------------------------------------------------------------------------------------------
// Blockerende Pause von "ms" dauer
void delay_ms (uint16_t ms);
// ---------------------------------------------------------------------------------------------------------------------
// Läßt in Endlosschleife LED(s) schnell blinken
// Zur Statusanzeige bei Erstmaligem Start mit Tasterdruck (Zeit gesetzt)
// oder zur Fehleranzeige bei Kommunikationsfehler mit I2C-Geräten
void status_blink_endless (uint8_t led_byte);
// ---------------------------------------------------------------------------------------------------------------------
// Setzt Uhrzeit/Datum des DS3231 auf Daten aus 24C32-EEPROM/AVR-EEPROM/Flash-Variable
void set_rtc (void);
// ---------------------------------------------------------------------------------------------------------------------
// Schaltet täglichen Alarm ein (Zeit wie in 24C32-EEPROM/AVR-EEPROM/Flash-Variable)
// Immer aufrufen bevor der AVR schlafen geht
void set_daily_alarm (void);
// ---------------------------------------------------------------------------------------------------------------------
// Holt aktuelles Datum/Uhrzeit aus DS3231 und läßt LEDs danach blinken
// Keine Tasterabfrage zwischendurch
void show_date_and_time (void);
// ---------------------------------------------------------------------------------------------------------------------
// Prüft ob Taste durchgehend 5 Sekunden gedrückt
// Gibt TRUE zurück falls Ja, sonst FALSE
uint8_t check_key_pressed_5s (void);
// ---------------------------------------------------------------------------------------------------------------------
// Holt Byte zur LED-Steuerung aus 24C32-EEPROM/AVR-EEPROM/Flash-Variable
// Rückgabe: LED-Byte in *led_byte
//           TRUE wenn für Heute Daten vorliegen, sonst FALSE
uint8_t get_ledbyte_today (uint8_t *led_byte);
// ---------------------------------------------------------------------------------------------------------------------
// Steuert 10 Minuten die LEDs an
// Übergabe: LED-Byte in led_byte
// Rückgabe: TRUE wenn 10 Minuten ohne Tasterdruck um
//           falls der Nutzer auf den Taster gedrückt hat, wird abgebrochen und FALSE zurückgegeben
uint8_t led_blink_10min (uint8_t led_byte);
// ---------------------------------------------------------------------------------------------------------------------

#endif

