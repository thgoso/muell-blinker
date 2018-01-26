/*
 * load_data.h
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

#ifndef MY_LOAD_DATA
#define MY_LOAD_DATA

// ---------------------------------------------------------------------------------------------------------------------
// Adressen Datenbytes für LED-Status der Tage im EEPROM 24c32 bzw. Array-Index falls im AVR-Flash oder EEPROM
// 0-383 --> pro Monat 32 Bytes (Am Ende des Monats immer Füllbytes)
// 0-31=Januar, 32-60=Februar ... 352-383=Dezember
//
// Ab Adresse 384 Datenbytes Zeiten im PACKED BCD FORMAT
#define ADR_RTC_START_DATE    384
#define ADR_RTC_START_MONTH   385
#define ADR_RTC_START_YEAR    386
#define ADR_RTC_START_HOUR    387
#define ADR_RTC_START_MINUTE  388
#define ADR_WAKEUP_HOUR       389
#define ADR_WAKEUP_MINUTE     390
#define ADR_SLEEP_HOUR        391
#define ADR_SLEEP_MINUTE      392
#define ADR_MESZ_MAR          393
#define ADR_MESZ_OCT          394

// ---------------------------------------------------------------------------------------------------------------------
// Liest Daten-Byte aus AVR-Flash oder AVR-EEPROM oder AT24C32-EEPROM aus (je nach LOAD_FROM in config.h)
// Übergabe Adresse = 0 - 394
uint8_t load_data_byte (uint16_t adr);
// ---------------------------------------------------------------------------------------------------------------------

#endif
