/*
 * ds3231.h
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

#ifndef MY_DS3231
#define MY_DS3231

// ---------------------------------------------------------------------------------------------------------------------
// Benötigte DS Register/Adressen für Funktionsübergaben
#define DS_SECONDS            0x00
#define DS_MINUTES            0x01
#define DS_HOURS              0x02
#define DS_DATE               0x04
#define DS_MONTH              0x05
#define DS_YEAR               0x06
// ---------------------------------------------------------------------------------------------------------------------
// Liest Register aus DS3231, Rückgabe = PACKED BCD Format
uint8_t ds_read_reg (uint8_t reg);
// ---------------------------------------------------------------------------------------------------------------------
// Beschreibt Register des DS3231, Übergabe Registername(Adresse), Wert in PACKED BCD FORM
void ds_write_reg (uint8_t reg, uint8_t pbcd_val);
// ---------------------------------------------------------------------------------------------------------------------
// Schaltet Alarmfunktion "Alarm when Hours, Minutes, Seconds match" ein
// Nach Aufruf der Funktion wird INT0 auf High gesetzt
// Ist die Alarmzeit erreicht, wird INT0 auf Low gezogen (so lange bis Alarm erneut gesetzt wird)
// Übergabe Alarmzeit Stunde, Minute in PACKED BCD FORM
void ds_set_alarm_time (uint8_t pbcd_hour, uint8_t pbcd_minute);
// ---------------------------------------------------------------------------------------------------------------------

#endif
