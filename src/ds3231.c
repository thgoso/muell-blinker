/*
 * ds3231.c
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
#include "i2cmaster.h"
#include "set_led.h"

#define DS_ALRM1_SECONDS      0x07
#define DS_ALRM1_MINUTES      0x08
#define DS_ALRM1_HOURS        0x09
#define DS_ALRM1_DATE         0x0A
#define DS_CNTRL              0x0E
#define DS_SREG               0x0F

/***********************************************************************************************************************
* Liest ein Register des DS3231 aus und gibt es zurück
* Übergeben werden muß Registeradresse bzw. der Name
* Die Daten vom DS liegen IMMER im PACKED BCD FORMAT vor
***********************************************************************************************************************/
uint8_t ds_read_reg (uint8_t reg)
{
  // Aufhängen mit schnell Blinkender Statusanzeige für Fehler I2C-DS3231,
  // Falls dieses nicht antwortet
  if (i2c_start(DS_I2CADR_W) != 0) leds_blink_endless(LED_ERR_DS);
  if (i2c_write(reg) != 0) leds_blink_endless(LED_ERR_DS);
  if (i2c_rep_start(DS_I2CADR_R) != 0) leds_blink_endless(LED_ERR_DS);
  uint8_t byte = i2c_readNak();
  i2c_stop();
  return byte;
}
/***********************************************************************************************************************
* Schreibt "pbcd_val" in das Register "reg" des DS
* übergebener Wert muß wie für den DS üblich im PACKED BCD FORMAT vorliegen
* "reg" muß Registeradresse oder Bezeichner sein
***********************************************************************************************************************/
void ds_write_reg (uint8_t reg, uint8_t pbcd_val)
{
  // Aufhängen mit schnell Blinkender Statusanzeige für Fehler I2C-DS3231,
  // Falls dieses nicht antwortet
  if (i2c_start(DS_I2CADR_W) != 0) leds_blink_endless(LED_ERR_DS);
  if (i2c_write(reg) != 0) leds_blink_endless(LED_ERR_DS);
  if (i2c_write(pbcd_val) != 0) leds_blink_endless(LED_ERR_DS);
  i2c_stop();
}
/***********************************************************************************************************************
* Schaltet Alarmfunktion DS3231 ein
*
* Der DS erzeugt am Ausgang SQW/INT einen LOW-Pegel wenn die Alarmzeit erreicht ist
* Modus "Alarm when hours, minutes, and seconds match"
* SQW/INT bleibt SO LANGE Low, bis der Alarm erneut gesetzt wird
*
* Übergabe: Alarmzeit im Packed-BCD Format
***********************************************************************************************************************/
void ds_set_alarm_time (uint8_t pbcd_hour, uint8_t pbcd_minute)
{
  // Zuordnung Alarmregister und Bits/Werte
  //
  // Register Name     Adresse   |B7|B6|B5|B4|B3|B2|B1|B0|
  // DS_ALRM1_SECONDS   0x07     |M1|  10er  |    1er    |
  // DS_ALRM1_MINUTES   0x08     |M2|  10er  |    1er    |
  // DS_ALRM1_HOURS     0x09     |M3|  10er  |    1er    |
  // DS_ALRM1_DATE      0x10     |M4|  10er  |    1er    |
  //
  // Zur Nutzung von Alarmfunktion "Alarm when hours, minutes, and seconds match"
  // Müssen die Bits M1, M2, M3 = 0 sein, und M4 = 1

  ds_write_reg(DS_ALRM1_SECONDS, 0x00);
  ds_write_reg(DS_ALRM1_MINUTES, pbcd_minute);
  ds_write_reg(DS_ALRM1_HOURS, pbcd_hour);
  ds_write_reg(DS_ALRM1_DATE, 0b10000000);

  // Einschalten Alarm1-INT, Ausschalten Alarm2-INT, Modus-SQW/INT setzten auf INT
  ds_write_reg(DS_CNTRL, 0b01000101);
  // Alarm-Statusflags in SREG zurücksetzten
  ds_write_reg(DS_SREG, 0b00000000);
  // Warten damit DS3231 INT0 auf High bringen kann (Low = Wecksignal)
  _delay_ms(200);
}
