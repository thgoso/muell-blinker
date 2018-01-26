/*
 * load_data.c
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
#include "config.h"
#include "set_led.h"

// ---------------------------------------------------------------------------------------------------------------------
// Datenarray in AVR-EEPROM oder AVR-Flash erstellen (falls entsprechende Variante gewählt)
#if DATA_LOAD == FROM_EEPROM_AVR
  #include <avr/eeprom.h>
  uint8_t my_ee_data[395] EEMEM = {MY_DATA};

#elif DATA_LOAD == FROM_FLASH_AVR
  static const __flash uint8_t my_flash_data[395] = {MY_DATA};

#elif DATA_LOAD == FROM_EEPROM_AT24C32
  #include "i2cmaster.h"

#endif

/***********************************************************************************************************************
* Funktion zum lesen von Datenbyte Adresse/Index 0-394
* Erstellt mit "Abfuhrtabelle.ods" und gespeichert in:
* - AVR-internem EEPROM my_data[] (Adresse = Index 0-394)
* - oder AVR-Flash my_data[] (Adresse = Index 0-394)
* - oder 24C32 EEPROM auf DS-Board (EEPROM Adresse = 0-394)
***********************************************************************************************************************/
uint8_t load_data_byte (uint16_t adr)
{
#if DATA_LOAD == FROM_FLASH_AVR
  return my_flash_data[adr];

#elif DATA_LOAD == FROM_EEPROM_AVR
  return eeprom_read_byte (&my_ee_data[adr]);

#elif DATA_LOAD == FROM_EEPROM_AT24C32
  uint8_t adr_high = ((uint8_t)(adr>>8));
  uint8_t adr_low  = ((uint8_t)(adr));
  uint8_t data;

  // Aufhängen mit schnell Blinkender Statusanzeige für Fehler I2C-EEPROM,
  // Falls dieses nicht antwortet
  if (i2c_start(AT_I2CADR_W) != 0) leds_blink_endless(LED_ERR_AT);
  if (i2c_write(adr_high) != 0) leds_blink_endless(LED_ERR_AT);
  if (i2c_write(adr_low) != 0) leds_blink_endless(LED_ERR_AT);
  if (i2c_rep_start(AT_I2CADR_R) != 0) leds_blink_endless(LED_ERR_AT);
  data = i2c_readNak();
  i2c_stop();
  return data;
#endif
}
