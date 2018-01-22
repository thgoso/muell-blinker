/*
 * config.h
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

#ifndef MY_CONFIG
#define MY_CONFIG

// ---------------------------------------------------------------------------------------------------------------------
// Die Daten können gespeichert werden im:        - AVR-EEPROM (für Typen mit größerem EEPROM als ATtiny24)
//                                                - AVR-Flash (in dieser Variante für ATtiny24 möglich)
//                                                - 24C32-EEPROM des DS-Boards (EEPROMMER von Nöten)

// Variante AT24C32 EEPROM:                       - Daten selber im EEPROM Adresse 0-390 = 391 Bytes speichern
// Variante AVR-Flash oder AVR-EEPROM:            - Hier MY_DATA definieren und die 391 Bytes Daten darin ablegen
#define FROM_FLASH_AVR        1
#define FROM_EEPROM_AVR       2
#define FROM_EEPROM_AT24C32   3
// ---------------------------------------------------------------------------------------------------------------------
// Hier Angeben woher die mittels "Abfuhrtabelle.ods" erstellten Daten geholt werden sollen:
#define DATA_LOAD             FROM_EEPROM_AT24C32
// ---------------------------------------------------------------------------------------------------------------------
// Hier bei Variante FROM_FLASH_AVR oder FROM_EEPROM_AVR Abfuhrtabelle einfügen

/*
#define MY_DATA               0x01,0x08,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x00,0x00, \
                              0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x05,0x08,0x00,0x00,0x00,0x00,0x00, \
                              0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x00,0x00,0x00,0x00,0x00,0x02, \
                              0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x00, \
                              0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00, \
                              0x00,0x00,0x00,0x01,0x08,0x00,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00, \
                              0x00,0x01,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x08, \
                              0x00,0x00,0x00,0x00,0x00,0x04,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x01,0x08,0x00, \
                              0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x00,0x00,0x00, \
                              0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x05,0x0A,0x00,0x00,0x00,0x00,0x00,0x00, \
                              0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00, \
                              0x00,0x00,0x00,0x00,0x00,0x01,0x0A,0x04,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00, \
                              0x00,0x00,0x01,0x0A,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x01, \
                              0x0A,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x0A,0x00, \
                              0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x0A,0x04,0x00,0x00,0x00, \
                              0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x00,0x00,0x00,0x00,0x00,0x00, \
                              0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x04,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x02, \
                              0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00, \
                              0x00,0x00,0x01,0x08,0x04,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00, \
                              0x01,0x08,0x00,0x00,0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x00, \
                              0x00,0x00,0x00,0x00,0x04,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x00,0x01,0x08,0x00,0x00, \
                              0x00,0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x02,0x08,0x00,0x00,0x00,0x00,0x00, \
                              0x00,0x00,0x00,0x02,0x00,0x00,0x00,0x00,0x00,0x00,0x18,0x01,0x18,0x20,0x00,0x12,0x00
*/
// ---------------------------------------------------------------------------------------------------------------------
// Adressen I2C Busteilnehmer ggf. anpassen       // I2C Slave Adresse DS3231
#define DS_I2CADR_R           0b11010001          // Für Lesezugriffe
#define DS_I2CADR_W           0b11010000          // Für Schreibzugriffe
                                                  // I2C Slave Adresse AT24C32
#define AT_I2CADR_R           0b10101101          // Für Lesezugriffe        <--- OHNE Lötbrücken auf DS-Board
#define AT_I2CADR_W           0b10101100          // Für Schreibzugriffe     <--- weniger Stromverbrauch
// ---------------------------------------------------------------------------------------------------------------------
// die LEDs und Taster müssen sich an einem gemeinsamen Port befinden
// Auf den Port wird immer ganz geschrieben... Weitere Hardware daran nicht erlaubt
// LEDs Taster tauschen möglich
#define LED_KEY_PORT          PORTA               // LEDs und Taster an PORTA
#define LED_KEY_PIN           PINA                // bzw. PINA
#define LED_KEY_DDR           DDRA                // bzw. DDR

#define KEY                   PA7                 // Taster an PA7

// LED Steuerung zur Statusanzeige Zeit gestellt, Fehler DS3231, Fehler AT24C32, Datum/Zeitanzeige
// Passende LED Kombinationen ggf. anpassen
#define LED_TIME_SET          ((1<<PA0)|(1<<PA1)|(1<<PA2)|(1<<PA3)|(1<<PA4)|(1<<PA5)|(1<<PA6))
#define LED_ERR_DS            ((1<<PA0)|(1<<PA1))
#define LED_ERR_AT            ((1<<PA2)|(1<<PA3))
#define LED_DATE              (1<<PA0)
#define LED_MONTH             (1<<PA1)
#define LED_HOUR              (1<<PA2)
#define LED_MINUTE            (1<<PA3)
// ---------------------------------------------------------------------------------------------------------------------
// Hier definieren ob die jeweiligen LEDs bei Ansteuerung zur Abholanzeige
// Blinken sollen (100ms an, 900ms aus) [Stromsparend] oder dauerleuchten sollen
#define MODE_BLINK            1
#define MODE_ON               2

#define LED_MODE              MODE_BLINK
// ---------------------------------------------------------------------------------------------------------------------
// I2C Bus Signale SDA, SCL
// IOs werden einzeln bedient, weitere Hardware an anderen PortPins erlaubt
#define SDA_PORT              PORTB               // SDA PortB
#define SCL_PORT              PORTB               // SCL PortB
#define SDA                   PB0                 // SDA an PB0
#define SCL                   PB1                 // SCL an PB1
// ---------------------------------------------------------------------------------------------------------------------
// Fehlercheck config.h
#if LED_MODE != MODE_BLINK && LED_MODE != MODE_ON
  #error config.h -> LED_MODE
#endif
#if DATA_LOAD != FROM_EEPROM_AT24C32 && DATA_LOAD != FROM_EEPROM_AVR && DATA_LOAD != FROM_FLASH_AVR
  #error config.h -> DATA_LOAD
#endif
#if DATA_LOAD == FROM_EEPROM_AT24C32
  #ifdef MY_DATA
    #warning config.h -> MY_DATA defined but DATA_LOAD FROM_EEPROM_AT24C32
  #endif
#endif
#if DATA_LOAD == FROM_EEPROM_AVR
  #ifndef MY_DATA
    #error config.h -> MY_DATA not defined but DATA_LOAD FROM_EEPROM_AVR
  #endif
#endif
#if DATA_LOAD == FROM_FLASH_AVR
  #ifndef MY_DATA
    #error config.h -> MY_DATA not defined but DATA_LOAD FROM_FLASH_AVR
  #endif
#endif
// ---------------------------------------------------------------------------------------------------------------------

#endif
