/*
 * io_func.c
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
#include <avr/eeprom.h>
#include <util/delay.h>
#include "config.h"
#include "i2cmaster.h"
#include "io_func.h"

// ---------------------------------------------------------------------------------------------------------------------
// Adressen Datenbytes für LED-Status der Tage
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
// ---------------------------------------------------------------------------------------------------------------------
// Benötigte DS Register/Adressen
#define DS_SECONDS            0x00
#define DS_MINUTES            0x01
#define DS_HOURS              0x02
#define DS_DATE               0x04
#define DS_MONTH              0x05
#define DS_YEAR               0x06
#define DS_ALRM1_SECONDS      0x07
#define DS_ALRM1_MINUTES      0x08
#define DS_ALRM1_HOURS        0x09
#define DS_ALRM1_DATE         0x0A
#define DS_CNTRL              0x0E
#define DS_SREG               0x0F
// ---------------------------------------------------------------------------------------------------------------------
// Datenarray in AVR-EEPROM oder AVR-Flash erstellen (falls entsprechende Variante gewählt)
#if DATA_LOAD == FROM_EEPROM_AVR
  uint8_t my_ee_data[391] EEMEM = {MY_DATA};

#elif DATA_LOAD == FROM_FLASH_AVR
  static const __flash uint8_t my_flash_data[391] = {MY_DATA};

#endif











// Private Funktionen
/***********************************************************************************************************************
* Funktion zum lesen von Datenbyte Adresse/Index 0-390
* Erstellt mit "Abfuhrtabelle.ods" und gespeichert in:
* - AVR-internem EEPROM my_data[] (Adresse = Index 0-390)
* - oder AVR-Flash my_data[] (Adresse = Index 0-390)
* - oder 24C32 EEPROM auf DS-Board (EEPROM Adresse = 0-390)
***********************************************************************************************************************/
static uint8_t load_data_byte (uint16_t adr)
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
  if (i2c_start(AT_I2CADR_W) != 0) status_blink_endless(LED_ERR_AT);
  if (i2c_write(adr_high) != 0) status_blink_endless(LED_ERR_AT);
  if (i2c_write(adr_low) != 0) status_blink_endless(LED_ERR_AT);
  if (i2c_rep_start(AT_I2CADR_R) != 0) status_blink_endless(LED_ERR_AT);
  data = i2c_readNak();
  i2c_stop();
  return data;
#endif
}
/***********************************************************************************************************************
* Liest ein Register des DS3231 aus und gibt es zurück
* Übergeben werden muß Registeradresse bzw. der Name
* Die Daten vom DS liegen IMMER im PACKED BCD FORMAT vor
***********************************************************************************************************************/
static uint8_t ds_read_reg (uint8_t reg)
{
  // Aufhängen mit schnell Blinkender Statusanzeige für Fehler I2C-DS3231,
  // Falls dieses nicht antwortet
  if (i2c_start(DS_I2CADR_W) != 0) status_blink_endless(LED_ERR_DS);
  if (i2c_write(reg) != 0) status_blink_endless(LED_ERR_DS);
  if (i2c_rep_start(DS_I2CADR_R) != 0) status_blink_endless(LED_ERR_DS);
  uint8_t byte = i2c_readNak();
  i2c_stop();
  return byte;
}
/***********************************************************************************************************************
* Schreibt "val" in das Register "reg" des DS
* übergebener Wert muß wie für den DS üblich im PACKED BCD FORMAT vorliegen
* "reg" muß Registeradresse oder Bezeichner sein
***********************************************************************************************************************/
static void ds_write_reg (uint8_t reg, uint8_t val)
{
  // Aufhängen mit schnell Blinkender Statusanzeige für Fehler I2C-DS3231,
  // Falls dieses nicht antwortet
  if (i2c_start(DS_I2CADR_W) != 0) status_blink_endless(LED_ERR_DS);
  if (i2c_write(reg) != 0) status_blink_endless(LED_ERR_DS);
  if (i2c_write(val) != 0) status_blink_endless(LED_ERR_DS);
  i2c_stop();
}
/***********************************************************************************************************************
* Konvertiert ein übergebenes Byte vom Packed-BCD nach normal binär
* Übergabe: Byte im PBCD-Format
* Rückgabe: Wert gewandelt
***********************************************************************************************************************/
static void pbcd_to_bin (uint8_t *byte)
{
  uint8_t ones = (*byte & 0b00001111);
  uint8_t tens = (((*byte >>4) & 0b00001111) * 10);
  *byte = (ones + tens);
}
/***********************************************************************************************************************
* Blockierende Pause von 900ms Dauer
* Taste wird zwischendurch abgefragt
* Sollte diese gedrückt werden, wird unterbrochen und es geht mit TRUE zurück
* Sollte die Sekunde ohne Tasterdruck verstrichen sein, wird FALSE zurückgegeben
***********************************************************************************************************************/
static uint8_t check_key_pressed_900ms (void)
{
  uint8_t cnt;

  for (cnt=18; cnt>0; cnt--) {                    // 18 Durchgänge
    if ((LED_KEY_PIN & (1<<KEY)) == 0) {          // Raus bei Tastendruck
      return TRUE;
    }
    delay_ms(50);                                 // 50ms * 18 Durchgänge = 900 ms
  }
  return FALSE;
}
/***********************************************************************************************************************
* Blinkhilfsfunktion zur Anzeige von Datum/Zeit mittels LEDs
* LEDs werden so oft an/aus geschaltet wie "pbcd_byte" angibt, danach eine Sekunde Pause
* Während blinken KEINE Tastenabfrage
* Übergabe: Zeitbyte im PACKED-BCD-Format
*           LED-Byte zum steuern von LED_KEY_PORT
***********************************************************************************************************************/
static void blink_loop (uint8_t pbcd_byte, uint8_t led_byte)
{
  pbcd_to_bin(&pbcd_byte);                        // Wandeln nach normal binär
  led_byte |= (1<<KEY);                           // Taste Pull-Up = Ein, LEDs nach Vorgabe

  for ( ;pbcd_byte>0; pbcd_byte--) {
    LED_KEY_PORT = led_byte;                      // LED-Byte anlegen
    delay_ms(100);
    LED_KEY_PORT = (0b00000000 | (1<<KEY));       // Alle LEDs aus, Taster interner Pull-Up = an
    delay_ms(500);
  }
  delay_ms(1000);
}
/**********************************************************************************************************************/











// Öffentliche Funktionen
/***********************************************************************************************************************
* Initialisiert IOs des AVR
***********************************************************************************************************************/
void init_ioports (void)
{
  LED_KEY_DDR = (0b11111111 & ~(1<<KEY));         // Alles Ausgänge, Taster = Eingang
  LED_KEY_PORT = (0b00000000 | (1<<KEY));         // Alle LEDs aus, Taster interner Pull-Up = an
  i2c_init();                                     // SDA, SCL
}
/***********************************************************************************************************************
* blockierende Pause
* Übergabe: Wartetdauer in ms
***********************************************************************************************************************/
void delay_ms (uint16_t ms)
{
  for ( ;ms>0; ms--) {
    _delay_ms(1);
  }
}
/***********************************************************************************************************************
* Prüft ob 5 Sekunden lang der Taster durchgängig gedrückt wird
* Gibt TRUE zurück falls Ja, sonst FALSE
***********************************************************************************************************************/
uint8_t check_key_pressed_5s (void)
{
  uint8_t cnt;

  for (cnt=50; cnt>0; cnt--) {                    // In 50 Durchgängen
    delay_ms(100);                                // 100ms warten = 5 Sek
    if ((LED_KEY_PIN & (1<<KEY)) != 0) {          // Wenn nicht gedrückt oder losgelassen
      return FALSE;
    }
  }
  return TRUE;                                    // Zeit um, Taste gedrückt gewesen
}
/***********************************************************************************************************************
* Endlosschleife mit schnell blinkenden LEDs zur Status/Fehleranzeige
* Übergabe: LED-Byte mit passend gesetzten Bits LEDs die EIN sein sollen
***********************************************************************************************************************/
void status_blink_endless (uint8_t led_byte)
{
  led_byte |= (1<<KEY);                           // Taste Pull-Up = Ein, LEDs nach Vorgabe

  while (1) {
    LED_KEY_PORT = (0b00000000 | (1<<KEY));       // Alle LEDs aus, Taster interner Pull-Up = an
    delay_ms(233);
    LED_KEY_PORT = led_byte;                      // Entsprechende LEDs an
    delay_ms(100);
  }
}
/***********************************************************************************************************************
* Steuert 10 Minuten die LEDs an, prüft zwischendurch Taste auf Druck
* Bricht bei Druck ab und gibt FALSE zurück
* Wenn 10 Minuten lang Anzeige ohne Tastendruck erfolgte wird TRUE zurückgegeben
* Übergabe: LED-Byte mit passend gesetzten Bits
***********************************************************************************************************************/
uint8_t led_blink_10min (uint8_t led_byte)
{
  uint16_t cnt;

  led_byte |= (1<<KEY);                           // Taste Pull-Up = Ein, LEDs nach Vorgabe

  for (cnt=600; cnt>0; cnt--) {                   // Schleife 600 Durchgänge == 10 Minute LEDs steuern
    LED_KEY_PORT = led_byte;                      // LEDs einschalten (KEY ist gesetzt/High... Pull-Up)
    delay_ms(100);                                // 100ms aufblitzen
#if LED_MODE == MODE_BLINK
    LED_KEY_PORT = (0b00000000 | (1<<KEY));       // Alle wieder auss, 900ms aus lassen, Taster abfragen
#endif
    if (check_key_pressed_900ms() == TRUE) {
      return FALSE;                               // Falls Taster gedrück, abbruch mit FALSE
    }
  }
  return TRUE;                                    // Ohne Tasterdruck durch
}
/***********************************************************************************************************************
* Die Startuhrzeit/Datum wird aus dem ext.EEPROM/int.EEPROM/Flash geladen und der DS3231 wird gestellt
***********************************************************************************************************************/
void set_rtc (void)
{
  uint8_t tmp;

  // Startsekunde auf "05" stellen, da beim Einschalten Taste 5 Sekunden gedrückt wurde
  ds_write_reg(DS_SECONDS, 0x05);
  tmp = load_data_byte(ADR_RTC_START_MINUTE);
  ds_write_reg(DS_MINUTES, tmp);
  tmp = load_data_byte(ADR_RTC_START_HOUR);
  ds_write_reg(DS_HOURS, tmp);
  tmp = load_data_byte(ADR_RTC_START_DATE);
  ds_write_reg(DS_DATE, tmp);
  tmp = load_data_byte(ADR_RTC_START_MONTH);
  ds_write_reg(DS_MONTH, tmp);
  tmp = load_data_byte(ADR_RTC_START_YEAR);
  ds_write_reg(DS_YEAR, tmp);
}
/***********************************************************************************************************************
* Läd Tägliche Weckzeit aus dem externen EEPROM oder internen EEPROM oder Flash und setzt den täglichen Alarm neu
* Funktion immer aufrufen, bevor der AVR Schlafen geht
*
* Der DS erzeugt am Ausgang SQW/INT einen LOW-Pegel wenn die Alarmzeit erreicht ist
* Modus "Alarm when hours, minutes, and seconds match"
***********************************************************************************************************************/
void set_daily_alarm (void)
{
  // Tägliche Weckzeit aus ext.EEPROM/int.EEPROM/Flash laden
  uint8_t hour = load_data_byte(ADR_WAKEUP_HOUR);
  uint8_t minute = load_data_byte(ADR_WAKEUP_MINUTE);

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
  ds_write_reg(DS_ALRM1_MINUTES, minute);
  ds_write_reg(DS_ALRM1_HOURS, hour);
  ds_write_reg(DS_ALRM1_DATE, 0b10000000);

  // Einschalten Alarm1, Ausschalten Alarm2 INT/SQW Modus setzten
  ds_write_reg(DS_CNTRL, 0b01000101);
  // Alarm-Statusflags in SREG zurücksetzten
  ds_write_reg(DS_SREG, 0b00000000);
  // Warten damit DS3231 INT0 auf High bringen kann (Low = Wecksignal)
  delay_ms(200);
}
/***********************************************************************************************************************
* Liest aktuelles Datum (TT.MM) vom DS3231
* Liest die Abfuhrdaten für dieses Datum aus dem int.EEPROM/ext.EEPROM/Flash
* und gibt das Byte zur LED Ansteuerung zurück in *led_byte
* Rückgabe: TRUE wenn für Heute Daten vorliegen (LEDs gesteuert werden müssen)
*           FALSE wenn für Heute keine Daten vorliegen (LEDs aus)
***********************************************************************************************************************/
uint8_t get_ledbyte_today (uint8_t *led_byte)
{
  uint8_t day, month;
  uint16_t adr;

  day = ds_read_reg(DS_DATE);                     // Datum vom DS abholen
  month = ds_read_reg(DS_MONTH);                  // Format = PACKED BCD
  pbcd_to_bin(&day);                              // Beide wandeln PBCD --> Binär
  pbcd_to_bin(&month);                            // Adresse (EEPROM/Array) für Datum berechnen: ((Monat-1)*32)+(Tag-1)
  day--;                                          // 0...30
  month--;                                        // 0...11
  adr = month;
  adr *= 32;                                      // 0,32,64,96,128,160,192,224,256,288,320,352
  adr = adr + day;                                // + Tag (0...30)
  *led_byte = load_data_byte(adr);                // LED-Byte (Abfuhrdaten) aus int.EEPROM/ext.EEPROM/Flash laden
  *led_byte &= ~(1<<KEY);                         // KEY-PIN ausmaskieren/löschen
  if (led_byte == 0) return FALSE;
  else return TRUE;
}
/***********************************************************************************************************************
* Liest aktuelles Datum (TT.MM) und Uhrzeit vom DS3231 Läßt LEDs so oft blinken wie die Zeitbytes sind
* Keine Tastenabfrage während blinken
*
* z.B. es ist der 10. Febraur 15:00 Uhr
* LED_DATE blinkt   10 mal
* LED_MONTH blinkt   2 mal
* LED_HOUR blink    15 mal
* LED_MINUTE blinkt 10 mal
***********************************************************************************************************************/
void show_date_and_time (void)
{
  uint8_t tmp;

  tmp=ds_read_reg(DS_DATE);
  blink_loop(tmp, LED_DATE);
  tmp=ds_read_reg(DS_MONTH);
  blink_loop(tmp, LED_MONTH);
  tmp=ds_read_reg(DS_HOURS);
  blink_loop(tmp, LED_HOUR);
  tmp=ds_read_reg(DS_MINUTES);
  blink_loop(tmp, LED_MINUTE);
}
/**********************************************************************************************************************/
