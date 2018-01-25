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
// LEDs am LED_KEY_PORT setzten, am Tasteranschluss jedoch Pull-Up an lassen
#define SET_LEDS(ledbyte)     (LED_KEY_PORT = (ledbyte | (1<<KEY)))
#define CLEAR_LEDS            (LED_KEY_PORT = (0b00000000 | (1<<KEY)))
// Taster abfragen, LEDs ausmaskieren... Liefert 0 wenn Taster gedrückt
#define READ_KEY              (LED_KEY_PIN & (1<<KEY))
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
#define ADR_MESZ_MAR          391
#define ADR_MESZ_OCT          392
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
  uint8_t my_ee_data[393] EEMEM = {MY_DATA};

#elif DATA_LOAD == FROM_FLASH_AVR
  static const __flash uint8_t my_flash_data[393] = {MY_DATA};

#endif
// ---------------------------------------------------------------------------------------------------------------------





// Private Funktionen
/***********************************************************************************************************************
* Konvertiert ein übergebenes Byte vom Packed-BCD nach normal binär
* Übergabe: Byte im PBCD-Format
* Rückgabe: Wert gewandelt
***********************************************************************************************************************/
static inline void __attribute__((always_inline)) pbcd_to_bin (uint8_t *byte)
{
  uint8_t ones = (*byte & 0b00001111);
  uint8_t tens = (((*byte >>4) & 0b00001111) * 10);
  *byte = (ones + tens);
}
/***********************************************************************************************************************
* Endlosschleife mit schnell blinkenden LEDs zur Status/Fehleranzeige
* Übergabe: LED-Byte mit passend gesetzten Bits LEDs die EIN sein sollen
***********************************************************************************************************************/
static void __attribute__((noreturn)) status_blink_endless (uint8_t led_byte)
{
  while (1) {
    CLEAR_LEDS;
    _delay_ms(233);
    SET_LEDS(led_byte);
    _delay_ms(100);
  }
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

  for ( ;pbcd_byte>0; pbcd_byte--) {
    SET_LEDS(led_byte);
    _delay_ms(100);
    CLEAR_LEDS;
    _delay_ms(500);
  }
  _delay_ms(1000);
}
/***********************************************************************************************************************
* Funktion zum lesen von Datenbyte Adresse/Index 0-392
* Erstellt mit "Abfuhrtabelle.ods" und gespeichert in:
* - AVR-internem EEPROM my_data[] (Adresse = Index 0-392)
* - oder AVR-Flash my_data[] (Adresse = Index 0-392)
* - oder 24C32 EEPROM auf DS-Board (EEPROM Adresse = 0-392)
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
* Wenn Taste 5 Sekunden durchgehend gedrückt:
* Initialwerte Startzeit/Datum aus EEPROM/Flash laden und RTC stellen (Startzeit/Datum)
* Endlosschleife Statusblinken
* Wenn nicht gedrückt, Rückkehr ohne Änderungen
***********************************************************************************************************************/
void set_rtc_on_keypress (void)
{
  uint8_t tmp;

  for (tmp=50; tmp>0; tmp--) {                    // In 50 Durchgängen
    _delay_ms(100);                               // 100ms warten = 5 Sek
    if (READ_KEY != 0) return;                    // Wenn nicht gedrückt oder losgelassen
  }

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
  // DS_SREG komplett zurücksetzten
  ds_write_reg(DS_SREG, 0b00000000);

  // Endlosschleife Statusblinken
  status_blink_endless(LED_TIME_SET);
}
/***********************************************************************************************************************
* Holt aktuelle Uhrzeit/Datum vom DS3231 und gibt Daten in PACKED-BCD-Form zurück
***********************************************************************************************************************/
void get_date_and_time (pbcd_date_time_t *pbcd_date_time)
{
  pbcd_date_time->date.day    = ds_read_reg(DS_DATE);
  pbcd_date_time->date.month  = ds_read_reg(DS_MONTH);
  pbcd_date_time->time.hour   = ds_read_reg(DS_HOURS);
  pbcd_date_time->time.minute = ds_read_reg(DS_MINUTES);
}
/***********************************************************************************************************************
* Läßt LEDs so oft blinken wie die Zeitbytes in pbcd_data dies angeben
* Keine Tastenabfrage während blinken
*
* z.B. es ist der 10. Febraur 15:10 Uhr
* LED_DATE blinkt   10 mal
* LED_MONTH blinkt   2 mal
* LED_HOUR blink    15 mal
* LED_MINUTE blinkt 10 mal
***********************************************************************************************************************/
void show_date_and_time (const pbcd_date_time_t *pbcd_date_time)
{
  uint8_t tmp;

  tmp = pbcd_date_time->date.day;
  blink_loop(tmp, LED_DATE);
  tmp = pbcd_date_time->date.month;
  blink_loop(tmp, LED_MONTH);
  tmp = pbcd_date_time->time.hour;
  blink_loop(tmp, LED_HOUR);
  tmp = pbcd_date_time->time.minute;
  blink_loop(tmp, LED_MINUTE);
}
/***********************************************************************************************************************
* Liest die Abfuhrdaten für übergebenes Datum aus dem int.EEPROM/ext.EEPROM/Flash
* Gibt das Byte zur LED Ansteuerung zurück
* Wenn heute KEINE Abholung, Rückgabe = 0
***********************************************************************************************************************/
uint8_t get_ledbyte_for_date (const pbcd_date_t *pbcd_date)
{
  uint16_t adr;
  uint8_t  retval;
  uint8_t  day = pbcd_date->day;                  // Tag und Monat = PACKED-BCD
  uint8_t  month = pbcd_date->month;              // kopieren

  pbcd_to_bin(&day);                              // Beide wandeln PBCD --> Binär
  pbcd_to_bin(&month);                            // Adresse (EEPROM/Array) für Datum berechnen: ((Monat-1)*32)+(Tag-1)
  day--;                                          // 0...30
  month--;                                        // 0...11
  adr = month;
  adr *= 32;                                      // 0,32,64,96,128,160,192,224,256,288,320,352
  adr = adr + day;                                // + Tag (0...30)
  retval = load_data_byte(adr);                   // LED-Byte (Abfuhrdaten) aus int.EEPROM/ext.EEPROM/Flash laden
  retval &= ~(1<<KEY);                            // KEY-PIN ausmaskieren/löschen
  return retval;
}
/***********************************************************************************************************************
* Steuert 10 Minuten die LEDs an, prüft zwischendurch Taste auf Druck
* Bricht bei Druck ab und gibt KEY_PRESSE zurück
* Wenn 10 Minuten lang Anzeige ohne Tastendruck erfolgte wird KEY_UNPRESSED zurückgegeben
* Übergabe: LED-Byte mit passend gesetzten Bits
***********************************************************************************************************************/
uint8_t led_blink_10min (uint8_t led_byte)
{
  uint16_t cnt_10min;
  uint8_t  cnt_900ms;

  for (cnt_10min=600;cnt_10min>0;cnt_10min--) {   // Schleife 600 Durchgänge == 10 Minute LEDs steuern
    SET_LEDS(led_byte);                           // LEDs ein
    _delay_ms(100);                               // 100ms aufblitzen
#if LED_MODE == MODE_BLINK
    CLEAR_LEDS;                                   // Alle wieder aus, 900ms aus lassen
#endif
    for (cnt_900ms=18;cnt_900ms>0;cnt_900ms--) {  // Taster abfragen, 18 Durchgänge
      if (READ_KEY == 0) return KEY_PRESSED;      // Raus bei Tastendruck
      _delay_ms(50);                              // 50ms * 18 Durchgänge = 900 ms
    }
  }

  return KEY_UNPRESSED;                           // Ohne Tasterdruck durch
}
/***********************************************************************************************************************
* Läd die tägliche Weckzeit aus EEPROM/Flash und gibt diese in *pbcd_time zurück
***********************************************************************************************************************/
void get_daily_wakeup_time (pbcd_time_t *pbcd_time)
{
  pbcd_time->hour = load_data_byte(ADR_WAKEUP_HOUR);
  pbcd_time->minute = load_data_byte(ADR_WAKEUP_MINUTE);
}
/***********************************************************************************************************************
* Schaltet Alarmfunktion DS3231 ein
* Funktion immer aufrufen, bevor der AVR Schlafen geht
*
* Der DS erzeugt am Ausgang SQW/INT einen LOW-Pegel wenn die Alarmzeit erreicht ist
* Modus "Alarm when hours, minutes, and seconds match"
* SQW/INT bleibt SO LANGE Low, bis der Alarm erneut gesetzt wird
*
* Übergabe: Alarmzeit im Packed-BCD Format
***********************************************************************************************************************/
void set_alarm_time (const pbcd_time_t *pbcd_time)
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
  ds_write_reg(DS_ALRM1_MINUTES, pbcd_time->minute);
  ds_write_reg(DS_ALRM1_HOURS, pbcd_time->hour);
  ds_write_reg(DS_ALRM1_DATE, 0b10000000);

  // Einschalten Alarm1-INT, Ausschalten Alarm2-INT, Modus-SQW/INT setzten auf INT
  ds_write_reg(DS_CNTRL, 0b01000101);
  // Alarm-Statusflags in SREG zurücksetzten
  ds_write_reg(DS_SREG, 0b00000000);
  // Warten damit DS3231 INT0 auf High bringen kann (Low = Wecksignal)
  _delay_ms(200);
}
/***********************************************************************************************************************
* Prüft ob übergebenes Datum ein Tag ist, an dem Zeitumstellung ME(S)Z stattfindet
* Falls ja, wird das Stundenregister des DS3231 angepaßt
* Falls nei, geht's ohne Änderungen zurück
*
* Funktion MUSS IMMER Nachts um 01:30 aufgerufen werden
* Übergabe: Aktuelles Datum in *pbcd_date
***********************************************************************************************************************/
void switch_time_on_switchdate_1_30 (const pbcd_date_t *pbcd_date)
{
  switch (pbcd_date->month) {
    case 0x03:
      // Heute Zeitumstellung "??.03" eine Stunde vor von 1 Uhr auf 2 Uhr
      if (load_data_byte(ADR_MESZ_MAR) == pbcd_date->day) ds_write_reg(DS_HOURS, 0x02);
      break;
    case 0x10:
      // Heute Zeitumstellung "??.10" eine Stunde zurück von 1 Uhr auf 0 Uhr
      if (load_data_byte(ADR_MESZ_OCT) == pbcd_date->day) ds_write_reg(DS_HOURS, 0x00);
      break;
  }
}
