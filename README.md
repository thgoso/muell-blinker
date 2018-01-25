# Mülltonnen-Blinker

## Zutaten:
- ATtiny24
- Taster
- max. 7 LEDs
  - eine pro Mülltonne in passender Farbe
- RTC-Board mit DS3231, EEPROM 24C32 und Knopzelle (gibt's im großen Onlineauktionshaus)
  - Pull-Up-Widerstände für SDA, SCL, SQW/INT sind dort integriert

![Bild](https://github.com/thgoso/muell-blinker/blob/master/infos/DS3231-Board.jpg)

## Verwendung:
Zeigt mittels blinkender LED(s) an daß die Müllabfuhr kommt und welche Tonne(n) an die Straße gebracht werden müssen.

![Bild](https://github.com/thgoso/muell-blinker/blob/master/schaltung/Schaltung.jpg)

## Daten erstellen:
Mittels Datei "Abfuhrkalender.ods" im Ordner infos oder selbst erzeugen.

**Abfuhrdaten müssen:** 
- Entweder im EEPROM (24C32) des DS3231 Boards gespeichert werden (EEPROMMER erforderlich)
- oder im internen EEPROM des AVR (nicht möglich mit ATtiny24, jedoch bei Typen mit großem EEPROM)
- oder im Flash des AVR gespeichert werden (beim ATtiny24 ebenfalls möglich)

Dazu sind für jeden Monat 32 Bytes genutzt (Somit hat jeder Monat gleich viel Tage)

**Die Daten gehören:**
- ins 24C32 EEPROM ab Adresse 0
- ins AVR interne EEPROM (Einbinden in config.h)
- In den Flash des AVR (Einbinden in config.h)

| Adresse/Index Dezimal | Datum                                     |
| --------------------- | ----------------------------------------- |
| 0-31                  | 1.1 - 31.1 = 31 Datenbytes + 1 Füllbyte   |
| 32-63                 | 1.2 - 29.2 = 29 Datenbytes + 3 Füllbytes  |
| 64-96                 | 1.3 - 31.3 = 31 Datenbytes + 1 Füllbyte   |
| .                     | .                                         |
| .                     | .                                         |
| .                     | .                                         |
| 352-383               | 1.12 - 31.12 = 31 Datenbytes + 1 Füllbyte |

Im Datenbyte wird gespeichert, welche LED(s) am zugehörigen Tag angesteuert werden.
Das Bit 7 ist ohne Bedeutung. Zuordnung Bits im Datenbyte zu LEDs:

| Bit7 | Bit6   | Bit5   | Bit4   | Bit3   | Bit2   | Bit1   | Bit0   | Datenbyte  | LEDs          |
| ---- | ------ | ------ | ------ | ------ | ------ | ------ | ------ | ---------- | ------------- |
| *NC* | *LED6* | *LED5* | *LED4* | *LED3* | *LED2* | *LED1* | *LED0* | *Binär*    | *Ein*         |
| X    | 1      | 1      | 1      | 1      | 1      | 1      | 1      | 0bX1111111 | 6,5,4,3,2,1,0 |
| X    | 0      | 0      | 0      | 0      | 0      | 0      | 0      | 0bX0000000 | Keine         |
| X    | 0      | 1      | 0      | 1      | 0      | 1      | 0      | 0bX0101010 | 5,3,1         |


Ab EEPROM Adresse Dez 384 (bzw. Array Index) müssen folgende Daten gespeichert werden:

| Adresse | Byte                       | Format     |
| ------- | -------------------------- | ---------- |
| 384     | RTC Start Tag              | PACKED BCD |
| 385     | RTC Start Monat            | PACKED BCD |
| 386     | RTC Start Jahr 2-Stellig   | PACKED BCD |
| 387     | RTC Start Stunde           | PACKED BCD |
| 388     | RTC Start Minute           | PACKED BCD |
| 389     | Weckzeit Täglich Stunde    | PACKED BCD |
| 390     | Weckzeit Täglich Minute    | PACKED BCD |
| 391     | Tag Zeitumstellung März    | PACKED BCD |
| 392     | Tag Zeitumstellung Oktober | PACKED BCD |

**RTC Startdatum/Zeit:**

Die Zeit zu der das Gerät das ERSTE mal (nach brennen der Daten) MIT gedrücktem Taster eingeschaltet wird.
Auf diese Zeit/Datum wird die RTC dann im Moment des Einschaltens (MIT GEDRÜCKTEM TASTER) gestellt.

**Weckzeit Täglich:**

Möglicher Bereich: 03:00 Uhr bis 23:00 Uhr.

Jeden Tag zu dieser Uhrzeit weckt die RTC den AVR auf. Dieser steuert ggf. die LEDs an.

**Zeitumstellung ME(S)Z:**

Der Tag im März und Oktober an dem die Uhren umgestellt werden.

**Format Bytes:**

Die Datenbytes müssen im Packed BCD Format vorliegen... Beispiel:

Erstmaliger Start nach brennen = 01.02.2018 - 15:00 Uhr

Weckzeit AVR täglich = 06:00 Uhr

Zeitumstellung MEZ > MESZ = 25. März

Zeitumstellung MESZ > MEZ = 28. Oktober

| Adresse Dezimal  | 384  | 385  | 386  | 387  | 388  | 389  | 390  | 391  | 392  |
| ---------------- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- | ---- |
| Byte Hexadezimal | 0x01 | 0x02 | 0x18 | 0x15 | 0x00 | 0x06 | 0x00 | 0x25 | 0x28 |

Zum Erstellen der EEPROM, Flash Daten kann die Datei "Abfuhrkalender.ods" genutzt werden.
Nach Eingabe der Daten (grüner Bereich) kann man sich die automatisch erstellten
EEPROM/Flash Daten (schwarzer Bereich) in die Zwischenablage kopieren
und damit eine Datei zum brennen des EEPROMS auf dem DS-Board erstellen, oder diese Daten
in der Datei "config.h" fest einbauen.

Linke Tabellenseite bitte mit "1" füllen (Abholung, LED an) oder leeren mit Druck auf "ENTFERNEN" (keine Abholung LED aus)

Linux Nutzer können sich die "Abfuhrkalender.ods" füllen, speichern und danach
als CSV-Datei exportieren. Feldtrenner bitte "TAB" nutzen, Texttrenner leer lassen.
Danach kann man sich aus dem CSV leicht eine Intel-Hex-Datei für's EEPROM 24C32 erstellen:
```
conv_csv_to_hex.sh Dateiname.csv > Dateiname.hex
```

### Datenspeicher = 24C32 EEPROM auf DS-Board:
- AVR einmalig brennen, Jährlich neue Abfuhrtabelle auf EEPROM des DS-Board speichern.
### Datenspeicher = AVR-Internes EEPROM:
- Jährlich neue Abfuhrtabelle im AVR-EEPROM speichern.
### Datenspeicher = AVR-Flash:
- Jährlich Software neu erstellen mit einkompilierter Abfuhrtabelle und AVR brennen.

### Jährlich zu Jahresbeginn:
- Neuen Abfallkalender erstellen --> Abfuhrkalender.ods nutzen oder Daten selbst erzeugen
- Neuen Abfuhrkalender im jeweiligen Speicher unterbringen
- ggf. Batteriewechsel auf dem DS3231-Board
- Alle Schaltungsteile wieder miteinander verbinden
- WARTEN bis Zeit zum einschalten erreicht ist (Im obigen Beispiel 1.2.2018 15:00)
- DANN Schaltung mit GEDRÜCKTEM Taster einschalten, 5 Sekunden gedrückt halten 
  ALLE LEDs blinken, Datum Uhrzeit RTC wird gesetzt auf 1.2.2018 15:00
- Schaltung vom Strom trennen und OHNE gedrückten Schalter wieder einschalten

### Nach Bedarf:
**Batteriewechsel Hauptgerät:**
- Austauschen und OHNE gedrückten Taster wieder einschalten
- Das DS-Board läuft so lange mit seiner Knopfzelle

**Batteriewechsel DS-Board:**
- Sollte selten nötig sein, da das Board im Normalfall über die Batterie des Hauptgeräts versorgt wird
- Verfahren wie zu Jahresbeginn, da die RTC dann gestellt werden muß

### Programmablauf:
1. AVR wird täglich vom DS3231 zur Weckzeit geweckt.
   - Falls LED-Daten zum Datum hinterlegt
     - LEDs passend zum Datum ansteuern bis:
       - Es ca. 00:00 Uhr geworden ist oder Nutzer den Taster drückt
         - LEDs aus, AVR geht bis 01:30 Uhr schlafen
   - Falls heute keine LEDs gesteuert werden müssen
     - LEDs aus, AVR geht bis 01:30 Uhr schlafen
2. AVR wird jede Nacht um 01:30 vom DS3231 geweckt
   - Prüfung ob heute die Zeit umgestellt werden muß
     - Falls ja, wird die RCT eine Stunde vor oder zurück gestellt
     - AVR geht bis zur täglichen Weckzeit schlafen

### Statusblinken in Endlosschleife:
*100ms an / 233ms aus*
- Alle LEDs: Zeitregister RTC beim Start (Knopdruck 5Sek) erfolgreich gestellt
  - Neustart ohne gedrückte Taste erforderlich
- LED0 & LED1: Fehler beim Zugriff auf DS3231
  - Slave Adresse in config.h falsch 
  - DS3231 defekt
- LED2 & LED3: Fehler beim Zugriff auf AT23C32
  - Slave Adresse in config.h falsch
  - AT24C32 defekt

### Blinkanzeige nach Start OHNE Tasterdruck:
*100ms an / 500ms aus*
- Datum & Uhrzeit werden angezeigt
  - LED0 blinkt 20 mal
  - LED1 blinkt 10 mal
  - LED2 blinkt 15 mal
  - LED3 blinkt 25 mal
- Es ist der 20. Oktober, 15:25 Uhr
 
### Normalblinken
*100ms an / 900ms aus*
- Die LEDs zeigen welche Tonnen vor die Tür müssen

### Nutzung
Der Taster wird, wie oben bereits beschrieben, zum einmaligen stellen der RTC zu Jahresbeginn benötigt.
Im Normalbetrieb kann der Taster betätigt werden um die LEDs auszuschalten und den AVR wieder in Tiefschlaf zu versetzten,
sozusagen als Bestätigung "Ich hab die Tonnen rausgebracht. Bei geschickter Wahl von "Weckzeit" des AVR läßt sich somit
die Leuchtdauer der LEDs reduzieren.

![Bild](https://github.com/thgoso/muell-blinker/blob/master/infos/Testaufbau.jpg)

### Stromverbrauch ca.
Je nach LEDs und Vorwiderständen bei mir 3-5 mA wenn LEDs angesteuert werden und 150 µA wenn der AVR schläft. Auf dem DS-Board wurde die rote Power-On-LED entfernt.
