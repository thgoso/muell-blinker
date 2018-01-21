#/bin/bash

if [ -z "$1" ] ; then
  echo "1. Abfuhrkalender.ods füllen, speichern und als CSV-Datei exportieren."
  echo "   Feldtrenner = TAB, Texttrenner = Keiner"
  echo "2. Dieses Skript aufrufen mit $0 Input.csv"
  echo "3. Datei Input.csv wird gelesen, Ausgabe Intel-Hex-Format"
  exit 1
fi

if ! [ -f "$1" ] ; then
  echo "$0: $1 nicht gefunden"
  exit 1
fi

rm -f "/tmp/Abfuhrkalender.bin"
rm -f "/tmp/Abfuhrkalender.hex"

# Aus csv ab Zeile 4 jeweils Spalte 12 (HEX Databyte) lesen; Binärdatei erstellen
cat "$1" | awk -F '\t' '{if (NR > 3) print $12}' | xxd -r -p >> "/tmp/Abfuhrkalender.bin"
# Binärdatei --> Intel-Hex
avr-objcopy -I binary -O ihex "/tmp/Abfuhrkalender.bin" "/tmp/Abfuhrkalender.hex"
# Ausgabe in stdout
cat "/tmp/Abfuhrkalender.hex"

rm -f "/tmp/Abfuhrkalender.bin"
rm -f "/tmp/Abfuhrkalender.hex"

exit 0

