#!/bin/bash

# conv.sh
#
# Copyright 2018 Thomas Gollmer <th_goso@freenet.de>
#
# This program is free software; you can redistribute it and/or modify
# it under the terms of the GNU General Public License as published by
# the Free Software Foundation; either version 3 of the License, or
# (at your option) any later version.
#
# This program is distributed in the hope that it will be useful,
# but WITHOUT ANY WARRANTY; without even the implied warranty of
# MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
# GNU General Public License for more details.
#
# You should have received a copy of the GNU General Public License
# along with this program; if not, write to the Free Software
# Foundation, Inc., 51 Franklin Street, Fifth Floor, Boston,
# MA 02110-1301, USA.


# ----------------------------------------------------------------------------------------------------------------------
# hexdata[] erstellen aus ics-Datei
# Param: /Pfad/Name.ics
function make_from_ics() {
    IFS=$'\n'
	# Array mit Daten und passend dazu Tonnennamen erstellen
    echo ""
	echo "Analyse $1:"
	daten=($(cat "$1" | awk -F ":" '/^DTSTART/ {print substr($2,7,2) "." substr($2,5,2) "." substr($2,1,4)}'))
    from=$(cat "$1" | egrep "^SUMMARY|^DESCRIPTION" | sort | uniq)
    echo "$from"
    echo ""
    echo "Sind die Tonnenbezeichner in [S]UMMARY oder [D]ESCRIPTION untergebracht ?"
    while ((1)) ; do
      read -p "Eingabe s oder d :" quest
      if [ "$quest" = "s" ] ; then
        tonnen=($(cat "$1" | tr "\r" "\n" | awk -F ":" '/^SUMMARY/ {print $2}'))
        break
      elif [ "$quest" = "d" ] ; then
        tonnen=($(cat "$1" | tr "\r" "\n" | awk -F ":" '/^DESCRIPTION/ {print $2}'))
        break
      fi
    done
	num_data="${#daten[@]}"

	# Abfrage Tag abziehen ?
    echo ""
	echo "Sollen die LEDs einen Tag [v]or der Abholung"
	echo "oder [a]m Tag der Abholung gesteuert werden ?"
	while ((1)) ; do
	  read -p "Eingabe v oder a :" quest
	  if [ "$quest" = "a" ] || [ "$quest" = "v" ] ; then break ; fi
	done

	# ggf. zu jedem Datum einen Tag abziehen, Übersicht anzeigen
	if [ "$quest" = "v" ] ; then
	  for ((cnt=0; cnt<num_data; cnt++)) ; do
		echo -n "${daten[$cnt]} - "
		tmp=$(echo "${daten[$cnt]}" | awk -F "." '{print $3 "-" $2 "-" $1}')
		daten[$cnt]=$(date +%d.%m.%Y -d "$tmp - 1 day")
		echo "${daten[$cnt]} - ${tonnen[$cnt]}"
	  done
	  echo "^Abholung^   ^LEDs--an^   ^Tonne^"
	else
	  for ((cnt=0; cnt<num_data; cnt++)) ; do
		echo "${daten[$cnt]} - ${daten[$cnt]} - ${tonnen[$cnt]}"
	  done
	  echo "^Abholung^   ^LEDs--an^   ^Tonne^"
	fi

	# LED-Zuordnung abfragen led[] = Tonnenbezeichner, val[] = Dezimalwert LED-Steuerung für diese Tonne
	echo ""
	echo "Tonnen bitte den LEDs zuordnen LED[0] ... LED[6]"
	echo "Soll eine Tonne unberücksichtigt bleiben, 'x' eingeben"
	leds=($(echo "${tonnen[*]}" | sort | uniq))
	num_leds="${#leds[@]}"
	for ((cnt=0; cnt<num_leds; cnt++)) ; do
	  while ((1)) ; do
		read -p "${leds[$cnt]} = 0,1,2,3,4,5,6,x :" quest
		if [ "$quest" = "0" ] ; then 
		  val[$cnt]="1"
		  break
		elif [ "$quest" = "1" ] ; then 
		  val[$cnt]="2"
		  break
		elif [ "$quest" = "2" ] ; then 
		  val[$cnt]="4"
		  break
		elif [ "$quest" = "3" ] ; then 
		  val[$cnt]="8"
		  break
		elif [ "$quest" = "4" ] ; then 
		  val[$cnt]="16"
		  break
		elif [ "$quest" = "5" ] ; then 
		  val[$cnt]="32"
		  break
		elif [ "$quest" = "6" ] ; then 
		  val[$cnt]="64"
		  break
		elif [ "$quest" = "x" ] ; then 
		  val[$cnt]="0"
		  break
		fi
	  done
	done

	# in tonnen[] nun die Bezeichner durch Byte-Werte-Dezimal ersetzen
	for ((cnt_data=0; cnt_data<num_data; cnt_data++)) ; do
	  for ((cnt_leds=0; cnt_leds<num_leds; cnt_leds++)) ; do
		if [ "${tonnen[$cnt_data]}" = "${leds[$cnt_leds]}" ] ; then
		  tonnen[$cnt_data]="${val[$cnt_leds]}"
		fi
	  done
	done

	# daten[] hält jetzt das Datum vor, tonnen[] das dazugehörige LED-Byte
	# Ausgabedaten dezimal erstellen in hexdata[]
	for ((cnt=0; cnt<=394; cnt++)) ; do
	  hexdata[$cnt]="0"
	done
	for ((cnt=0; cnt<num_data; cnt++)) ; do
	  # Adresse (EEPROM/Array) für Datum berechnen: ((Monat-1)*32)+(Tag-1)
	  adr=$(echo "${daten[$cnt]}" | awk -F "." '{print (($2 - 1) * 32) + ($1 - 1)}')
	  # LED-Byte neu berechnen
	  hexdata[$adr]=$((${tonnen[$cnt]} | ${hexdata[$adr]}))
	done
	# Ausgabedaten in hexdata[] jetzt erst umwandeln in hex
	for ((cnt=0; cnt<=394; cnt++)) ; do
	  hex=$(printf "%02X" "${hexdata[$cnt]}")
	  hexdata[$cnt]="$hex"
	done

	# Abfrage Restwerte
	echo ""
	echo "Restliche Angaben"
	read -p "RTC Startdatum TT.MM.JJ " quest
	hexdata[384]=$(echo "$quest" | awk -F "." '{print $1}')
	hexdata[385]=$(echo "$quest" | awk -F "." '{print $2}')
	hexdata[386]=$(echo "$quest" | awk -F "." '{print $3}')
	read -p "RTC Startzeit HH:MM " quest
	hexdata[387]=$(echo "$quest" | awk -F ":" '{print $1}')
	hexdata[388]=$(echo "$quest" | awk -F ":" '{print $2}')
	read -p "Weckzeit täglich HH:MM " quest
	hexdata[389]=$(echo "$quest" | awk -F ":" '{print $1}')
	hexdata[390]=$(echo "$quest" | awk -F ":" '{print $2}')
	read -p "Schlafenszeit täglich HH:MM " quest
	hexdata[391]=$(echo "$quest" | awk -F ":" '{print $1}')
	hexdata[392]=$(echo "$quest" | awk -F ":" '{print $2}')
	read -p "Zeitumstellung März TT " quest
	hexdata[393]=$quest
	read -p "Zeitumstellung Oktober TT " quest
	hexdata[394]=$quest
}
# ----------------------------------------------------------------------------------------------------------------------
# hexdata[] erstellen aus ods-Datei
# Param: /Pfad/Name.ics
function make_from_ods() {
  IFS=$'\n'
  uuid=$(cat /proc/sys/kernel/random/uuid)
  cp "$1" "/tmp/$uuid.ods"
  echo "Lese $1:"
  soffice --headless --convert-to csv "/tmp/$uuid.ods" --outdir "/tmp" &> /dev/null
  if ! [ -f "/tmp/$uuid.csv" ] ; then
    echo "Lesen aus Dokument nicht möglich !"
    exit 1
  fi
  hexdata=($(cat "/tmp/$uuid.csv" | awk -F ',' '{if (NR > 3) print $12}'))
  rm -f "/tmp/$uuid.csv"
  rm -f "/tmp/$uuid.ods"
}
# ----------------------------------------------------------------------------------------------------------------------
# Main
old_IFS="$IFS"

# Eingabedatei abfragen, Bytes lesen nach hexdata[0...394]
read -p "Eingabedatei :" datei_ein
typ=$(file "$datei_ein" | egrep -o -i "vcalendar|opendocument spreadsheet" | tr "[:upper:]" "[:lower:]")
if [ "$typ" = "vcalendar" ] ; then
  echo "Dateityp: vCalendar"
  make_from_ics "$datei_ein"
elif [ "$typ" = "opendocument spreadsheet" ] ; then
  echo "Dateityp: OpenDocument Spreadsheet"
  make_from_ods "$datei_ein"
else 
  echo "Dateityp: unbekannt"
  exit 0
fi

# Ausgabedatei abfragen
while ((1)) ; do
  read -p "Ausgabedatei :" datei_aus
  if [ -n "$datei_aus" ] ; then break ; fi
done

# Ausgabe erzeugen
echo "1)     Textdatei HEX-Byte"
echo "2)     Textdatei HEX-Adr HEX-Byte" 
echo "3)     #define zum einkompilieren"
echo "4)     Intel-Hex"
echo "5)     S-Record"
echo "6)     Binärdatei"

echo "Enter) Ende"
read -p "Ausgabeformat wählen: " opt

case "$opt" in
  "1") echo "Schreibe $datei_aus"
       rm -f "$datei_aus"
       for ((cnt=0; cnt<=394; cnt++)) ; do
         echo "0x${hexdata[$cnt]}" >> "$datei_aus"
       done
       echo "" >> "$datei_aus"
       ;;
  "2") echo "Schreibe $datei_aus"
       rm -f "$datei_aus"
       for ((cnt=0; cnt<=394; cnt++)) ; do
         adr=$(printf "%04X" "$cnt")
         echo "0x$adr 0x${hexdata[$cnt]}" >> "$datei_aus"
       done
       echo "" >> "$datei_aus"
       ;;
  "3") echo "Schreibe $datei_aus"
       echo "#define MY_DATA \\" > "$datei_aus"
       for ((cnt_sp=0; cnt_sp<=394; cnt_sp+=16)) ; do
         for ((cnt_li=0; cnt_li<=15; cnt_li++)) ; do
           adr=$(($cnt_sp + $cnt_li))
           if [ "$adr" = "394" ] ; then 
             echo "0x${hexdata[$adr]}" >> "$datei_aus"
             break 
           fi
           if [ "$cnt_li" = "15" ] ; then
               echo "0x${hexdata[$adr]}, \\" >> "$datei_aus"
             else
               echo -n "0x${hexdata[$adr]}," >> "$datei_aus"
           fi
         done
       done
       echo "" >> "$datei_aus"
       ;;
  "4") echo "Schreibe $datei_aus"
       uuid=$(cat /proc/sys/kernel/random/uuid)
       for ((cnt=0; cnt<=394; cnt++)) ; do
         echo "${hexdata[$cnt]}" | xxd -r -p >> "/tmp/$uuid.tmp"
       done
       avr-objcopy -I binary -O ihex "/tmp/$uuid.tmp" "$datei_aus"
       rm -f "/tmp/$uuid.tmp"
       ;;
  "5") echo "Schreibe $datei_aus"
       uuid=$(cat /proc/sys/kernel/random/uuid)
       for ((cnt=0; cnt<=394; cnt++)) ; do
         echo "${hexdata[$cnt]}" | xxd -r -p >> "/tmp/$uuid.tmp"
       done
       avr-objcopy -I binary -O srec "/tmp/$uuid.tmp" "$datei_aus"
       rm -f "/tmp/$uuid.tmp"
       ;;
  "6") echo "Schreibe $datei_aus"
       rm -f "$datei_aus"
       for ((cnt=0; cnt<=394; cnt++)) ; do
         echo "${hexdata[$cnt]}" | xxd -r -p >> "$datei_aus"
       done
       ;;
esac

IFS="$old_IFS"
exit 0

