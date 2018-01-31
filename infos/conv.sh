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
# Prüft ob Übergabe eine positive Zahl im angegebenen Bereich ist
# Param: Eingabestring, erlaubte Untergrenze, erlaubte Obergrenze
# Rückgabe: Zahl in stdout, wenn gültig
# Rückgabe: Leerstring in stdout, wenn ungültig
function check_number() {
  local retval=""

  if [ -n "$1" ] ; then
    if $(echo "$1" | grep -q '^[0-9]\+$') ; then
      if [ "$1" -ge "$2" ] && [ "$1" -le "$3" ] ; then
        retval="$1"
      fi
    fi
  fi
  echo "$retval"
}
# ----------------------------------------------------------------------------------------------------------------------
# Prüft ob übergebenes Datum "tt.mm.jj" oder "tt.mm.jjjj" OK ist
# Param: Datum im Format "tt.mm.jj" oder "tt.mm.jjjj"
# Rückgabe: Datum in stdout formatiert nach "tt.mm.jj" 
# Rückgabe: Leerstring, wenn Datum ungültig
function check_date() {
  local tt=$(echo "$1" | awk -F "." '{print $1}')
  local mm=$(echo "$1" | awk -F "." '{print $2}')
  local jj=$(echo "$1" | awk -F "." '{print $3}')
  local retval=$(date +%d.%m.%y -d "$jj-$mm-$tt" 2> /dev/null)
  echo "$retval"
}
# ----------------------------------------------------------------------------------------------------------------------
# g_hexdata[] erstellen aus ics-Datei
# Param: /Pfad/Name.ics
function make_from_ics() {
  local daten from tonnen leds val  # Arrays
  local cnt cnt_data cnt_leds       # Zähler
  local num_leds num_data           # Arraygrenzen
  local quest adr hex hh mm tmp     # Eingaben

  # Array mit Daten und passend dazu Tonnennamen erstellen
  # daten[] enthält immer ein Abholdatum tonnen[] die zugehörige Tonne
  echo "Analyse $1"
  echo "--------------------------------------------------------------------------------"
  daten=($(cat "$1" | awk -F ":" '/^DTSTART/ {print substr($2,7,2) "." substr($2,5,2) "." substr($2,1,4)}'))
  from=$(cat "$1" | egrep "^SUMMARY|^DESCRIPTION" | sort | uniq)
  echo "$from"
  echo "--------------------------------------------------------------------------------"
  while ((1)) ; do
    read -p "Sind die Tonnennamen in [s]ummary oder [d]escription zu finden? : " quest
    if [ "$quest" = "s" ] ; then
      tonnen=($(cat "$1" | tr "\r" "\n" | awk -F ":" '/^SUMMARY/ {print $2}'))
      break
    elif [ "$quest" = "d" ] ; then
      tonnen=($(cat "$1" | tr "\r" "\n" | awk -F ":" '/^DESCRIPTION/ {print $2}'))
      break
    fi
    echo -e -n "\033[1A\033[2K"
  done
  num_data="${#daten[@]}"

  # Abfrage Tag abziehen ?
  while ((1)) ; do
    read -p "LEDs einen Tag [v]or oder [a]m Tag der Abholung ansteuern? : " quest
    if [ "$quest" = "a" ] || [ "$quest" = "v" ] ; then break ; fi
    echo -e -n "\033[1A\033[2K"
  done

  # ggf. zu jedem Datum in daten[] einen Tag abziehen, Übersicht anzeigen
  echo "--------------------------------------------------------------------------------"
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
  echo "--------------------------------------------------------------------------------"

  # LED-Zuordnung abfragen leds[] = je ein Tonnenbezeichner
  # val[] = Dezimalwert LED-Steuerung für diese Tonne
  echo "Tonnen bitte den LEDs zuordnen LED[0]=PA0 ... LED[6]=PA6"
  echo "Soll eine Tonne unberücksichtigt bleiben 'x' eingeben"
  leds=($(echo "${tonnen[*]}" | sort | uniq))
  num_leds="${#leds[@]}"
  for ((cnt=0; cnt<num_leds; cnt++)) ; do
    while ((1)) ; do
      read -p "${leds[$cnt]} = 0,1,2,3,4,5,6,x : " quest
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
      echo -e -n "\033[1A\033[2K"
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
  # Ausgabedaten dezimal erstellen in g_hexdata[]
  for ((cnt=0; cnt<=394; cnt++)) ; do
    g_hexdata[$cnt]="0"
  done
  for ((cnt=0; cnt<num_data; cnt++)) ; do
    # Adresse (EEPROM/Array) für Datum berechnen: ((Monat-1)*32)+(Tag-1)
    adr=$(echo "${daten[$cnt]}" | awk -F "." '{print (($2 - 1) * 32) + ($1 - 1)}')
    # LED-Byte neu berechnen
    g_hexdata[$adr]=$((${tonnen[$cnt]} | ${g_hexdata[$adr]}))
  done

  # Ausgabedaten in g_hexdata[] jetzt erst umwandeln in hex
  for ((cnt=0; cnt<=394; cnt++)) ; do
    hex=$(printf "%02X" "${g_hexdata[$cnt]}")
    g_hexdata[$cnt]="$hex"
  done

  # Abfrage Restwerte
  echo "--------------------------------------------------------------------------------"
  while ((1)) ; do
    read -p "RTC Startdatum TT.MM.JJ : " quest
    quest=$(check_date "$quest")
    if [ -n "$quest" ] ; then
      echo -e "\033[1A\033[2KRTC Startdatum TT.MM.JJ : $quest"
      g_hexdata[384]=$(echo "$quest" | awk -F "." '{print $1}')
      g_hexdata[385]=$(echo "$quest" | awk -F "." '{print $2}')
      g_hexdata[386]=$(echo "$quest" | awk -F "." '{print $3}')
      break
    fi
    echo -e -n "\033[1A\033[2K"
  done

  while ((1)) ; do
    read -p "RTC Startzeit HH:MM : " quest
    hh=$(echo "$quest" | awk -F ":" '{print $1}')
    mm=$(echo "$quest" | awk -F ":" '{print $2}')
    hh=$(check_number "$hh" "0" "23")
    mm=$(check_number "$mm" "0" "59")
    if [ -n "$hh" ] && [ -n "$mm" ] ; then
      printf "\033[1A\033[2KRTC Startzeit HH:MM : %02i:%02i\n" "$hh" "$mm"
      g_hexdata[387]=$hh
      g_hexdata[388]=$mm
      break
    fi
    echo -e -n "\033[1A\033[2K"
  done

  while ((1)) ; do
    read -p "Weckzeit täglich HH:MM : " quest
    hh=$(echo "$quest" | awk -F ":" '{print $1}')
    mm=$(echo "$quest" | awk -F ":" '{print $2}')
    hh=$(check_number "$hh" "3" "14")
    mm=$(check_number "$mm" "0" "59")
    if [ -n "$hh" ] && [ -n "$mm" ] ; then
      printf "\033[1A\033[2KWeckzeit täglich HH:MM : %02i:%02i\n" "$hh" "$mm"
      g_hexdata[389]=$hh
      g_hexdata[390]=$mm
      break
    fi
    echo -e -n "\033[1A\033[2K"
  done

  while ((1)) ; do
    read -p "Schlafenszeit täglich HH:MM : " quest
    hh=$(echo "$quest" | awk -F ":" '{print $1}')
    mm=$(echo "$quest" | awk -F ":" '{print $2}')
    hh=$(check_number "$hh" "11" "22")
    mm=$(check_number "$mm" "0" "59")
    if [ -n "$hh" ] && [ -n "$mm" ] ; then
      printf "\033[1A\033[2KSchlafenszeit täglich HH:MM : %02i:%02i\n" "$hh" "$mm"
      g_hexdata[391]=$hh
      g_hexdata[392]=$mm
      break
    fi
    echo -e -n "\033[1A\033[2K"
  done

  while ((1)) ; do
    read -p "Zeitumstellung März TT : " quest
    quest=$(check_number "$quest" "20" "31")
    if [ -n "$quest" ] ; then
      echo -e "\033[1A\033[2KZeitumstellung März TT : $quest"
      g_hexdata[393]=$quest
      break
    fi
    echo -e -n "\033[1A\033[2K"
  done
  
  while ((1)) ; do
    read -p "Zeitumstellung Oktober TT : " quest
    quest=$(check_number "$quest" "20" "31")
    if [ -n "$quest" ] ; then
      echo -e "\033[1A\033[2KZeitumstellung Oktober TT : $quest"
      g_hexdata[394]=$quest
      break
    fi
    echo -e -n "\033[1A\033[2K"
  done
  echo "--------------------------------------------------------------------------------"
}
# ----------------------------------------------------------------------------------------------------------------------
# g_hexdata[] erstellen aus ods-Datei
# Param: /Pfad/Name.ods
function make_from_ods() {
  local uuid=$(cat /proc/sys/kernel/random/uuid)
  cp "$1" "/tmp/$uuid.ods"
  echo "Lese $1"
  soffice --headless --convert-to csv "/tmp/$uuid.ods" --outdir "/tmp" &> /dev/null
  if ! [ -f "/tmp/$uuid.csv" ] ; then
    echo "Lesen aus Dokument nicht möglich !"
    IFS="$g_old_IFS"
    exit 1
  fi
  g_hexdata=($(cat "/tmp/$uuid.csv" | awk -F ',' '{if (NR > 3) print $12}'))
  rm -f "/tmp/$uuid.csv"
  rm -f "/tmp/$uuid.ods"
}
# ----------------------------------------------------------------------------------------------------------------------
# Main
g_old_IFS="$IFS"
IFS=$'\n'

# Eingabedatei abfragen, Bytes lesen nach g_hexdata[0...394]
read -p "Eingabedatei : " datei_ein
typ=$(file "$datei_ein" | egrep -o -i "vcalendar|opendocument spreadsheet" | tr "[:upper:]" "[:lower:]")
if [ "$typ" = "vcalendar" ] ; then
  echo "Dateityp : vCalendar"
  make_from_ics "$datei_ein"
elif [ "$typ" = "opendocument spreadsheet" ] ; then
  echo "Dateityp : OpenDocument Spreadsheet"
  make_from_ods "$datei_ein"
else 
  echo "Dateityp : unbekannt"
  IFS="$g_old_IFS"
  exit 1
fi

# Ausgabedatei abfragen
while ((1)) ; do
  read -p "Ausgabedatei : " datei_aus
  if [ -n "$datei_aus" ] ; then break ; fi
  echo -e -n "\033[1A\033[2K"
done

# Ausgabe erzeugen
echo "1) Textdatei HEX-Byte"
echo "2) Textdatei HEX-Adr HEX-Byte" 
echo "3) #define zum einkompilieren"
echo "4) Intel-Hex"
echo "5) S-Record"
echo "6) Binärdatei"
echo "7) Ende"
while ((1)) ; do
  read -p "Ausgabeformat wählen : " opt
  opt=$(check_number "$opt" "1" "7")
  if [ -n "$opt" ] ; then break ; fi
  echo -e -n "\033[1A\033[2K"
done

case "$opt" in
  "1") echo "Schreibe $datei_aus"
       rm -f "$datei_aus"
       for ((cnt=0; cnt<=394; cnt++)) ; do
         echo "0x${g_hexdata[$cnt]}" >> "$datei_aus"
       done
       echo "" >> "$datei_aus"
       ;;
  "2") echo "Schreibe $datei_aus"
       rm -f "$datei_aus"
       for ((cnt=0; cnt<=394; cnt++)) ; do
         adr=$(printf "%04X" "$cnt")
         echo "0x$adr 0x${g_hexdata[$cnt]}" >> "$datei_aus"
       done
       echo "" >> "$datei_aus"
       ;;
  "3") echo "Schreibe $datei_aus"
       echo "#define MY_DATA \\" > "$datei_aus"
       for ((cnt_sp=0; cnt_sp<=394; cnt_sp+=16)) ; do
         for ((cnt_li=0; cnt_li<=15; cnt_li++)) ; do
           adr=$(($cnt_sp + $cnt_li))
           if [ "$adr" = "394" ] ; then 
             echo "0x${g_hexdata[$adr]}" >> "$datei_aus"
             break 
           fi
           if [ "$cnt_li" = "15" ] ; then
               echo "0x${g_hexdata[$adr]}, \\" >> "$datei_aus"
             else
               echo -n "0x${g_hexdata[$adr]}," >> "$datei_aus"
           fi
         done
       done
       echo "" >> "$datei_aus"
       ;;
  "4") echo "Schreibe $datei_aus"
       uuid=$(cat /proc/sys/kernel/random/uuid)
       for ((cnt=0; cnt<=394; cnt++)) ; do
         echo "${g_hexdata[$cnt]}" | xxd -r -p >> "/tmp/$uuid.tmp"
       done
       avr-objcopy -I binary -O ihex "/tmp/$uuid.tmp" "$datei_aus"
       rm -f "/tmp/$uuid.tmp"
       ;;
  "5") echo "Schreibe $datei_aus"
       uuid=$(cat /proc/sys/kernel/random/uuid)
       for ((cnt=0; cnt<=394; cnt++)) ; do
         echo "${g_hexdata[$cnt]}" | xxd -r -p >> "/tmp/$uuid.tmp"
       done
       avr-objcopy -I binary -O srec "/tmp/$uuid.tmp" "$datei_aus"
       rm -f "/tmp/$uuid.tmp"
       ;;
  "6") echo "Schreibe $datei_aus"
       rm -f "$datei_aus"
       for ((cnt=0; cnt<=394; cnt++)) ; do
         echo "${g_hexdata[$cnt]}" | xxd -r -p >> "$datei_aus"
       done
       ;;
  "7") echo "Keine Daten geschrieben"
       ;;
esac

IFS="$g_old_IFS"
exit 0

