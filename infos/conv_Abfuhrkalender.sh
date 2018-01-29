#/bin/bash

file_name="Abfuhrkalender"
dir_tmp="/tmp"

if ! [ -f "${file_name}.ods" ] ; then
  echo "${file_name}.ods nicht gefunden"
  exit 1
fi

# Temp-Dateien
rm -f "${dir_tmp}/${file_name}.csv"
rm -f "${dir_tmp}/${file_name}.bin"

# Abfuhrkalender.ods --> /tmp/Abfuhrkalender.csv
echo "Konvertiere ${file_name}.ods --> ${dir_tmp}/${file_name}.csv"
soffice --headless --convert-to csv "${file_name}.ods" --outdir "$dir_tmp" &> /dev/null
if ! [ -f "${dir_tmp}/${file_name}.csv" ] ; then
  echo "Konvertieren nicht möglich!"
  exit 1
fi

# /tmp/Abfuhrkalender.csv --> /tmp/Abfuhrkalender.bin
echo "Konvertiere ${dir_tmp}/${file_name}.csv --> ${dir_tmp}/${file_name}.bin"
cat "${dir_tmp}/${file_name}.csv" | awk -F ',' '{if (NR > 3) print $12}' | xxd -r -p > "${dir_tmp}/${file_name}.bin"
if ! [ -f "${dir_tmp}/${file_name}.bin" ] ; then
  echo "Konvertieren nicht möglich!"
  exit 1
fi

# Ausgabe erzeugen
echo "1)     Textdatei HEX-Byte"
echo "2)     Textdatei HEX-Adr HEX-Byte" 
echo "3)     Binärdatei"
echo "4)     Intel-Hex"
echo "5)     S-Record"
echo "Enter) Ende"
read -p "Ausgabeformat wählen: " opt

case "$opt" in
  "1") echo "Schreibe ${file_name}.txt"
       cat "${dir_tmp}/${file_name}.csv" | awk -F ',' '{if (NR > 3) print "0x" $12}' > "${file_name}.txt"
       echo "" >> "${file_name}.txt"
       ;;
  "2") echo "Schreibe ${file_name}.txt"
       cat "${dir_tmp}/${file_name}.csv" | awk -F ',' '{if (NR > 3) print "0x" $11 " 0x" $12}' > "${file_name}.txt"
       echo "" >> "${file_name}.txt"
       ;;
  "3") echo "Schreibe ${file_name}.bin"
       cp "${dir_tmp}/${file_name}.bin" "${file_name}.bin"
       ;;
  "4") echo "Schreibe ${file_name}.hex"
       avr-objcopy -I binary -O ihex "${dir_tmp}/${file_name}.bin" "${file_name}.hex"
       ;;
  "5") echo "Schreibe ${file_name}.hex"
       avr-objcopy -I binary -O srec "${dir_tmp}/${file_name}.bin" "${file_name}.hex"
       ;;
esac

# Temp-Dateien
rm -f "${dir_tmp}/${file_name}.csv"
rm -f "${dir_tmp}/${file_name}.bin"

exit 0

