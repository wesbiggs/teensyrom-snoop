#!/bin/bash
# A compile check of the TeensyROM main sketch for the Teensy 4.1's Cortex-M7 WITHOUT the Teensyduino
# install (pjrc.com is unreachable from some build containers): the Teensy core and libraries from
# GitHub, Debian/Ubuntu's arm-none-eabi-g++ (GCC 13, not Teensyduino 1.61's 11.3), a rough stand-in for
# the Arduino sketch merge (merge.py) and a stub Bounce.h.  Compile only, no link; the point is
# compile errors in the patch and the code GCC makes for isrPHI2 / SnoopTap.
#
#   tools/armcheck/build.sh <3p dir> <TeensyROM checkout> <name> [patch...]
#   <3p dir> holds cores/ (PaulStoffregen/cores) and libs/{SD,SdFat,SPI,FNET,NativeEthernet,EEPROM,Time,CRC32}
#   (PaulStoffregen/SD, PaulStoffregen/SdFat, PaulStoffregen/SPI, vjmuzik/FNET, vjmuzik/NativeEthernet,
#    PaulStoffregen/EEPROM, PaulStoffregen/Time, bakercp/CRC32).
# Writes build/armcheck/<name>.o, .log and .isr.s (the disassembly of isrPHI2 and SnoopTap).
set -u
H=$1; TR=$2; NAME=$3; shift 3
HERE=$(cd "$(dirname "$0")" && pwd)
OUT=$(pwd)/build/armcheck; mkdir -p "$OUT"
T="$OUT/tree-$NAME"
rm -rf "$T" && mkdir -p "$T" && git -C "$TR" archive HEAD | tar -x -C "$T"
for p in "$@"; do P=$(cd "$(dirname "$p")" && pwd)/$(basename "$p"); (cd "$T" && git apply --whitespace=nowarn "$P") || exit 1; done
S=$T/Source/Teensy
python3 "$HERE/merge.py" "$S" "$S/merged.cpp" >/dev/null
INC="-I$H/cores/teensy4 -I$HERE/stub -I$S -I$S/src/USBHost_t36 -I$S/src/PN532"
for l in SD/src SdFat/src SPI FNET/src NativeEthernet/src EEPROM Time CRC32/src; do INC="$INC -I$H/libs/$l"; done
arm-none-eabi-g++ -c -O2 -g -mthumb -mcpu=cortex-m7 -mfloat-abi=hard -mfpu=fpv5-d16 \
  -D__IMXRT1062__ -DTEENSYDUINO=161 -DARDUINO_TEENSY41 -DARDUINO=10819 -DF_CPU=600000000 \
  -DUSB_MIDI_SERIAL -DLAYOUT_US_ENGLISH -DFab04_Features -DFNET_CFG_TLS=0 -DVM_EXTENSIONS_ENABLED \
  -std=gnu++17 -fno-exceptions -fpermissive -fno-rtti -fno-threadsafe-statics -felide-constructors \
  -Wno-error=narrowing -ffunction-sections -fdata-sections -w \
  $INC "$S/merged.cpp" -o "$OUT/$NAME.o" > "$OUT/$NAME.log" 2>&1
RC=$?
echo "$NAME: exit $RC ($(grep -c 'error:' "$OUT/$NAME.log") errors)"
[ $RC -eq 0 ] || exit $RC
arm-none-eabi-objdump -d --no-show-raw-insn -C "$OUT/$NAME.o" | awk '/<(isrPHI2\(\)|SnoopTap\(unsigned short\))>:/{p=1} p{print} p&&/^$/{p=0}' > "$OUT/$NAME.isr.s"
echo "isrPHI2 saves: $(awk '/<isrPHI2\(\)>:/{p=1} p&&/stmdb|push/{print; exit}' "$OUT/$NAME.isr.s" | sed 's/^[^\t]*\t//')"
