#!/bin/bash

shlen() { perl -pe 'print, last if /^\s*#[\s\-]*OUTPUT/i' < "${0}" | wc -c ; }
trunc() { truncate --no-create --size=$( shlen ) "${0}" ; }
append() { "${@}" 2>&1 | sed -e 's/^/# /' | tee -a "${0}" | sed -e 's/^# //' ; }

for f in paths functions aliases; do 
	. "$( goshconfig -d )/tinygo/${f}.bash"; 
done

# ----------- MAIN -----------
src=$( realpath -e "${0%/*}" )
trunc; append tinygo-build -x -b -o "${src##*/}.elf" "${src}"

# ---------- OUTPUT ----------
# [ source   ] /usr/local/go/src/github.com/ardnew/teensy40-matrix
# [ tinygo   ] /usr/local/src/tinygo/build/tinygo
# [ target   ] teensy40
# [ command  ] /usr/local/src/tinygo/build/tinygo build -target=teensy40 -size=full -o=teensy40-matrix.elf /usr/local/go/src/github.com/ardnew/teensy40-matrix
#    code  rodata    data     bss |   flash     ram | package
#    5864     101       8     130 |    5973     138 | (bootstrap)
#    1964      55     320    1238 |    2339    1558 | device/nxp
#    1658     156      32    4621 |    1846    4653 | github
#       0     454       0       0 |     454       0 | handleHardFault$string
#     154      24       0       4 |     178       4 | internal/task
#    4440     257      48       0 |    4745      48 | machine
#       0       0       0     130 |       0     130 | machine$alloc
#    4246       0       0       0 |    4246       0 | main
#       0      32       0       0 |      32       0 | math/bits
#    1396     104       0      48 |    1500      48 | runtime
#      98       0       0       0 |      98       0 | runtime/interrupt
#   19820    1183     408    6171 |   21411    6579 | (sum)
#   29904       -     408   14432 |   30312   14840 | (all)
# [ objcopy  ] /usr/local/src/tinygo/llvm-build/bin/llvm-objcopy -O ihex -R .eeprom teensy40-matrix.elf ./teensy40-matrix.hex
# [ objcopy  ] /usr/local/src/tinygo/llvm-build/bin/llvm-objcopy -O binary teensy40-matrix.elf ./teensy40-matrix.bin
