#!/bin/bash
for bauds in $(
    sed -r 's/^#define\s+B([1-9][0-9]+)\s+.*/\1/p;d' < \
        /usr/include/asm-generic/termbits.h ) ;do
    echo $bauds
    stty -F /dev/ttyACM0 $bauds && echo Ok.
done 2>&1 |
    pr -at2
