#!/bin/sh

# Regens MCM files from the PNG file containing all the characters
# Uses fiam's max7456tool https://github.com/ArduCustom/max7456tool

for mcmfile in *.mcm; do
    pngfile="$(basename "$mcmfile" .mcm).png"
    max7456tool build -m 3 "$pngfile" "$mcmfile"
done
