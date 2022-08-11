#!/bin/sh

make CROSS_COMPILE=aarch64-bst-linux- ARCH=arm64 O=build $1 $2 $3 $4 $5 $6 $7 $8
