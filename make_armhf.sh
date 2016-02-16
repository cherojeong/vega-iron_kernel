#!/bin/sh
export LC_ALL=C
make -j8 O=../out ARCH=arm CROSS_COMPILE=arm-linux-gnueabihf- KBUILD_BUILD_USER=cherojeong KBUILD_BUILD_HOST=gmail.com zImage 2>&1 | tee build.txt
