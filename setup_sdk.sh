#!/bin/bash

#
# project toolchain setup
# Copyright (C) 2022 YuzukiTsuru <gloomyghost@gloomyghost.com>. All rights reserved.
#

echo '=======================Setup Toolchain======================='
wget https://armkeil.blob.core.windows.net/developer/Files/downloads/gnu-rm/8-2019q3/RC1.1/gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2 -O gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2
echo '=======================Unarchive Toolchain======================='
tar jxf gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2
rm -rf tools/gcc-arm-none-eabi-8-2019-q3-update
rm -rf gcc-arm-none-eabi-8-2019-q3-update-linux.tar.bz2
echo '=======================Add Toolchain======================='
mv gcc-arm-none-eabi-8-2019-q3-update/ tools/
echo '=======================Add Toolchain path======================='
cp gcc.mk.in gcc.mk
sed -i 's/~\/tools\/gcc-arm-none-eabi-8-2019-q3-update\/bin/#REPLACE-WITH-PATH\/tools\/gcc-arm-none-eabi-8-2019-q3-update\/bin/g' gcc.mk
sed -i 's?#REPLACE-WITH-PATH?'`pwd`'?' gcc.mk
echo '=======================Set Tools======================='
chmod 777 tools/mkimage
chmod 777 tools/phoenixMC
echo '=======================ALL Done!======================='

