#!/bin/bash

#
# project toolchain setup
# Copyright (C) 2022 YuzukiTsuru <gloomyghost@gloomyghost.com>. All rights reserved.
#

echo '=======================Setup Toolchain======================='
rm -rf gcc-arm-none-eabi-10.3-2021.07-x86_64-linux.tar.bz2
wget https://developer.arm.com/-/media/Files/downloads/gnu-rm/10.3-2021.07/gcc-arm-none-eabi-10.3-2021.07-x86_64-linux.tar.bz2 -O gcc-arm-none-eabi-10.3-2021.07-x86_64-linux.tar.bz2
echo '=======================Unarchive Toolchain======================='
echo '=======================Please Wait======================='
tar jxf gcc-arm-none-eabi-10.3-2021.07-x86_64-linux.tar.bz2
rm -rf tools/gcc-arm-none-eabi-10.3-2021.07
rm -rf gcc-arm-none-eabi-10.3-2021.07-x86_64-linux.tar.bz2
echo '=======================Add Toolchain======================='
mv gcc-arm-none-eabi-10.3-2021.07/ tools/
echo '=======================Add Toolchain path======================='
cp gcc.mk.in gcc.mk
sed -i 's/~\/tools\/gcc-arm-none-eabi-8-2019-q3-update\/bin/#REPLACE-WITH-PATH\/tools\/gcc-arm-none-eabi-10.3-2021.07\/bin/g' gcc.mk
sed -i 's?#REPLACE-WITH-PATH?'`pwd`'?' gcc.mk
echo '=======================Set Tools======================='
chmod 777 tools/mkimage
chmod 777 tools/phoenixMC
echo '=======================ALL Done!======================='

