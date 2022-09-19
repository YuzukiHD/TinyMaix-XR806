#!/bin/bash

imgcfg="$1"
sign_dir="sign_script"
gen_signature="gen_signature.sh"

if [ $# -ne 1 ]; then
	echo "Usage $0 <image_cfg_file>"
	exit 1
fi

if [ ! -e ${imgcfg} ]; then
	echo "${imgcfg} not exist!"
	exit 1
fi

if [ ! -e $sign_dir/$gen_signature ]; then
	echo "$sign_dir/$gen_signature not exist!"
	exit 1
fi

# remove useless keyword and keep the value only
imginfo=`grep bin $imgcfg`
printf "image cfg: $imgcfg\n"
printf "image information:\n$imginfo\n"
imginfo=`echo "$imginfo" | sed "s/id\|\"bin\"\|cert\|flash_offs\|sram_offs\|ep\|attr\|\"\| \|{\|}\|\:\|\t\|\r//g"`

for image in ${imginfo[*]}; do
	imagename=`echo $image  | awk -F "[,]" '{print $2}'`
	cert=`echo $image | awk -F "[,]" '{print $3}'`
	attr=`echo $image  | awk -F "[,]" '{print $7}'`
	printf "imagename $imagename, cert $cert, attr $attr\n"
	if [ "$cert" != "null" -a $[attr & 0x4] -eq 4 ];then
		printf "generate sign [\033[31m${cert}\033[0m] for image [\033[31m${imagename}\033[0m]!!\n"
		cp $imagename $sign_dir && cd $sign_dir
		img_prefix=${imagename%_*}
		if [ "$img_prefix" == "boot" ]; then
			mv $imagename boot.bin
			imagename="boot.bin"
		fi
		./$gen_signature $imagename $cert
		mv $cert ../ && cd ..
	fi
done

