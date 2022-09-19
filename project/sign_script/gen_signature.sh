#!/bin/bash
#
#@Description: 1. use private_key to sign the input parameter.
#              2. if "signed_file_name" is not appointed in the parameter,
#                 the output is named "xxx_sign.bin" by default.

rs_signature="rs_sample.bin"
key_dir="ecc_key"
gen_ecc256_key="gen_ecc256_key.sh"
private_key=$key_dir/"ecc_private_key.pem"
public_key=$key_dir/"ecc_public_key.pem"
pk_coordinate=$key_dir/"pk_sample.bin"
input=$1
output=$2

if [ $# -lt 1 ] || [ $# -gt 2 ]; then
	echo "Usage $0 <input_file> <signed_file_name>"
	exit 1
fi

if [ ! -e $1 ]; then
	echo "The input file does not exist."
	exit 1
fi

extract_signature_rs()
{
	local offset=0
	local length=0

	dd if=$1 of=temp.txt bs=1 count=1 skip=3
	length=`cat temp.txt`;length=`printf "%x" "'$length"`
	if [[ "$length" == "21" ]]; then
		dd if=$1 of=$rs_signature bs=1 count=32 skip=5
		offset=37
	else
		dd if=$1 of=$rs_signature bs=1 count=32 skip=4
		offset=36
	fi
	
	dd if=$1 of=temp.txt bs=1 count=1 skip=$(($offset+1))
	length=`cat temp.txt`;length=`printf "%x" "'$length"`
	if [[ "$length" == "21" ]]; then
		dd if=$1 of=$rs_signature bs=1 count=32 skip=$(($offset+3)) seek=32
	else
		dd if=$1 of=$rs_signature bs=1 count=32 skip=$(($offset+2)) seek=32
	fi
	
	rm -f temp.txt
}

calculate_signature()
{
	openssl dgst -sign $private_key -sha256 -out ecc_signature.bin $1
}

verify_signature()
{
	openssl dgst -verify $public_key -sha256 -signature ecc_signature.bin $1
}

gen_sign()
{
	calculate_signature $input
	extract_signature_rs ecc_signature.bin
	verify_signature $input
	if [ $? -ne 0 ]; then
		printf "input image verified signature failure!\n"
		exit 1
	fi

	rm -f ecc_signature.bin
}

check_key_exist()
{
	if [ ! -e ${private_key} ] || [ ! -e ${public_key} ] || [ ! -e ${pk_coordinate} ]; then
		printf "\033[0;31;1m"
		echo "The secret key file in directory("$key_dir") is incomplete, Try to auto generate it!"
		printf "\033[0m"
		cd $key_dir && ./$gen_ecc256_key && cd -
	fi
}

main_func()
{
	check_key_exist
	gen_sign $input

	if [ $# -eq 2 ]; then
		if [ "$input" == "boot.bin" ]; then
			cat $pk_coordinate > $output
			cat $rs_signature >> $output
		else
			cat $rs_signature > $output
		fi
	else
		echo "Output by default name: xxx_sign.bin!"
		local prefix=${input%.*}
		if [ "$input" == "boot.bin" ]; then
			cat $pk_coordinate > boot_pubk_sign.bin
			cat $rs_signature >> boot_pubk_sign.bin
		else
			cat $rs_signature > ${prefix}_sign.bin
		fi
	fi

	rm -f $rs_signature $input
}

main_func $input $output
