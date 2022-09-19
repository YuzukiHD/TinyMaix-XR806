#!/bin/bash
#
#@Description: 1. generate ecc256 private_key and public_key,
#              2. use sha256 to calculate public_key digest.

private_key="ecc_private_key.pem"
public_key="ecc_public_key.pem"
digest="sha256.txt"
pk_coordinate="pk_sample.bin"

generate_key_pair()
{
	openssl ecparam -name prime256v1 -genkey -out $private_key
	openssl ec -in $private_key -pubout -out $public_key
	openssl ec -inform PEM -in $public_key -pubin -outform DER -out $public_key.der
}

extract_pk_coordinate()
{
	dd if=$public_key.der of=$pk_coordinate bs=1 count=64 skip=27
}

calculate_sha256()
{
	openssl dgst -sha256 -out $digest $1
	printf "\033[0;31;1m"
	cat $digest
	printf "\033[0m"
}

gen_key()
{
	generate_key_pair
	extract_pk_coordinate
	calculate_sha256 $pk_coordinate

	echo public key size is `stat -c "%s" $pk_coordinate`
}

gen_key
