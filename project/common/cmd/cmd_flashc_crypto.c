/*
 * Copyright (C) 2017 XRADIO TECHNOLOGY CO., LTD. All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *    1. Redistributions of source code must retain the above copyright
 *       notice, this list of conditions and the following disclaimer.
 *    2. Redistributions in binary form must reproduce the above copyright
 *       notice, this list of conditions and the following disclaimer in the
 *       documentation and/or other materials provided with the
 *       distribution.
 *    3. Neither the name of XRADIO TECHNOLOGY CO., LTD. nor the names of
 *       its contributors may be used to endorse or promote products derived
 *       from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
 *  A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
 *  OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
 *  SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
 *  LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
 *  DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
 *  THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
 *  (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
 *  OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
 */

#include "image/flash.h"
#include "cmd_debug.h"
#include "cmd_util.h"
#include "driver/chip/hal_flash.h"
#include "driver/chip/psram/hal_psramctrl.h"
#include "driver/chip/hal_dcache.h"
#include "sys/io.h"
#include "sys/sys_heap.h"

#if ((CONFIG_CHIP_ARCH_VER == 3) && defined(CONFIG_FLASH_CRYPTO))
#ifdef CONFIG_TRUSTZONE
#include "trustzone/nsc_table.h"
#include "trustzone/tz_rpc.h"
#endif

/*
 * Flash Crypto disable interface take effect option for test. If the flash
 * crypto disable interface is avaliable, configure this macro to 1; Otherwise
 * configure the macro is 0.
 */
#define FLASH_CRYPTO_DISABLE_CAN_USE    1

typedef struct {
	uint32_t crypto_type;
	uint32_t crypto_addr;
	uint32_t crypto_length;
	int32_t  crypto_channel;
	OS_Thread_t thread;
	OS_Semaphore_t sem;
	OS_Queue_t queue;
	uint32_t test_times;
} cmd_fc_t;
static cmd_fc_t g_fc_info;

#define MFLASH 0

static uint8_t flash_enc_nonce[6] = {0x50, 0x00, 0x06, 0x20, 0x00, 0x00};

// The number of columns comprising a state in AES. This is a constant in AES.
// Value=4
#define Nb 4

// The number of rounds in AES Cipher. It is simply initiated to zero. The
// actual value is recieved in the program.
int Nr;

// The number of 32 bit words in the key. It is simply initiated to zero. The
// actual value is recieved in the program.
int Nk;

// in - it is the array that holds the plain text to be encrypted.
// out - it is the array that holds the key for encryption.
// state - the array that holds the intermediate results during encryption.
//unsigned char in[16], out[16], state[4][4];
unsigned char state[4][4];
// The array that stores the round keys.
unsigned char RoundKey[240];

// The Key input to the AES Program
//unsigned char Key[32];

static const int rsbox[256] = {
	0x52, 0x09, 0x6a, 0xd5, 0x30, 0x36, 0xa5, 0x38, 0xbf, 0x40, 0xa3, 0x9e, 0x81, 0xf3, 0xd7, 0xfb,
	0x7c, 0xe3, 0x39, 0x82, 0x9b, 0x2f, 0xff, 0x87, 0x34, 0x8e, 0x43, 0x44, 0xc4, 0xde, 0xe9, 0xcb,
	0x54, 0x7b, 0x94, 0x32, 0xa6, 0xc2, 0x23, 0x3d, 0xee, 0x4c, 0x95, 0x0b, 0x42, 0xfa, 0xc3, 0x4e,
	0x08, 0x2e, 0xa1, 0x66, 0x28, 0xd9, 0x24, 0xb2, 0x76, 0x5b, 0xa2, 0x49, 0x6d, 0x8b, 0xd1, 0x25,
	0x72, 0xf8, 0xf6, 0x64, 0x86, 0x68, 0x98, 0x16, 0xd4, 0xa4, 0x5c, 0xcc, 0x5d, 0x65, 0xb6, 0x92,
	0x6c, 0x70, 0x48, 0x50, 0xfd, 0xed, 0xb9, 0xda, 0x5e, 0x15, 0x46, 0x57, 0xa7, 0x8d, 0x9d, 0x84,
	0x90, 0xd8, 0xab, 0x00, 0x8c, 0xbc, 0xd3, 0x0a, 0xf7, 0xe4, 0x58, 0x05, 0xb8, 0xb3, 0x45, 0x06,
	0xd0, 0x2c, 0x1e, 0x8f, 0xca, 0x3f, 0x0f, 0x02, 0xc1, 0xaf, 0xbd, 0x03, 0x01, 0x13, 0x8a, 0x6b,
	0x3a, 0x91, 0x11, 0x41, 0x4f, 0x67, 0xdc, 0xea, 0x97, 0xf2, 0xcf, 0xce, 0xf0, 0xb4, 0xe6, 0x73,
	0x96, 0xac, 0x74, 0x22, 0xe7, 0xad, 0x35, 0x85, 0xe2, 0xf9, 0x37, 0xe8, 0x1c, 0x75, 0xdf, 0x6e,
	0x47, 0xf1, 0x1a, 0x71, 0x1d, 0x29, 0xc5, 0x89, 0x6f, 0xb7, 0x62, 0x0e, 0xaa, 0x18, 0xbe, 0x1b,
	0xfc, 0x56, 0x3e, 0x4b, 0xc6, 0xd2, 0x79, 0x20, 0x9a, 0xdb, 0xc0, 0xfe, 0x78, 0xcd, 0x5a, 0xf4,
	0x1f, 0xdd, 0xa8, 0x33, 0x88, 0x07, 0xc7, 0x31, 0xb1, 0x12, 0x10, 0x59, 0x27, 0x80, 0xec, 0x5f,
	0x60, 0x51, 0x7f, 0xa9, 0x19, 0xb5, 0x4a, 0x0d, 0x2d, 0xe5, 0x7a, 0x9f, 0x93, 0xc9, 0x9c, 0xef,
	0xa0, 0xe0, 0x3b, 0x4d, 0xae, 0x2a, 0xf5, 0xb0, 0xc8, 0xeb, 0xbb, 0x3c, 0x83, 0x53, 0x99, 0x61,
	0x17, 0x2b, 0x04, 0x7e, 0xba, 0x77, 0xd6, 0x26, 0xe1, 0x69, 0x14, 0x63, 0x55, 0x21, 0x0c, 0x7d
};

static const int sbox[256] = {
	//0     1    2      3     4    5     6     7      8    9     A      B    C     D     E     F
	0x63, 0x7c, 0x77, 0x7b, 0xf2, 0x6b, 0x6f, 0xc5, 0x30, 0x01, 0x67, 0x2b, 0xfe, 0xd7, 0xab, 0x76, //0
	0xca, 0x82, 0xc9, 0x7d, 0xfa, 0x59, 0x47, 0xf0, 0xad, 0xd4, 0xa2, 0xaf, 0x9c, 0xa4, 0x72, 0xc0, //1
	0xb7, 0xfd, 0x93, 0x26, 0x36, 0x3f, 0xf7, 0xcc, 0x34, 0xa5, 0xe5, 0xf1, 0x71, 0xd8, 0x31, 0x15, //2
	0x04, 0xc7, 0x23, 0xc3, 0x18, 0x96, 0x05, 0x9a, 0x07, 0x12, 0x80, 0xe2, 0xeb, 0x27, 0xb2, 0x75, //3
	0x09, 0x83, 0x2c, 0x1a, 0x1b, 0x6e, 0x5a, 0xa0, 0x52, 0x3b, 0xd6, 0xb3, 0x29, 0xe3, 0x2f, 0x84, //4
	0x53, 0xd1, 0x00, 0xed, 0x20, 0xfc, 0xb1, 0x5b, 0x6a, 0xcb, 0xbe, 0x39, 0x4a, 0x4c, 0x58, 0xcf, //5
	0xd0, 0xef, 0xaa, 0xfb, 0x43, 0x4d, 0x33, 0x85, 0x45, 0xf9, 0x02, 0x7f, 0x50, 0x3c, 0x9f, 0xa8, //6
	0x51, 0xa3, 0x40, 0x8f, 0x92, 0x9d, 0x38, 0xf5, 0xbc, 0xb6, 0xda, 0x21, 0x10, 0xff, 0xf3, 0xd2, //7
	0xcd, 0x0c, 0x13, 0xec, 0x5f, 0x97, 0x44, 0x17, 0xc4, 0xa7, 0x7e, 0x3d, 0x64, 0x5d, 0x19, 0x73, //8
	0x60, 0x81, 0x4f, 0xdc, 0x22, 0x2a, 0x90, 0x88, 0x46, 0xee, 0xb8, 0x14, 0xde, 0x5e, 0x0b, 0xdb, //9
	0xe0, 0x32, 0x3a, 0x0a, 0x49, 0x06, 0x24, 0x5c, 0xc2, 0xd3, 0xac, 0x62, 0x91, 0x95, 0xe4, 0x79, //A
	0xe7, 0xc8, 0x37, 0x6d, 0x8d, 0xd5, 0x4e, 0xa9, 0x6c, 0x56, 0xf4, 0xea, 0x65, 0x7a, 0xae, 0x08, //B
	0xba, 0x78, 0x25, 0x2e, 0x1c, 0xa6, 0xb4, 0xc6, 0xe8, 0xdd, 0x74, 0x1f, 0x4b, 0xbd, 0x8b, 0x8a, //C
	0x70, 0x3e, 0xb5, 0x66, 0x48, 0x03, 0xf6, 0x0e, 0x61, 0x35, 0x57, 0xb9, 0x86, 0xc1, 0x1d, 0x9e, //D
	0xe1, 0xf8, 0x98, 0x11, 0x69, 0xd9, 0x8e, 0x94, 0x9b, 0x1e, 0x87, 0xe9, 0xce, 0x55, 0x28, 0xdf, //E
	0x8c, 0xa1, 0x89, 0x0d, 0xbf, 0xe6, 0x42, 0x68, 0x41, 0x99, 0x2d, 0x0f, 0xb0, 0x54, 0xbb, 0x16  //F
};


int getSBoxValue(int num)
{
	return sbox[num];
}

int getSBoxInvert(int num)
{
	return rsbox[num];
}


// The round constant word array, Rcon[i], contains the values given by
// x to th e power (i-1) being powers of x (x is denoted as {02}) in the field GF(28)
// Note that i starts at 1, not 0).
static const int Rcon[255] = {
	0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36, 0x6c, 0xd8, 0xab, 0x4d, 0x9a,
	0x2f, 0x5e, 0xbc, 0x63, 0xc6, 0x97, 0x35, 0x6a, 0xd4, 0xb3, 0x7d, 0xfa, 0xef, 0xc5, 0x91, 0x39,
	0x72, 0xe4, 0xd3, 0xbd, 0x61, 0xc2, 0x9f, 0x25, 0x4a, 0x94, 0x33, 0x66, 0xcc, 0x83, 0x1d, 0x3a,
	0x74, 0xe8, 0xcb, 0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36, 0x6c, 0xd8,
	0xab, 0x4d, 0x9a, 0x2f, 0x5e, 0xbc, 0x63, 0xc6, 0x97, 0x35, 0x6a, 0xd4, 0xb3, 0x7d, 0xfa, 0xef,
	0xc5, 0x91, 0x39, 0x72, 0xe4, 0xd3, 0xbd, 0x61, 0xc2, 0x9f, 0x25, 0x4a, 0x94, 0x33, 0x66, 0xcc,
	0x83, 0x1d, 0x3a, 0x74, 0xe8, 0xcb, 0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20, 0x40, 0x80, 0x1b,
	0x36, 0x6c, 0xd8, 0xab, 0x4d, 0x9a, 0x2f, 0x5e, 0xbc, 0x63, 0xc6, 0x97, 0x35, 0x6a, 0xd4, 0xb3,
	0x7d, 0xfa, 0xef, 0xc5, 0x91, 0x39, 0x72, 0xe4, 0xd3, 0xbd, 0x61, 0xc2, 0x9f, 0x25, 0x4a, 0x94,
	0x33, 0x66, 0xcc, 0x83, 0x1d, 0x3a, 0x74, 0xe8, 0xcb, 0x8d, 0x01, 0x02, 0x04, 0x08, 0x10, 0x20,
	0x40, 0x80, 0x1b, 0x36, 0x6c, 0xd8, 0xab, 0x4d, 0x9a, 0x2f, 0x5e, 0xbc, 0x63, 0xc6, 0x97, 0x35,
	0x6a, 0xd4, 0xb3, 0x7d, 0xfa, 0xef, 0xc5, 0x91, 0x39, 0x72, 0xe4, 0xd3, 0xbd, 0x61, 0xc2, 0x9f,
	0x25, 0x4a, 0x94, 0x33, 0x66, 0xcc, 0x83, 0x1d, 0x3a, 0x74, 0xe8, 0xcb, 0x8d, 0x01, 0x02, 0x04,
	0x08, 0x10, 0x20, 0x40, 0x80, 0x1b, 0x36, 0x6c, 0xd8, 0xab, 0x4d, 0x9a, 0x2f, 0x5e, 0xbc, 0x63,
	0xc6, 0x97, 0x35, 0x6a, 0xd4, 0xb3, 0x7d, 0xfa, 0xef, 0xc5, 0x91, 0x39, 0x72, 0xe4, 0xd3, 0xbd,
	0x61, 0xc2, 0x9f, 0x25, 0x4a, 0x94, 0x33, 0x66, 0xcc, 0x83, 0x1d, 0x3a, 0x74, 0xe8, 0xcb
};

// This function produces Nb(Nr+1) round keys. The round keys are used in each
// round to encrypt the states.
void KeyExpansion(unsigned char *Key)
{
	int i, j;
	unsigned char temp[4], k;

	// The first round key is the key itself.
	for (i = 0; i < Nk; i++) {
		RoundKey[i * 4]     = Key[i * 4];
		RoundKey[i * 4 + 1] = Key[i * 4 + 1];
		RoundKey[i * 4 + 2] = Key[i * 4 + 2];
		RoundKey[i * 4 + 3] = Key[i * 4 + 3];
	}

	// All other round keys are found from the previous round keys.
	while (i < (Nb * (Nr + 1))) {
		for (j = 0; j < 4; j++) {
			temp[j] = RoundKey[(i - 1) * 4 + j];
		}
		if (i % Nk == 0) {
			// This function rotates the 4 bytes in a word to the left once.
			// [a0,a1,a2,a3] becomes [a1,a2,a3,a0]

			// Function RotWord()
			{
				k       = temp[0];
				temp[0] = temp[1];
				temp[1] = temp[2];
				temp[2] = temp[3];
				temp[3] = k;
			}

			// SubWord() is a function that takes a four-byte input word and
			// applies the S-box to each of the four bytes to produce an output word.

			// Function Subword()
			{
				temp[0] = getSBoxValue(temp[0]);
				temp[1] = getSBoxValue(temp[1]);
				temp[2] = getSBoxValue(temp[2]);
				temp[3] = getSBoxValue(temp[3]);
			}

			temp[0] =  temp[0] ^ Rcon[i/Nk];
		} else if (Nk > 6 && i % Nk == 4) {
			// Function Subword()
			{
				temp[0] = getSBoxValue(temp[0]);
				temp[1] = getSBoxValue(temp[1]);
				temp[2] = getSBoxValue(temp[2]);
				temp[3] = getSBoxValue(temp[3]);
			}
		}
		RoundKey[i * 4 + 0] = RoundKey[(i - Nk) * 4 + 0] ^ temp[0];
		RoundKey[i * 4 + 1] = RoundKey[(i - Nk) * 4 + 1] ^ temp[1];
		RoundKey[i * 4 + 2] = RoundKey[(i - Nk) * 4 + 2] ^ temp[2];
		RoundKey[i * 4 + 3] = RoundKey[(i - Nk) * 4 + 3] ^ temp[3];
		i++;
	}
}

// This function adds the round key to state.
// The round key is added to the state by an XOR function.
void AddRoundKey(int round)
{
	int i, j;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			state[j][i] ^= RoundKey[round * Nb * 4 + i * Nb + j];
		}
	}
}

// The SubBytes Function Substitutes the values in the
// state matrix with values in an S-box.
void SubBytes(void)
{
	int i, j;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			state[i][j] = getSBoxValue(state[i][j]);

		}
	}
}

// The SubBytes Function Substitutes the values in the
// state matrix with values in an S-box.
void InvSubBytes(void)
{
	int i, j;
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			state[i][j] = getSBoxInvert(state[i][j]);

		}
	}
}


// The ShiftRows() function shifts the rows in the state to the left.
// Each row is shifted with different offset.
// Offset = Row number. So the first row is not shifted.
void ShiftRows(void)
{
	unsigned char temp;

	// Rotate first row 1 columns to left
	temp        = state[1][0];
	state[1][0] = state[1][1];
	state[1][1] = state[1][2];
	state[1][2] = state[1][3];
	state[1][3] = temp;

	// Rotate second row 2 columns to left
	temp        = state[2][0];
	state[2][0] = state[2][2];
	state[2][2] = temp;

	temp        = state[2][1];
	state[2][1] = state[2][3];
	state[2][3] = temp;

	// Rotate third row 3 columns to left
	temp        = state[3][0];
	state[3][0] = state[3][3];
	state[3][3] = state[3][2];
	state[3][2] = state[3][1];
	state[3][1] = temp;
}

// The ShiftRows() function shifts the rows in the state to the left.
// Each row is shifted with different offset.
// Offset = Row number. So the first row is not shifted.
void InvShiftRows(void)
{
	unsigned char temp;

	// Rotate first row 1 columns to right
	temp        = state[1][3];
	state[1][3] = state[1][2];
	state[1][2] = state[1][1];
	state[1][1] = state[1][0];
	state[1][0] = temp;

	// Rotate second row 2 columns to right
	temp        = state[2][0];
	state[2][0] = state[2][2];
	state[2][2] = temp;

	temp        = state[2][1];
	state[2][1] = state[2][3];
	state[2][3] = temp;

	// Rotate third row 3 columns to right
	temp        = state[3][0];
	state[3][0] = state[3][1];
	state[3][1] = state[3][2];
	state[3][2] = state[3][3];
	state[3][3] = temp;
}


// xtime is a macro that finds the product of {02} and the argument to xtime
// modulo {1b}
#define xtime(x)   ((x << 1) ^ (((x >> 7) & 1) * 0x1b))

// Multiplty is a macro used to multiply numbers in the field GF(2^8)
#define Multiply(x, y) ( \
	((y & 1) * x) ^ ((y>>1 & 1) * xtime(x)) \
	^ ((y>>2 & 1) * xtime(xtime(x))) \
	^ ((y>>3 & 1) * xtime(xtime(xtime(x)))) \
	^ ((y>>4 & 1) * xtime(xtime(xtime(xtime(x))))) \
	)

// MixColumns function mixes the columns of the state matrix
void MixColumns(void)
{
	int i;
	unsigned char Tmp, Tm, t;
	for (i = 0; i < 4; i++) {
		t   = state[0][i];
		Tmp = state[0][i] ^ state[1][i] ^ state[2][i] ^ state[3][i];
		Tm  = state[0][i] ^ state[1][i] ; Tm = xtime(Tm); state[0][i] ^= Tm ^ Tmp;
		Tm  = state[1][i] ^ state[2][i] ; Tm = xtime(Tm); state[1][i] ^= Tm ^ Tmp;
		Tm  = state[2][i] ^ state[3][i] ; Tm = xtime(Tm); state[2][i] ^= Tm ^ Tmp;
		Tm  = state[3][i] ^ t ; Tm = xtime(Tm); state[3][i] ^= Tm ^ Tmp;
	}
}

// MixColumns function mixes the columns of the state matrix.
// The method used to multiply may be difficult to understand for the
// inexperienced. Please use the references to gain more information.
void InvMixColumns(void)
{
	int i;
	unsigned char a, b, c, d;
	for (i = 0; i < 4; i++) {
		a = state[0][i];
		b = state[1][i];
		c = state[2][i];
		d = state[3][i];

		state[0][i] = Multiply(a, 0x0e) ^ Multiply(b, 0x0b) ^ Multiply(c, 0x0d)
		              ^ Multiply(d, 0x09);
		state[1][i] = Multiply(a, 0x09) ^ Multiply(b, 0x0e) ^ Multiply(c, 0x0b)
		              ^ Multiply(d, 0x0d);
		state[2][i] = Multiply(a, 0x0d) ^ Multiply(b, 0x09) ^ Multiply(c, 0x0e)
		              ^ Multiply(d, 0x0b);
		state[3][i] = Multiply(a, 0x0b) ^ Multiply(b, 0x0d) ^ Multiply(c, 0x09)
		              ^ Multiply(d, 0x0e);
	}
}

// Cipher is the main function that encrypts the PlainText.
void Cipher(unsigned char *in, unsigned char *out)
{
	int i, j, round = 0;

	//Copy the input PlainText to state array.
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			state[j][i] = in[i * 4 + j];
		}
	}

	// Add the First round key to the state before starting the rounds.
	AddRoundKey(0);

	// There will be Nr rounds.
	// The first Nr-1 rounds are identical.
	// These Nr-1 rounds are executed in the loop below.
	for (round = 1; round < Nr; round++) {
		SubBytes();
		ShiftRows();
		MixColumns();
		AddRoundKey(round);
	}

	// The last round is given below.
	// The MixColumns function is not here in the last round.
	SubBytes();
	ShiftRows();
	AddRoundKey(Nr);

	// The encryption process is over.
	// Copy the state array to output array.
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			out[i * 4 + j] = state[j][i];
		}
	}
}

// InvCipher is the main function that decrypts the CipherText.
void InvCipher(unsigned char *in, unsigned char *out)
{
	int i, j, round = 0;

	//Copy the input CipherText to state array.
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			state[j][i] = in[i * 4 + j];
		}
	}

	// Add the First round key to the state before starting the rounds.
	AddRoundKey(Nr);

	// There will be Nr rounds.
	// The first Nr-1 rounds are identical.
	// These Nr-1 rounds are executed in the loop below.
	for (round = Nr - 1; round > 0; round--) {
		InvShiftRows();
		InvSubBytes();
		AddRoundKey(round);
		InvMixColumns();
	}

	// The last round is given below.
	// The MixColumns function is not here in the last round.
	InvShiftRows();
	InvSubBytes();
	AddRoundKey(0);

	// The decryption process is over.
	// Copy the state array to output array.
	for (i = 0; i < 4; i++) {
		for (j = 0; j < 4; j++) {
			out[i * 4 + j] = state[j][i];
		}
	}
}

#if FLASH_CRYPTO_DISABLE_CAN_USE

static void soft_aes_enc(unsigned char *key, int key_size,
                         unsigned char *in_buf, unsigned char *out_buf,
                         unsigned long text_size, unsigned char *iv_buf)
{
	unsigned long i, k;
	unsigned char in[16], out[16];

	Nk = key_size / 32;
	Nr = Nk + 6;

	KeyExpansion(key);

	for (k = 0; k < 16; k++)
		in[k] = iv_buf[k];
	for (i = 0; i < text_size / 16; i++) {
		Cipher(in, out);
		for (k = 0; k < 16; k++) {
			out_buf[i * 16 + k] = in_buf[i * 16 + k] ^ out[k];
			in[k] = out[k];
		}
	}
}

static void soft_flash_enc(uint8_t *nonce, uint8_t *key, uint32_t faddr,
                           uint32_t len, uint8_t *in, uint8_t *out)
{
	uint8_t  iv[16];
	uint32_t i, ahb_addr;
	uint32_t block_num = len >> 4;

	memset(iv, 0, sizeof(iv));

	for (i = 0; i < block_num; i++) {
		ahb_addr = faddr + (i << 4);
 		iv[0]  = nonce[1];
 		iv[1]  = nonce[0];
 		iv[2]  = nonce[5];
 		iv[3]  = nonce[4];
 		iv[4]  = nonce[3];
 		iv[5]  = nonce[2];
 		iv[6]  = ahb_addr >> 24;
 		iv[7]  = ahb_addr >> 16;
 		iv[8]  = ahb_addr >> 8;
 		iv[9]  = (nonce[4] >> 4) | (ahb_addr & 0xF0);
		iv[10] = ((nonce[4] << 4) & 0xF0) | (nonce[3] >> 4);
		iv[11] = ((nonce[3] << 4) & 0xF0) | (nonce[2] >> 4);
		iv[12] = ((nonce[2] << 4) & 0xF0) | (ahb_addr >> 28);
		iv[13] = ahb_addr >> 20;
		iv[14] = ahb_addr >> 12;
		iv[15] = ahb_addr >> 4;
		reversed_order(iv, sizeof(iv));
		soft_aes_enc(key, 128, (uint8_t *)((uint32_t)in + (i << 4)),
		             (uint8_t *)((uint32_t)out + (i << 4)), 16, iv);
	}
}
#endif

/*
 * cmd_flash_crypto_bench_exec1
 * drv flashcrypto flash_bench m=0 l=1024
 * drv flashcrypto flash_bench <m=0/1> <l=size>
 * m: 0:flash write/read, 1:flash crypto write/read
 */
static enum cmd_status cmd_flash_crypto_bench_exec(char *cmd)
{
	int32_t   err;
	uint32_t  throuth_mb, throuth_kb;
	OS_Time_t tick_usew, tick_user, tick_startw, tick_startr;
	int32_t   cnt;
	uint8_t   *src = NULL;

	uint32_t  addr = 0x180000;
	uint32_t  mode, size, bench_size;
#if FLASH_CRYPTO_DISABLE_CAN_USE
	uint8_t   key[16] = {0x15, 0x22, 0x67, 0x55, 0x1a, 0x3b, 0x5c, 0x34, 0x79,
	                     0x7f, 0x11, 0x35, 0xbd, 0xf4, 0x88, 0x3b};
#endif

	cnt = cmd_sscanf(cmd, "m=%u l=%u", &mode, &size);
	if (cnt != 2) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_FAIL;
	}

#if !FLASH_CRYPTO_DISABLE_CAN_USE
	if (mode == 1) {
		CMD_ERR("flash crypto disable function is disable, can't test crypto"
		        " read/write!\n");
		return CMD_STATUS_FAIL;
	}
#endif

	src = cmd_malloc(size);
	if (src == NULL) {
		CMD_ERR("no memory\n");
		return CMD_STATUS_FAIL;
	}

	if (mode == 1)
		HAL_FlashCrypto_Init(flash_enc_nonce);

	if (HAL_Flash_Open(MFLASH, 5000) != HAL_OK) {
		CMD_ERR("flash driver open failed\n");
		return CMD_STATUS_FAIL;
	}

	/* write normal function test */
	for (int src_temp = 0; src_temp < size / 4; src_temp++)
		src[src_temp] = src_temp;
#if 1
		if (HAL_Flash_Overwrite(MFLASH, addr, src, size) != HAL_OK) {
			CMD_ERR("flash write failed !\n");
		}

#else
		if (flash_erase(MFLASH, addr, 0x8000)) {
			CMD_ERR("Flash Erase failure !\n");
		}
		if (HAL_Flash_Write(MFLASH, addr, src, size)) {
			CMD_ERR("flash write failed\n");
		}
#endif
	if (HAL_Flash_Check(MFLASH, addr, src, size) != 0) {
		CMD_ERR("flash write not success !\n");
	}
	cmd_free(src);
	src = NULL;

	for (int i = 0; i < 1000; i++) {
		int j;
		bench_size = size * (1 << i);
		uint8_t *buf = cmd_malloc(bench_size);
		if (!buf) {
			CMD_ERR("test end for malloc buff failed!\n");
			CMD_DBG("%s test end\n", __func__);
			goto out;
		}

		for (j = 0; j < bench_size; j++) {
			buf[j] = (j & 0xff);
		}

		tick_startw = OS_GetTicks();
		if (mode == 0) {
#if 1
			err = HAL_Flash_Overwrite(MFLASH, addr, buf, bench_size);
#else
			if (flash_erase(MFLASH, addr, 0x8000)) {
				CMD_ERR("Flash Erase failure !\n");
				err = -1;
			}
			if (HAL_Flash_Write(MFLASH, addr, buf, bench_size)) {
				CMD_ERR("flash write failed\n");
				err = -1;
			}
#endif

		} else if (mode == 1) {
#if FLASH_CRYPTO_DISABLE_CAN_USE
#if 0
			err = HAL_Flash_Overwrite_Crypto(MFLASH, addr, buf, bench_size, key);
#else
			if (flash_erase(MFLASH, addr, 0x8000)) {
				CMD_ERR("Flash Erase failure !\n");
				err = -1;
			}
			if (flash_write_crypto(MFLASH, addr, src, size, key) != size) {
				CMD_ERR("flash write failed\n");
				err = -1;
			}
#endif
#endif
		}
		tick_usew = OS_GetTicks() - tick_startw;
		if (!tick_usew)
			tick_usew = 1;
		if (err) {
			CMD_ERR("flash write err!\n");
			goto next;
		} else {
			throuth_kb = bench_size * 1000 / 1024
			             / (uint32_t)OS_TicksToMSecs(tick_usew);
			throuth_mb = throuth_kb / 1000;
			CMD_DBG("%s flash write ok, ", __func__);
			CMD_LOG(CMD_DBG_ON, "%3d", bench_size/1024);
			CMD_LOG(CMD_DBG_ON, " KB use:%3d ms, throughput:%d.%d MB/S\n",
			        (uint32_t)OS_TicksToMSecs(tick_usew),
			        throuth_mb, throuth_kb - throuth_mb);
		}

		for (j = 0; j < bench_size; j++)
			buf[j] = 0;

		tick_startr = OS_GetTicks();
		if (mode == 0) {
			err = HAL_Flash_Read(MFLASH, addr, buf, bench_size);
		} else if (mode == 1) {
#if FLASH_CRYPTO_DISABLE_CAN_USE
			if (flash_read_crypto(MFLASH, addr, buf, bench_size, key) != bench_size) {
				err = 1;
			}
#endif
		}
		tick_user = OS_GetTicks() - tick_startr;
		if (!tick_user)
			tick_user = 1;
		if (err) {
			CMD_ERR("flash read err!\n");
			goto next;
		}

		err = 0;
		for (j = 0; j < bench_size; j++) {
			if (buf[j] != (j & 0xff)) {
				err = -1;
				break;
			}
		}

		if (err) {
			CMD_ERR("bench_size:%d write data err:0x%x should:0x%x, idx:%d!\n",
			        bench_size, buf[j], (j & 0xff), j);
			print_hex_dump_words((const void *)&buf[j], 256);
			goto next;
		}

		throuth_kb = bench_size * 1000 / 1024 / (uint32_t)OS_TicksToMSecs(tick_user);
		throuth_mb = throuth_kb / 1000;
		CMD_DBG("%s flash read  ok, ", __func__);
		CMD_LOG(CMD_DBG_ON, "%3d", bench_size/1024);
		CMD_LOG(CMD_DBG_ON, " KB use:%3d ms, throughput:%d.%d MB/S\n",
		        (uint32_t)OS_TicksToMSecs(tick_user),
		        throuth_mb, throuth_kb - throuth_mb);
		goto next;
next:
		cmd_free(buf);
		buf = NULL;
		OS_MSleep(10);
		if (err)
			break;
	}

out:
	if (HAL_Flash_Close(MFLASH) != HAL_OK) {
		CMD_ERR("flash driver close failed\n");
	}
	return CMD_STATUS_OK;
}

#if FLASH_CRYPTO_DISABLE_CAN_USE
static enum cmd_status cmd_flash_crypto_exec(char *cmd)
{
	int32_t cnt;
	uint32_t ret = CMD_STATUS_FAIL, i;
	uint8_t  *src, *soft_result, *ahb_result;
	uint32_t addr = 0x180000;
	uint32_t size = 0x400;
	uint8_t  key[16] = {0x15, 0x22, 0x67, 0x55, 0x1a, 0x3b, 0x5c, 0x34, 0x79,
	                    0x7f, 0x11, 0x35, 0xbd, 0xf4, 0x88, 0x3b};
	uint8_t  data_fill[4] = {0xa1, 0xb2, 0xc3, 0xd4};

	cnt = cmd_sscanf(cmd, "l=0x%x", &size);
	if (cnt != 1) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	src = cmd_malloc(size);
	if (src == NULL) {
		CMD_ERR("no memory\n");
		cmd_free(src);
		return ret;
	}
	soft_result = cmd_malloc(size);
	if (soft_result == NULL) {
		CMD_ERR("no memory\n");
		cmd_free(soft_result);
		return ret;
	}
	ahb_result = cmd_malloc(size);
	if (ahb_result == NULL) {
		CMD_ERR("no memory\n");
		cmd_free(ahb_result);
		return ret;
	}

	HAL_FlashCrypto_Init(flash_enc_nonce);

	if (HAL_Flash_Open(MFLASH, 5000) != HAL_OK) {
		CMD_ERR("flash driver open failed\n");
		return CMD_STATUS_FAIL;
	}

	/* write normal function test */
	if (HAL_Flash_Overwrite(MFLASH, addr, src, size) != HAL_OK) {
		CMD_ERR("flash write failed !\n");
	}
	if (HAL_Flash_Check(MFLASH, addr, src, size) != 0) {
		CMD_ERR("flash write not success !\n");
	}

	for (i = 0; i < 1; i++) {
		cmd_memset(src, data_fill[i], size);
#if 1
		if (flash_erase(MFLASH, addr, 0x1000)) {
			CMD_ERR("Flash Erase failure !\n");
			goto error_exit;
		}
		if (flash_write_crypto(MFLASH, addr, src, size, key) != size) {
			CMD_ERR("flash read failed\n");
			goto error_exit;
		}
#else
		if (HAL_Flash_Overwrite_Crypto(MFLASH, addr, src, size, key) != HAL_OK) {
			CMD_ERR("flash write failed !\n");
		}
#endif
		if (flash_read_crypto(MFLASH, addr, ahb_result, size, key) != size) {
			CMD_ERR("flash read failed\n");
			goto error_exit;
		}
		if (cmd_memcmp(src, ahb_result, size)) {
			CMD_ERR("flash decrypt test error !\n");
			CMD_DBG("the source data :\n");
			print_hex_dump_bytes(src, size);
			CMD_DBG("HW decrypt result :\n");
			print_hex_dump_bytes(ahb_result, size);
			goto error_exit;
		}

		soft_flash_enc(flash_enc_nonce, key, addr, size, src, soft_result);

		cmd_memset(ahb_result, 0x0, size);
		if (flash_read(MFLASH, addr, ahb_result, size) != size) {
			CMD_ERR("flash read failed\n");
			goto error_exit;
		}

		if (cmd_memcmp(soft_result, ahb_result, size)) {
			CMD_ERR("flash encrypt test error !\n");
			CMD_DBG("SW encrypt result :\n");
			print_hex_dump_bytes(soft_result, size);
			CMD_DBG("HW encrypt result :\n");
			print_hex_dump_bytes(ahb_result, size);
			goto error_exit;
		}
	}

	CMD_LOG(1, "flash crypto test successful !\n");
	ret = CMD_STATUS_OK;

error_exit:
	if (HAL_Flash_Close(MFLASH) != HAL_OK) {
		CMD_ERR("flash driver close failed\n");
		return CMD_STATUS_FAIL;
	}
	cmd_free(src);
	cmd_free(soft_result);
	cmd_free(ahb_result);
	return ret;
}
#endif

#if (defined(CONFIG_PSRAM))
static enum cmd_status cmd_psram_crypto_exec(char *cmd)
{
	int32_t cnt;
	uint32_t ret = CMD_STATUS_FAIL, i, mode_cpu_ndma;
	uint8_t  *src = NULL, *soft_result = NULL;
	uint8_t  *ahb_result = NULL, *psram_buf = NULL;
	uint32_t size = 0x400;
	uint8_t  key[16] = {0x15, 0x22, 0x67, 0x55, 0x1a, 0x3b, 0x5c, 0x34, 0x79,
	                    0x7f, 0x11, 0x35, 0xbd, 0xf4, 0x88, 0x3b};
	uint8_t  data_fill[4] = {0xa1, 0xb2, 0xc3, 0xd4};
	DMA_ChannelInitParam dmaParam;
	DMA_Channel dma_ch;
	OS_Semaphore_t *dma_sem;

	cnt = cmd_sscanf(cmd, "l=0x%x m=%u", &size, &mode_cpu_ndma);
	if (cnt != 2) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	src = cmd_malloc(size);
	if (src == NULL) {
		CMD_ERR("no memory\n");
		goto error_exit;
	}
	soft_result = cmd_malloc(size);
	if (soft_result == NULL) {
		CMD_ERR("no memory\n");
		goto error_exit;
	}
	ahb_result = cmd_malloc(size);
	if (ahb_result == NULL) {
		CMD_ERR("no memory\n");
		goto error_exit;
	}
	psram_buf = psram_malloc(size);
	if (psram_buf == NULL) {
		CMD_ERR("no psram heap memory\n");
		goto error_exit;
	}

	HAL_FlashCrypto_Init(flash_enc_nonce);

	/* write normal function test */
	cmd_memset(src, 0x5a, size);
	cmd_memcpy((void *)psram_buf, src, size);
	HAL_Dcache_Clean((uint32_t)psram_buf, size);
	cmd_memcpy(ahb_result, (void *)psram_buf, size);
	if (cmd_memcmp(src, ahb_result, size)) {
		CMD_ERR("psram normal wr-test fail !\n");
	}

	for (i = 0; i < 1; i++) {
		cmd_memset(src, data_fill[i], size);
		cmd_memset(ahb_result, 0x0, size);
#if FLASH_CRYPTO_DISABLE_CAN_USE
		int32_t crypto_ch = FlashCryptoRequest((uint32_t)psram_buf,
		                                       (uint32_t)psram_buf + size - 1,
		                                       key);
#else
		FlashCryptoRequest((uint32_t)psram_buf, (uint32_t)psram_buf + size - 1,
		                   key);
#endif
		if (!mode_cpu_ndma) {
			//CPU
			CMD_DBG("psram crypto test through CPU\n");
			cmd_memcpy((void *)psram_buf, src, size);
			HAL_Dcache_Clean((uint32_t)psram_buf, size);
			cmd_memcpy(ahb_result, (void *)psram_buf, size);
		} else {
			//DMA
			CMD_DBG("psram crypto test through DMA\n");

			//DMA channel request
			dma_ch = HAL_DMA_Request();
			if (dma_ch == DMA_CHANNEL_INVALID) {
				CMD_ERR("request dma channel fail !\n");
				goto error_exit;
			}

			dma_sem = &g_fc_info.sem;
			OS_SemaphoreCreate(dma_sem, 0, 1);

			dmaParam.irqType = DMA_IRQ_TYPE_END;
			dmaParam.endCallback = (DMA_IRQCallback)OS_SemaphoreRelease;
			dmaParam.endArg = dma_sem;

			//src->psram_buf
			dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
			                                          DMA_WAIT_CYCLE_2,
			                                          DMA_BYTE_CNT_MODE_REMAIN,
			                                          DMA_DATA_WIDTH_8BIT,
			                                          DMA_BURST_LEN_4,
			                                          DMA_ADDR_MODE_INC,
			                                          DMA_PERIPH_FLASHC,
			                                          DMA_DATA_WIDTH_8BIT,
			                                          DMA_BURST_LEN_1,
			                                          DMA_ADDR_MODE_INC,
			                                          DMA_PERIPH_SRAM);
			HAL_DMA_Init(dma_ch, &dmaParam);
			HAL_DMA_Start(dma_ch, (uint32_t)src, (uint32_t)psram_buf, size);

			if (OS_SemaphoreWait(dma_sem, 2000) != OS_OK)
				CMD_ERR("sem wait failed: %d\n", ret);

			HAL_DMA_Stop(dma_ch);
			HAL_DMA_DeInit(dma_ch);

			//psram_buf->ahb_result
			dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
			                                          DMA_WAIT_CYCLE_2,
			                                          DMA_BYTE_CNT_MODE_REMAIN,
			                                          DMA_DATA_WIDTH_8BIT,
			                                          DMA_BURST_LEN_1,
			                                          DMA_ADDR_MODE_INC,
			                                          DMA_PERIPH_SRAM,
			                                          DMA_DATA_WIDTH_8BIT,
			                                          DMA_BURST_LEN_4,
			                                          DMA_ADDR_MODE_INC,
			                                          DMA_PERIPH_FLASHC);
			HAL_DMA_Init(dma_ch, &dmaParam);
			HAL_DMA_Start(dma_ch, (uint32_t)psram_buf, (uint32_t)ahb_result, size);

			if (OS_SemaphoreWait(dma_sem, 2000) != OS_OK)
				CMD_ERR("sem wait failed: %d\n", ret);

			HAL_DMA_Stop(dma_ch);
			HAL_DMA_DeInit(dma_ch);

			//DMA channel release
			HAL_DMA_Release(dma_ch);

			OS_SemaphoreDelete(dma_sem);
		}

		if (cmd_memcmp(src, ahb_result, size)) {
			CMD_ERR("psram decrypt test error !\n");
			CMD_DBG("the source data :\n");
			print_hex_dump_bytes(src, size);
			CMD_DBG("HW decrypt result :\n");
			print_hex_dump_bytes(ahb_result, size);
			goto error_exit;
		}

#if FLASH_CRYPTO_DISABLE_CAN_USE
		FlashCryptoRelease(crypto_ch);
		soft_flash_enc(flash_enc_nonce, key,
		               (uint32_t)psram_buf - CONFIG_PSRAM_START, size, src,
		               soft_result);

		cmd_memset(ahb_result, 0x0, size);
		if (!mode_cpu_ndma) {
			//CPU
			HAL_Dcache_Flush((uint32_t)psram_buf, size);
			cmd_memcpy(ahb_result, (void *)psram_buf, size);
		} else {
			//DMA
			//DMA channel request
			dma_ch = HAL_DMA_Request();
			if (dma_ch == DMA_CHANNEL_INVALID) {
				CMD_ERR("request dma channel fail !\n");
				goto error_exit;
			}

			dma_sem = &g_fc_info.sem;
			OS_SemaphoreCreate(dma_sem, 0, 1);

			dmaParam.irqType = DMA_IRQ_TYPE_END;
			dmaParam.endCallback = (DMA_IRQCallback)OS_SemaphoreRelease;
			dmaParam.endArg = dma_sem;

			//psram_buf->ahb_result
			dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
			                                          DMA_WAIT_CYCLE_2,
			                                          DMA_BYTE_CNT_MODE_REMAIN,
			                                          DMA_DATA_WIDTH_8BIT,
			                                          DMA_BURST_LEN_1,
			                                          DMA_ADDR_MODE_INC,
			                                          DMA_PERIPH_SRAM,
			                                          DMA_DATA_WIDTH_8BIT,
			                                          DMA_BURST_LEN_4,
			                                          DMA_ADDR_MODE_INC,
			                                          DMA_PERIPH_FLASHC);
			HAL_DMA_Init(dma_ch, &dmaParam);
			HAL_DMA_Start(dma_ch, (uint32_t)psram_buf, (uint32_t)ahb_result, size);

			if (OS_SemaphoreWait(dma_sem, 2000) != OS_OK)
				CMD_ERR("sem wait failed: %d\n", ret);

			HAL_DMA_Stop(dma_ch);
			HAL_DMA_DeInit(dma_ch);

			//DMA channel release
			HAL_DMA_Release(dma_ch);

			OS_SemaphoreDelete(dma_sem);
		}

		if (cmd_memcmp(soft_result, ahb_result, size)) {
			CMD_ERR("psram encrypt test error !\n");
			CMD_DBG("SW encrypt result :\n");
			print_hex_dump_bytes(soft_result, size);
			CMD_DBG("HW encrypt result :\n");
			print_hex_dump_bytes(ahb_result, size);
			goto error_exit;
		}
#endif
	}

	CMD_LOG(1, "psarm crypto test successful !\n");
	ret = CMD_STATUS_OK;

error_exit:
	cmd_free(src);
	cmd_free(soft_result);
	cmd_free(ahb_result);
	psram_free(psram_buf);
	return ret;
}

static enum cmd_status cmd_flash_crypto_setup_exec(char *cmd)
{
	int32_t  cnt;
	uint32_t mode, start_addr, size;
	uint8_t  key[16] = {0x15, 0x22, 0x67, 0x55, 0x1a, 0x3b, 0x5c, 0x34, 0x79,
	                    0x7f, 0x11, 0x35, 0xbd, 0xf4, 0x88, 0x3b};

	cnt = cmd_sscanf(cmd, "m=%u s=0x%x l=0x%x", &mode, &start_addr, &size);
	if (cnt != 3) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	if (start_addr < 0x100000) {
		CMD_ERR("flash test address should not less than 1MB, 0x%x!\n",
		        start_addr);
	}

	if (mode) {
		start_addr += CONFIG_PSRAM_START;
	}
	g_fc_info.crypto_type = mode;
	g_fc_info.crypto_addr = start_addr;
	g_fc_info.crypto_length = size;

	HAL_FlashCrypto_Init(flash_enc_nonce);
	g_fc_info.crypto_channel = FlashCryptoRequest(start_addr,
	                                              start_addr + size - 1, key);

	return CMD_STATUS_OK;
}
#endif

static enum cmd_status cmd_flash_crypto_test_exec(char *cmd)
{
	int32_t  cnt;
	uint32_t ret = CMD_STATUS_FAIL, i;
	uint8_t  *src, *ahb_result;
	uint32_t start_addr, size;

	cnt = cmd_sscanf(cmd, "s=0x%x l=0x%x", &start_addr, &size);
	if (cnt != 2) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	src = cmd_malloc(size);
	if (src == NULL) {
		CMD_ERR("no memory\n");
		cmd_free(src);
		return ret;
	}
	ahb_result = cmd_malloc(size);
	if (ahb_result == NULL) {
		CMD_ERR("no memory\n");
		cmd_free(ahb_result);
		return ret;
	}

	for (i = 0; i < size; i++) {
		src[i] = (i & 0xff);
	}
	cmd_memset(ahb_result, 0x0, size);

	if (g_fc_info.crypto_type) {
		start_addr += CONFIG_PSRAM_START;
		cmd_memcpy((void *)start_addr, src, size);
		cmd_memcpy(ahb_result, (void *)start_addr, size);
	} else {
		if (HAL_Flash_Open(MFLASH, 5000) != HAL_OK) {
			CMD_ERR("flash driver open failed\n");
			goto error_exit;
		}
		if (HAL_Flash_Overwrite(MFLASH, start_addr, src, size) != HAL_OK) {
			CMD_ERR("flash write failed !\n");
		}
		if (flash_read(MFLASH, start_addr, ahb_result, size) != size) {
			CMD_ERR("flash read failed\n");
		}
		if (HAL_Flash_Close(MFLASH) != HAL_OK) {
			CMD_ERR("flash driver close failed\n");
		}
	}

	if (cmd_memcmp(src, ahb_result, size)) {
		CMD_ERR("flash decrypt test error !\n");
		CMD_DBG("the source data :\n");
		print_hex_dump_bytes(src, size);
		CMD_DBG("HW decrypt result :\n");
		print_hex_dump_bytes(ahb_result, size);
		goto error_exit;
	}

	CMD_LOG(1, "flash crypto test successful !\n");
	ret = CMD_STATUS_OK;

error_exit:
	cmd_free(src);
	cmd_free(ahb_result);
	return ret;
}

#if FLASH_CRYPTO_DISABLE_CAN_USE
static enum cmd_status cmd_flash_crypto_finish_exec(char *cmd)
{
	FlashCryptoRelease(g_fc_info.crypto_channel);

	return CMD_STATUS_OK;
}
#endif

static void fc_press_test_task(void *arg)
{
	uint8_t  *src = NULL, *ahb_result = NULL;
	uint32_t i, start_addr, size, test_times = 0;
	cmd_fc_t *priv = &g_fc_info;

	start_addr = priv->crypto_addr;
	size = priv->crypto_length > 4096 ? 4096 : priv->crypto_length;

	if (!priv->crypto_type) {
		if (HAL_Flash_Open(MFLASH, 5000) != HAL_OK) {
			CMD_ERR("flash driver open failed\n");
			goto exit;
		}
	}
	src = cmd_malloc(size);
	if (src == NULL) {
		CMD_ERR("no memory\n");
		goto exit;
	}
	ahb_result = cmd_malloc(size);
	if (ahb_result == NULL) {
		CMD_ERR("no memory\n");
		goto exit;
	}
	for (i = 0; i < size; i++) {
		src[i] = (i & 0xff);
	}

	while (test_times++ < priv->test_times) {
		cmd_memset(ahb_result, 0x0, size);

		if (priv->crypto_type) {
			cmd_memcpy((void *)start_addr, src, size);
			HAL_Dcache_Clean(start_addr, size);
			cmd_memcpy(ahb_result, (void *)start_addr, size);
		} else {
			if (HAL_Flash_Overwrite(MFLASH, start_addr, src, size) != HAL_OK) {
				CMD_ERR("flash write failed !\n");
			}
			if (flash_read(MFLASH, start_addr, ahb_result, size) != size) {
				CMD_ERR("flash read failed\n");
			}
		}

		if (cmd_memcmp(src, ahb_result, size)) {
			CMD_ERR("flash decrypt test error !\n");
			CMD_DBG("the source data :\n");
			print_hex_dump_bytes(src, size);
			CMD_DBG("HW decrypt result :\n");
			print_hex_dump_bytes(ahb_result, size);
			CMD_LOG(1, "test failed on the %d times.\n", test_times);
			goto exit;
		} else {
			if (!(test_times % 50)) {
				CMD_LOG(1, "flash crypto pressure test %04u times...\n", test_times);
			}
		}

		OS_MSleep(100);
	}

	CMD_LOG(1, "flash crypto pressure test pass !\n");

exit:
	if (!priv->crypto_type) {
		if (HAL_Flash_Close(MFLASH) != HAL_OK) {
			CMD_ERR("flash driver close failed\n");
		}
	}
	cmd_free(src);
	cmd_free(ahb_result);
	OS_SemaphoreRelease(&priv->sem);
}

static enum cmd_status cmd_flash_crypto_pressure_exec(char *cmd)
{
	int32_t  cnt;
	uint32_t test_times;
	cmd_fc_t *priv = &g_fc_info;

	cnt = cmd_sscanf(cmd, "t=%u", &test_times);
	if (cnt != 1) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}
	priv->test_times = test_times;

	OS_SemaphoreCreate(&priv->sem, 0, 1);

	if (OS_ThreadCreate(&priv->thread,
	                    "fcrypto-press-test",
	                    fc_press_test_task,
	                    NULL,
	                    OS_PRIORITY_NORMAL,
	                    1024) != OS_OK) {
		CMD_LOG(1, "create Task[fcrypto-press-test] failed\n");
		return CMD_STATUS_FAIL;
	}

	OS_SemaphoreWait(&priv->sem, OS_WAIT_FOREVER);
	OS_SemaphoreDelete(&priv->sem);
	OS_ThreadDelete(&priv->thread);

	return CMD_STATUS_OK;
}

#if FLASH_CRYPTO_DISABLE_CAN_USE
static void fc_bus_switch_test_task1(void *arg)
{
	uint8_t  *src = NULL, *ahb_result = NULL;
	uint32_t i, start_addr, size;
	cmd_fc_t *priv = &g_fc_info;
	static uint32_t run_time = 0;
	uint8_t key[16] = {0x15, 0x22, 0x67, 0x55, 0x1a, 0x3b, 0x5c, 0x34, 0x79,
	                   0x7f, 0x11, 0x35, 0xbd, 0xf4, 0x88, 0x3b};
	OS_Time_t tick_current = 0, tick_record = 0;
	OS_Time_t tick_end = OS_GetTicks() + OS_SecsToTicks(priv->test_times);

	start_addr = priv->crypto_addr;
	size = priv->crypto_length > 4096 ? 4096 : priv->crypto_length;

	src = cmd_malloc(size);
	if (src == NULL) {
		CMD_ERR("no memory\n");
		goto exit;
	}
	ahb_result = cmd_malloc(size);
	if (ahb_result == NULL) {
		CMD_ERR("no memory\n");
		goto exit;
	}
	for (i = 0; i < size; i++) {
		src[i] = (i & 0xff);
	}

	HAL_FlashCrypto_Init(flash_enc_nonce);
#if 1
	if (flash_erase(MFLASH, start_addr, 0x8000)) {
		CMD_ERR("Flash Erase failure !\n");
		goto exit;
	}
	if (flash_write_crypto(MFLASH, start_addr, src, size, key) != size) {
		CMD_ERR("flash read failed\n");
		goto exit;
	}
#else
	if (HAL_Flash_Overwrite_Crypto(MFLASH, start_addr, src, size, key) != HAL_OK) {
		CMD_ERR("flash write failed !\n");
		goto exit;
	}
#endif

	while (tick_current < tick_end) {
		cmd_memset(ahb_result, 0x0, size);
		if (flash_read_crypto(MFLASH, start_addr, ahb_result, size, key) != size) {
			CMD_ERR("flash read failed\n");
		}

		if (cmd_memcmp(src, ahb_result, size)) {
			CMD_ERR("flash decrypt test error !\n");
			CMD_DBG("the source data :\n");
			print_hex_dump_bytes(src, size);
			CMD_DBG("HW decrypt result :\n");
			print_hex_dump_bytes(ahb_result, size);
			break;
		}

		OS_MSleep(4);
		tick_current = OS_GetTicks();
		if (OS_TicksToSecs(tick_current - tick_record) >= 10) {
			run_time += 10;
			CMD_LOG(1, "test run normally %04ds...\n", run_time);
			tick_record = tick_current;
		}
	}
	if (tick_current >= tick_end) {
		CMD_LOG(1, "thread[%s] test successful !\n", __func__);
	}

exit:
	run_time = 0;
	cmd_free(src);
	cmd_free(ahb_result);
	OS_ThreadDelete(NULL);
}

static void fc_bus_switch_test_task2(void *arg)
{
	int ret;
	uint32_t addr, size = 0x200, i;
	uint8_t *wbuf;
	cmd_fc_t *priv = &g_fc_info;
	OS_Time_t tick_current = 0;
	OS_Time_t tick_end = OS_GetTicks() + OS_SecsToTicks(priv->test_times);

	wbuf = cmd_malloc(size);
	if (wbuf == NULL) {
		CMD_ERR("no heap memory\n");
		goto exit;
	}
	for (i = 0; i < size; i++) {
		wbuf[i] = (i & 0xff);
	}
	addr = priv->crypto_addr + priv->crypto_length + 0x1000;

	while (tick_current < tick_end) {
		if (HAL_Flash_Open(MFLASH, 3000) != HAL_OK) {
			CMD_ERR("flash driver open failed\n");
			break;
		}
		if ((ret = HAL_Flash_Overwrite(MFLASH, addr, wbuf, size)) != HAL_OK) {
			CMD_ERR("flash write failed: %d\n", ret);
			break;
		}
		if ((ret = HAL_Flash_Check(MFLASH, addr, wbuf, size)) != 0) {
			CMD_ERR("flash write not success %d\n", ret);
			break;
		}
		if (HAL_Flash_Close(MFLASH) != HAL_OK) {
			CMD_ERR("flash driver close failed\n");
			break;
		}

		OS_MSleep(3);
		tick_current = OS_GetTicks();
	}
	if (tick_current >= tick_end) {
		CMD_LOG(1, "thread[%s] test successful !\n", __func__);
	}

exit:
	cmd_free(wbuf);
	OS_ThreadDelete(NULL);
}

static enum cmd_status cmd_flash_crypto_bus_switch_exec(char *cmd)
{
	int32_t  cnt;
	uint32_t start_addr, size, test_times;
	cmd_fc_t *priv = &g_fc_info;

	cnt = cmd_sscanf(cmd, "s=0x%x l=0x%x t=%u", &start_addr, &size, &test_times);
	if (cnt != 3) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}
	g_fc_info.crypto_addr = start_addr;
	g_fc_info.crypto_length = size;
	g_fc_info.test_times = test_times;

	if (OS_ThreadCreate(&priv->thread,
	                    "fc_bus_switch_test_task1",
	                    fc_bus_switch_test_task1,
	                    NULL,
	                    OS_PRIORITY_NORMAL,
	                    1024) != OS_OK) {
		CMD_LOG(1, "create Task[fc_bus_switch_test_task1] failed\n");
		return CMD_STATUS_FAIL;
	}
	OS_ThreadSetInvalid(&priv->thread);
	if (OS_ThreadCreate(&priv->thread,
	                    "fc_bus_switch_test_task2",
	                    fc_bus_switch_test_task2,
	                    NULL,
	                    OS_PRIORITY_NORMAL,
	                    1024) != OS_OK) {
		CMD_LOG(1, "create Task[fc_bus_switch_test_task2] failed\n");
		return CMD_STATUS_FAIL;
	}
	OS_ThreadSetInvalid(&priv->thread);

	return CMD_STATUS_OK;
}
#endif

#if (defined(CONFIG_PSRAM))
/*
 * @brief To make the statistics of cache miss/hit counter more accurate,
 *        please make sure that no code&data in psram.
 */
static enum cmd_status cmd_flash_crypto_cache_exec(char *cmd)
{
#if FLASH_CRYPTO_DISABLE_CAN_USE
	int32_t crypto_ch = 0;
#endif
	int32_t cnt;
	enum cmd_status ret = CMD_STATUS_FAIL;
	uint8_t  *src = NULL, *ahb_result = NULL, *psram_buf = NULL;
	uint32_t size = 0x400, i, cache_write_cnt = 0, cache_read_cnt = 0;
	uint8_t  key[16] = {0x15, 0x22, 0x67, 0x55, 0x1a, 0x3b, 0x5c, 0x34, 0x79,
	                    0x7f, 0x11, 0x35, 0xbd, 0xf4, 0x88, 0x3b};
	DMA_ChannelInitParam dmaParam;
	DMA_Channel dma_ch;
	OS_Semaphore_t *dma_sem;

	cnt = cmd_sscanf(cmd, "l=0x%x", &size);
	if (cnt != 1) {
		CMD_ERR("invalid param number %d\n", cnt);
		return CMD_STATUS_INVALID_ARG;
	}

	src = cmd_malloc(size);
	if (src == NULL) {
		CMD_ERR("no memory\n");
		goto error_exit;
	}
	ahb_result = cmd_malloc(size);
	if (ahb_result == NULL) {
		CMD_ERR("no memory\n");
		goto error_exit;
	}
	psram_buf = psram_malloc(size);
	if (psram_buf == NULL) {
		CMD_ERR("no psram heap memory\n");
		goto error_exit;
	}
	for (i = 0; i < size; i++) {
		src[i] = (i & 0xff);
	}

	HAL_FlashCrypto_Init(flash_enc_nonce);
#if FLASH_CRYPTO_DISABLE_CAN_USE
	crypto_ch = FlashCryptoRequest((uint32_t)psram_buf,
	                               (uint32_t)psram_buf + size - 1, key);
#else
	FlashCryptoRequest((uint32_t)psram_buf, (uint32_t)psram_buf + size - 1,
	                   key);
#endif

	OS_ThreadSuspendScheduler();
	HAL_Dcache_CleanAll();
	HAL_Dcache_FlushAll();
	HAL_Dcache_DumpMissHit();
	cache_write_cnt = HAL_REG_32BIT(0x40009090);
	cache_read_cnt  = HAL_REG_32BIT(0x40009094);
	for (i = 0; i < size; i++) {
		psram_buf[i] = src[i];
	}
	CMD_LOG(1, "after write %d bytes psram :\n", size);
	HAL_Dcache_DumpMissHit();
	cache_write_cnt = HAL_REG_32BIT(0x40009090) - cache_write_cnt;
	cache_read_cnt  = HAL_REG_32BIT(0x40009094) - cache_read_cnt;
	CMD_LOG(1, "cache_write_cnt -> 0x%08x, cache_read_cnt -> 0x%08x\n",
	        cache_write_cnt, cache_read_cnt);
	OS_ThreadResumeScheduler();

	dma_ch = HAL_DMA_Request();
	if (dma_ch == DMA_CHANNEL_INVALID) {
		CMD_ERR("request dma channel fail !\n");
		goto error_exit;
	}

	dma_sem = &g_fc_info.sem;
	OS_SemaphoreCreate(dma_sem, 0, 1);

	dmaParam.irqType = DMA_IRQ_TYPE_END;
	dmaParam.endCallback = (DMA_IRQCallback)OS_SemaphoreRelease;
	dmaParam.endArg = dma_sem;

	dmaParam.cfg = HAL_DMA_MakeChannelInitCfg(DMA_WORK_MODE_SINGLE,
	                                          DMA_WAIT_CYCLE_2,
	                                          DMA_BYTE_CNT_MODE_REMAIN,
	                                          DMA_DATA_WIDTH_8BIT,
	                                          DMA_BURST_LEN_4,
	                                          DMA_ADDR_MODE_INC,
	                                          DMA_PERIPH_FLASHC,
	                                          DMA_DATA_WIDTH_8BIT,
	                                          DMA_BURST_LEN_1,
	                                          DMA_ADDR_MODE_INC,
	                                          DMA_PERIPH_SRAM);
	HAL_DMA_Init(dma_ch, &dmaParam);
	HAL_DMA_Start(dma_ch, (uint32_t)psram_buf, (uint32_t)ahb_result, size);

	if (OS_SemaphoreWait(dma_sem, 2000) != OS_OK)
		CMD_ERR("sem wait failed: %d\n", ret);

	HAL_DMA_Stop(dma_ch);
	HAL_DMA_DeInit(dma_ch);
	HAL_DMA_Release(dma_ch);

	OS_SemaphoreDelete(dma_sem);

	if (cmd_memcmp(src, ahb_result, size)) {
		CMD_ERR("psram crypto wr-test fail !\n");
		goto error_exit;
	}

	CMD_LOG(1, "psarm crypto wr-test successful !\n");
	ret = CMD_STATUS_OK;

error_exit:
#if FLASH_CRYPTO_DISABLE_CAN_USE
	FlashCryptoRelease(crypto_ch);
#endif
	cmd_free(src);
	cmd_free(ahb_result);
	psram_free(psram_buf);
	return ret;
}
#endif

static const struct cmd_data g_flashc_crypto_cmds[] = {
#if (defined(CONFIG_PSRAM))
	{ "psram",            cmd_psram_crypto_exec },
	{ "fc-setup",         cmd_flash_crypto_setup_exec },
#endif
	{ "fc-test",          cmd_flash_crypto_test_exec },
#if FLASH_CRYPTO_DISABLE_CAN_USE
	{ "fc-finish",        cmd_flash_crypto_finish_exec },
	{ "flash",            cmd_flash_crypto_exec },
	{ "bus-switch-scene", cmd_flash_crypto_bus_switch_exec },
#endif
	{ "fc-pressure",      cmd_flash_crypto_pressure_exec },
	{ "flash_bench",      cmd_flash_crypto_bench_exec },
#if (defined(CONFIG_PSRAM))
	{ "fc-cache-scene",   cmd_flash_crypto_cache_exec }
#endif
};

enum cmd_status cmd_flashc_crypto_exec(char *cmd)
{
	return cmd_exec(cmd, g_flashc_crypto_cmds,
	                cmd_nitems(g_flashc_crypto_cmds));
}

#endif
