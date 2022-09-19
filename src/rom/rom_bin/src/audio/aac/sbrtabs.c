#ifdef CONFIG_ROM
#include "sbr.h"

/* coefficient table 4.A.87, n_format = Q31
 * reordered as:
 *   cTab[0],  cTab[64],  cTab[128], cTab[192], cTab[256],
 *   cTab[2],  cTab[66],  cTab[130], cTab[194], cTab[258],
 *   ...
 *   cTab[64], cTab[128], cTab[192], cTab[256], cTab[320]
 *
 * NOTE: cTab[1, 2, ... , 318, 319] = cTab[639, 638, ... 322, 321]
 *   except cTab[384] = -cTab[256], cTab[512] = -cTab[128]
 */
const int n_cTabA[165] = {
	0x00000000, 0x0055dba1, 0x01b2e41d, 0x09015651, 0x2e3a7532, 0xffed978a, 0x006090c4, 0x01fd3ba0, 0x08a24899, 0x311af3a4,
	0xfff0065d, 0x006b47fa, 0x024bf7a1, 0x082f552e, 0x33ff670e, 0xffef7b8b, 0x0075fded, 0x029e35b4, 0x07a8127d, 0x36e69691,
	0xffee1650, 0x00807994, 0x02f3e48d, 0x070bbf58, 0x39ce0477, 0xffecc31b, 0x008a7dd7, 0x034d01f0, 0x06593912, 0x3cb41219,
	0xffeb50b2, 0x009424c6, 0x03a966bb, 0x0590a67d, 0x3f962fb8, 0xffe9ca76, 0x009d10bf, 0x04083fec, 0x04b0adcb, 0x4272a385,
	0xffe88ba8, 0x00a520bb, 0x04694101, 0x03b8f8dc, 0x4547daea, 0xffe79e16, 0x00abe79e, 0x04cc2fcf, 0x02a99097, 0x4812f848,
	0xffe6d466, 0x00b1978d, 0x05303f87, 0x01816e06, 0x4ad237a2, 0xffe65416, 0x00b5c867, 0x05950122, 0x0040c496, 0x4d83976c,
	0xffe66dd0, 0x00b8394b, 0x05f9c051, 0xfee723c6, 0x5024d70e, 0xffe69423, 0x00b8c6b0, 0x065dd56a, 0xfd7475d8, 0x52b449de,
	0xffe75361, 0x00b73ab0, 0x06c0f0c0, 0xfbe8f5bd, 0x552f8ff7, 0xffe85b4b, 0x00b36acd, 0x0721bf22, 0xfa44a069, 0x579505f5,
	0xffea353a, 0x00acbd2f, 0x077fedb3, 0xf887507c, 0x59e2f69e, 0xffec8409, 0x00a3508f, 0x07da2b7f, 0xf6b1f3c3, 0x5c16d0ae,
	0xffef2395, 0x0096dcc2, 0x08303897, 0xf4c473c6, 0x5e2f6367, 0xfff294c3, 0x00872c63, 0x0880ffdd, 0xf2bf6ea4, 0x602b0c7f,
	0xfff681d6, 0x007400b8, 0x08cb4e23, 0xf0a3959f, 0x6207f220, 0xfffb42b0, 0x005d36df, 0x090ec1fc, 0xee71b2fe, 0x63c45243,
	0x00007134, 0x00426f36, 0x0949eaac, 0xec2a3f5f, 0x655f63f2, 0x0006b1cf, 0x0023b989, 0x097c1ee8, 0xe9cea84a, 0x66d76725,
	0x000d31b5, 0x0000e790, 0x09a3e163, 0xe75f8bb8, 0x682b39a4, 0x001471f8, 0xffda17f2, 0x09c0e59f, 0xe4de0cb0, 0x6959709d,
	0x001c3549, 0xffaea5d6, 0x09d19ca9, 0xe24b8f66, 0x6a619c5e, 0x0024dd50, 0xff7ee3f1, 0x09d5560b, 0xdfa93ab5, 0x6b42a864,
	0x002d8e42, 0xff4aabc8, 0x09caeb0f, 0xdcf898fb, 0x6bfbdd98, 0x003745f9, 0xff120d70, 0x09b18a1d, 0xda3b176a, 0x6c8c4c7a,
	0x004103f4, 0xfed4bec3, 0x09881dc5, 0xd7722f04, 0x6cf4073e, 0x004b6c46, 0xfe933dc0, 0x094d7ec2, 0xd49fd55f, 0x6d32730f,
	0x0055dba1, 0x01b2e41d, 0x09015651, 0x2e3a7532, 0x6d474e1d,
};

/* coefficient table 4.A.87, n_format = Q31
 * reordered as cTab[0], cTab[64], cTab[128], ... cTab[576], cTab[1], cTab[65], cTab[129], ... cTab[639]
 * keeping full table (not using symmetry) to allow sequential access in synth filter inner loop
 * n_format = Q31
 */
const int n_cTabS[640] = {
	0x00000000, 0x0055dba1, 0x01b2e41d, 0x09015651, 0x2e3a7532, 0x6d474e1d, 0xd1c58ace, 0x09015651, 0xfe4d1be3, 0x0055dba1,
	0xffede50e, 0x005b5371, 0x01d78bfc, 0x08d3e41b, 0x2faa221c, 0x6d41d963, 0xd3337b3d, 0x09299ead, 0xfe70b8d1, 0x0050b177,
	0xffed978a, 0x006090c4, 0x01fd3ba0, 0x08a24899, 0x311af3a4, 0x6d32730f, 0xd49fd55f, 0x094d7ec2, 0xfe933dc0, 0x004b6c46,
	0xffefc9b9, 0x0065fde5, 0x02244a24, 0x086b1eeb, 0x328cc6f0, 0x6d18520e, 0xd60a46e5, 0x096d0e21, 0xfeb48d0d, 0x00465348,
	0xfff0065d, 0x006b47fa, 0x024bf7a1, 0x082f552e, 0x33ff670e, 0x6cf4073e, 0xd7722f04, 0x09881dc5, 0xfed4bec3, 0x004103f4,
	0xffeff6ca, 0x0070c8a5, 0x0274ba43, 0x07ee507c, 0x3572ec70, 0x6cc59bab, 0xd8d7f21f, 0x099ec3dc, 0xfef3f6ab, 0x003c1fa4,
	0xffef7b8b, 0x0075fded, 0x029e35b4, 0x07a8127d, 0x36e69691, 0x6c8c4c7a, 0xda3b176a, 0x09b18a1d, 0xff120d70, 0x003745f9,
	0xffeedfa4, 0x007b3875, 0x02c89901, 0x075ca90c, 0x385a49c4, 0x6c492217, 0xdb9b5b12, 0x09c018ce, 0xff2ef725, 0x00329ab6,
	0xffee1650, 0x00807994, 0x02f3e48d, 0x070bbf58, 0x39ce0477, 0x6bfbdd98, 0xdcf898fb, 0x09caeb0f, 0xff4aabc8, 0x002d8e42,
	0xffed651d, 0x0085c217, 0x03201116, 0x06b559c3, 0x3b415115, 0x6ba4629f, 0xde529086, 0x09d1fa23, 0xff6542d1, 0x00293718,
	0xffecc31b, 0x008a7dd7, 0x034d01f0, 0x06593912, 0x3cb41219, 0x6b42a864, 0xdfa93ab5, 0x09d5560b, 0xff7ee3f1, 0x0024dd50,
	0xffebe77b, 0x008f4bfc, 0x037ad438, 0x05f7fb90, 0x3e25b17e, 0x6ad73e8d, 0xe0fc421e, 0x09d52709, 0xff975c01, 0x002064f8,
	0xffeb50b2, 0x009424c6, 0x03a966bb, 0x0590a67d, 0x3f962fb8, 0x6a619c5e, 0xe24b8f66, 0x09d19ca9, 0xffaea5d6, 0x001c3549,
	0xffea9192, 0x0098b855, 0x03d8afe6, 0x05237f9d, 0x41058bc6, 0x69e29784, 0xe396a45d, 0x09cab9f2, 0xffc4e365, 0x0018703f,
	0xffe9ca76, 0x009d10bf, 0x04083fec, 0x04b0adcb, 0x4272a385, 0x6959709d, 0xe4de0cb0, 0x09c0e59f, 0xffda17f2, 0x001471f8,
	0xffe940f4, 0x00a1039c, 0x043889c6, 0x0437fb0a, 0x43de620a, 0x68c7269b, 0xe620c476, 0x09b3d77f, 0xffee183b, 0x0010bc63,
	0xffe88ba8, 0x00a520bb, 0x04694101, 0x03b8f8dc, 0x4547daea, 0x682b39a4, 0xe75f8bb8, 0x09a3e163, 0x0000e790, 0x000d31b5,
	0xffe83a07, 0x00a8739d, 0x049aa82f, 0x03343533, 0x46aea856, 0x6785c24d, 0xe89971b7, 0x099140a7, 0x00131c75, 0x0009aa3f,
	0xffe79e16, 0x00abe79e, 0x04cc2fcf, 0x02a99097, 0x4812f848, 0x66d76725, 0xe9cea84a, 0x097c1ee8, 0x0023b989, 0x0006b1cf,
	0xffe7746e, 0x00af374c, 0x04fe20be, 0x02186a91, 0x4973fef1, 0x661fd6b8, 0xeafee7f1, 0x0963ed46, 0x0033b927, 0x00039609,
	0xffe6d466, 0x00b1978d, 0x05303f87, 0x01816e06, 0x4ad237a2, 0x655f63f2, 0xec2a3f5f, 0x0949eaac, 0x00426f36, 0x00007134,
	0xffe6afee, 0x00b3d15c, 0x05626209, 0x00e42fa2, 0x4c2ca3df, 0x64964063, 0xed50a31d, 0x092d7970, 0x00504f41, 0xfffdfa25,
	0xffe65416, 0x00b5c867, 0x05950122, 0x0040c496, 0x4d83976c, 0x63c45243, 0xee71b2fe, 0x090ec1fc, 0x005d36df, 0xfffb42b0,
	0xffe681c6, 0x00b74c37, 0x05c76fed, 0xff96db90, 0x4ed62be3, 0x62ea6474, 0xef8d4d7b, 0x08edfeaa, 0x006928a0, 0xfff91fca,
	0xffe66dd0, 0x00b8394b, 0x05f9c051, 0xfee723c6, 0x5024d70e, 0x6207f220, 0xf0a3959f, 0x08cb4e23, 0x007400b8, 0xfff681d6,
	0xffe66fac, 0x00b8fe0d, 0x062bf5ec, 0xfe310657, 0x516eefb9, 0x611d58a3, 0xf1b461ab, 0x08a75da4, 0x007e0393, 0xfff48700,
	0xffe69423, 0x00b8c6b0, 0x065dd56a, 0xfd7475d8, 0x52b449de, 0x602b0c7f, 0xf2bf6ea4, 0x0880ffdd, 0x00872c63, 0xfff294c3,
	0xffe6fed4, 0x00b85f70, 0x068f8b44, 0xfcb1d740, 0x53f495aa, 0x5f30ff5f, 0xf3c4e887, 0x08594887, 0x008f87aa, 0xfff0e7ef,
	0xffe75361, 0x00b73ab0, 0x06c0f0c0, 0xfbe8f5bd, 0x552f8ff7, 0x5e2f6367, 0xf4c473c6, 0x08303897, 0x0096dcc2, 0xffef2395,
	0xffe80414, 0x00b58c8c, 0x06f1825d, 0xfb19b7bd, 0x56654bdd, 0x5d26be9b, 0xf5be0fa9, 0x08061671, 0x009da526, 0xffedc418,
	0xffe85b4b, 0x00b36acd, 0x0721bf22, 0xfa44a069, 0x579505f5, 0x5c16d0ae, 0xf6b1f3c3, 0x07da2b7f, 0x00a3508f, 0xffec8409,
	0xffe954d0, 0x00b06b68, 0x075112a2, 0xf96916f5, 0x58befacd, 0x5b001db8, 0xf79fa13a, 0x07ad8c26, 0x00a85e94, 0xffeb3849,
	0xffea353a, 0x00acbd2f, 0x077fedb3, 0xf887507c, 0x59e2f69e, 0x59e2f69e, 0xf887507c, 0x077fedb3, 0x00acbd2f, 0xffea353a,
	0xffeb3849, 0x00a85e94, 0x07ad8c26, 0xf79fa13a, 0x5b001db8, 0x58befacd, 0xf96916f5, 0x075112a2, 0x00b06b68, 0xffe954d0,
	0xffec8409, 0x00a3508f, 0x07da2b7f, 0xf6b1f3c3, 0x5c16d0ae, 0x579505f5, 0xfa44a069, 0x0721bf22, 0x00b36acd, 0xffe85b4b,
	0xffedc418, 0x009da526, 0x08061671, 0xf5be0fa9, 0x5d26be9b, 0x56654bdd, 0xfb19b7bd, 0x06f1825d, 0x00b58c8c, 0xffe80414,
	0xffef2395, 0x0096dcc2, 0x08303897, 0xf4c473c6, 0x5e2f6367, 0x552f8ff7, 0xfbe8f5bd, 0x06c0f0c0, 0x00b73ab0, 0xffe75361,
	0xfff0e7ef, 0x008f87aa, 0x08594887, 0xf3c4e887, 0x5f30ff5f, 0x53f495aa, 0xfcb1d740, 0x068f8b44, 0x00b85f70, 0xffe6fed4,
	0xfff294c3, 0x00872c63, 0x0880ffdd, 0xf2bf6ea4, 0x602b0c7f, 0x52b449de, 0xfd7475d8, 0x065dd56a, 0x00b8c6b0, 0xffe69423,
	0xfff48700, 0x007e0393, 0x08a75da4, 0xf1b461ab, 0x611d58a3, 0x516eefb9, 0xfe310657, 0x062bf5ec, 0x00b8fe0d, 0xffe66fac,
	0xfff681d6, 0x007400b8, 0x08cb4e23, 0xf0a3959f, 0x6207f220, 0x5024d70e, 0xfee723c6, 0x05f9c051, 0x00b8394b, 0xffe66dd0,
	0xfff91fca, 0x006928a0, 0x08edfeaa, 0xef8d4d7b, 0x62ea6474, 0x4ed62be3, 0xff96db90, 0x05c76fed, 0x00b74c37, 0xffe681c6,
	0xfffb42b0, 0x005d36df, 0x090ec1fc, 0xee71b2fe, 0x63c45243, 0x4d83976c, 0x0040c496, 0x05950122, 0x00b5c867, 0xffe65416,
	0xfffdfa25, 0x00504f41, 0x092d7970, 0xed50a31d, 0x64964063, 0x4c2ca3df, 0x00e42fa2, 0x05626209, 0x00b3d15c, 0xffe6afee,
	0x00007134, 0x00426f36, 0x0949eaac, 0xec2a3f5f, 0x655f63f2, 0x4ad237a2, 0x01816e06, 0x05303f87, 0x00b1978d, 0xffe6d466,
	0x00039609, 0x0033b927, 0x0963ed46, 0xeafee7f1, 0x661fd6b8, 0x4973fef1, 0x02186a91, 0x04fe20be, 0x00af374c, 0xffe7746e,
	0x0006b1cf, 0x0023b989, 0x097c1ee8, 0xe9cea84a, 0x66d76725, 0x4812f848, 0x02a99097, 0x04cc2fcf, 0x00abe79e, 0xffe79e16,
	0x0009aa3f, 0x00131c75, 0x099140a7, 0xe89971b7, 0x6785c24d, 0x46aea856, 0x03343533, 0x049aa82f, 0x00a8739d, 0xffe83a07,
	0x000d31b5, 0x0000e790, 0x09a3e163, 0xe75f8bb8, 0x682b39a4, 0x4547daea, 0x03b8f8dc, 0x04694101, 0x00a520bb, 0xffe88ba8,
	0x0010bc63, 0xffee183b, 0x09b3d77f, 0xe620c476, 0x68c7269b, 0x43de620a, 0x0437fb0a, 0x043889c6, 0x00a1039c, 0xffe940f4,
	0x001471f8, 0xffda17f2, 0x09c0e59f, 0xe4de0cb0, 0x6959709d, 0x4272a385, 0x04b0adcb, 0x04083fec, 0x009d10bf, 0xffe9ca76,
	0x0018703f, 0xffc4e365, 0x09cab9f2, 0xe396a45d, 0x69e29784, 0x41058bc6, 0x05237f9d, 0x03d8afe6, 0x0098b855, 0xffea9192,
	0x001c3549, 0xffaea5d6, 0x09d19ca9, 0xe24b8f66, 0x6a619c5e, 0x3f962fb8, 0x0590a67d, 0x03a966bb, 0x009424c6, 0xffeb50b2,
	0x002064f8, 0xff975c01, 0x09d52709, 0xe0fc421e, 0x6ad73e8d, 0x3e25b17e, 0x05f7fb90, 0x037ad438, 0x008f4bfc, 0xffebe77b,
	0x0024dd50, 0xff7ee3f1, 0x09d5560b, 0xdfa93ab5, 0x6b42a864, 0x3cb41219, 0x06593912, 0x034d01f0, 0x008a7dd7, 0xffecc31b,
	0x00293718, 0xff6542d1, 0x09d1fa23, 0xde529086, 0x6ba4629f, 0x3b415115, 0x06b559c3, 0x03201116, 0x0085c217, 0xffed651d,
	0x002d8e42, 0xff4aabc8, 0x09caeb0f, 0xdcf898fb, 0x6bfbdd98, 0x39ce0477, 0x070bbf58, 0x02f3e48d, 0x00807994, 0xffee1650,
	0x00329ab6, 0xff2ef725, 0x09c018ce, 0xdb9b5b12, 0x6c492217, 0x385a49c4, 0x075ca90c, 0x02c89901, 0x007b3875, 0xffeedfa4,
	0x003745f9, 0xff120d70, 0x09b18a1d, 0xda3b176a, 0x6c8c4c7a, 0x36e69691, 0x07a8127d, 0x029e35b4, 0x0075fded, 0xffef7b8b,
	0x003c1fa4, 0xfef3f6ab, 0x099ec3dc, 0xd8d7f21f, 0x6cc59bab, 0x3572ec70, 0x07ee507c, 0x0274ba43, 0x0070c8a5, 0xffeff6ca,
	0x004103f4, 0xfed4bec3, 0x09881dc5, 0xd7722f04, 0x6cf4073e, 0x33ff670e, 0x082f552e, 0x024bf7a1, 0x006b47fa, 0xfff0065d,
	0x00465348, 0xfeb48d0d, 0x096d0e21, 0xd60a46e5, 0x6d18520e, 0x328cc6f0, 0x086b1eeb, 0x02244a24, 0x0065fde5, 0xffefc9b9,
	0x004b6c46, 0xfe933dc0, 0x094d7ec2, 0xd49fd55f, 0x6d32730f, 0x311af3a4, 0x08a24899, 0x01fd3ba0, 0x006090c4, 0xffed978a,
	0x0050b177, 0xfe70b8d1, 0x09299ead, 0xd3337b3d, 0x6d41d963, 0x2faa221c, 0x08d3e41b, 0x01d78bfc, 0x005b5371, 0xffede50f,
};
#endif
