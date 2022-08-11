/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * ov10652 sensor config for BST Cameras Driver
 *
 * This file contains proprietary information that is the sole intellectual
 * property of Black Sesame Technologies, Inc. and its affiliates.
 * No portions of this material may be reproduced in any
 * form without the written permission of:
 * Black Sesame Technologies, Inc. and its affiliates
 * 2255 Martin Ave. Suite D
 * Santa Clara, CA 95050
 * Copyright @2016: all right reserved.
 *
 * This program is free software; you can redistribute it and/or modify it
 * under the terms and conditions of the GNU General Public License,
 * version 2, as published by the Free Software Foundation.
 *
 * This program is distributed in the hope it will be useful, but WITHOUT
 * ANY WARRANTY; without even the implied warranty of MERCHANTABILITY or
 * FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License for
 * more details.
 */
#ifndef __OV10652_CONFIG_H__
#define __OV10652_CONFIG_H__

struct ov10652_sensor_base_cfg {
	uint16_t reg;
	uint8_t value;
};

static struct ov10652_sensor_base_cfg sensor_base_settings[] = {
	{ 0x3013, 0x01 },
	{ 0x3000, 0x03 },
	{ 0x3001, 0x70 },
	{ 0x3002, 0x07 },
	{ 0x3003, 0x01 },
	{ 0x3004, 0x03 },
	{ 0x3005, 0x60 },
	{ 0x3006, 0x07 },
	{ 0x3007, 0x01 },
	{ 0x3008, 0x01 },
	{ 0x3014, 0x00 },
	{ 0x3015, 0x00 },
	{ 0x3020, 0xc0 },
	{ 0x3026, 0x05 },
	{ 0x3027, 0x00 },
	{ 0x3029, 0x00 },
	{ 0x3040, 0x02 },
	{ 0x3041, 0x00 },
	{ 0x3042, 0x13 },
	{ 0x3043, 0x00 },
	{ 0x3044, 0x00 },
	{ 0x3045, 0x00 },
	{ 0x3046, 0x05 },
	{ 0x3047, 0xc3 },
	{ 0x3048, 0x06 },
	{ 0x3049, 0x77 },
	{ 0x304a, 0x22 },
	{ 0x304b, 0x39 },
	{ 0x304c, 0x08 },
	{ 0x304d, 0x46 },
	{ 0x304e, 0x38 },
	{ 0x304f, 0xc7 },
	{ 0x3050, 0x2a },
	{ 0x3051, 0x2a },
	{ 0x3052, 0xea },
	{ 0x3053, 0xea },
	{ 0x3054, 0x3f },
	{ 0x3090, 0x04 },
	{ 0x3091, 0x7f },
	{ 0x3092, 0xfe },
	{ 0x3093, 0x19 },
	{ 0x3094, 0xd7 },
	{ 0x3095, 0x09 },
	{ 0x3096, 0x78 },
	{ 0x3098, 0x05 },
	{ 0x30b0, 0x00 },
	{ 0x30b1, 0x00 },
	{ 0x30b2, 0x00 },
	{ 0x30b3, 0x00 },
	{ 0x30b4, 0x07 },
	{ 0x30b5, 0x27 },
	{ 0x30b6, 0x03 },
	{ 0x30b7, 0xb3 },
	{ 0x30b8, 0x07 },
	{ 0x30b9, 0x20 },
	{ 0x30ba, 0x03 },
	{ 0x30bb, 0xac },
	{ 0x30bc, 0x07 },
	{ 0x30bd, 0x7e },
	{ 0x30be, 0x07 },
	{ 0x30bf, 0x9a },
	{ 0x30c0, 0x00 },
	{ 0x30c1, 0x04 },
	{ 0x30c2, 0x00 },
	{ 0x30c3, 0x04 },
	{ 0x30c4, 0x00 },
	{ 0x30c5, 0x40 },
	{ 0x30c8, 0x03 },
	{ 0x30ca, 0x00 },
	{ 0x30cb, 0x12 },
	{ 0x30ce, 0x00 },
	{ 0x30cf, 0x00 },
	{ 0x30d2, 0xf5 },
	{ 0x30e1, 0x04 },
	{ 0x30e5, 0x79 },
	{ 0x30e6, 0x20 },
	{ 0x30e7, 0x79 },
	{ 0x30e8, 0x20 },
	{ 0x30e9, 0x79 },
	{ 0x30ea, 0x20 },
	{ 0x30eb, 0xf9 },
	{ 0x30ec, 0x20 },
	{ 0x30f8, 0x50 },
	{ 0x30f9, 0x88 },
	{ 0x30fa, 0x61 },
	{ 0x30fb, 0x86 },
	{ 0x30fc, 0x63 },
	{ 0x30fd, 0x7a },
	{ 0x30fe, 0x54 },
	{ 0x30ff, 0x32 },
	{ 0x3100, 0x48 },
	{ 0x3110, 0x00 },
	{ 0x3111, 0x00 },
	{ 0x3112, 0x01 },
	{ 0x3113, 0xc9 },
	{ 0x3114, 0x05 },
	{ 0x3115, 0x00 },
	{ 0x3116, 0x10 },
	{ 0x3117, 0x10 },
	{ 0x3130, 0x2a },
	{ 0x3131, 0x01 },
	{ 0x313c, 0x00 },
	{ 0x313d, 0x00 },
	{ 0x313e, 0x00 },
	{ 0x313f, 0x00 },
	{ 0x3140, 0x00 },
	{ 0x3141, 0x00 },
	{ 0x3142, 0x00 },
	{ 0x3143, 0x80 },
	{ 0x3144, 0x00 },
	{ 0x3145, 0x80 },
	{ 0x3146, 0x00 },
	{ 0x3147, 0x80 },
	{ 0x3148, 0x1c },
	{ 0x3149, 0xff },
	{ 0x314a, 0xff },
	{ 0x314b, 0xff },
	{ 0x314c, 0x10 },
	{ 0x314d, 0x10 },
	{ 0x314e, 0x10 },
	{ 0x3154, 0x0f },
	{ 0x3155, 0xff },
	{ 0x3197, 0x00 },
	{ 0x31d0, 0x52 },
	{ 0x31d2, 0x00 },
	{ 0x31d3, 0x48 },
	{ 0x3210, 0x73 },
	{ 0x3230, 0x30 },
	{ 0x3231, 0xb7 },
	{ 0x3232, 0xca },
	{ 0x3233, 0xcc },
	{ 0x32bc, 0x01 },
	{ 0x32bd, 0x9f },
	{ 0x32be, 0xff },
	{ 0x35a4, 0x48 },
	{ 0x35d5, 0x05 },
	{ 0x6000, 0xfd },
	{ 0x6001, 0x00 },
	{ 0x6002, 0xc1 },
	{ 0x6003, 0xc5 },
	{ 0x6004, 0xeb },
	{ 0x6005, 0x00 },
	{ 0x6006, 0x00 },
	{ 0x6007, 0xb9 },
	{ 0x6008, 0xba },
	{ 0x6009, 0xa4 },
	{ 0x600a, 0x52 },
	{ 0x600b, 0x50 },
	{ 0x600c, 0xb5 },
	{ 0x600d, 0xea },
	{ 0x600e, 0x01 },
	{ 0x600f, 0x10 },
	{ 0x6010, 0xa7 },
	{ 0x6011, 0xb7 },
	{ 0x6012, 0x5c },
	{ 0x6013, 0x9e },
	{ 0x6014, 0x22 },
	{ 0x6015, 0xd3 },
	{ 0x6016, 0x23 },
	{ 0x6017, 0xea },
	{ 0x6018, 0x00 },
	{ 0x6019, 0xf6 },
	{ 0x601a, 0xcc },
	{ 0x601b, 0x96 },
	{ 0x601c, 0x35 },
	{ 0x601d, 0xab },
	{ 0x601e, 0xb7 },
	{ 0x601f, 0x00 },
	{ 0x6020, 0x00 },
	{ 0x6021, 0x00 },
	{ 0x6022, 0x00 },
	{ 0x6023, 0x00 },
	{ 0x6024, 0x00 },
	{ 0x6025, 0x00 },
	{ 0x6026, 0x00 },
	{ 0x6027, 0x00 },
	{ 0x6028, 0x00 },
	{ 0x6029, 0x00 },
	{ 0x602a, 0x61 },
	{ 0x602b, 0x9c },
	{ 0x602c, 0x94 },
	{ 0x602d, 0x90 },
	{ 0x602e, 0xb1 },
	{ 0x602f, 0xb2 },
	{ 0x6030, 0x54 },
	{ 0x6031, 0xa0 },
	{ 0x6032, 0xa2 },
	{ 0x6033, 0x34 },
	{ 0x6034, 0x00 },
	{ 0x6035, 0x36 },
	{ 0x6036, 0xd3 },
	{ 0x6037, 0x10 },
	{ 0x6038, 0x34 },
	{ 0x6039, 0xde },
	{ 0x603a, 0xcc },
	{ 0x603b, 0x03 },
	{ 0x603c, 0xe7 },
	{ 0x603d, 0x02 },
	{ 0x603e, 0x31 },
	{ 0x603f, 0xea },
	{ 0x6040, 0x03 },
	{ 0x6041, 0xd2 },
	{ 0x6042, 0xce },
	{ 0x6043, 0x1c },
	{ 0x6044, 0xcf },
	{ 0x6045, 0x21 },
	{ 0x6046, 0xd0 },
	{ 0x6047, 0x26 },
	{ 0x6048, 0xd2 },
	{ 0x6049, 0xbc },
	{ 0x604a, 0xcc },
	{ 0x604b, 0x54 },
	{ 0x604c, 0xcc },
	{ 0x604d, 0x00 },
	{ 0x604e, 0xe9 },
	{ 0x604f, 0x00 },
	{ 0x6050, 0xcc },
	{ 0x6051, 0x0b },
	{ 0x6052, 0xd2 },
	{ 0x6053, 0xd3 },
	{ 0x6054, 0x0a },
	{ 0x6055, 0x1a },
	{ 0x6056, 0xea },
	{ 0x6057, 0x02 },
	{ 0x6058, 0xd4 },
	{ 0x6059, 0x84 },
	{ 0x605a, 0xba },
	{ 0x605b, 0xa4 },
	{ 0x605c, 0x56 },
	{ 0x605d, 0x26 },
	{ 0x605e, 0xe7 },
	{ 0x605f, 0x07 },
	{ 0x6060, 0xe9 },
	{ 0x6061, 0x00 },
	{ 0x6062, 0x54 },
	{ 0x6063, 0xcc },
	{ 0x6064, 0x65 },
	{ 0x6065, 0xe7 },
	{ 0x6066, 0x03 },
	{ 0x6067, 0xea },
	{ 0x6068, 0x03 },
	{ 0x6069, 0xe9 },
	{ 0x606a, 0x00 },
	{ 0x606b, 0x1a },
	{ 0x606c, 0xcc },
	{ 0x606d, 0xa2 },
	{ 0x606e, 0xea },
	{ 0x606f, 0x02 },
	{ 0x6070, 0xa0 },
	{ 0x6071, 0x20 },
	{ 0x6072, 0x60 },
	{ 0x6073, 0xc2 },
	{ 0x6074, 0xe1 },
	{ 0x6075, 0xc6 },
	{ 0x6076, 0xb9 },
	{ 0x6077, 0xa5 },
	{ 0x6078, 0x51 },
	{ 0x6079, 0x50 },
	{ 0x607a, 0xb5 },
	{ 0x607b, 0x5c },
	{ 0x607c, 0x35 },
	{ 0x607d, 0xd4 },
	{ 0x607e, 0x64 },
	{ 0x607f, 0x7c },
	{ 0x6080, 0x74 },
	{ 0x6081, 0xa0 },
	{ 0x6082, 0xa2 },
	{ 0x6083, 0x61 },
	{ 0x6084, 0xcc },
	{ 0x6085, 0x93 },
	{ 0x6086, 0xf6 },
	{ 0x6087, 0x34 },
	{ 0x6088, 0x00 },
	{ 0x6089, 0x36 },
	{ 0x608a, 0xd3 },
	{ 0x608b, 0x10 },
	{ 0x608c, 0x34 },
	{ 0x608d, 0xdf },
	{ 0x608e, 0xcc },
	{ 0x608f, 0x03 },
	{ 0x6090, 0xe7 },
	{ 0x6091, 0x04 },
	{ 0x6092, 0x31 },
	{ 0x6093, 0xea },
	{ 0x6094, 0x03 },
	{ 0x6095, 0xd2 },
	{ 0x6096, 0xbb },
	{ 0x6097, 0xcc },
	{ 0x6098, 0x1c },
	{ 0x6099, 0xd2 },
	{ 0x609a, 0xb3 },
	{ 0x609b, 0xbd },
	{ 0x609c, 0xcc },
	{ 0x609d, 0x54 },
	{ 0x609e, 0xcc },
	{ 0x609f, 0x00 },
	{ 0x60a0, 0xe9 },
	{ 0x60a1, 0x00 },
	{ 0x60a2, 0xcc },
	{ 0x60a3, 0x0b },
	{ 0x60a4, 0xd2 },
	{ 0x60a5, 0xd3 },
	{ 0x60a6, 0x0a },
	{ 0x60a7, 0x1a },
	{ 0x60a8, 0xea },
	{ 0x60a9, 0x02 },
	{ 0x60aa, 0x71 },
	{ 0x60ab, 0x26 },
	{ 0x60ac, 0xe7 },
	{ 0x60ad, 0x08 },
	{ 0x60ae, 0xd4 },
	{ 0x60af, 0x84 },
	{ 0x60b0, 0xe9 },
	{ 0x60b1, 0x00 },
	{ 0x60b2, 0x70 },
	{ 0x60b3, 0xca },
	{ 0x60b4, 0xcc },
	{ 0x60b5, 0x65 },
	{ 0x60b6, 0xe7 },
	{ 0x60b7, 0x05 },
	{ 0x60b8, 0xea },
	{ 0x60b9, 0x03 },
	{ 0x60ba, 0xe9 },
	{ 0x60bb, 0x00 },
	{ 0x60bc, 0x1a },
	{ 0x60bd, 0xea },
	{ 0x60be, 0x02 },
	{ 0x60bf, 0xcc },
	{ 0x60c0, 0xa2 },
	{ 0x60c1, 0xa0 },
	{ 0x60c2, 0x20 },
	{ 0x60c3, 0x60 },
	{ 0x60c4, 0xe3 },
	{ 0x60c5, 0x03 },
	{ 0x60c6, 0xe5 },
	{ 0x60c7, 0x41 },
	{ 0x60c8, 0xc3 },
	{ 0x60c9, 0xe2 },
	{ 0x60ca, 0xc7 },
	{ 0x60cb, 0x35 },
	{ 0x60cc, 0xb9 },
	{ 0x60cd, 0xa5 },
	{ 0x60ce, 0x51 },
	{ 0x60cf, 0x50 },
	{ 0x60d0, 0xb5 },
	{ 0x60d1, 0x41 },
	{ 0x60d2, 0x5c },
	{ 0x60d3, 0x61 },
	{ 0x60d4, 0x2a },
	{ 0x60d5, 0xcc },
	{ 0x60d6, 0x93 },
	{ 0x60d7, 0xf6 },
	{ 0x60d8, 0x34 },
	{ 0x60d9, 0x00 },
	{ 0x60da, 0x36 },
	{ 0x60db, 0xd3 },
	{ 0x60dc, 0x10 },
	{ 0x60dd, 0x34 },
	{ 0x60de, 0xcc },
	{ 0x60df, 0x03 },
	{ 0x60e0, 0x31 },
	{ 0x60e1, 0xd2 },
	{ 0x60e2, 0xeb },
	{ 0x60e3, 0x00 },
	{ 0x60e4, 0xff },
	{ 0x60e5, 0xf6 },
	{ 0x60e6, 0xbb },
	{ 0x60e7, 0xcc },
	{ 0x60e8, 0x1c },
	{ 0x60e9, 0xd2 },
	{ 0x60ea, 0xfe },
	{ 0x60eb, 0xcc },
	{ 0x60ec, 0x54 },
	{ 0x60ed, 0x5c },
	{ 0x60ee, 0xcc },
	{ 0x60ef, 0x00 },
	{ 0x60f0, 0xd2 },
	{ 0x60f1, 0xd3 },
	{ 0x60f2, 0x10 },
	{ 0x60f3, 0xcc },
	{ 0x60f4, 0x0b },
	{ 0x60f5, 0xd2 },
	{ 0x60f6, 0x26 },
	{ 0x60f7, 0xd3 },
	{ 0x60f8, 0x50 },
	{ 0x60f9, 0x1a },
	{ 0x60fa, 0xcc },
	{ 0x60fb, 0xa2 },
	{ 0x60fc, 0xa0 },
	{ 0x60fd, 0x20 },
	{ 0x60fe, 0x50 },
	{ 0x60ff, 0x60 },
	{ 0x6100, 0x40 },
	{ 0x6101, 0xeb },
	{ 0x6102, 0x00 },
	{ 0x6103, 0x00 },
	{ 0x6104, 0xc0 },
	{ 0x6105, 0xe0 },
	{ 0x6106, 0xc4 },
	{ 0x6107, 0xb9 },
	{ 0x6108, 0xa3 },
	{ 0x6109, 0x51 },
	{ 0x610a, 0x50 },
	{ 0x610b, 0xb5 },
	{ 0x610c, 0x00 },
	{ 0x610d, 0x5c },
	{ 0x610e, 0x35 },
	{ 0x610f, 0xba },
	{ 0x6110, 0xae },
	{ 0x6111, 0x00 },
	{ 0x6112, 0x00 },
	{ 0x6113, 0x00 },
	{ 0x6114, 0x00 },
	{ 0x6115, 0x00 },
	{ 0x6116, 0xaa },
	{ 0x6117, 0xb7 },
	{ 0x6118, 0x00 },
	{ 0x6119, 0x00 },
	{ 0x611a, 0x00 },
	{ 0x611b, 0x00 },
	{ 0x611c, 0xa6 },
	{ 0x611d, 0xb7 },
	{ 0x611e, 0x00 },
	{ 0x611f, 0x9d },
	{ 0x6120, 0xd3 },
	{ 0x6121, 0x14 },
	{ 0x6122, 0xb0 },
	{ 0x6123, 0xb7 },
	{ 0x6124, 0x00 },
	{ 0x6125, 0xd3 },
	{ 0x6126, 0x34 },
	{ 0x6127, 0x9c },
	{ 0x6128, 0x94 },
	{ 0x6129, 0x90 },
	{ 0x612a, 0xc8 },
	{ 0x612b, 0xba },
	{ 0x612c, 0xb0 },
	{ 0x612d, 0x7c },
	{ 0x612e, 0x74 },
	{ 0x612f, 0xa0 },
	{ 0x6130, 0xa2 },
	{ 0x6131, 0x61 },
	{ 0x6132, 0x61 },
	{ 0x6133, 0xcc },
	{ 0x6134, 0x93 },
	{ 0x6135, 0xf6 },
	{ 0x6136, 0x34 },
	{ 0x6137, 0x00 },
	{ 0x6138, 0x36 },
	{ 0x6139, 0xd3 },
	{ 0x613a, 0x10 },
	{ 0x613b, 0x34 },
	{ 0x613c, 0xdc },
	{ 0x613d, 0xcc },
	{ 0x613e, 0x03 },
	{ 0x613f, 0xe7 },
	{ 0x6140, 0x00 },
	{ 0x6141, 0x31 },
	{ 0x6142, 0xea },
	{ 0x6143, 0x03 },
	{ 0x6144, 0xcc },
	{ 0x6145, 0x19 },
	{ 0x6146, 0xd2 },
	{ 0x6147, 0xbb },
	{ 0x6148, 0xcc },
	{ 0x6149, 0x1e },
	{ 0x614a, 0xd2 },
	{ 0x614b, 0xfe },
	{ 0x614c, 0xce },
	{ 0x614d, 0x54 },
	{ 0x614e, 0xcf },
	{ 0x614f, 0x59 },
	{ 0x6150, 0xd0 },
	{ 0x6151, 0x5f },
	{ 0x6152, 0xcc },
	{ 0x6153, 0x00 },
	{ 0x6154, 0xe9 },
	{ 0x6155, 0x00 },
	{ 0x6156, 0xcc },
	{ 0x6157, 0x0b },
	{ 0x6158, 0xd2 },
	{ 0x6159, 0xd3 },
	{ 0x615a, 0x0a },
	{ 0x615b, 0xd9 },
	{ 0x615c, 0x68 },
	{ 0x615d, 0xda },
	{ 0x615e, 0x6c },
	{ 0x615f, 0x1a },
	{ 0x6160, 0xea },
	{ 0x6161, 0x02 },
	{ 0x6162, 0xd4 },
	{ 0x6163, 0x84 },
	{ 0x6164, 0x71 },
	{ 0x6165, 0x26 },
	{ 0x6166, 0xe7 },
	{ 0x6167, 0x06 },
	{ 0x6168, 0xba },
	{ 0x6169, 0xb0 },
	{ 0x616a, 0xe9 },
	{ 0x616b, 0x00 },
	{ 0x616c, 0x70 },
	{ 0x616d, 0x26 },
	{ 0x616e, 0x61 },
	{ 0x616f, 0xcc },
	{ 0x6170, 0x65 },
	{ 0x6171, 0xe7 },
	{ 0x6172, 0x01 },
	{ 0x6173, 0xea },
	{ 0x6174, 0x03 },
	{ 0x6175, 0xe9 },
	{ 0x6176, 0x00 },
	{ 0x6177, 0xd9 },
	{ 0x6178, 0x76 },
	{ 0x6179, 0xda },
	{ 0x617a, 0x7a },
	{ 0x617b, 0x1a },
	{ 0x617c, 0xea },
	{ 0x617d, 0x02 },
	{ 0x617e, 0x12 },
	{ 0x617f, 0xcc },
	{ 0x6180, 0xa2 },
	{ 0x6181, 0xd6 },
	{ 0x6182, 0x62 },
	{ 0x6183, 0xb9 },
	{ 0x6184, 0xba },
	{ 0x6185, 0xaf },
	{ 0x6186, 0xff },
	{ 0x6187, 0x00 },
	{ 0x6188, 0xd2 },
	{ 0x6189, 0xa0 },
	{ 0x618a, 0x01 },
	{ 0x618b, 0x50 },
	{ 0x618c, 0x60 },
	{ 0x618d, 0xcc },
	{ 0x618e, 0x9e },
	{ 0x618f, 0xd5 },
	{ 0x6190, 0xba },
	{ 0x6191, 0xb0 },
	{ 0x6192, 0xb7 },
	{ 0x6193, 0x00 },
	{ 0x6194, 0x9d },
	{ 0x6195, 0xd3 },
	{ 0x6196, 0x34 },
	{ 0x6197, 0x9c },
	{ 0x6198, 0x94 },
	{ 0x6199, 0x90 },
	{ 0x619a, 0xc8 },
	{ 0x619b, 0xba },
	{ 0x619c, 0xb0 },
	{ 0x619d, 0xd5 },
	{ 0x619e, 0x00 },
	{ 0x619f, 0x00 },
	{ 0x61a0, 0x01 },
	{ 0x61a1, 0x1a },
	{ 0x61a2, 0xcc },
	{ 0x61a3, 0x16 },
	{ 0x61a4, 0x12 },
	{ 0x61a5, 0xea },
	{ 0x61a6, 0x02 },
	{ 0x61a7, 0xcc },
	{ 0x61a8, 0xb5 },
	{ 0x61a9, 0xcc },
	{ 0x61aa, 0x65 },
	{ 0x61ab, 0xea },
	{ 0x61ac, 0x02 },
	{ 0x61ad, 0xd2 },
	{ 0x61ae, 0x04 },
	{ 0x61af, 0xd5 },
	{ 0x61b0, 0x1a },
	{ 0x61b1, 0xcc },
	{ 0x61b2, 0x16 },
	{ 0x61b3, 0xea },
	{ 0x61b4, 0x02 },
	{ 0x61b5, 0x12 },
	{ 0x61b6, 0xcc },
	{ 0x61b7, 0xb5 },
	{ 0x61b8, 0xcc },
	{ 0x61b9, 0x65 },
	{ 0x61ba, 0xea },
	{ 0x61bb, 0x03 },
	{ 0x61bc, 0xd2 },
	{ 0x61bd, 0x1a },
	{ 0x61be, 0xcc },
	{ 0x61bf, 0x16 },
	{ 0x61c0, 0xea },
	{ 0x61c1, 0x02 },
	{ 0x61c2, 0x12 },
	{ 0x61c3, 0xcc },
	{ 0x61c4, 0xb5 },
	{ 0x61c5, 0xcc },
	{ 0x61c6, 0x65 },
	{ 0x61c7, 0xea },
	{ 0x61c8, 0x03 },
	{ 0x61c9, 0xd2 },
	{ 0x61ca, 0x1a },
	{ 0x61cb, 0xcc },
	{ 0x61cc, 0x16 },
	{ 0x61cd, 0xea },
	{ 0x61ce, 0x02 },
	{ 0x61cf, 0x12 },
	{ 0x61d0, 0xcc },
	{ 0x61d1, 0xb5 },
	{ 0x61d2, 0xcc },
	{ 0x61d3, 0x65 },
	{ 0x61d4, 0xea },
	{ 0x61d5, 0x03 },
	{ 0x61d6, 0xd2 },
	{ 0x61d7, 0xd5 },
	{ 0x61d8, 0x1a },
	{ 0x61d9, 0xcc },
	{ 0x61da, 0x16 },
	{ 0x61db, 0x12 },
	{ 0x61dc, 0xea },
	{ 0x61dd, 0x02 },
	{ 0x61de, 0xcc },
	{ 0x61df, 0xa3 },
	{ 0x61e0, 0xcc },
	{ 0x61e1, 0x7c },
	{ 0x61e2, 0xea },
	{ 0x61e3, 0x03 },
	{ 0x61e4, 0xd2 },
	{ 0x61e5, 0xd5 },
	{ 0x61e6, 0x00 },
	{ 0x61e7, 0x00 },
	{ 0x61e8, 0x1a },
	{ 0x61e9, 0xcc },
	{ 0x61ea, 0x16 },
	{ 0x61eb, 0xea },
	{ 0x61ec, 0x02 },
	{ 0x61ed, 0x12 },
	{ 0x61ee, 0xcc },
	{ 0x61ef, 0xa3 },
	{ 0x61f0, 0xcc },
	{ 0x61f1, 0x7c },
	{ 0x61f2, 0xea },
	{ 0x61f3, 0x03 },
	{ 0x61f4, 0xd2 },
	{ 0x61f5, 0x1a },
	{ 0x61f6, 0xcc },
	{ 0x61f7, 0x16 },
	{ 0x61f8, 0xea },
	{ 0x61f9, 0x02 },
	{ 0x61fa, 0x12 },
	{ 0x61fb, 0xcc },
	{ 0x61fc, 0xa3 },
	{ 0x61fd, 0xcc },
	{ 0x61fe, 0x7c },
	{ 0x61ff, 0xea },
	{ 0x6200, 0x03 },
	{ 0x6201, 0xd2 },
	{ 0x6202, 0x1a },
	{ 0x6203, 0xcc },
	{ 0x6204, 0x16 },
	{ 0x6205, 0xea },
	{ 0x6206, 0x02 },
	{ 0x6207, 0x12 },
	{ 0x6208, 0xcc },
	{ 0x6209, 0xa3 },
	{ 0x620a, 0xcc },
	{ 0x620b, 0x7c },
	{ 0x620c, 0xea },
	{ 0x620d, 0x03 },
	{ 0x620e, 0xd2 },
	{ 0x620f, 0xd5 },
	{ 0x6210, 0xcc },
	{ 0x6211, 0x16 },
	{ 0x6212, 0x00 },
	{ 0x6213, 0x12 },
	{ 0x6214, 0xcc },
	{ 0x6215, 0xb5 },
	{ 0x6216, 0xd5 },
	{ 0x6800, 0x00 },
	{ 0x6801, 0x0f },
	{ 0x6802, 0x00 },
	{ 0x6803, 0x0f },
	{ 0x6804, 0x00 },
	{ 0x6805, 0xff },
	{ 0x6806, 0x05 },
	{ 0x6807, 0x10 },
	{ 0x6808, 0x04 },
	{ 0x6809, 0x8d },
	{ 0x680a, 0x06 },
	{ 0x680b, 0x13 },
	{ 0x680c, 0x04 },
	{ 0x680d, 0x8c },
	{ 0x680e, 0x00 },
	{ 0x680f, 0x01 },
	{ 0x6810, 0x06 },
	{ 0x6811, 0x1f },
	{ 0x6812, 0x00 },
	{ 0x6813, 0x00 },
	{ 0x6814, 0x00 },
	{ 0x6815, 0xff },
	{ 0x6816, 0x06 },
	{ 0x6817, 0x1b },
	{ 0x6818, 0x06 },
	{ 0x6819, 0x1a },
	{ 0x681a, 0x00 },
	{ 0x681b, 0x02 },
	{ 0x681c, 0x06 },
	{ 0x681d, 0x10 },
	{ 0x681e, 0x00 },
	{ 0x681f, 0x00 },
	{ 0x6820, 0x05 },
	{ 0x6821, 0x13 },
	{ 0x6822, 0x00 },
	{ 0x6823, 0xff },
	{ 0x6824, 0x04 },
	{ 0x6825, 0x8d },
	{ 0x6826, 0x04 },
	{ 0x6827, 0x8c },
	{ 0x6828, 0x05 },
	{ 0x6829, 0x10 },
	{ 0x682a, 0x00 },
	{ 0x682b, 0xff },
	{ 0x682c, 0x04 },
	{ 0x682d, 0x80 },
	{ 0x682e, 0x05 },
	{ 0x682f, 0x03 },
	{ 0x6830, 0x00 },
	{ 0x6831, 0xff },
	{ 0x6832, 0x08 },
	{ 0x6833, 0x2b },
	{ 0x6834, 0x08 },
	{ 0x6835, 0x32 },
	{ 0x6836, 0x00 },
	{ 0x6837, 0xff },
	{ 0x6838, 0x08 },
	{ 0x6839, 0x2b },
	{ 0x683a, 0x08 },
	{ 0x683b, 0x32 },
	{ 0x683c, 0x07 },
	{ 0x683d, 0x20 },
	{ 0x683e, 0x08 },
	{ 0x683f, 0x41 },
	{ 0x6840, 0x00 },
	{ 0x6841, 0xff },
	{ 0x6842, 0x08 },
	{ 0x6843, 0x2b },
	{ 0x6844, 0x08 },
	{ 0x6845, 0x37 },
	{ 0x6846, 0x07 },
	{ 0x6847, 0x10 },
	{ 0x6848, 0x08 },
	{ 0x6849, 0x41 },
	{ 0x684a, 0x00 },
	{ 0x684b, 0xff },
	{ 0x684c, 0x08 },
	{ 0x684d, 0x2b },
	{ 0x684e, 0x08 },
	{ 0x684f, 0x3c },
	{ 0x6850, 0x07 },
	{ 0x6851, 0x00 },
	{ 0x6852, 0x08 },
	{ 0x6853, 0x41 },
	{ 0x6854, 0x00 },
	{ 0x6855, 0xff },
	{ 0x6856, 0x07 },
	{ 0x6857, 0x00 },
	{ 0x6858, 0x06 },
	{ 0x6859, 0x9f },
	{ 0x685a, 0x01 },
	{ 0x685b, 0xaf },
	{ 0x685c, 0x01 },
	{ 0x685d, 0x0f },
	{ 0x685e, 0x01 },
	{ 0x685f, 0x90 },
	{ 0x6860, 0x01 },
	{ 0x6861, 0xc8 },
	{ 0x6862, 0x00 },
	{ 0x6863, 0xff },
	{ 0x6864, 0x01 },
	{ 0x6865, 0xac },
	{ 0x6866, 0x01 },
	{ 0x6867, 0x0c },
	{ 0x6868, 0x01 },
	{ 0x6869, 0x90 },
	{ 0x686a, 0x01 },
	{ 0x686b, 0xe8 },
	{ 0x686c, 0x00 },
	{ 0x686d, 0xff },
	{ 0x686e, 0x01 },
	{ 0x686f, 0xad },
	{ 0x6870, 0x01 },
	{ 0x6871, 0x0d },
	{ 0x6872, 0x01 },
	{ 0x6873, 0x90 },
	{ 0x6874, 0x01 },
	{ 0x6875, 0xe8 },
	{ 0x6876, 0x00 },
	{ 0x6877, 0xff },
	{ 0x6878, 0x01 },
	{ 0x6879, 0xae },
	{ 0x687a, 0x01 },
	{ 0x687b, 0x0e },
	{ 0x687c, 0x01 },
	{ 0x687d, 0x90 },
	{ 0x687e, 0x01 },
	{ 0x687f, 0xe8 },
	{ 0x6880, 0x00 },
	{ 0x6881, 0xff },
	{ 0x6882, 0x01 },
	{ 0x6883, 0xb0 },
	{ 0x6884, 0x01 },
	{ 0x6885, 0xb1 },
	{ 0x6886, 0x01 },
	{ 0x6887, 0xb2 },
	{ 0x6888, 0x01 },
	{ 0x6889, 0xb3 },
	{ 0x688a, 0x01 },
	{ 0x688b, 0xb4 },
	{ 0x688c, 0x01 },
	{ 0x688d, 0xb5 },
	{ 0x688e, 0x01 },
	{ 0x688f, 0xb6 },
	{ 0x6890, 0x01 },
	{ 0x6891, 0xb7 },
	{ 0x6892, 0x01 },
	{ 0x6893, 0xb8 },
	{ 0x6894, 0x01 },
	{ 0x6895, 0xb9 },
	{ 0x6896, 0x01 },
	{ 0x6897, 0xba },
	{ 0x6898, 0x01 },
	{ 0x6899, 0xbb },
	{ 0x689a, 0x01 },
	{ 0x689b, 0xbc },
	{ 0x689c, 0x01 },
	{ 0x689d, 0xbd },
	{ 0x689e, 0x01 },
	{ 0x689f, 0xbe },
	{ 0x68a0, 0x01 },
	{ 0x68a1, 0xbf },
	{ 0x68a2, 0x01 },
	{ 0x68a3, 0xc0 },
	{ 0x68a4, 0x06 },
	{ 0x68a5, 0x1f },
	{ 0x68a6, 0x00 },
	{ 0x68a7, 0xff },
	{ 0x68a8, 0x07 },
	{ 0x68a9, 0x00 },
	{ 0x68aa, 0x01 },
	{ 0x68ab, 0xf6 },
	{ 0x68ac, 0x01 },
	{ 0x68ad, 0xf6 },
	{ 0x68ae, 0x04 },
	{ 0x68af, 0x8c },
	{ 0x68b0, 0x00 },
	{ 0x68b1, 0xff },
	{ 0x68b2, 0x07 },
	{ 0x68b3, 0x10 },
	{ 0x68b4, 0x01 },
	{ 0x68b5, 0xf6 },
	{ 0x68b6, 0x01 },
	{ 0x68b7, 0xf6 },
	{ 0x68b8, 0x04 },
	{ 0x68b9, 0x8c },
	{ 0x68ba, 0x07 },
	{ 0x68bb, 0x00 },
	{ 0x68bc, 0x00 },
	{ 0x68bd, 0xff },
	{ 0x68be, 0x07 },
	{ 0x68bf, 0x20 },
	{ 0x68c0, 0x01 },
	{ 0x68c1, 0xf6 },
	{ 0x68c2, 0x01 },
	{ 0x68c3, 0xf6 },
	{ 0x68c4, 0x04 },
	{ 0x68c5, 0x8c },
	{ 0x68c6, 0x07 },
	{ 0x68c7, 0x00 },
	{ 0x68c8, 0x00 },
	{ 0x68c9, 0xff },
	{ 0x68ca, 0x05 },
	{ 0x68cb, 0x13 },
	{ 0x68cc, 0x04 },
	{ 0x68cd, 0x8d },
	{ 0x68ce, 0x04 },
	{ 0x68cf, 0x8c },
	{ 0x68d0, 0x06 },
	{ 0x68d1, 0x50 },
	{ 0x68d2, 0x01 },
	{ 0x68d3, 0x20 },
	{ 0x68d4, 0x01 },
	{ 0x68d5, 0x31 },
	{ 0x68d6, 0x01 },
	{ 0x68d7, 0x32 },
	{ 0x68d8, 0x01 },
	{ 0x68d9, 0x33 },
	{ 0x68da, 0x01 },
	{ 0x68db, 0x34 },
	{ 0x68dc, 0x01 },
	{ 0x68dd, 0x35 },
	{ 0x68de, 0x01 },
	{ 0x68df, 0x36 },
	{ 0x68e0, 0x01 },
	{ 0x68e1, 0x37 },
	{ 0x68e2, 0x01 },
	{ 0x68e3, 0x38 },
	{ 0x68e4, 0x01 },
	{ 0x68e5, 0x39 },
	{ 0x68e6, 0x01 },
	{ 0x68e7, 0x3a },
	{ 0x68e8, 0x01 },
	{ 0x68e9, 0x3b },
	{ 0x68ea, 0x01 },
	{ 0x68eb, 0x3c },
	{ 0x68ec, 0x01 },
	{ 0x68ed, 0x3d },
	{ 0x68ee, 0x01 },
	{ 0x68ef, 0x3e },
	{ 0x68f0, 0x01 },
	{ 0x68f1, 0x3f },
	{ 0x68f2, 0x02 },
	{ 0x68f3, 0xa0 },
	{ 0x68f4, 0x06 },
	{ 0x68f5, 0x10 },
	{ 0x68f6, 0x00 },
	{ 0x68f7, 0xff },
	{ 0x68f8, 0x05 },
	{ 0x68f9, 0x13 },
	{ 0x68fa, 0x04 },
	{ 0x68fb, 0x8d },
	{ 0x68fc, 0x04 },
	{ 0x68fd, 0x8c },
	{ 0x68fe, 0x06 },
	{ 0x68ff, 0x50 },
	{ 0x6900, 0x01 },
	{ 0x6901, 0x00 },
	{ 0x6902, 0x01 },
	{ 0x6903, 0x11 },
	{ 0x6904, 0x01 },
	{ 0x6905, 0x12 },
	{ 0x6906, 0x01 },
	{ 0x6907, 0x13 },
	{ 0x6908, 0x01 },
	{ 0x6909, 0x14 },
	{ 0x690a, 0x01 },
	{ 0x690b, 0x15 },
	{ 0x690c, 0x01 },
	{ 0x690d, 0x16 },
	{ 0x690e, 0x01 },
	{ 0x690f, 0x17 },
	{ 0x6910, 0x01 },
	{ 0x6911, 0x18 },
	{ 0x6912, 0x01 },
	{ 0x6913, 0x19 },
	{ 0x6914, 0x01 },
	{ 0x6915, 0x1a },
	{ 0x6916, 0x01 },
	{ 0x6917, 0x1b },
	{ 0x6918, 0x01 },
	{ 0x6919, 0x1c },
	{ 0x691a, 0x01 },
	{ 0x691b, 0x1d },
	{ 0x691c, 0x01 },
	{ 0x691d, 0x1e },
	{ 0x691e, 0x01 },
	{ 0x691f, 0x1f },
	{ 0x6920, 0x02 },
	{ 0x6921, 0xa0 },
	{ 0x6922, 0x06 },
	{ 0x6923, 0x10 },
	{ 0x6924, 0x00 },
	{ 0x6925, 0xff },
	{ 0x6926, 0x04 },
	{ 0x6927, 0x88 },
	{ 0x6928, 0x08 },
	{ 0x6929, 0x7f },
	{ 0x692a, 0x00 },
	{ 0x692b, 0xff },
	{ 0x692c, 0x06 },
	{ 0x692d, 0x10 },
	{ 0x692e, 0x08 },
	{ 0x692f, 0x7f },
	{ 0x6930, 0x04 },
	{ 0x6931, 0x50 },
	{ 0x6932, 0x00 },
	{ 0x6933, 0x00 },
	{ 0x6934, 0x05 },
	{ 0x6935, 0x03 },
	{ 0x6936, 0x04 },
	{ 0x6937, 0x40 },
	{ 0x6938, 0x04 },
	{ 0x6939, 0x48 },
	{ 0x693a, 0x00 },
	{ 0x693b, 0xff },
	{ 0x693c, 0x04 },
	{ 0x693d, 0x40 },
	{ 0x693e, 0x05 },
	{ 0x693f, 0x03 },
	{ 0x6940, 0x06 },
	{ 0x6941, 0x00 },
	{ 0x6942, 0x00 },
	{ 0x6943, 0xff },
	{ 0x6944, 0x04 },
	{ 0x6945, 0x80 },
	{ 0x6946, 0x03 },
	{ 0x6947, 0x0b },
	{ 0x6948, 0x05 },
	{ 0x6949, 0x03 },
	{ 0x694a, 0x00 },
	{ 0x694b, 0x0c },
	{ 0x694c, 0x05 },
	{ 0x694d, 0x12 },
	{ 0x694e, 0x00 },
	{ 0x694f, 0x00 },
	{ 0x6950, 0x05 },
	{ 0x6951, 0x30 },
	{ 0x6952, 0x00 },
	{ 0x6953, 0x01 },
	{ 0x6954, 0x05 },
	{ 0x6955, 0x34 },
	{ 0x6956, 0x05 },
	{ 0x6957, 0x3c },
	{ 0x6958, 0x03 },
	{ 0x6959, 0x9a },
	{ 0x695a, 0x05 },
	{ 0x695b, 0x03 },
	{ 0x695c, 0x00 },
	{ 0x695d, 0x07 },
	{ 0x695e, 0x05 },
	{ 0x695f, 0x12 },
	{ 0x6960, 0x00 },
	{ 0x6961, 0x00 },
	{ 0x6962, 0x05 },
	{ 0x6963, 0x30 },
	{ 0x6964, 0x00 },
	{ 0x6965, 0x01 },
	{ 0x6966, 0x05 },
	{ 0x6967, 0x34 },
	{ 0x6968, 0x05 },
	{ 0x6969, 0x3c },
	{ 0x696a, 0x03 },
	{ 0x696b, 0x99 },
	{ 0x696c, 0x05 },
	{ 0x696d, 0x03 },
	{ 0x696e, 0x00 },
	{ 0x696f, 0x05 },
	{ 0x6970, 0x05 },
	{ 0x6971, 0x12 },
	{ 0x6972, 0x00 },
	{ 0x6973, 0x00 },
	{ 0x6974, 0x05 },
	{ 0x6975, 0x30 },
	{ 0x6976, 0x00 },
	{ 0x6977, 0x01 },
	{ 0x6978, 0x05 },
	{ 0x6979, 0x34 },
	{ 0x697a, 0x05 },
	{ 0x697b, 0x3c },
	{ 0x697c, 0x03 },
	{ 0x697d, 0x98 },
	{ 0x697e, 0x05 },
	{ 0x697f, 0x03 },
	{ 0x6980, 0x00 },
	{ 0x6981, 0x01 },
	{ 0x6982, 0x05 },
	{ 0x6983, 0x12 },
	{ 0x6984, 0x00 },
	{ 0x6985, 0x00 },
	{ 0x6986, 0x05 },
	{ 0x6987, 0x30 },
	{ 0x6988, 0x00 },
	{ 0x6989, 0x01 },
	{ 0x698a, 0x05 },
	{ 0x698b, 0x34 },
	{ 0x698c, 0x05 },
	{ 0x698d, 0x3c },
	{ 0x698e, 0x03 },
	{ 0x698f, 0x97 },
	{ 0x6990, 0x05 },
	{ 0x6991, 0x03 },
	{ 0x6992, 0x00 },
	{ 0x6993, 0x01 },
	{ 0x6994, 0x05 },
	{ 0x6995, 0x12 },
	{ 0x6996, 0x00 },
	{ 0x6997, 0x00 },
	{ 0x6998, 0x05 },
	{ 0x6999, 0x30 },
	{ 0x699a, 0x00 },
	{ 0x699b, 0x01 },
	{ 0x699c, 0x05 },
	{ 0x699d, 0x34 },
	{ 0x699e, 0x05 },
	{ 0x699f, 0x3c },
	{ 0x69a0, 0x03 },
	{ 0x69a1, 0x96 },
	{ 0x69a2, 0x05 },
	{ 0x69a3, 0x03 },
	{ 0x69a4, 0x00 },
	{ 0x69a5, 0x01 },
	{ 0x69a6, 0x05 },
	{ 0x69a7, 0x12 },
	{ 0x69a8, 0x00 },
	{ 0x69a9, 0x00 },
	{ 0x69aa, 0x05 },
	{ 0x69ab, 0x30 },
	{ 0x69ac, 0x00 },
	{ 0x69ad, 0x01 },
	{ 0x69ae, 0x05 },
	{ 0x69af, 0x34 },
	{ 0x69b0, 0x05 },
	{ 0x69b1, 0x3c },
	{ 0x69b2, 0x03 },
	{ 0x69b3, 0x95 },
	{ 0x69b4, 0x05 },
	{ 0x69b5, 0x03 },
	{ 0x69b6, 0x00 },
	{ 0x69b7, 0x01 },
	{ 0x69b8, 0x05 },
	{ 0x69b9, 0x12 },
	{ 0x69ba, 0x00 },
	{ 0x69bb, 0x00 },
	{ 0x69bc, 0x05 },
	{ 0x69bd, 0x30 },
	{ 0x69be, 0x00 },
	{ 0x69bf, 0x01 },
	{ 0x69c0, 0x05 },
	{ 0x69c1, 0x34 },
	{ 0x69c2, 0x05 },
	{ 0x69c3, 0x3c },
	{ 0x69c4, 0x03 },
	{ 0x69c5, 0x94 },
	{ 0x69c6, 0x05 },
	{ 0x69c7, 0x03 },
	{ 0x69c8, 0x00 },
	{ 0x69c9, 0x03 },
	{ 0x69ca, 0x05 },
	{ 0x69cb, 0x12 },
	{ 0x69cc, 0x00 },
	{ 0x69cd, 0x00 },
	{ 0x69ce, 0x05 },
	{ 0x69cf, 0x30 },
	{ 0x69d0, 0x00 },
	{ 0x69d1, 0x01 },
	{ 0x69d2, 0x05 },
	{ 0x69d3, 0x34 },
	{ 0x69d4, 0x05 },
	{ 0x69d5, 0x3c },
	{ 0x69d6, 0x03 },
	{ 0x69d7, 0x93 },
	{ 0x69d8, 0x05 },
	{ 0x69d9, 0x03 },
	{ 0x69da, 0x00 },
	{ 0x69db, 0x01 },
	{ 0x69dc, 0x05 },
	{ 0x69dd, 0x12 },
	{ 0x69de, 0x00 },
	{ 0x69df, 0x00 },
	{ 0x69e0, 0x05 },
	{ 0x69e1, 0x30 },
	{ 0x69e2, 0x00 },
	{ 0x69e3, 0x01 },
	{ 0x69e4, 0x05 },
	{ 0x69e5, 0x34 },
	{ 0x69e6, 0x05 },
	{ 0x69e7, 0x3c },
	{ 0x69e8, 0x03 },
	{ 0x69e9, 0x92 },
	{ 0x69ea, 0x05 },
	{ 0x69eb, 0x03 },
	{ 0x69ec, 0x00 },
	{ 0x69ed, 0x01 },
	{ 0x69ee, 0x05 },
	{ 0x69ef, 0x12 },
	{ 0x69f0, 0x00 },
	{ 0x69f1, 0x00 },
	{ 0x69f2, 0x05 },
	{ 0x69f3, 0x30 },
	{ 0x69f4, 0x00 },
	{ 0x69f5, 0x01 },
	{ 0x69f6, 0x05 },
	{ 0x69f7, 0x34 },
	{ 0x69f8, 0x05 },
	{ 0x69f9, 0x3c },
	{ 0x69fa, 0x03 },
	{ 0x69fb, 0x91 },
	{ 0x69fc, 0x05 },
	{ 0x69fd, 0x03 },
	{ 0x69fe, 0x00 },
	{ 0x69ff, 0x01 },
	{ 0x6a00, 0x05 },
	{ 0x6a01, 0x12 },
	{ 0x6a02, 0x00 },
	{ 0x6a03, 0x00 },
	{ 0x6a04, 0x05 },
	{ 0x6a05, 0x30 },
	{ 0x6a06, 0x00 },
	{ 0x6a07, 0x01 },
	{ 0x6a08, 0x05 },
	{ 0x6a09, 0x34 },
	{ 0x6a0a, 0x05 },
	{ 0x6a0b, 0x3c },
	{ 0x6a0c, 0x03 },
	{ 0x6a0d, 0x90 },
	{ 0x6a0e, 0x05 },
	{ 0x6a0f, 0x03 },
	{ 0x6a10, 0x00 },
	{ 0x6a11, 0x01 },
	{ 0x6a12, 0x05 },
	{ 0x6a13, 0x12 },
	{ 0x6a14, 0x00 },
	{ 0x6a15, 0x00 },
	{ 0x6a16, 0x05 },
	{ 0x6a17, 0x30 },
	{ 0x6a18, 0x00 },
	{ 0x6a19, 0x01 },
	{ 0x6a1a, 0x05 },
	{ 0x6a1b, 0x34 },
	{ 0x6a1c, 0x05 },
	{ 0x6a1d, 0x3c },
	{ 0x6a1e, 0x02 },
	{ 0x6a1f, 0x90 },
	{ 0x6a20, 0x05 },
	{ 0x6a21, 0x03 },
	{ 0x6a22, 0x00 },
	{ 0x6a23, 0xff },
	{ 0x6a24, 0x00 },
	{ 0x6a25, 0xff },
	{ 0x6a26, 0x00 },
	{ 0x6a27, 0xff },
	{ 0x6a28, 0x00 },
	{ 0x6a29, 0xff },
	{ 0x6a2a, 0x00 },
	{ 0x6a2b, 0xff },
	{ 0x3199, 0x60 },
	{ 0x3197, 0x04 },
	{ 0x319b, 0x80 },
	{ 0x319d, 0x60 },
	{ 0x31a7, 0x07 },
	{ 0x31a8, 0x30 },
	{ 0x3231, 0xb6 },
	{ 0x3012, 0x01 },
};

#endif