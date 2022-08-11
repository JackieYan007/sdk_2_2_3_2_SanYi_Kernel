/* SPDX-License-Identifier: GPL-2.0-or-later */
/*
 * imx424 sensor config for BST Cameras Driver
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
#ifndef __IMX424_CONFIG_H__
#define __IMX424_CONFIG_H__

struct imx424_sensor_base_cfg {
	uint16_t reg;
	uint8_t value;
};

static struct imx424_sensor_base_cfg sensor_base_settings[] = {
	{ 0x0090, 0x16 },
	{ 0x1004, 0x4E },
	{ 0x1005, 0x0C },
	{ 0x1021, 0x3E },
	{ 0x1022, 0x0C },
	{ 0x102C, 0x87 },
	{ 0x102D, 0x00 },
	{ 0x102E, 0x3F },
	{ 0x102F, 0x00 },
	{ 0x1030, 0x3F },
	{ 0x1031, 0x00 },
	{ 0x1032, 0x1F },
	{ 0x1033, 0x01 },
	{ 0x1034, 0x47 },
	{ 0x1035, 0x00 },
	{ 0x1036, 0x77 },
	{ 0x1037, 0x00 },
	{ 0x1038, 0x47 },
	{ 0x1039, 0x00 },
	{ 0x103A, 0x6F },
	{ 0x103B, 0x00 },
	{ 0x103C, 0x37 },
	{ 0x103D, 0x00 },
	{ 0x1040, 0xB0 },
	{ 0x1041, 0x77 },
	{ 0x1042, 0x0A },
	{ 0x1044, 0x20 },
	{ 0x1045, 0xFA },
	{ 0x1046, 0x05 },
	{ 0x1047, 0x01 },
	{ 0x1048, 0xD0 },
	{ 0x1049, 0x39 },
	{ 0x104A, 0xB8 },
	{ 0x104B, 0x5F },
	{ 0x104D, 0x75 },
	{ 0x104E, 0x00 },
	{ 0x1050, 0xC8 },
	{ 0x1051, 0x00 },
	{ 0x2005, 0x04 },
	{ 0x2006, 0x00 },
	{ 0x2008, 0x04 },
	{ 0x2009, 0x00 },
	{ 0x200A, 0x04 },
	{ 0x200B, 0x00 },
	{ 0x200C, 0x04 },
	{ 0x200D, 0x00 },
	{ 0x200E, 0x04 },
	{ 0x200F, 0x00 },
	{ 0x2010, 0x04 },
	{ 0x2011, 0x00 },
	{ 0x201D, 0xFF },
	{ 0x201E, 0x1F },
	{ 0x2020, 0xFF },
	{ 0x2021, 0x1F },
	{ 0x2022, 0xFF },
	{ 0x2023, 0x1F },
	{ 0x2024, 0xFF },
	{ 0x2025, 0x1F },
	{ 0x202B, 0x01 },
	{ 0x2047, 0x01 },
	{ 0x2056, 0x0B },
	{ 0x2057, 0x0B },
	{ 0x207A, 0x09 },
	{ 0x207C, 0x00 },
	{ 0x207D, 0x00 },
	{ 0x2080, 0x03 },
	{ 0x2087, 0xB9 },
	{ 0x2088, 0xDC },
	{ 0x2089, 0x00 },
	{ 0x208A, 0x1F },
	{ 0x20F9, 0x08 },
	{ 0x2108, 0x00 },
	{ 0x2109, 0x00 },
	{ 0x224A, 0x01 },
	{ 0x224C, 0x14 },
	{ 0x224D, 0x00 },
	{ 0x2254, 0x42 },
	{ 0x2255, 0x00 },
	{ 0x2258, 0x40 },
	{ 0x2259, 0x00 },
	{ 0x225E, 0x3E },
	{ 0x225F, 0x00 },
	{ 0x2265, 0x00 },
	{ 0x2266, 0x08 },
	{ 0x2268, 0xFB },
	{ 0x2269, 0x0B },
	{ 0x226A, 0x77 },
	{ 0x226B, 0x0D },
	{ 0x226C, 0x00 },
	{ 0x226E, 0x01 },
	{ 0x2270, 0x03 },
	{ 0x2290, 0x7E },
	{ 0x2291, 0x0F },
	{ 0x2292, 0x01 },
	{ 0x2294, 0x1E },
	{ 0x2295, 0x1B },
	{ 0x2299, 0x08 },
	{ 0x229A, 0x08 },
	{ 0x229B, 0x08 },
	{ 0x229C, 0x01 },
	{ 0x229D, 0x03 },
	{ 0x229E, 0x07 },
	{ 0x229F, 0x0A },
	{ 0x22A0, 0x0C },
	{ 0x22A1, 0x0E },
	{ 0x22A2, 0x10 },
	{ 0x22EC, 0x0E },
	{ 0x22ED, 0x01 },
	{ 0x2330, 0x48 },
	{ 0x2331, 0x00 },
	{ 0x2342, 0x48 },
	{ 0x2343, 0x00 },
	{ 0x2348, 0x4D },
	{ 0x2349, 0x00 },
	{ 0x246F, 0x06 },
	{ 0x2470, 0x09 },
	{ 0x2471, 0x05 },
	{ 0x2475, 0x0A },
	{ 0x2477, 0x05 },
	{ 0x249A, 0x04 },
	{ 0x249C, 0x05 },
	{ 0x24A0, 0x00 },
	{ 0x24A2, 0x00 },
	{ 0x24A6, 0x0C },
	{ 0x24B0, 0x03 },
	{ 0x24B1, 0x03 },
	{ 0x24B4, 0x07 },
	{ 0x24B5, 0x07 },
	{ 0x3002, 0x03 },
	{ 0x3023, 0x0A },
	{ 0x3024, 0xBF },
	{ 0x3025, 0x07 },
	{ 0x3026, 0x87 },
	{ 0x3028, 0xBF },
	{ 0x3029, 0x0B },
	{ 0x302A, 0xAF },
	{ 0x302C, 0x07 },
	{ 0x302D, 0x0D },
	{ 0x3032, 0x0A },
	{ 0x3034, 0xCC },
	{ 0x3035, 0x07 },
	{ 0x3036, 0x87 },
	{ 0x3038, 0xBB },
	{ 0x3039, 0x0B },
	{ 0x303A, 0xAF },
	{ 0x303C, 0xFD },
	{ 0x303D, 0x0C },
	{ 0x3044, 0xF5 },
	{ 0x3045, 0x03 },
	{ 0x3046, 0x7E },
	{ 0x3047, 0x02 },
	{ 0x3048, 0x27 },
	{ 0x3049, 0x0C },
	{ 0x304C, 0x4B },
	{ 0x304D, 0x14 },
	{ 0x304E, 0xA0 },
	{ 0x304F, 0x00 },
	{ 0x3050, 0x68 },
	{ 0x3051, 0x16 },
	{ 0x3052, 0xFE },
	{ 0x3053, 0x0F },
	{ 0x3054, 0x42 },
	{ 0x3055, 0x04 },
	{ 0x3056, 0xE6 },
	{ 0x3057, 0x01 },
	{ 0x3058, 0x68 },
	{ 0x3059, 0x16 },
	{ 0x305A, 0xFA },
	{ 0x305B, 0x0A },
	{ 0x30CF, 0x00 },
	{ 0x405C, 0xAF },
	{ 0x405D, 0x0A },
	{ 0x6203, 0x30 },
	{ 0x6204, 0x00 },
	{ 0x6603, 0x30 },
	{ 0x6604, 0x00 },
	{ 0x7007, 0x55 },
	{ 0x4030, 0x79 },
	{ 0x4031, 0x00 },
	{ 0x4032, 0x0F },
	{ 0x4034, 0x73 },
	{ 0x4035, 0x00 },
	{ 0x4036, 0x1F },
	{ 0x4038, 0x73 },
	{ 0x4039, 0x00 },
	{ 0x403A, 0x1F },
	{ 0x403C, 0x56 },
	{ 0x403D, 0x00 },
	{ 0x403E, 0x18 },
	{ 0x4040, 0xAC },
	{ 0x4041, 0x00 },
	{ 0x4042, 0x2F },
	{ 0x4044, 0x73 },
	{ 0x4045, 0x00 },
	{ 0x4046, 0x1F },
	{ 0x4048, 0x73 },
	{ 0x4049, 0x00 },
	{ 0x404A, 0x1F },
	{ 0x404C, 0xF4 },
	{ 0x404D, 0x00 },
	{ 0x404E, 0x19 },
	{ 0x4050, 0x55 },
	{ 0x4051, 0x00 },
	{ 0x4052, 0x18 },
	{ 0x4054, 0x4D },
	{ 0x4055, 0x00 },
	{ 0x4056, 0x15 },
	{ 0x4058, 0x79 },
	{ 0x4059, 0x00 },
	{ 0x405A, 0x0F },
	{ 0x6202, 0x00 },
	{ 0x6203, 0x30 },
	{ 0x6602, 0x00 },
	{ 0x6603, 0x30 },
	{ 0x0014, 0x00 },
	{ 0x0015, 0x07 },
	{ 0x0016, 0x00 },
	{ 0x0018, 0x70 },
	{ 0x0019, 0x00 },
	{ 0x001A, 0x00 },
	{ 0x001C, 0x07 },
	{ 0x001D, 0x00 },
	{ 0x001E, 0x00 },
	{ 0x0020, 0x00 },
	{ 0x0021, 0x00 },
	{ 0x0022, 0x00 },
	{ 0x0023, 0x00 },
	{ 0x0024, 0x00 },
	{ 0x0025, 0x00 },
	{ 0x8010, 0x00 },
	{ 0x8011, 0x05 },
	{ 0x8012, 0x00 },
	{ 0x8013, 0x07 },
	{ 0x8014, 0x00 },
	{ 0x8015, 0x0A },
	{ 0x8016, 0x00 },
	{ 0x8017, 0x0E },
	{ 0x8018, 0x00 },
	{ 0x8019, 0x05 },
	{ 0x801A, 0x00 },
	{ 0x801B, 0x07 },
	{ 0x801C, 0x00 },
	{ 0x801D, 0x0F },
	{ 0x801E, 0x00 },
	{ 0x801F, 0x15 },
	{ 0x8020, 0x00 },
	{ 0x8021, 0x05 },
	{ 0x8022, 0x00 },
	{ 0x8023, 0x07 },
	{ 0x8024, 0x00 },
	{ 0x8025, 0x0A },
	{ 0x8026, 0x00 },
	{ 0x8027, 0x0E },
	{ 0x8028, 0x00 },
	{ 0x8029, 0x05 },
	{ 0x802A, 0x00 },
	{ 0x802B, 0x07 },
	{ 0x802C, 0x00 },
	{ 0x802D, 0x0F },
	{ 0x802E, 0x00 },
	{ 0x802F, 0x15 },
	{ 0x803D, 0x00 },
	{ 0x803E, 0x00 },
	{ 0x8040, 0xF0 },
	{ 0x8041, 0x00 },
	{ 0x8042, 0x00 },
	{ 0x8044, 0xF0 },
	{ 0x8045, 0x00 },
	{ 0x8048, 0xA0 },
	{ 0x8049, 0x0F },
	{ 0x804A, 0x00 },
	{ 0x804C, 0xA0 },
	{ 0x804D, 0x0F },
	{ 0x8050, 0x00 },
	{ 0x8051, 0xFA },
	{ 0x8052, 0x00 },
	{ 0x8054, 0x46 },
	{ 0x8055, 0x1E },
	{ 0x8058, 0x00 },
	{ 0x8059, 0xFF },
	{ 0x805A, 0x0F },
	{ 0x805C, 0x0F },
	{ 0x805D, 0x3F },
	{ 0x8060, 0x00 },
	{ 0x8061, 0xFF },
	{ 0x8062, 0x0F },
	{ 0x8064, 0x0F },
	{ 0x8065, 0x3F },
	{ 0x8068, 0x00 },
	{ 0x8069, 0xFF },
	{ 0x806A, 0x0F },
	{ 0x806C, 0x0F },
	{ 0x806D, 0x3F },
	{ 0x8070, 0x00 },
	{ 0x8071, 0xFF },
	{ 0x8072, 0x0F },
	{ 0x8074, 0x0F },
	{ 0x8075, 0x3F },
	{ 0x8078, 0x00 },
	{ 0x8079, 0xFF },
	{ 0x807A, 0x0F },
	{ 0x807C, 0x0F },
	{ 0x807D, 0x3F },
	{ 0x8080, 0x00 },
	{ 0x8081, 0xFF },
	{ 0x8082, 0x0F },
	{ 0x8084, 0x0F },
	{ 0x8085, 0x3F },
	{ 0x8088, 0x00 },
	{ 0x8089, 0xFF },
	{ 0x808A, 0x0F },
	{ 0x808C, 0x0F },
	{ 0x808D, 0x3F },
	{ 0x8090, 0x00 },
	{ 0x8091, 0xFF },
	{ 0x8092, 0x0F },
	{ 0x8094, 0x0F },
	{ 0x8095, 0x3F },
	{ 0x8098, 0x00 },
	{ 0x8099, 0xFF },
	{ 0x809A, 0x0F },
	{ 0x809C, 0x0F },
	{ 0x809D, 0x3F },
	{ 0x80A0, 0x00 },
	{ 0x80A1, 0xFF },
	{ 0x80A2, 0x0F },
	{ 0x80A4, 0x0F },
	{ 0x80A5, 0x3F },
	{ 0x80A8, 0x00 },
	{ 0x80A9, 0xFF },
	{ 0x80AA, 0x0F },
	{ 0x80AC, 0x0F },
	{ 0x80AD, 0x3F },
	{ 0x80B0, 0x00 },
	{ 0x80B1, 0xFF },
	{ 0x80B2, 0x0F },
	{ 0x80B4, 0x0F },
	{ 0x80B5, 0x3F },
	{ 0x80B8, 0x00 },
	{ 0x80B9, 0xFF },
	{ 0x80BA, 0x0F },
	{ 0x80BC, 0x0F },
	{ 0x80BD, 0x3F },
	{ 0x80C0, 0x00 },
	{ 0x80C1, 0xFF },
	{ 0x80C2, 0x0F },
	{ 0x80C4, 0x0F },
	{ 0x80C5, 0x3F },
	{ 0x80C8, 0x00 },
	{ 0x80C9, 0xFF },
	{ 0x80CA, 0x0F },
	{ 0x80CC, 0x0F },
	{ 0x80CD, 0x3F },
	{ 0x80D0, 0x00 },
	{ 0x80D1, 0xFF },
	{ 0x80D2, 0x0F },
	{ 0x80D4, 0x0F },
	{ 0x80D5, 0x3F },
	{ 0x80D8, 0x00 },
	{ 0x80D9, 0xFF },
	{ 0x80DA, 0x0F },
	{ 0x80DC, 0x0F },
	{ 0x80DD, 0x3F },
	{ 0x80E0, 0x00 },
	{ 0x80E1, 0xFF },
	{ 0x80E2, 0x0F },
	{ 0x80E4, 0x0F },
	{ 0x80E5, 0x3F },
	{ 0x80E8, 0x00 },
	{ 0x80E9, 0xFF },
	{ 0x80EA, 0x0F },
	{ 0x80EC, 0x0F },
	{ 0x80ED, 0x3F },
	{ 0x80F0, 0x00 },
	{ 0x80F1, 0xFF },
	{ 0x80F2, 0x0F },
	{ 0x80F4, 0x0F },
	{ 0x80F5, 0x3F },
	{ 0x80F8, 0x00 },
	{ 0x80F9, 0xFF },
	{ 0x80FA, 0x0F },
	{ 0x80FC, 0x0F },
	{ 0x80FD, 0x3F },
	{ 0x8100, 0x00 },
	{ 0x8101, 0xFF },
	{ 0x8102, 0x0F },
	{ 0x8104, 0x0F },
	{ 0x8105, 0x3F },
	{ 0x8108, 0x00 },
	{ 0x8109, 0xFF },
	{ 0x810A, 0x0F },
	{ 0x810C, 0x0F },
	{ 0x810D, 0x3F },
	{ 0x8110, 0x00 },
	{ 0x8111, 0xFF },
	{ 0x8112, 0x0F },
	{ 0x8114, 0x0F },
	{ 0x8115, 0x3F },
	{ 0x8118, 0x00 },
	{ 0x8119, 0xFF },
	{ 0x811A, 0x0F },
	{ 0x811C, 0x0F },
	{ 0x811D, 0x3F },
	{ 0x8120, 0x00 },
	{ 0x8121, 0xFF },
	{ 0x8122, 0x0F },
	{ 0x8124, 0x0F },
	{ 0x8125, 0x3F },
	{ 0x8128, 0x00 },
	{ 0x8129, 0xFF },
	{ 0x812A, 0x0F },
	{ 0x812C, 0x0F },
	{ 0x812D, 0x3F },
	{ 0x8130, 0x00 },
	{ 0x8131, 0xFF },
	{ 0x8132, 0x0F },
	{ 0x8134, 0x0F },
	{ 0x8135, 0x3F },
	{ 0x8138, 0x00 },
	{ 0x8139, 0xFF },
	{ 0x813A, 0x0F },
	{ 0x813C, 0x0F },
	{ 0x813D, 0x3F },
	{ 0x8140, 0x00 },
	{ 0x8141, 0xFF },
	{ 0x8142, 0x0F },
	{ 0x8144, 0x0F },
	{ 0x8145, 0x3F },
	{ 0x814A, 0x6C },
	{ 0x814B, 0x07 },
	{ 0x814C, 0xA0 },
	{ 0x814D, 0x0F },
	{ 0x814E, 0xD8 },
	{ 0x814F, 0x0E },
	{ 0x8150, 0x40 },
	{ 0x8151, 0x1F },
	{ 0x8152, 0x6C },
	{ 0x8153, 0x07 },
	{ 0x8154, 0xA0 },
	{ 0x8155, 0x0F },
	{ 0x8156, 0x44 },
	{ 0x8157, 0x16 },
	{ 0x8158, 0xE0 },
	{ 0x8159, 0x2E },
	{ 0x815A, 0x98 },
	{ 0x815B, 0x08 },
	{ 0x815C, 0xA0 },
	{ 0x815D, 0x0F },
	{ 0x815E, 0x30 },
	{ 0x815F, 0x11 },
	{ 0x8160, 0x40 },
	{ 0x8161, 0x1F },
	{ 0x8162, 0x6C },
	{ 0x8163, 0x07 },
	{ 0x8164, 0xA0 },
	{ 0x8165, 0x0F },
	{ 0x8166, 0x44 },
	{ 0x8167, 0x16 },
	{ 0x8168, 0xE0 },
	{ 0x8169, 0x2E },
	{ 0x8180, 0xFF },
	{ 0x8181, 0x0F },
	{ 0x8182, 0xFE },
	{ 0x8183, 0x1F },
	{ 0x8184, 0xFF },
	{ 0x8185, 0x0F },
	{ 0x8186, 0xFD },
	{ 0x8187, 0x2F },
	{ 0x8188, 0xFF },
	{ 0x8189, 0x0F },
	{ 0x818A, 0xFE },
	{ 0x818B, 0x1F },
	{ 0x818C, 0xFF },
	{ 0x818D, 0x0F },
	{ 0x818E, 0xFD },
	{ 0x818F, 0x2F },
	{ 0x8190, 0xFF },
	{ 0x8191, 0x0F },
	{ 0x8192, 0xFE },
	{ 0x8193, 0x1F },
	{ 0x8194, 0xFF },
	{ 0x8195, 0x0F },
	{ 0x8196, 0xFD },
	{ 0x8197, 0x2F },
	{ 0x8198, 0x08 },
	{ 0x8199, 0x07 },
	{ 0x819A, 0x10 },
	{ 0x819B, 0x0E },
	{ 0x819C, 0x08 },
	{ 0x819D, 0x07 },
	{ 0x819E, 0x18 },
	{ 0x819F, 0x15 },
	{ 0x81A0, 0x6C },
	{ 0x81A1, 0x07 },
	{ 0x81A2, 0xD8 },
	{ 0x81A3, 0x0E },
	{ 0x81A4, 0x6C },
	{ 0x81A5, 0x07 },
	{ 0x81A6, 0x44 },
	{ 0x81A7, 0x16 },
	{ 0x81A8, 0x98 },
	{ 0x81A9, 0x08 },
	{ 0x81AA, 0xA0 },
	{ 0x81AB, 0x0F },
	{ 0x81AC, 0x30 },
	{ 0x81AD, 0x11 },
	{ 0x81AE, 0x40 },
	{ 0x81AF, 0x1F },
	{ 0x81B0, 0x98 },
	{ 0x81B1, 0x08 },
	{ 0x81B2, 0xA0 },
	{ 0x81B3, 0x0F },
	{ 0x81B4, 0xC8 },
	{ 0x81B5, 0x19 },
	{ 0x81B6, 0xE0 },
	{ 0x81B7, 0x2E },
	{ 0x81B8, 0xFC },
	{ 0x81B9, 0x08 },
	{ 0x81BA, 0xA0 },
	{ 0x81BB, 0x0F },
	{ 0x81BC, 0xF8 },
	{ 0x81BD, 0x11 },
	{ 0x81BE, 0x40 },
	{ 0x81BF, 0x1F },
	{ 0x81C0, 0xFC },
	{ 0x81C1, 0x08 },
	{ 0x81C2, 0xA0 },
	{ 0x81C3, 0x0F },
	{ 0x81C4, 0xF4 },
	{ 0x81C5, 0x1A },
	{ 0x81C6, 0xE0 },
	{ 0x81C7, 0x2E },
	{ 0x81C8, 0x00 },
	{ 0x81C9, 0x06 },
	{ 0x81CA, 0x00 },
	{ 0x81CB, 0x08 },
	{ 0x81CC, 0x00 },
	{ 0x81CD, 0x0C },
	{ 0x81CE, 0x00 },
	{ 0x81CF, 0x10 },
	{ 0x81D0, 0x00 },
	{ 0x81D1, 0x06 },
	{ 0x81D2, 0x00 },
	{ 0x81D3, 0x08 },
	{ 0x81D4, 0x00 },
	{ 0x81D5, 0x12 },
	{ 0x81D6, 0x00 },
	{ 0x81D7, 0x18 },
	{ 0x0034, 0x80 },
	{ 0x8000, 0x00 },
	{ 0x8001, 0x04 },
	{ 0x8002, 0x00 },
	{ 0x8003, 0x04 },
	{ 0x803C, 0x00 },
	{ 0x8030, 0x01 },
	{ 0x803A, 0x01 },
	{ 0x803B, 0x01 },
	{ 0x406D, 0x2D },
	{ 0x7004, 0x01 },
	{ 0x7005, 0x00 },
	{ 0x7006, 0x01 },
	{ 0x7009, 0xFF },
	{ 0x700A, 0x3F },
	{ 0x0000, 0x00 },
};

#endif