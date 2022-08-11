/* Copyright (c) 2018 Black Sesame Technologies Inc. ALL RIGHTS RESERVED.
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in
 * all copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN
 * THE SOFTWARE.
 */

/*!
 * @file    lwnn_rt_fw_kapi.h
 *
 * @brief   This file contains the type and constant definitions of API between
 *          the runtime firmware and the kernel driver.
 *
 * @author  Shichen Lin
 */

#ifndef LWNN_RT_FW_KAPI_H
#define LWNN_RT_FW_KAPI_H

#define BST_LWNN_INIT_START              1
#define BST_LWNN_INIT_END                2

#pragma pack(push)  /* push current alignment to stack */
#pragma pack(4)     /* set alignment to 4 byte boundary */

struct bst_lwnn_fw_ver_info {
  uint8_t month;
  uint8_t date;
  uint16_t year;
  uint8_t major;
  uint8_t minor;
  uint8_t patch;
};

struct bst_lwnn_rt_init {
  uint32_t init_status; // the status field to indicate the handshake progress
  //from arm
  dsp_ptr assigned_mem; // the start address of assigned memory 
  uint32_t assigned_mem_size; // the size of assigned memory
  uint32_t dest_core_id;  // the IPC destination core id of the ARM core
  uint32_t src_core_id; // the IPC source core id of the DSP
  dsp_ptr ipc_register_addr; // the IPC buffer array address
  //to arm
  struct bst_lwnn_fw_ver_info ver_info; // firmware version information
};

#pragma pack(pop)

#endif