/********************************************************************************
 *
 *  Copyright (C) 2017 	NEXTCHIP Inc. All rights reserved.
 *  Module		: i2c.c
 *  Description	:
 *  Author		:
 *  Date         :
 *  Version		: Version 1.0
 *
 ********************************************************************************
 *  History      :
 *
 *
 ********************************************************************************/
#include <linux/string.h>
#include <linux/delay.h>
#include <linux/i2c.h>
#include <linux/i2c-dev.h>

extern struct i2c_client* jaguar1_client;

void __I2CWriteByte8(unsigned char chip_addr, unsigned char reg_addr, unsigned char value)
{
	i2c_smbus_write_byte_data(jaguar1_client, reg_addr, value);
}

unsigned char __I2CReadByte8(unsigned char chip_addr, unsigned char reg_addr)
{
	return i2c_smbus_read_byte_data(jaguar1_client, reg_addr);
}
