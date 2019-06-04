/*
 * Copyright (C) 2014-2015
 * SANYO Techno Solutions Tottori Co., Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <i2c.h>
#include <asm/gpio.h>

#include "vx5b1d.h"

#define TAG "VX5B1D"

#define VX5B1D_I2C_BUS		4	/* TWSI5 */
#define VX5B1D_I2C_SLAVE	0x64
#define VX5B1D_RESET_PIN	121

static int power_type;

/*******************************************************************************
 * VX5B1D Initialization Sequence
 ******************************************************************************/
static const uint32_t vx5b1d_reg_table[] =
{
0x700, 0x34900040,
0x704, 0x102ED,
0x70C, 0x00004604,
0x710, 0x2C0000B,
0x714, 0x00,
0x718, 0x00000102,
0x71C, 0xA8002F,
0x720, 0x0,
0x154, 0x00000000,
0x154, 0x80000000,
0x700, 0x34900040,
//0x70C, 0x00005E6E,
0x70C, 0x00005E42,
0x718, 0x00000202,
0x154, 0x00000000,
0x154, 0x80000000,
0x120, 0x5,
0x124, 0x13990500,
0x128, 0x102803,
0x12C, 0x27,
//0x124, 0x14190500,
//0x128, 0x302803,
//0x12C, 0x28,
0x130, 0x3C18,
0x134, 0x00000005,
//0x138, 0xFF8000,
0x138, 0xDD8000,
//0x13C, 0x100,
0x13C, 0x1000100,
0x140, 0x10000,
//0x20C, 0x2,
0x20C, 0x1A,
//0x20C, 0x3A,
0x21C, 0x514,
//0x224, 0x0,
0x224, 0x1,
0x228, 0x320000,
0x22C, 0xFF08,
0x230, 0x1,
0x234, 0xCA033E10,
0x238, 0x00000060,
0x23C, 0x82E86030,
0x244, 0x001E0285,
0x258, 0xA004E,
0x158, 0x0,
0x158, 0x1,
0x37C, 0x00001063,
0x380, 0x82A86030,
0x384, 0x2861408B,
0x388, 0x00130285,
0x38C, 0x10630009,
0x394, 0x400B82A8,
0x600, 0x16CC78C,
0x604, 0x3FFFFC00,
0x608, 0x9EC,
0x154, 0x00000000,
0x154, 0x80000000,
};

static const uint32_t vx5b1d_reg_table_p_spi[] =
{
0x700 , 0x34900070 ,
0x704 , 0x10310 ,
0x70C , 0x00004600 ,
0x710 , 0x024F000F ,
0x714 , 0x0 ,
0x718 , 0x00000102 ,
0x71C , 0x98282F ,
0x720 , 0x0 ,
0x154 , 0x00000000 ,
0x154 , 0x80000000 ,
0x700 , 0x34900070 ,
0x70C , 0x00005E62 ,
0x718 , 0x00000202 ,
0x154 , 0x00000000 ,
0x154 , 0x80000000 ,
0x120 , 0x5 ,
0x124 , 0x5280320 ,
0x128 , 0x104015 ,
0x12C , 0x3B ,
0x130 , 0x3C18 ,
0x134 , 0x00000005 ,
0x138 , 0x998000 ,
0x13C , 0x61000000 ,
0x140 , 0x10000 ,
0x20C , 0x1A ,
0x21C , 0x514 ,
0x224 , 0x0 ,
0x228 , 0x320000 ,
0x22C , 0xFF09 ,
0x230 , 0x1 ,
0x234 , 0xCA033E10 ,
0x238 , 0x00000060 ,
0x23C , 0x82E86030 ,
0x240 , 0x2861409C ,//7
0x244 , 0x001E0785 ,//7
//0x244 , 0x001E0285 ,
0x258 , 0x10014 ,
0x158 , 0x0 ,
0x158 , 0x1 ,
0x374 , 0xEA033A10 , // Charge Pump 5uA
0x37C , 0x00001063 ,
0x380 , 0x82A86030 ,
0x384 , 0x2861408B ,
0x388 , 0x00130285 ,
0x38C , 0x10630009 ,
0x394 , 0x400B82A8 ,
0x600 , 0x16CC78C ,
0x604 , 0x3FFFFFE0 ,
0x608 , 0x9EC ,
0x154 , 0x00000000 ,
0x154 , 0x80000000 ,
};

static int vx5b1d_reg_read16(uint16_t addr, uint16_t *data)
{
	int ret;
	uint8_t buf[5];
	unsigned int i2c_bus_bak = i2c_get_bus_num();

	buf[0] = 0x0E;	/* I2C Direct Access Read */
	buf[1] = (uint8_t)(addr >> 8);
	buf[2] = (uint8_t)(addr);
	buf[3] = 0x00;
	buf[4] = 0x02;

	ret = i2c_set_bus_num(VX5B1D_I2C_BUS);
	if (ret != 0)
		goto cleanup;
	ret = i2c_write(VX5B1D_I2C_SLAVE, 0, 0, buf, sizeof(buf));
	if (ret != 0)
		goto cleanup;
	ret = i2c_read(VX5B1D_I2C_SLAVE, 0, 0, buf, 2);
	if (ret != 0)
		goto cleanup;
	*data = (buf[1] << 8) | buf[0];

cleanup:
	i2c_set_bus_num(i2c_bus_bak);
	return ret;
}

static int vx5b1d_reg_read32(uint16_t addr, uint32_t *data)
{
	int ret;
	uint8_t buf[5];
	unsigned int i2c_bus_bak = i2c_get_bus_num();

	buf[0] = 0x0E;	/* I2C Direct Access Read */
	buf[1] = (uint8_t)(addr >> 8);
	buf[2] = (uint8_t)(addr);
	buf[3] = 0x00;
	buf[4] = 0x04;

	ret = i2c_set_bus_num(VX5B1D_I2C_BUS);
	if (ret != 0)
		goto cleanup;
	ret = i2c_write(VX5B1D_I2C_SLAVE, 0, 0, buf, sizeof(buf));
	if (ret != 0)
		goto cleanup;
	ret = i2c_read(VX5B1D_I2C_SLAVE, 0, 0, buf, 4);
	if (ret != 0)
		goto cleanup;
	*data = (buf[3] << 24) | (buf[2] << 16) | (buf[1] << 8) | buf[0];

cleanup:
	i2c_set_bus_num(i2c_bus_bak);
	return ret;
}

static int vx5b1d_reg_write16(uint16_t addr, uint32_t data)
{
	int ret;
	uint8_t buf[5];
	unsigned int i2c_bus_bak = i2c_get_bus_num();

	buf[0] = 0x0A;	/* I2C Direct Access Write */
	buf[1] = (uint8_t)(addr >> 8);
	buf[2] = (uint8_t)(addr);
	buf[3] = (uint8_t)(data);
	buf[4] = (uint8_t)(data >> 8);

	ret = i2c_set_bus_num(VX5B1D_I2C_BUS);
	if (ret != 0)
		goto cleanup;
	ret = i2c_write(VX5B1D_I2C_SLAVE, 0, 0, buf, sizeof(buf));

cleanup:
	i2c_set_bus_num(i2c_bus_bak);
	return ret;
}

static int vx5b1d_reg_write32(uint16_t addr, uint32_t data)
{
	int ret;
	uint8_t buf[7];
	unsigned int i2c_bus_bak = i2c_get_bus_num();

	buf[0] = 0x0A;	/* I2C Direct Access Write */
	buf[1] = (uint8_t)(addr >>  8);
	buf[2] = (uint8_t)(addr);
	buf[3] = (uint8_t)(data);
	buf[4] = (uint8_t)(data >>  8);
	buf[5] = (uint8_t)(data >> 16);
	buf[6] = (uint8_t)(data >> 24);

	ret = i2c_set_bus_num(VX5B1D_I2C_BUS);
	if (ret != 0)
		goto cleanup;
	ret = i2c_write(VX5B1D_I2C_SLAVE, 0, 0, buf, sizeof(buf));

cleanup:
	i2c_set_bus_num(i2c_bus_bak);
	return ret;
}

static int vx5b1d_spi_write(uint16_t addr ,uint8_t data)
{
	vx5b1d_reg_write32(0x44, 0xF); //16bit
	vx5b1d_reg_write32(0x20, 0x2000 | ((addr >> 8) & 0x00FF));
	vx5b1d_reg_write32(0x40, 0x1); //tx start
	udelay(10);
	vx5b1d_reg_write32(0x44, 0xF); //16bit
	vx5b1d_reg_write32(0x20, 0x0000 | (addr & 0x00FF));
	vx5b1d_reg_write32(0x40, 0x1); //tx start
	udelay(10);
	vx5b1d_reg_write32(0x44, 0xF); //16bit
	vx5b1d_reg_write32(0x20, 0x4000 | data);
	vx5b1d_reg_write32(0x40, 0x1); //tx start
	udelay(10);
}

/*******************************************************************************
 *
 * Reset IC
 *
 ******************************************************************************/
void vx5b1d_reset(void)
{
	gpio_direction_output(VX5B1D_RESET_PIN, 1);
	mdelay(1);
	gpio_direction_output(VX5B1D_RESET_PIN, 0);
	mdelay(10);
	gpio_direction_output(VX5B1D_RESET_PIN, 1);
}

/*******************************************************************************
 *
 * Initialization
 *
 ******************************************************************************/
int vx5b1d_init(int landscape,int power,int micom)
{
	int i;
	uint32_t ready;

	power_type = power;

	i2c_set_bus_num(VX5B1D_I2C_BUS);

	ready = 0;
	while (!ready) {
		mdelay(1);
		if (0 != vx5b1d_reg_read32(0x200, &ready)) {
			printf(TAG ": i2c read error !!\n");
			return -1;
		}
	}

	if(landscape)
	{
		for (i=0; i<ARRAY_SIZE(vx5b1d_reg_table); i+=2) {
			int ret;
			uint32_t addr = vx5b1d_reg_table[i+0];
			uint32_t data = vx5b1d_reg_table[i+1];
			ret = vx5b1d_reg_write32(addr, data);
			if (ret != 0) {
				printf(TAG ": cannot write register. (seq=%d)\n", i);
				return -1;
			}
		}
	}
	else
	{
		for (i=0; i<ARRAY_SIZE(vx5b1d_reg_table_p_spi); i+=2) {
			int ret;
			uint32_t addr = vx5b1d_reg_table_p_spi[i+0];
			uint32_t data = vx5b1d_reg_table_p_spi[i+1];

			if(micom)
			{
					if(addr == 0x138)
						data = 0xDF800;
					if(addr == 0x13C)
						data = 0x1000000;
					if(addr == 0x140)
						data = 0x1A000;
			}
			ret = vx5b1d_reg_write32(addr, data);
			if (ret != 0) {
				printf(TAG ": cannot write register. (seq=%d)\n", i);
				return -1;
			}
		}

		if(micom == 0)
		{
			/* 0xF602 command send*/
			vx5b1d_spi_write(0xFF00,0xAA);
			vx5b1d_spi_write(0xFF01,0x55);
			vx5b1d_spi_write(0xFF02,0x25);
			vx5b1d_spi_write(0xFF03,0x01);
			vx5b1d_spi_write(0xF602,0x03);
		}
	}

	return 0;
}

/*******************************************************************************
 *
 * PWM control
 *
 ******************************************************************************/
int vx5b1d_pwm_ctrl(int enable, uint32_t freq, uint16_t h_time)
{
	int ret = 0;
	uint32_t data;

	ret = vx5b1d_reg_read32(0x138, &data);
	if (ret != 0) {
		printf(TAG ": PWM configuration failed. (read error)\n");
		return -1;
	}

	if (enable) {
		data &= ~0x200000;	/* GPIO[6]: output */
		ret += vx5b1d_reg_write32(0x138, data);
		ret += vx5b1d_reg_write32(0x160, freq);
		ret += vx5b1d_reg_write32(0x164, h_time);
		ret += vx5b1d_reg_write32(0x15c, 0x05);
		ret += vx5b1d_reg_write32(0x114, 0xc6302);
	}
	else {
		if(power_type)
		{
			data &= ~0x200000;	/* GPIO[6]: output */
			data |=  0x00080;   /* High*/
			ret += vx5b1d_reg_write32(0x138, data);
			ret += vx5b1d_reg_write32(0x15c, 0x01);
			ret += vx5b1d_reg_write32(0x114, 0xc6300);
		}
		else
		{
			data |= 0x200000;	/* GPIO[6]: input */
			ret += vx5b1d_reg_write32(0x138, data);
			ret += vx5b1d_reg_write32(0x15c, 0x01);
			ret += vx5b1d_reg_write32(0x114, 0xc6300);
		}
	}
	ret += vx5b1d_reg_write32(0x154, 0);
	ret += vx5b1d_reg_write32(0x154, 0x80000000);

	if (ret != 0) {
		printf(TAG ": PWM configuration failed. (write error)\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
 *
 * VEE control
 *
 ******************************************************************************/
int vx5b1d_vee_ctrl(int enable, u16 vee)
{
	int ret = 0;
	uint32_t data[2];

	ret += vx5b1d_reg_read32(0x70c, &data[0]);
	ret += vx5b1d_reg_read32(0x710, &data[1]);
	if (ret != 0) {
		printf(TAG ": VEE configuration failed. (write error)\n");
		return -1;
	}

	ret += vx5b1d_reg_write32(0x154, 0);
	ret += vx5b1d_reg_write32(0x174, 0x7f);
	ret += vx5b1d_reg_write32(0x70c, data[0] | 0x10);
	ret += vx5b1d_reg_write32(0x710, data[1] | 0x40);
	ret += vx5b1d_reg_write32(0x154, 0x80000000);

	ret += vx5b1d_reg_write32(0x410, 0x05e50300);
	ret += vx5b1d_reg_write32(0x400, enable? 0x1f07:0x1f06);
	ret += vx5b1d_reg_write16(0x402, vee);
	ret += vx5b1d_reg_write32(0x404, 0x55550003);
	ret += vx5b1d_reg_write32(0x154, 0);
	ret += vx5b1d_reg_write32(0x154, 0x80000000);

	if(power_type) {
		ret += vx5b1d_reg_write32(0x434, 0x55550039);
	}

	if (ret != 0) {
		printf(TAG ": VEE configuration failed. (write error)\n");
		return -1;
	}
	return 0;
}

/*******************************************************************************
 *
 * U-Boot command
 *
 ******************************************************************************/
static int do_vx5_dump(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
static int do_vx5_rd16(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
static int do_vx5_rd32(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
static int do_vx5_wr16(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
static int do_vx5_wr32(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[]);
static cmd_tbl_t cmd_vx5_sub[] = {
	U_BOOT_CMD_MKENT(dump, 2, 0, do_vx5_dump, "", ""),
	U_BOOT_CMD_MKENT(rd16, 4, 0, do_vx5_rd16, "", ""),
	U_BOOT_CMD_MKENT(rd32, 4, 0, do_vx5_rd32, "", ""),
	U_BOOT_CMD_MKENT(wr16, 4, 0, do_vx5_wr16, "", ""),
	U_BOOT_CMD_MKENT(wr32, 4, 0, do_vx5_wr32, "", ""),
};

static int do_vx5(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	cmd_tbl_t *c = find_cmd_tbl(argv[1], &cmd_vx5_sub[0], ARRAY_SIZE(cmd_vx5_sub));
	if (c == NULL) {
		return CMD_RET_USAGE;
	}
	return c->cmd(cmdtp, flag, argc-1, &argv[1]);	
}

U_BOOT_CMD(
	vx5, 4, 1, do_vx5,
	"read/write VX5B1D register(s)",
	    "dump                         - dump registers\n"
	"vx5 rd16 <addr(hex)> [num]       - read register(s) (16-bit)\n"
	"vx5 rd32 <addr(hex)> [num]       - read register(s) (32-bit)\n"
	"vx5 wr16 <addr(hex)> <data(hex)> - write register   (16-bit)\n"
	"vx5 wr32 <addr(hex)> <data(hex)> - write register   (32-bit)\n"
);

static int do_vx5_dump(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	static const struct {
		uint16_t	start;
		uint16_t	end;
		int			align;
	} tbl[] = {
		{ 0x100, 0x174, 4 },	/* Common Registers */
		{ 0x200, 0x258, 4 },	/* MIPI DSI Client Registers */
		{ 0x300, 0x394, 4 },	/* ??? */
		{ 0x400, 0x4FE, 2 },	/* VEE HD+ Registers */
		{ 0x600, 0x608, 4 },	/* LVDS Registers */
		{ 0x700, 0x720, 4 },	/* Clock and Reset Registers */
	};
	unsigned int i;

	for (i=0; i<ARRAY_SIZE(tbl); i++) {
		uint16_t addr = tbl[i].start;
		while (addr <= tbl[i].end) {
			if ((addr & 0x0F) == 0) {
				printf("\n%04x:", addr);
			}
			if (tbl[i].align == 4) {
				uint32_t data;
				int ret = vx5b1d_reg_read32(addr, &data);
				if (ret != 0) {
					printf("read error (addr=0x%03x)\n", addr);
					return 1;
				}
				printf(" %08x", data);
			}
			else {
				uint16_t data;
				int ret = vx5b1d_reg_read16(addr, &data);
				if (ret != 0) {
					printf("read error (addr=0x%03x)\n", addr);
					return 1;
				}
				printf(" %04x", data);
			}
			addr += tbl[i].align;
		}
		puts("\n");
	}

	return 0;
}

static int do_vx5_rd16(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong addr;
	long i, num;

	switch (argc)
	{
	case 2:
		addr = simple_strtoul(argv[1], NULL, 16);
		num = 1;
		break;

	case 3:
		addr = simple_strtoul(argv[1], NULL, 16);
		num = simple_strtol(argv[2], NULL, 0);
		break;

	default:
		return CMD_RET_USAGE;
	}

	addr &= 0xFFC;
	for (i=0; i<num; i++) {
		uint16_t data;
		int ret = vx5b1d_reg_read16((uint16_t)addr, &data);
		if (ret != 0) {
			printf("read error (addr=0x%03lx)\n", addr);
			return 1;
		}
		printf("reg[0x%03lx]=0x%04x\n", addr, data);
		if (((addr + 1) & 0xFF) == 0xFF) {
			break;
		}
		addr += 2;
	}

	return 0;
}

static int do_vx5_rd32(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong addr;
	long i, num;

	switch (argc)
	{
	case 2:
		addr = simple_strtoul(argv[1], NULL, 16);
		num = 1;
		break;

	case 3:
		addr = simple_strtoul(argv[1], NULL, 16);
		num = simple_strtol(argv[2], NULL, 0);
		break;

	default:
		return CMD_RET_USAGE;
	}

	addr &= 0xFFC;
	for (i=0; i<num; i++) {
		uint32_t data;
		int ret = vx5b1d_reg_read32((uint16_t)addr, &data);
		if (ret != 0) {
			printf("read error (addr=0x%03lx)\n", addr);
			return 1;
		}
		printf("reg[0x%03lx]=0x%08x\n", addr, data);
		if (((addr + 3) & 0xFF) == 0xFF) {
			break;
		}
		addr += 4;
	}

	return 0;
}

static int do_vx5_wr16(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong addr, data;
	int ret;

	switch (argc)
	{
	case 3:
		addr = simple_strtoul(argv[1], NULL, 16);
		data = simple_strtoul(argv[2], NULL, 16);
		break;

	default:
		return CMD_RET_USAGE;
	}

	addr &= 0xFFE;
	data &= 0xFFFF;
	ret = vx5b1d_reg_write16((uint16_t)addr, (uint16_t)data);
	if (ret != 0) {
		printf("write error (addr=0x%03lx, data=0x%04lx)\n", addr, data);
		return 1;
	}
	printf("reg[0x%03lx]=0x%04lx\n", addr, data);

	return 0;
}

static int do_vx5_wr32(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	ulong addr, data;
	int ret;

	switch (argc)
	{
	case 3:
		addr = simple_strtoul(argv[1], NULL, 16);
		data = simple_strtoul(argv[2], NULL, 16);
		break;

	default:
		return CMD_RET_USAGE;
	}

	addr &= 0xFFC;
	ret = vx5b1d_reg_write32((uint16_t)addr, data);
	if (ret != 0) {
		printf("write error (addr=0x%03lx, data=0x%08lx)\n", addr, data);
		return 1;
	}
	printf("reg[0x%03lx]=0x%08lx\n", addr, data);

	return 0;
}
