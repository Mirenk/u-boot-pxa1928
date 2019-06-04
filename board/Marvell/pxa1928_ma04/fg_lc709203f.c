/*
 * Copyright (C) 2014
 * SANYO Techno Solutions Tottori Co., Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

//#define DEBUG

#include <common.h>
#include <errno.h>
#include <i2c.h>
#include <power/pmic.h>
#include <power/battery.h>
#include <power/marvell88pm_pmic.h>

#include "lc709203f.h"

static u8 lc709203f_crc8(u8 before, u8 after)
{
	int i;
	u16 tmp = 0;

	tmp = (u16)(before ^ after);
	tmp <<= 8;

	for (i=0; i<8; i++) {
		if (tmp & 0x8000) {
			tmp ^= 0x8380;
		}
		tmp <<= 1;
	}

	return (u8)(tmp >> 8);
}

int lc709203f_reg_read(struct pmic *p, u8 addr, u16 *data)
{
	int ret;
	u8 buf[3];
	u8 crc8;
	unsigned int i2c_bus_bak = i2c_get_bus_num();
	int count = 0;

	*data = 0xFFFF;

	ret = i2c_set_bus_num(p->bus);
	if (ret != 0) {
		goto cleanup;
	}

loop:
	ret = i2c_read(p->hw.i2c.addr, addr, 1, buf, sizeof(buf));
	if (ret != 0) {
		printf("%s: read error !! (addr=0x%02x)\n", p->name, addr);
		if(++count > 5)
			goto cleanup;
		else
			goto loop;
	}

	crc8 = 0;
	crc8 = lc709203f_crc8(crc8, p->hw.i2c.addr << 1);
	crc8 = lc709203f_crc8(crc8, addr);
	crc8 = lc709203f_crc8(crc8, 0x17);
	crc8 = lc709203f_crc8(crc8, buf[0]);
	crc8 = lc709203f_crc8(crc8, buf[1]);
	if (crc8 != buf[2]) {
		printf("%s: CRC error !! (addr=0x%02x)\n", p->name, addr);
		goto cleanup;
	}

	*data = ((u16)buf[1] << 8) | buf[0];

	debug("lc709203f_reg_read: bus %d hw 0x%02x addr 0x%02x data %04x\n", p->bus ,p->hw.i2c.addr, addr ,*data);

cleanup:
	i2c_set_bus_num(i2c_bus_bak);
	return ret;
}

int lc709203f_reg_write(struct pmic *p, u8 addr, u16 data)
{
	int ret;
	u8 buf[3];
	u8 crc8;
	unsigned int i2c_bus_bak = i2c_get_bus_num();

	debug("lc709203f_reg_write: bus %d hw 0x%02x addr 0x%02x data 0x%04x\n", p->bus ,p->hw.i2c.addr, addr ,data);

	buf[0] = data & 0x00ff;
	buf[1] = data >> 8;
	crc8 = 0;
	crc8 = lc709203f_crc8(crc8, p->hw.i2c.addr << 1);
	crc8 = lc709203f_crc8(crc8, addr);
	crc8 = lc709203f_crc8(crc8, buf[0]);
	crc8 = lc709203f_crc8(crc8, buf[1]);
	buf[2] = crc8;

	ret = i2c_set_bus_num(p->bus);
	if (ret != 0) {
		goto cleanup;
	}
	ret = i2c_write(p->hw.i2c.addr, addr, 1, buf, sizeof(buf));
	if (ret != 0) {
		printf("%s: write error !! (addr=0x%02x, data=0x%02x)\n",
		       p->name, addr, data);
		goto cleanup;
	}

cleanup:
	i2c_set_bus_num(i2c_bus_bak);
	return ret;
}

static int lc709203f_update_battery(struct pmic *p, struct pmic *bat)
{
	struct power_battery *pb = bat->pbat;
	u16 val;
	int ret = 0;

	debug("lc709203f_update_battery\n");

	ret = lc709203f_reg_read(p, LC709203F_REG_MODE, &val);
	if ((ret != 0) || (val != LC709203F_NORMAL_MODE)) {
		puts("Can't find lc709203f fuel gauge\n");
		return -1;
	}

	ret |= lc709203f_reg_read(p, LC709203F_REG_CELL_VOLTAGE, &val);
	pb->bat->voltage_uV = val * 1000;
	debug("vcell %dmv\n",val);

	ret |= lc709203f_reg_read(p, LC709203F_REG_RSOC, &val);
	pb->bat->state_of_chrg = val;
	debug("soc   %d%%\n",val);

	return ret;
}

static int lc709203f_check_battery(struct pmic *p, struct pmic *bat)
{
	struct power_battery *pb = bat->pbat;
	u16 val;
	int ret = 0;

	debug("lc709203f_check_battery\n");

	ret = lc709203f_reg_read(p, LC709203F_REG_MODE, &val);
	if ((ret != 0) || (val != LC709203F_NORMAL_MODE)) {
		puts("Can't find lc709203f fuel gauge\n");
		return -1;
	}

	ret = lc709203f_reg_read(p, LC709203F_REG_VERSION, &val);
	pb->bat->version = val;
	debug("fg ver: 0x%x\n", pb->bat->version);

	ret |= lc709203f_reg_read(p, LC709203F_REG_CELL_VOLTAGE, &val);
	pb->bat->voltage_uV = val * 1000;
	printf("vcell %dmv\n",val);

	ret |= lc709203f_reg_read(p, LC709203F_REG_RSOC, &val);
	pb->bat->state_of_chrg = val;
	printf("soc   %d%%\n",val);

	return ret;
}

static struct power_fg lc709203f_fg_ops = {
	.fg_battery_check = lc709203f_check_battery,
	.fg_battery_update = lc709203f_update_battery,
};

int lc709203f_fg_alloc(unsigned char bus, struct pmic_chip_desc *chip)
{
	static const char name[] = "LC709203F_FG";
	struct pmic *p = pmic_alloc();

	if (!p) {
		printf("%s: POWER allocation error!\n", __func__);
		return -ENOMEM;
	}

	debug("Board Fuel Gauge init\n");

	if (!chip) {
		printf("%s: PMIC chip is empty.\n", __func__);
		return -EINVAL;
	}
	chip->fuelgauge_name = name;

	p->name = name;
	p->interface = PMIC_I2C;
	p->number_of_regs = 0xFF;
	p->hw.i2c.addr = LC709203F_I2C_ADDR;
	p->hw.i2c.tx_num = 3;
	p->sensor_byte_order = PMIC_SENSOR_BYTE_ORDER_BIG;
	p->bus = bus;

	p->fg = &lc709203f_fg_ops;
	return 0;
}
