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
#include <power/marvell88pm_pmic.h>

#include "bq24160.h"

static int bq24160_reg_read(struct pmic *p, u8 addr, u8 *data)
{
	int ret;
	unsigned int i2c_bus_bak = i2c_get_bus_num();
	int count = 0;

	debug("bq24160_reg_read: bus %d hw 0x%02x addr 0x%02x\n",
		p->bus ,p->hw.i2c.addr, addr);

	ret = i2c_set_bus_num(p->bus);
	if (ret != 0) {
		goto cleanup;
	}

loop:
	ret = i2c_read(p->hw.i2c.addr, addr, 1, data, 1);
	if (ret != 0) {
		printf("%s: read error !! (addr=0x%02x)\n", p->name, addr);
		if(++count > 5)
			goto cleanup;
		else
			goto loop;
	}

cleanup:
	i2c_set_bus_num(i2c_bus_bak);

	return ret;
}

static int bq24160_reg_write(struct pmic *p, u8 addr, u8 data)
{
	int ret;
	unsigned int i2c_bus_bak = i2c_get_bus_num();

	debug("bq24160_reg_write: bus %d hw 0x%02x addr 0x%02x data 0x%02x\n",
		p->bus ,p->hw.i2c.addr, addr ,data);

	ret = i2c_set_bus_num(p->bus);
	if (ret != 0) {
		goto cleanup;
	}
	ret = i2c_write(p->hw.i2c.addr, addr, 1, &data, 1);
	if (ret != 0) {
		printf("%s: write error !! (addr=0x%02x, data=0x%02x)\n",
		       p->name, addr, data);
		goto cleanup;
	}

cleanup:
	i2c_set_bus_num(i2c_bus_bak);
	return ret;
}

static int bq24160_charger_type(struct pmic *p)
{
	int ret;
	u8 val;
	int value;
	int type = CHARGER_NO;

	ret = bq24160_reg_read(p, BQ24160_STATUS_CONTROL, &val);
	if (ret != 0) {
		puts("Can't find bq24160_charger\n");
		return -1;
	}

	debug("BQ24160_STATUS_CONTROL: 0x%02x\n", val);

	value = (val >> 4) & 0x07;

	switch(value)
	{
		case 0:
			break;
		case 1:
			type = CHARGER_TA;
			break;
		case 2:
			type = CHARGER_USB;
			break;
		case 3:
			type = CHARGER_TA;
			break;
		case 4:
			type = CHARGER_USB;
			break;
		case 5:
			type = CHARGER_TA;
			break;
		case 6:
			break;
		case 7:
			break;
		default:
			break;
	}

	return type;
}

static int bq24160_charger_bat_present(struct pmic *p)
{
	int ret;
	u8 val,val_ctr;

	ret = bq24160_reg_read(p, BQ24160_CONTROL, &val);
	if (ret != 0) {
		puts("Can't find bq24160_charger\n");
		return ret;
	}

	val |= 0x02;
	val &=~ 0x80;
	ret = bq24160_reg_write(p, BQ24160_CONTROL, val);
	if (ret != 0) {
		puts("Can't find bq24160_charger\n");
		return ret;
	}

	ret = bq24160_reg_read(p, BQ24160_BATTERY_SUPPLY_STATUS, &val);
	if (ret != 0) {
		puts("Can't find bq24160_charger\n");
		return CHARGER_NO;
	}

	debug("BQ24160_BATTERY_SUPPLY_STATUS: 0x%02x\n", val);

	if(((val & 0x06) >> 1) == 0x02)
	{
		debug("Battery Not Present\n");

		ret = bq24160_reg_read(p, BQ24160_CONTROL, &val_ctr);
		if (ret != 0) {
			puts("Can't find bq24160_charger\n");
			return ret;
		}

		val_ctr &=~ 0x80;

		ret = bq24160_reg_write(p, BQ24160_CONTROL, val_ctr | 0x02);
		if (ret != 0) {
			puts("Can't find bq24160_charger\n");
			return ret;
		}

		ret = bq24160_reg_write(p, BQ24160_BATTERY_SUPPLY_STATUS, val | 0x01);
		if (ret != 0) {
			puts("Can't find bq24160_charger\n");
			return ret;
		}

		return 0;
	}
	else
	{
		debug("Battery Present\n");
		ret = bq24160_reg_read(p, BQ24160_CONTROL, &val);
		if (ret != 0) {
			puts("Can't find bq24160_charger\n");
			return ret;
		}

		val &= ~0x82;

		ret = bq24160_reg_write(p, BQ24160_CONTROL, val);
		if (ret != 0) {
			puts("Can't find bq24160_charger\n");
			return ret;
		}

		return 1;
	}
}

static int bq24160_charger_state(struct pmic *p, int state, int current)
{
	int ret;
	u8 val;

	ret = bq24160_reg_read(p, BQ24160_CONTROL, &val);
	if (ret != 0) {
		puts("Can't find bq24160_charger\n");
		return ret;
	}

	val &=~ 0x80;

	if(state == CHARGER_DISABLE)
	{
		val |= 0x02;
	}
	else if(state == CHARGER_ENABLE)
	{
		val &= ~0x02;
	}

	ret = bq24160_reg_write(p, BQ24160_CONTROL, val);
	if (ret != 0) {
		puts("Can't find bq24160_charger\n");
		return ret;
	}

	return 0;
}

static void bq24160_charger_init(struct pmic *p)
{
	int ret;
	u8 val;

	ret = bq24160_reg_read(p, BQ24160_V_P_R, &val);
	if (ret != 0) {
		puts("Can't find bq24160_charger\n");
		return;
	}
	debug("BQ24160_V_P_R: 0x%02x\n", val);

	ret = bq24160_reg_read(p, BQ24160_STATUS_CONTROL, &val);
	if (ret != 0) {
		puts("Can't find bq24160_charger\n");
		return;
	}
	debug("BQ24160_STATUS_CONTROL: 0x%02x\n", val);

	ret = bq24160_reg_read(p, BQ24160_BATTERY_VOLTAGE, &val);
	if (ret != 0) {
		puts("Can't find bq24160_charger\n");
		return;
	}
	debug("BQ24160_BATTERY_VOLTAGE: 0x%02x\n", val);

//	if (val != 0xa4) {/*4.32*/
	if (val != 0x98) {/*4.26*/
//	if (val != 0x9a) {/*4.26 2.5A*/
		puts("bq24160_charger set initial value\n");
//		ret = bq24160_reg_write(p, BQ24160_BATTERY_VOLTAGE, 0xa4);
		ret = bq24160_reg_write(p, BQ24160_BATTERY_VOLTAGE, 0x98);
//		ret = bq24160_reg_write(p, BQ24160_BATTERY_VOLTAGE, 0x9a);
		if (ret != 0) {
			puts("Can't find bq24160_charger\n");
			return;
		}
	}
	ret = bq24160_reg_read(p, BQ24160_B_T_F_C, &val);
	if (ret != 0) {
		puts("Can't find bq24160_charger\n");
		return;
	}
	debug("BQ24160_B_T_F_C: 0x%02x\n", val);

	if (val != 0x52) {/*1.3A*/
		puts("bq24160_charger set initial value\n");
		ret = bq24160_reg_write(p, BQ24160_B_T_F_C, 0x52);
		if (ret != 0) {
			puts("Can't find bq24160_charger\n");
			return;
		}
	}
}

static struct power_chrg power_chrg_ops = {
	.chrg_type = bq24160_charger_type,
	.chrg_bat_present = bq24160_charger_bat_present,
	.chrg_state = bq24160_charger_state,
};

int bq24160_chrg_alloc(unsigned char bus, struct pmic_chip_desc *chip)
{
	static const char name[] = BQ24160_CHRG_NAME;
	struct pmic *p = pmic_alloc();

	if (!p) {
		printf("%s: POWER allocation error!\n", __func__);
		return -ENOMEM;
	}

	debug("Board Charger init\n");

	if (!chip) {
		printf("%s: PMIC chip is empty.\n", __func__);
		return -EINVAL;
	}
	chip->charger_name = name;

	p->name = name;
	p->interface = PMIC_I2C;
	p->number_of_regs = 0xFF;
	p->hw.i2c.addr = BQ24160_I2C_ADDR;
	p->hw.i2c.tx_num = 1;
	p->bus = bus;

	p->chrg = &power_chrg_ops;

	bq24160_charger_init(p);

	return 0;
}
