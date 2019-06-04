/*
 * Copyright (C) 2014
 * SANYO Techno Solutions Tottori Co., Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

//#define DEBUG

#include <common.h>
#include <errno.h>
#include <power/pmic.h>
#include <power/battery.h>
#include <power/marvell88pm_pmic.h>

#include "lc709203f.h"
#include "bq24160.h"

static struct battery ma01_battery;

#ifdef CONFIG_POWER_LC709203F

#define MA01_LC709203F_THERM_B	0x109A
#define MA01_LC709203F_APA		0x00C8

static int ma01_lc709203f_check(struct pmic *p)
{
	int ret;
	u16 value;

	ret = lc709203f_reg_read(p, LC709203F_REG_MODE, &value);
	if (ret != 0)
	{
		return -1;
	}
	else if(value != LC709203F_NORMAL_MODE) {
		return 1;
	}
	ret = lc709203f_reg_read(p, LC709203F_REG_THERM_B, &value);
	if ((ret != 0) || (value != MA01_LC709203F_THERM_B)) {
		return 1;
	}
	ret = lc709203f_reg_read(p, LC709203F_REG_APA, &value);
	if ((ret != 0) || (value != MA01_LC709203F_APA)) {
		return 1;
	}

	return 0;
}

static int ma01_lc709203f_init(struct pmic *p)
{
	int ret = 0;

	ret += lc709203f_reg_write(p, LC709203F_REG_MODE, LC709203F_NORMAL_MODE);
	ret += lc709203f_reg_write(p, LC709203F_REG_INIT_RSOC, 0xAA55);
	ret += lc709203f_reg_write(p, LC709203F_REG_THERM_B, MA01_LC709203F_THERM_B);
	ret += lc709203f_reg_write(p, LC709203F_REG_APA, MA01_LC709203F_APA);
	ret += lc709203f_reg_write(p, LC709203F_REG_THERM_STATUS, 0x0001);

	return ret;
}

#endif /* CONFIG_POWER_LC709203F */

static int ma01_battery_init(struct pmic *bat,
							 struct pmic *fg,
							 struct pmic *chrg,
							 struct pmic *muic)
{
	int ret;

	if (0 != strcmp(fg->name, MA01_BATTERY_FG)) {
		debug("%s: Invalid fuel gauge \"%s\"\n", bat->name, fg->name);
		return -1;
	}
	if (0 != strcmp(chrg->name, MA01_BATTERY_CHRG)) {
		debug("%s: Invalid charger \"%s\"\n", bat->name, chrg->name);
		return -1;
	}

	bat->pbat->fg = fg;
	bat->pbat->chrg = chrg;

	bat->fg = fg->fg;
	bat->chrg = chrg->chrg;

#ifdef CONFIG_POWER_LC709203F
	ret = ma01_lc709203f_check(fg);
	if(ret == -1)
	{
		debug("%s: No BAT found.\n", bat->name);
		return 0;
	}
	if (ret != 0) {
		/* Initialize */
		if (-1 == bat->chrg->chrg_type(chrg)) {
			debug("%s:  charger IC Not found.\n", bat->name);
			return -1;
		}
		if (0 == bat->chrg->chrg_type(chrg)) {
			debug("%s: No charger found.\n", bat->name);
			return -1;
		}
		ret = bat->chrg->chrg_state(chrg, CHARGER_DISABLE, 0);
		if (ret != 0) {
			debug("%s: Cannot stop charge.\n", bat->name);
			return -1;
		}
		udelay(2000000);

		ret = ma01_lc709203f_init(fg);
		if (ret != 0) {
			debug("%s: init failed.\n", bat->name);
			return -1;
		}
		ret = bat->chrg->chrg_state(chrg, CHARGER_ENABLE, 0);
		if (ret != 0) {
			debug("%s: Cannot stop charge.\n", bat->name);
			return -1;
		}
		udelay(500000);
	}
#endif /* CONFIG_POWER_LC709203F */

	return 0;
}

static int ma01_battery_charge(struct pmic *bat, unsigned int voltage)
{
	/* TODO */
	return 0;
}

static struct power_battery ma01_power_bat = {
	.bat = &ma01_battery,
	.battery_init = ma01_battery_init,
	.battery_charge = ma01_battery_charge,
};

int ma01_bat_alloc(unsigned char bus, struct pmic_chip_desc *chip)
{
	static const char name[] = "MA01_BATTERY";
	struct pmic *p = pmic_alloc();

	if (!p) {
		printf("%s: POWER allocation error!\n", __func__);
		return -ENOMEM;
	}

	debug("Board BAT init\n");

	if (!chip) {
		printf("%s: PMIC chip is empty.\n", __func__);
		return -EINVAL;
	}
	chip->battery_name = name;

	p->interface = PMIC_NONE;
	p->name = name;
	p->bus = bus;

	p->pbat = &ma01_power_bat;
	return 0;
}
