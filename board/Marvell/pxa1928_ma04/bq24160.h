/*
 * Copyright (C) 2014
 * SANYO Techno Solutions Tottori Co., Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __BQ24160_H__
#define __BQ24160_H__

#ifndef CONFIG_POWER_BQ24160
#error CONFIG_POWER_BQ24160 is not defined.
#endif /* !CONFIG_POWER_BQ24160 */

#define BQ24160_CHRG_NAME "BQ24160_CHRG"
#define BQ24160_I2C_ADDR	0x6B

extern int bq24160_chrg_alloc(unsigned char bus, struct pmic_chip_desc *chip);

#define BQ24160_STATUS_CONTROL			0x00
#define BQ24160_BATTERY_SUPPLY_STATUS	0x01
#define BQ24160_CONTROL					0x02
#define BQ24160_BATTERY_VOLTAGE			0x03
#define BQ24160_V_P_R					0x04
#define BQ24160_B_T_F_C					0x05

#define BQ24160_SAFTY_TIMER				0x07

#endif /* __BQ24160_H__ */
