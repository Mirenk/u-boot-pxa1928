/*
 * Copyright (C) 2014
 * SANYO Techno Solutions Tottori Co., Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __LC709203F_H__
#define __LC709203F_H__

#ifndef CONFIG_POWER_LC709203F
#error CONFIG_POWER_LC709203F is not defined.
#endif /* !CONFIG_POWER_LC709203F */

#define LC709203F_I2C_ADDR			0x0B

#define LC709203F_REG_THERM_B		0x06
#define LC709203F_REG_INIT_RSOC		0x07
#define LC709203F_REG_CELL_VOLTAGE	0x09
#define LC709203F_REG_APA			0x0B
#define LC709203F_REG_RSOC			0x0D
#define LC709203F_REG_VERSION		0x11
#define LC709203F_REG_MODE			0x15
#define LC709203F_REG_THERM_STATUS	0x16

#define LC709203F_NORMAL_MODE		0x0001

extern int lc709203f_reg_read(struct pmic *p, u8 addr, u16 *data);
extern int lc709203f_reg_write(struct pmic *p, u8 addr, u16 data);

extern int lc709203f_fg_alloc(unsigned char bus, struct pmic_chip_desc *chip);

#endif /* __LC70923F_H__ */
