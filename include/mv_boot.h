/*
 * (C) Copyright 2014
 * Marvell Semiconductor <www.marvell.com>
 * Written-by: Xiang Wang <wangx@marvell.com>
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __MV_BOOT_H
#define __MV_BOOT_H

/* Functions */
void image_load_notify(unsigned long load_addr);
void image_flash_notify(unsigned long load_addr);

int is_boot_from_sdcard(void);
const char * get_mmc_boot_dev(void);

#endif /* __MV_BOOT_H */
