/*
 * board_eeprom.h read board eeprom content header
 *
 * Copyright (C) 2013 MARVELL Corporation
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#define EEPROM_BOARD_SN_LEN	   32
#define EEPROM_BOARD_ECO_LEN   16
#define EEPROM_RF_NAME_LEN     8
#define EEPROM_RF_INFO_LEN     16
/*
 * RF Info field in eeprom are as follows:
 * Address 0xE8: rf_name - 8 bytes, ASCII, not null terminated
 * Address 0xF0: rf_info 16 bytes as follows:
 * 0x00 - 0x07: RF_VERSION, ASCII
 * 0x08 - 0x09: RF_GEOGRAPHY, ASCII (EU,NA,CN,LA,JA...)
 * 0x0A - 0x0B: RF_BAND, TBD
 * 0x0C - 0x0F: future use, TBD
*/
#define RF_NAME_LEN         8
#define RF_VERSION_LEN      8
#define RF_GEOGRAPHY_LEN    2
#define RF_TBD_LEN          6
#define RF_INFO_LEN         (RF_VERSION_LEN + RF_GEOGRAPHY_LEN + RF_TBD_LEN)

struct board_eeprom_info {
	u8 board_type;
	char board_sn[EEPROM_BOARD_SN_LEN + 1];
	u8 board_eco[EEPROM_BOARD_ECO_LEN];
	char rf_name[EEPROM_RF_NAME_LEN + 1];
	u8 rf_info[EEPROM_RF_INFO_LEN];
};

enum BOARD_CONFIG {
	BOARD_CONFIG_UNKNOWN = 0,
	BOARD_CONFIG_1928,
	BOARD_CONFIG_1936,
	BOARD_CONFIG_1956,
	BOARD_CONFIG_TOTAL
};

int read_board_eeprom_info(enum BOARD_CONFIG board_config, struct board_eeprom_info *board_info);
