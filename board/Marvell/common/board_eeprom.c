/*
 * board_eeprom.c read board eeprom content
 *
 * Copyright (C) 2013 MARVELL Corporation
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <malloc.h>
#include <eeprom_34xx02.h>
#include <board_eeprom.h>

/* board_rev=3 - only EEPROM2 exists (e.g. 1936)
   board_rev=2 - both EEPROM1 & EEPROM2 exists (e.g. 1928)
   board_rev=1 - only EEPROM1 exists - not supported anymore */

/* fields content: board_rev, eeprom1 index, eeprom1 i2c num, eeprom 2 index, eeprom 2 i2c num */
static int board_config_table[BOARD_CONFIG_TOTAL][5] = {
		{2, 1, 0, 2, 4},/*unknown - legacy*/
		{2, 1, 0, 2, 4},/*1928*/
		{3, 0, 0, 2, 0},/*1936*/
		{3, 0, 0, 2, 0} /*1956*/
		};


int read_board_eeprom_info(enum BOARD_CONFIG board_config, struct board_eeprom_info *board_eeprom)
{
	int board_rev;
	u8 board_info[64];
	u8 layout_version[8];
	u8 empty_pattern[4] = {0xFF, 0xFF, 0xFF, 0xFF};
	struct eeprom_data eeprom_data;

	/* Get board info from eeprom */
	printf("\nRead board info from EEPROM:\n");
	memset(board_eeprom, 0, sizeof(struct board_eeprom_info));

	board_rev = board_config_table[board_config][0];

	/* read board_type from EEPROM1 */
	if (board_rev <= 2) {
		printf("EEPROM1 info:\n");
		eeprom_data.index = board_config_table[board_config][1];
		eeprom_data.i2c_num = board_config_table[board_config][2];
		if (eeprom_get_board_type(&eeprom_data, &board_eeprom->board_type)) {
			printf("ERROR read from EEPROM1\n");
			return -1;
		}
	}

	/* read all other fields from EEPROM2 */
	if (board_rev >= 2) {
		printf("EEPROM2 info:\n");
		eeprom_data.index = board_config_table[board_config][3];
		eeprom_data.i2c_num = board_config_table[board_config][4];

		/* get serial_number (if SH Legacy or new layout then use it as is) */
		eeprom_get_board_sn(&eeprom_data, (u8 *)board_eeprom->board_sn);

		/* make sure serial number is burned */
		if (strlen(board_eeprom->board_sn)) {
			/* read board prefix: "EDN"-PTK or "PXA"-SH */
			if (eeprom_get_board_category(&eeprom_data, board_info)) {
				printf("ERROR read from EERPOM2\n");
				return -1;
			}

			/* check layout version - based on content:
			   if(address[0x00]-address[0x02] == "PXA") then this is SH Legacy:
				   serial_number = board_sn field as is
			   else if(address[0x54]-address[0x57] == {0xFF, 0xFF, 0xFF, 0xFF}
				   then is is PTK Legacy:
				   serial_number = board_sn and combined with 4 digits
				   from address[0x0C]-address[0x0F]
			   else then this is the 'new common' layout
				   serial_number = board_sn field as is
			*/

			if (strncmp((const char *)board_info, "PXA", 3)) {
				/* no 'PXA' -> new layout or PTK Legacy */
				char info_sn[8];

				eeprom_get_eeprom_layout_version(&eeprom_data, layout_version);
				if (memcmp(layout_version, empty_pattern, 4) == 0) {
					/* PTK Legacy */
					/* remove last digit-make sure it's after the 'N' */
					if (board_eeprom->board_sn[strlen(board_eeprom->board_sn)-2]
					    == 'N')
						board_eeprom->board_sn[strlen(
						board_eeprom->board_sn) - 1] = 0;

					eeprom_get_board_info_serial_number(&eeprom_data,
									    (u8 *)info_sn);
					info_sn[4] = 0;/* null termination */

					strcat(board_eeprom->board_sn, info_sn);
				}
			}
			eeprom_get_platform_name(&eeprom_data, board_info);
			eeprom_get_chip_name(&eeprom_data, board_info);
			eeprom_get_chip_stepping(&eeprom_data, board_info);
			eeprom_get_board_reg_date(&eeprom_data);
			eeprom_get_board_state(&eeprom_data);
			eeprom_get_user_team(&eeprom_data);
			eeprom_get_current_user(&eeprom_data);
			eeprom_get_board_eco(&eeprom_data, board_eeprom->board_eco);
			eeprom_get_lcd_resolution(&eeprom_data, board_info);
			eeprom_get_lcd_screen_size(&eeprom_data, board_info);
			eeprom_get_ddr_type(&eeprom_data, board_info);
			eeprom_get_ddr_size_speed(&eeprom_data);
			eeprom_get_emmc_type(&eeprom_data, board_info);
			eeprom_get_emmc_size(&eeprom_data, board_info);
			eeprom_get_rf_name(&eeprom_data, (u8 *)board_eeprom->rf_name);
			eeprom_get_rf_info(&eeprom_data, board_eeprom->rf_info);

			printf("Board full serial number :%s\n", board_eeprom->board_sn);
		} else {
			printf("Board EEPROM2 is empty (or serial number empty)\n");
		}
	}

	printf("\n");

	return 0;
}
