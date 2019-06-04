/*
 * boot.c mrvl boot support
 *
 * (C) Copyright 2013
 * Marvell Semiconductor <www.marvell.com>
 * Written-by: Xiaofan Tian <tianxf@marvell.com>
 *
 * SPDX-License-Identifier:	 GPL-2.0+
 */

#include <common.h>
#include <libfdt.h>
#ifdef CONFIG_OBM_PARAM_ADDR
#include "obm2osl.h"
#include "tim.h"
#endif
#include <mv_boot.h>
#include <mv_recovery.h>
#include "cmdline.h"
#include <android_image.h>
#include <image.h>
#include <mmc.h>

struct recovery_reg_funcs *p_recovery_reg_funcs;
int recovery_key_detect_default(void)
{
	return 0;
}
int recovery_key_detect(void)
	__attribute__((weak, alias("recovery_key_detect_default")));
int recovery_default(int primary_valid, int recovery_valid,
		int magic_key, struct recovery_reg_funcs *funcs)
{
	return 0;
}
int recovery(int primary_valid, int recovery_valid,
		int magic_key, struct recovery_reg_funcs *funcs)
	__attribute__((weak, alias("recovery_default")));

int sdboot_key_detect_default(void)
{
	return 0;
}
int sdboot_key_detect(void)
	__attribute__((weak, alias("sdboot_key_detect_default")));

#ifdef CONFIG_OF_LIBFDT
void handle_dtb_default(struct fdt_header *devtree)
{
}

void handle_dtb(struct fdt_header *devtree)
	__attribute__((weak, alias("handle_dtb_default")));

unsigned int dtb_offset_default(void)
{
	return 0;
}

unsigned int dtb_offset(void)
	__attribute__((weak, alias("dtb_offset_default")));
#endif

#ifdef CONFIG_CMD_MMC
static int get_recovery_flag(void)
{
	int primary_valid = 1, recovery_valid = 1;
	int magic_key;
#ifdef CONFIG_OBM_PARAM_ADDR
	struct OBM2OSL *params = NULL;
#endif

	magic_key = recovery_key_detect();

#ifdef CONFIG_OBM_PARAM_ADDR
	params = (struct OBM2OSL *)(uintptr_t)(*(u32 *)CONFIG_OBM_PARAM_ADDR);

	if (!params || params->signature != OBM2OSL_IDENTIFIER) {
		printf("WARING: no obm parameters !!!\n");
		params = NULL;
	} else {
#if 0
		magic_key |= (params->booting_mode == RECOVERY_MODE);

		primary_valid = params->primary.validation_status;
		recovery_valid = params->recovery.validation_status;
#endif
	}
#endif
	return recovery(primary_valid, recovery_valid,
			magic_key, p_recovery_reg_funcs);
}

static int is_found_bootable_sdcard(void)
{
	struct OBM2OSL *obm2osl = (struct OBM2OSL *)
		(*(unsigned long *)CONFIG_OBM_PARAM_ADDR);
	struct TIM tim;
	int dev = 1;	/* mmc1 */
	struct mmc *mmc;
	static u32 buf[512/4];

	/* check flash signature of the boot device */
	if (set_tim_pointers((unsigned char *)(unsigned long)
				obm2osl->cp.tim_ddr_address, &tim) < 0) {
		printf("%s: get tim pointer error\n", __func__);
		return 0;	/* false */
	}
	if (0x4D4D4307 == tim.ctim->flash_info.boot_flashsign) {
		/* boot from SD card */
		return 1;	/* true */
	}
	/* boot from eMMC */

	/* check SD card slot */
	mmc = find_mmc_device(dev);
	if (mmc) {
		if (0 == mmc_init(mmc)) {
			/* check flash signature of the SD card */
			u32 blk = 9;
			int n = mmc->block_dev.block_read(
				dev, blk, 1, buf);
			if (n != 1) {
				/* read error */
				return 0;	/* false */
			}
			flush_cache((ulong)buf, sizeof(buf));
			if ((buf[1] == 0x54494D48 /* "TIMH" */) &&
			    (buf[5] == 0x4D4D4307 /* "MMC",07h */)) {
				return 2;	/* true SDcard insert */
			}
		}
	}

	return 0;	/* false */
}

__weak const char * get_mmc_boot_dev(void)
{
	return "mmc dev 0 0";
}

static unsigned char load_op;
static void load_image_head(struct andr_img_hdr *ahdr,
		unsigned long base_emmc_addr)
{
	char cmd[128];

	/* copy boot image and uImage header to parse */
	sprintf(cmd, "%s; mmc read %p %lx 0x9", CONFIG_MMC_BOOT_DEV,
		ahdr, base_emmc_addr / 512);
	run_command(cmd, 0);
}

#ifdef CONFIG_OF_LIBFDT
static unsigned long dtb_emmc_lookup(struct andr_img_hdr *ahdr,
		unsigned long base_emmc_addr)
{
	unsigned int zimage_size = 0;
	u8 uhead_buf[16];
	unsigned long dtb_emmc_addr;
	unsigned int kernel_offset;

	/*
	 * DTBs are appended after uImage so need parse uImage header
	 * to get zImage length. Only the first 16 uImage header bytes
	 * are needed for this parse.
	 */
	kernel_offset = ahdr->page_size;
	memcpy(uhead_buf, (char *)ahdr + kernel_offset, 16);
	/*
	 * This zImage_size means the zImage without uboot_head size
	 * and we can get the size value from the uboot_head[12]~[15]
	 */
	zimage_size = (uhead_buf[12] << 24) | (uhead_buf[13] << 16) |
		(uhead_buf[14] << 8) | uhead_buf[15];

	dtb_emmc_addr = IH_HEADER_SIZE + kernel_offset + zimage_size;
	dtb_emmc_addr = ALIGN(dtb_emmc_addr, ahdr->page_size);
	dtb_emmc_addr += base_emmc_addr;
	dtb_emmc_addr += dtb_offset();

	return dtb_emmc_addr;
}
#endif

#define MV_KERNEL_LOADED	(1<<0)
#define MV_RAMDISK_LOADED	(1<<1)
#define MV_DTB_LOADED		(1<<2)
#define MV_DTB_FLASH		(1<<3) /* flash dtb to flash */

void image_load_notify(unsigned long load_addr)
{
	switch (load_addr) {
	case CONFIG_LOADADDR: /* same value as RECOVERY_KERNEL_LOADADDR */
		load_op |= MV_KERNEL_LOADED;
		break;
	case RAMDISK_LOADADDR: /* same value as RECOVERY_RAMDISK_LOADADDR */
		load_op |= MV_RAMDISK_LOADED;
		break;
#ifdef CONFIG_OF_LIBFDT
	case DTB_LOADADDR:
		load_op |= MV_DTB_LOADED;
		break;
#endif
	}
}

void image_flash_notify(unsigned long load_addr)
{
#ifdef CONFIG_OF_LIBFDT
	char cmd[128];
	unsigned long base_emmc_addr, dtb_load_addr, dtb_emmc_addr;
	int recovery_flag = 0;
	struct andr_img_hdr *ahdr =
		(struct andr_img_hdr *)(CONFIG_LOADADDR - 0x1000 - 0x200);

	/*load_op bit[3] flash dtb to mmc */
	if (load_op & MV_DTB_FLASH) {
		recovery_flag = get_recovery_flag();
		base_emmc_addr = recovery_flag ?
			RECOVERYIMG_EMMC_ADDR : BOOTIMG_EMMC_ADDR;

		/* Load DTB */
		dtb_load_addr = load_addr;
		load_image_head(ahdr, base_emmc_addr);
		dtb_emmc_addr = dtb_emmc_lookup(ahdr, base_emmc_addr) / 512;

		printf("write dtb to mmc 0x%lx\n", dtb_emmc_addr);
		sprintf(cmd, "%s; mmc write %lx %lx %x", CONFIG_MMC_BOOT_DEV,
			dtb_load_addr, dtb_emmc_addr, DTB_SIZE / 512);
		run_command(cmd, 0);
	}
#endif
}


int do_mrvlboot(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char cmd[128];
	char cmdline[COMMAND_LINE_SIZE];
	char *bootargs;
	unsigned int kernel_offset = 0, ramdisk_offset = KERNEL_SIZE;
	unsigned int kernel_size = KERNEL_SIZE, ramdisk_size = RAMDISK_SIZE;
	unsigned long base_emmc_addr, kernel_load_addr, ramdisk_load_addr;
	int recovery_flag = 0;
#ifdef CONFIG_OF_LIBFDT
	unsigned long dtb_load_addr, dtb_emmc_addr;
	struct fdt_header *devtree;
#endif

	/* The bootimg/recoveryimg header + uImage header ddr parse address */
	struct andr_img_hdr *ahdr =
		(struct andr_img_hdr *)(CONFIG_LOADADDR - 0x1000 - 0x200);

	recovery_flag = get_recovery_flag();
	/*
	 * OBM doesn't load kernel and ramdisk for nontrusted boot, but it
	 * still passes validation_status and loading_status as 1 to uboot
	 * since uboot is loaded and validated.
	 * Always load the kernel and ramdisk here to solve this problem,
	 * negative impact of a little boot time.
	 */
	base_emmc_addr = recovery_flag ? RECOVERYIMG_EMMC_ADDR : BOOTIMG_EMMC_ADDR;
	load_image_head(ahdr, base_emmc_addr);

	if (!memcmp(ANDR_BOOT_MAGIC, ahdr->magic, ANDR_BOOT_MAGIC_SIZE)) {
		printf("This is Android %s image\n",
		       recovery_flag ? "recovery" : "boot");
		kernel_offset = ahdr->page_size;
		kernel_size = ALIGN(ahdr->kernel_size, ahdr->page_size);
		ramdisk_offset = kernel_offset + kernel_size;
		ramdisk_size = ALIGN(ahdr->ramdisk_size, ahdr->page_size);
	}
	/* calculate mmc block number */
	kernel_offset /= 512;
	ramdisk_offset /= 512;
	kernel_size /= 512;
	ramdisk_size /= 512;

	/* Load kernel */
	kernel_load_addr = CONFIG_LOADADDR;
	if ((load_op & MV_KERNEL_LOADED) == 0) {
		printf("Load kernel to 0x%lx\n", kernel_load_addr);
		sprintf(cmd, "mmc read %lx %lx %x", kernel_load_addr,
			base_emmc_addr / 512 + kernel_offset, kernel_size);
		run_command(cmd, 0);
#ifdef CONFIG_RSA
			#define SUFFIX_LEN 128
			extern int decode_hash_with_image(unsigned char *hash_128byte);
#else
			#define SUFFIX_LEN 32
#endif	
			int mi;
			unsigned char md5_kernel_calc[16];
			unsigned char md5_kernel_with[SUFFIX_LEN+4];
			unsigned char md5_kernel_buf[4];
		
			md5(kernel_load_addr,ahdr->kernel_size-SUFFIX_LEN,md5_kernel_calc);
			memcpy(md5_kernel_with,kernel_load_addr+ahdr->kernel_size-SUFFIX_LEN,SUFFIX_LEN);
#ifdef CONFIG_RSA
			mi=decode_hash_with_image(md5_kernel_with);
			if(mi>0){
				md5_kernel_with[mi]='\0';			
#else
				md5_kernel_with[32]='\0';
#endif
				for(mi=0;mi<16;mi++){
					md5_kernel_buf[0]=md5_kernel_with[mi*2];
					md5_kernel_buf[1]=md5_kernel_with[mi*2+1];
					md5_kernel_buf[2]='\0';
					md5_kernel_with[mi]=simple_strtoul(md5_kernel_buf,NULL,16);
				}
				if(memcmp(md5_kernel_calc,md5_kernel_with,16)){
					printf("\tAuthorization error - stop starting up.\n");
#ifdef CONFIG_STOP_AUTH_ERROR
					return -1;
#endif
				}
				else
					printf("\tThis is the authorized kernel.\n");
#ifdef CONFIG_RSA
			}
			else{
				printf("\tDecode error - stop starting up.\n");
#ifdef CONFIG_STOP_AUTH_ERROR
				return -1;
#endif
			}
#endif
	}

	/* Load ramdisk */
	ramdisk_load_addr = RAMDISK_LOADADDR;
	if ((load_op & MV_RAMDISK_LOADED) == 0) {
		printf("Load ramdisk to 0x%lx\n", ramdisk_load_addr);
		sprintf(cmd, "mmc read %lx %lx %x", ramdisk_load_addr,
			base_emmc_addr / 512 + ramdisk_offset, ramdisk_size);
		run_command(cmd, 0);

#ifdef CONFIG_RSA
			#define SUFFIX_LEN 128
			extern int decode_hash_with_image(unsigned char *hash_128byte);
#else
			#define SUFFIX_LEN 32
#endif
			int mi;
			unsigned char md5_ramdisk_calc[16];
			unsigned char md5_ramdisk_with[SUFFIX_LEN+4];
			unsigned char md5_ramdisk_buf[4];
		
			md5(ramdisk_load_addr,ahdr->ramdisk_size-SUFFIX_LEN,md5_ramdisk_calc);
			memcpy(md5_ramdisk_with,ramdisk_load_addr+ahdr->ramdisk_size-SUFFIX_LEN,SUFFIX_LEN);
#ifdef CONFIG_RSA
			mi=decode_hash_with_image(md5_ramdisk_with);
			if(mi>0){
				md5_ramdisk_with[mi]='\0';
#else
				md5_ramdisk_with[32]='\0';
#endif
				for(mi=0;mi<16;mi++){
					md5_ramdisk_buf[0]=md5_ramdisk_with[mi*2];
					md5_ramdisk_buf[1]=md5_ramdisk_with[mi*2+1];
					md5_ramdisk_buf[2]='\0';
					md5_ramdisk_with[mi]=simple_strtoul(md5_ramdisk_buf,NULL,16);
				}
				if(memcmp(md5_ramdisk_calc,md5_ramdisk_with,16)){
					printf("\tAuthorization error - stop starting up.\n");
#ifdef CONFIG_STOP_AUTH_ERROR
					return -1;
#endif
				}
				else
					printf("\tThis is the authorized ramdisk.\n");
#ifdef CONFIG_RSA
			}
			else{
				printf("\tDecode error - stop starting up.\n");
#ifdef CONFIG_STOP_AUTH_ERROR
				return -1;
#endif
			}
#endif
	}

	/* Modify ramdisk command line */
	bootargs = getenv("bootargs");
	strncpy(cmdline, bootargs, COMMAND_LINE_SIZE);
	remove_cmdline_param(cmdline, "initrd=");
	sprintf(cmdline + strlen(cmdline),  " initrd=0x%lx,10m rw",
		ramdisk_load_addr);
	if (recovery_flag)
		sprintf(cmdline + strlen(cmdline), " recovery=1");
	setenv("bootargs", cmdline);

#ifdef CONFIG_OF_LIBFDT
	/* Load DTB */
	dtb_load_addr = DTB_LOADADDR;
	if ((load_op & MV_DTB_LOADED) == 0) {
		dtb_emmc_addr = dtb_emmc_lookup(ahdr, base_emmc_addr);
		printf("Load dtb to 0x%lx\n", dtb_load_addr);
		sprintf(cmd, "mmc read %lx %lx %x", dtb_load_addr,
			dtb_emmc_addr / 512, DTB_SIZE / 512);
		run_command(cmd, 0);
	}

	devtree = (struct fdt_header *)dtb_load_addr;
	if (be32_to_cpu(devtree->magic) == FDT_MAGIC) {
		handle_dtb(devtree);
		sprintf(cmd, "fdt addr 0x%lx; bootm 0x%lx - 0x%lx",
			dtb_load_addr, kernel_load_addr, dtb_load_addr);
	} else
#endif
		sprintf(cmd, "bootm 0x%lx", kernel_load_addr);
	run_command(cmd, 0);

	return 0;
}

#ifdef CONFIG_OF_LIBFDT
int do_dtb(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char cmd[128];
	unsigned long base_emmc_addr, dtb_load_addr, dtb_emmc_addr;
	int recovery_flag = 0;
	struct andr_img_hdr *ahdr =
		(struct andr_img_hdr *)(CONFIG_LOADADDR - 0x1000 - 0x200);

	if (argc > 3)
		return cmd_usage(cmdtp);

	if (argc == 1) {
		recovery_flag = get_recovery_flag();
		/* Load DTB from emmc */
		dtb_load_addr = DTB_LOADADDR;
		base_emmc_addr = recovery_flag ?
			RECOVERYIMG_EMMC_ADDR : BOOTIMG_EMMC_ADDR;
		load_image_head(ahdr, base_emmc_addr);
		dtb_emmc_addr = dtb_emmc_lookup(ahdr, base_emmc_addr) / 512;
		printf("Load dtb to 0x%lx\n", dtb_load_addr);
		sprintf(cmd, "mmc read %lx %lx %x", dtb_load_addr,
				dtb_emmc_addr, DTB_SIZE / 512);
		run_command(cmd, 0);

		load_op |= MV_DTB_LOADED;
		return 0;
	}
	/*load_op bit[3] flash dtb to mmc */
	if ((argc == 3) && !strcmp(argv[1], "b"))
		load_op |= MV_DTB_FLASH;

	sprintf(cmd, "tftpboot 0x%x %s",
		DTB_LOADADDR, argc == 3 ? argv[2] : argv[1]);
	run_command(cmd, 0);

	return 0;
}

U_BOOT_CMD(
	dtb, 3, 1, do_dtb,
	"Load dtb files through tftp or emmc",
	"Usage:\ndtb  load dtb file from emmc, then mrvlboot sequence would skip dtb load"
	"\ndtb <dtb_file> or dtb d <dtb_filename> to download dtb only"
	"\ndtb b <dtb_filename> to download dtb and flash"
);
#endif
#else
int do_mrvlboot(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	return 0;
}
#endif

U_BOOT_CMD(
	mrvlboot, 1, 1, do_mrvlboot,
	"Do marvell specific boot",
	"Usage:\nmrvlboot"
);
