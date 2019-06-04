/*
 * Copyright (C) 2014-2015
 * SANYO Techno Solutions Tottori Co., Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#include <common.h>
#include <asm/io.h>
#include <asm/arch/cpu.h>
#include <asm/arch/pxa1928.h>
#ifdef CONFIG_SDHCI
#include <sdhci.h>
#endif /* CONFIG_SDHCI */
#include <mvmfp.h>
#include <mv_boot.h>
#include <mv_recovery.h>
#include <pxa_amp.h>
#include <asm/arch/mfp.h>
#include <asm/gpio.h>
#include <malloc.h>
#include <emmd_rsv.h>
#include <power/pmic.h>
#include <power/marvell88pm_pmic.h>
#include <power/pxa1928_freq.h>
#include <asm/gpio.h>
#include <asm/armv8/adtfile.h>
#ifdef CONFIG_OF_LIBFDT
#include <libfdt.h>
#endif /* CONFIG_OF_LIBFDT */
#include "../common/cmdline.h"
#if defined(CONFIG_MMP_DISP)
#include <stdio_dev.h>
#include <mmp_disp.h>
#include <leds-lm3532.h>
#include <leds-88pm828x.h>
#include <video.h>
#include <video_fb.h>
#include <environment.h>
#include "../common/panel.h"
#include "../common/marvell.h"
#include "../common/logo_debugmode.h"

#include "android_logo_landscape.h"
#include "insert_ac_landscape.h"
#include "android_battery_landscape.h"
#include "recovery_landscape.h"
#include "android_logo_portrait.h"
#include "insert_ac_portrait.h"
#include "android_battery_portrait.h"
#include "recovery_portrait.h"
#endif /* CONFIG_MMP_DISP */
#include "../common/obm2osl.h"
#include "../common/mrd_flag.h"
#include "../common/mv_cp.h"

#include "../common/charge.h"

static struct pmic_chip_desc *board_pmic_chip;

DECLARE_GLOBAL_DATA_PTR;

//#define A02 /*A02 setting*/

#include "vx5b1d.h"
#include "bq24160.h"
#include "lc709203f.h"

#define MACH_TYPE_PXA1928	3897

int get_lcd_landscape(void);
static int get_lcd_power(void);
static int get_board_version(void);
static int get_board_micom(void);

static unsigned int revision = 0;

/*
 *  two dvc control register bits can support 4 control level
 *  4(control level) * 4 (DVC level registers) can support 16level DVC
 */
#define DVC_CONTROL_REG		0x4F
#define DVC_SET_ADDR1		(1 << 0)
#define DVC_SET_ADDR2		(1 << 1)
#define DVC_CTRL_LVL		4

#define PMIC_I2C_BUS	0
#define CHG_I2C_BUS 	0
#define FG_I2C_BUS  	0

#define RTC_CLK     	(0xD4015000)
#define RTC_BRN0    	(0xD4010014)
#define FB_MAGIC_NUM	('f'<<24 | 'a'<<16 | 's'<<8 | 't')
#define PD_MAGIC_NUM	('p'<<24 | 'r'<<16 | 'o'<<8 | 'd')

#ifdef CONFIG_RAMDUMP
#include "../common/ramdump.h"

/*
 * Platform memory map definition for ramdump, see ramdump.h
 */
static struct rd_mem_desc rd_mems[] = {
	{ 0, 0 },	/* skip ion area */
	{ 0, 0 },	/* left memory */
};

struct rd_desc ramdump_desc = {
	.rdc_addr = EMMD_INDICATOR + EMMD_RDC_OFFSET,
	.phys_start = CONFIG_TZ_HYPERVISOR_SIZE,
	.mems = &rd_mems[0],
	.nmems = sizeof(rd_mems)/sizeof(rd_mems[0]),
};
#endif /* CONFIG_RAMDUMP */

static int fastboot = 0;
static int battery_fail = 0;

#ifdef CONFIG_REVISION_TAG
static u8 board_rev = 0;
static char board_sn[33];
static u32 ddr_speed;
u32 get_board_rev(void)
{
	return (u32)board_rev;
}

int serialno_read(char* serialno);
#endif /* CONFIG_REVISION_TAG */

unsigned int lvt = 0, nlvt = 0, nsvt = 0, svt = 0;
unsigned int iddq_1p05 = 0, iddq_1p30 = 0;
unsigned int mv_profile = 0xFF;

/*
   Define CPU/DDR default max frequency
   CPU: 1300MHz
   DDR: 528MHz
   GC3D: 624MHz
   GC2D: 312MHz
*/
#define CPU_MAX_FREQ_DEFAULT	1300
#define DDR_MAX_FREQ_DEFAULT	528
#define GC3D_MAX_FREQ_DEFAULT	624
#define GC2D_MAX_FREQ_DEFAULT	312

/*
   Define CPU/DDR max frequency
   CPU: 1508MHz
   DDR:  667MHz (discrete) / 800MHz (pop)
   GC3D: 797MHz
   GC2D: 416MHz
*/
#define CPU_MAX_FREQ		1508
//#define DDR_MAX_FREQ		667
#define DDR_MAX_FREQ		528
#define DDR_MAX_FREQ_POP	800
#define GC3D_MAX_FREQ		797
#define GC2D_MAX_FREQ		416

/*
 * GPIO pins
 */
#define MA01_LCD_POWER_ON	150
#define MA01_LCD_PWR_ON		135
#define MA01_LCD_BACKLIGHT_ON	151
#define MA01_VERSION1	116
#define MA01_VERSION2	117
#define MA01_VERSION3	114
#define MA01_VERSION4	115
#define MA01_LCD_MICOM	195
#define MA01_LCD_ORIENT	196
#define MA01_LCD_POWER	197

#ifdef CONFIG_CHECK_DISCRETE
static void unlock_aib_regs(void)
{
	struct pxa1928apbc_registers *apbc =
		(struct pxa1928apbc_registers *)PXA1928_APBC_BASE;

	writel(FIRST_ACCESS_KEY, &apbc->asfar);
	writel(SECOND_ACCESS_KEY, &apbc->assar);
}
#endif /* CONFIG_CHECK_DISCRETE */

/*
 * when boot kernel: UBOOT take Volume Up key (GPIO160) for recovery magic key
 * when power up: OBM take Volume Up key (GPIO160) for swdownloader
 */
#ifdef A02
static unsigned __attribute__((unused)) recovery_key = 160;
#else
static unsigned __attribute__((unused)) recovery_key = 189;
#endif
static unsigned __attribute__((unused)) sdboot_key = 162;

static int pxa1928_discrete = 0xff;
static int highperf;

static void check_discrete(void)
{
#ifdef CONFIG_CHECK_DISCRETE
	struct pxa1928aib_registers *aib =
		(struct pxa1928aib_registers *)PXA1928_AIB_BASE;

	/* unlock AIB registers */
	unlock_aib_regs();

	/* get discrete or pop info */
	pxa1928_discrete = (readl(&aib->nand) & (1 << 4)) ? 1 : 0;
#endif /* CONFIG_CHECK_DISCRETE */

	printf("PKG:   %s\n", (pxa1928_discrete == 0xff) ?
		"Discrete/PoP not checked on Zx chips" :
		(pxa1928_discrete ? "Discrete" : "PoP"));
}

static int chip_type = PXA1926_2L_DISCRETE;
static void check_chip_type(void)
{
	struct fuse_reg_info arg;

	if (cpu_is_pxa1928_a0()) {
		if (1 == pxa1928_discrete)
			printf("Chip is pxa1928 A0 Discrete\n");
		else
			printf("Chip is pxa1928 A0 POP\n");
		return;
	}
	/* for pxa1928 B0 */
	smc_get_fuse_info(LC_GET_FUSE_INFO_CMD, (void *)&arg);
	chip_type = arg.arg5;
	switch (chip_type) {
	case PXA1926_2L_DISCRETE:
		printf("Chip is PXA1926 B0 2L Discrete\n");
		break;

	case PXA1928_POP:
		printf("Chip is PXA1928 B0 PoP\n");
		break;

	case PXA1928_4L:
		printf("Chip is PXA1928 B0 4L\n");
		break;

	case PXA1928_2L:
		printf("Chip is PXA1928 B0 2L\n");
		break;

	default:
		chip_type = PXA1926_2L_DISCRETE;
		printf("Unknown chip type, " \
		       "set to PXA1926 B0 2L Discrete as default\n");
		break;
	}
	return;
}

#if defined(CONFIG_MMP_DISP)
static struct dsi_info ma01_dsi = {
	.id = 1,
	.lanes = 2,
	.bpp = 24,
	.burst_mode = DSI_BURST_MODE_BURST,
	.rgb_mode = DSI_LCD_INPUT_DATA_RGB_MODE_888,
	.hfp_en = 1,
	.hbp_en = 1,
	.master_mode = 1,
};

static struct lcd_videomode video_wxga_mode = {
	.name = "WXGA",
	.refresh = 60,
	.xres = 1280,
	.yres = 800,
	.pixclock = 14065,
	.left_margin = 48,
	.right_margin = 48,
	.upper_margin = 10,
	.lower_margin = 3,
	.hsync_len = 160,
	.vsync_len = 10,
	.sync = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
};

static struct lcd_videomode video_wxga_mode_p = {
	.name = "WXGA",
	.refresh = 60,
	.xres = 800,
	.yres = 1280,
	.pixclock = 8812,
	.left_margin = 60, // HBP
	.right_margin = 60, // HFP
	.upper_margin = 16,	// VBP
	.lower_margin = 21,	// VFP
	.hsync_len = 20,
	.vsync_len = 1,
	.sync = FB_SYNC_VERT_HIGH_ACT | FB_SYNC_HOR_HIGH_ACT,
};

static void panel_wxga_init_config(struct mmp_disp_info *fbi,int en);
static void lcd_calculate_sclk(struct mmp_disp_info *fb,
		int dsiclk, int pathclk);
static struct mmp_disp_plat_info mmp_mipi_lcd_info = {
	.index = 0,
	.id = "GFX Layer",
	/* FIXME */
	.sclk_src = 416000000,
	.sclk_div = 0x20001106,
	.num_modes = 1,
	.modes = &video_wxga_mode,
	.pix_fmt = PIX_FMT_RGBA888,
	.burst_len = 16,
	/*
	 * don't care about io_pin_allocation_mode and dumb_mode
	 * since the panel is hard connected with lcd panel path
	 * and dsi1 output
	 */
	.panel_rgb_reverse_lanes = 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena = 0,
	.invert_pixclock = 0,
	.panel_rbswap = 0,
	.active = 1,
	.enable_lcd = 1,
	.phy_type = DSI,
	.phy_info = &ma01_dsi,
	.dsi_panel_config = panel_wxga_init_config,
	.update_panel_info = NULL,
	.calculate_sclk = lcd_calculate_sclk,
	.dynamic_detect_panel = 0,
	.panel_manufacturer_id = 0,
};

static struct mmp_disp_plat_info mmp_mipi_lcd_info_p = {
	.index = 0,
	.id = "GFX Layer",
	/* FIXME */
	.sclk_src = 416000000,
	.sclk_div = 0x20001106,
	.num_modes = 1,
	.modes = &video_wxga_mode_p,
	.pix_fmt = PIX_FMT_RGBA888,
	.burst_len = 16,
	/*
	 * don't care about io_pin_allocation_mode and dumb_mode
	 * since the panel is hard connected with lcd panel path
	 * and dsi1 output
	 */
	.panel_rgb_reverse_lanes = 0,
	.invert_composite_blank = 0,
	.invert_pix_val_ena = 0,
	.invert_pixclock = 0,
	.panel_rbswap = 0,
	.active = 1,
	.enable_lcd = 1,
	.phy_type = DSI,
	.phy_info = &ma01_dsi,
	.dsi_panel_config = panel_wxga_init_config,
	.update_panel_info = NULL,
	.calculate_sclk = lcd_calculate_sclk,
	.dynamic_detect_panel = 0,
	.panel_manufacturer_id = 0,
};

static void lcd_power_en(int on)
{
	/* TODO */
	gpio_direction_output(MA01_LCD_POWER_ON, on?1:0);
	gpio_direction_output(MA01_LCD_PWR_ON, on?1:0);
}

static void turn_off_backlight(void)
{
	/* TODO */
	gpio_direction_output(MA01_LCD_BACKLIGHT_ON, 0);
}

static void turn_on_backlight(void)
{
#ifdef CONFIG_VIDEO_VX5B1D
	if(get_lcd_power())
	{
		vx5b1d_pwm_ctrl(1, 870, 0x0276);
		vx5b1d_vee_ctrl(1, 0x4300);
	}
	else
	{
		vx5b1d_pwm_ctrl(1, 0x43F8, 0x10C8);
		vx5b1d_vee_ctrl(1, 0x2C00);
	}
#endif /* CONFIG_VIDEO_VX5B1D */
	gpio_direction_output(MA01_LCD_BACKLIGHT_ON, 1);
}

static void panel_wxga_init_config(struct mmp_disp_info *fbi,int en)
{
	if(en) {
		vx5b1d_reset();
		if (0 != vx5b1d_init(get_lcd_landscape(),get_lcd_power(),get_board_micom()))
			return NULL;
	}
}

#define LCD_PN_SCLK			(0xd420b1a8)
#define PIXEL_SRC_DISP1			(0x1 << 29)
#define PIXEL_SRC_DSI1_PLL		(0x3 << 29)
#define PIXEL_SRC_SEL_MASK		(0x7 << 29)
#define DSI1_BIT_CLK_SRC_DISP1		(0x1 << 12)
#define DSI1_BIT_CLK_SRC_DSI1_PLL	(0x3 << 12)
#define DSI1_BIT_CLK_SRC_SEL_MASK	(0x3 << 12)
#define DSI1_BITCLK_DIV(div)		((div) << 8)
#define DSI1_BITCLK_DIV_MASK		((0xf) << 8)
#define PIXEL_CLK_DIV(div)		(div)
#define PIXEL_CLK_DIV_MASK		(0xff)

#define APMU_DISP_CLKCTRL		(0xd4282984)
#define DISP1_CLK_SRC_SEL_PLL1624	(0 << 12)
#define DISP1_CLK_SRC_SEL_PLL1416	(1 << 12)
#define DISP1_CLK_SRC_SEL_MASK		(7 << 12)
#define DISP1_CLK_DIV(div)		((div) << 8)
#define DISP1_CLK_DIV_MASK		(7 << 8)

#define MHZ_TO_HZ	(1000000)
#define PLL1_416M	(416000000)
#define PLL1_624M	(624000000)
static const int parent_rate_fixed_num = 2;
static const int parent_rate_fixed_tbl[2] = {
	416,
	624,
};

/*******************************************************************************
 *
 * SCLK Calculation
 *
 ******************************************************************************/
static void lcd_calculate_sclk(struct mmp_disp_info *fb,
		int dsiclk, int pathclk)
{
	u32 sclk, apmu, bclk_div = 1, pclk_div;
	int sclk_src, parent_rate, div_tmp;
	int offset, offset_min = dsiclk, i;

	for (i = 0; i < parent_rate_fixed_num; i++) {
		parent_rate = parent_rate_fixed_tbl[i];
		div_tmp = (parent_rate + dsiclk / 2) / dsiclk;
		if (!div_tmp)
			div_tmp = 1;

		offset = abs(parent_rate - dsiclk * div_tmp);

		if (offset < offset_min) {
			offset_min = offset;
			bclk_div = div_tmp;
			sclk_src = parent_rate;
		}
	}

	if (offset_min > 20) {
		/* out of fixed parent rate +-20M */
		fb->mi->sclk_src = dsiclk;
		bclk_div = 1;
	} else {
		fb->mi->sclk_src = sclk_src;
	}

	sclk = readl(LCD_PN_SCLK);
	sclk &= ~(PIXEL_SRC_SEL_MASK | DSI1_BIT_CLK_SRC_SEL_MASK |
			DSI1_BITCLK_DIV_MASK | PIXEL_CLK_DIV_MASK);
	pclk_div = (fb->mi->sclk_src + pathclk / 2) / pathclk;

	sclk = DSI1_BITCLK_DIV(bclk_div) | PIXEL_CLK_DIV(pclk_div);

	fb->mi->sclk_src *= MHZ_TO_HZ;
	apmu = readl(APMU_DISP_CLKCTRL);
	apmu &= ~(DISP1_CLK_SRC_SEL_MASK | DISP1_CLK_DIV_MASK);
	apmu |= DISP1_CLK_DIV(1);
	if (fb->mi->sclk_src == PLL1_416M) {
		apmu |= DISP1_CLK_SRC_SEL_PLL1416;
		sclk |= PIXEL_SRC_DISP1 | DSI1_BIT_CLK_SRC_DISP1;
	} else if (fb->mi->sclk_src == PLL1_624M) {
		apmu |= DISP1_CLK_SRC_SEL_PLL1624;
		sclk |= PIXEL_SRC_DISP1 | DSI1_BIT_CLK_SRC_DISP1;
	} else {
		pxa1928_pll3_set_rate(fb->mi->sclk_src);
		sclk |= PIXEL_SRC_DSI1_PLL | DSI1_BIT_CLK_SRC_DSI1_PLL;
	}

	writel(apmu, APMU_DISP_CLKCTRL);
	fb->mi->sclk_div = sclk;
}

/*******************************************************************************
 *
 * LCD Initialization
 *
 * (call from video_hw_init())
 *
 ******************************************************************************/
static void *lcd_init(void)
{
	void *ret;
	struct mmp_disp_plat_info *mi;
	int landscape;

	landscape = get_lcd_landscape();
	if(landscape)
	{
		mi = &mmp_mipi_lcd_info;
	}
	else
	{
		mi = &mmp_mipi_lcd_info_p;
	}

	turn_off_backlight();
	lcd_power_en(1);

	ret = (void *)mmp_disp_init(mi);

//	turn_on_backlight();
	return ret;
}

struct mmp_disp_info *fbi;
/*******************************************************************************
 *
 * Video Hardware Initialization
 *
 * init_sequence_r[]
 *     --> stdio_init()
 *         --> drv_video_init()
 *             --> video_init()
 *                 --> video_hw_init()
 *
 ******************************************************************************/
void *video_hw_init(void)
{
	static GraphicDevice ctfb;

	struct mmp_disp_plat_info *mi;
	unsigned long t1, hsynch, vsynch;

	if (pxa_is_warm_reset())
		return NULL;

	fbi = lcd_init();
	if (fbi == NULL)
		return NULL;

	mi = fbi->mi;
	ctfb.winSizeX = ALIGN(mi->modes->xres, 16);
	ctfb.winSizeY = mi->modes->yres;

	/* calculate hsynch and vsynch freq (info only) */
	t1 = (mi->modes->left_margin + mi->modes->xres +
	      mi->modes->right_margin + mi->modes->hsync_len) / 8;
	t1 *= 8;
	t1 *= mi->modes->pixclock;
	t1 /= 1000;
	hsynch = 1000000000L / t1;
	t1 *= (mi->modes->upper_margin + mi->modes->yres +
	       mi->modes->lower_margin + mi->modes->vsync_len);
	vsynch = 1000000000L / t1;

	/* fill in Graphic device struct */
	sprintf(ctfb.modeIdent, "%dx%dx%d %ldkHz %ldHz", ctfb.winSizeX,
		ctfb.winSizeY, mi->bits_per_pixel, (hsynch / 1000),
		vsynch);

	ctfb.frameAdrs = (unsigned int)(uintptr_t) fbi->fb_start;
	ctfb.plnSizeX = ctfb.winSizeX;
	ctfb.plnSizeY = ctfb.winSizeY;
	ctfb.gdfBytesPP = 4;
	ctfb.gdfIndex = GDF_32BIT_X888RGB;

	ctfb.isaBase = 0x9000000;
	ctfb.pciBase = (unsigned int)(uintptr_t) fbi->fb_start;
	ctfb.memSize = fbi->fb_size;

	/* Cursor Start Address */
	ctfb.dprBase = (unsigned int) (uintptr_t)fbi->fb_start + (ctfb.winSizeX \
			* ctfb.winSizeY * ctfb.gdfBytesPP);
	if ((ctfb.dprBase & 0x0fff) != 0) {
		/* allign it */
		ctfb.dprBase &= 0xfffff000;
		ctfb.dprBase += 0x00001000;
	}
	ctfb.vprBase = (unsigned int)(uintptr_t) fbi->fb_start;
	ctfb.cprBase = (unsigned int)(uintptr_t) fbi->fb_start;

	return &ctfb;
}

int get_lcd_landscape(void)
{
	int ret,portrait,version;

	version = get_board_version();

	if(version == 0) //WS,A02
	{
		/*GPIO_196*/
		gpio_direction_input(MA01_LCD_ORIENT);

		portrait = gpio_get_value(MA01_LCD_ORIENT);

		if(portrait)
			ret = 0;
		else
			ret = 1;
	}
	else if(version == 1) //PP
	{
		/*GPIO_196*/
		gpio_direction_input(MA01_LCD_ORIENT);

		portrait = gpio_get_value(MA01_LCD_ORIENT);

		if(portrait)
			ret = 0;
		else
			ret = 1;
	}
	else if(version == 2) //PP?
	{
		/*GPIO_196*/
		gpio_direction_input(MA01_LCD_ORIENT);

		portrait = gpio_get_value(MA01_LCD_ORIENT);

		if(portrait)
			ret = 0;
		else
			ret = 1;
	}
	else if(version == 3) //MP
	{
		/*GPIO_196*/
		gpio_direction_input(MA01_LCD_ORIENT);

		portrait = gpio_get_value(MA01_LCD_ORIENT);

		if(portrait)
			ret = 0;
		else
			ret = 1;
	}
	else
	{
		/*GPIO_196*/
		gpio_direction_input(MA01_LCD_ORIENT);

		portrait = gpio_get_value(MA01_LCD_ORIENT);

		if(portrait)
			ret = 0;
		else
			ret = 1;
	}

	/*add revision*/
	{
		unsigned int tmpu,tmpl;
		tmpu = revision / 1000;
		tmpl = (revision % 100);
		revision = (tmpu * 1000) + (!ret * 100) + tmpl;
	}

	return ret;
}

static int get_lcd_power(void)
{
	int ret,power;

	/*GPIO_197*/
	gpio_direction_input(MA01_LCD_POWER);

	power = gpio_get_value(MA01_LCD_POWER);

	if(power)
		ret = 0;
	else
		ret = 1;

	return ret;
}

static int get_board_micom(void)
{
	int ret,micom;

	/*GPIO_195*/
	gpio_direction_input(MA01_LCD_MICOM);

	micom = gpio_get_value(MA01_LCD_MICOM);

	if(micom)
		ret = 0;
	else
		ret = 1;

	return ret;
}

static void show_logo(int type)
{
	char cmd[100];
	char *cmdline;
	struct mmp_disp_plat_info *fb = fbi->mi;
	struct lcd_videomode *fb_mode = fb->modes;
	int landscape = get_lcd_landscape();

	if(type == 0)
	{
		/* Show android logo */
		if(landscape)
		{
			sprintf(cmd,"bmp display_center %p %d %d", POWERED_BY_ANDROID_LANDSCAPE,
				fb_mode->xres, fb_mode->yres);
		}
		else
		{
			sprintf(cmd,"bmp display_center %p %d %d", POWERED_BY_ANDROID_PORTRAIT,
				fb_mode->xres, fb_mode->yres);
		}
	}
	else if(type == 1)
	{
		/* Show android battery logo */
		if(landscape)
		{
			sprintf(cmd, "bmp display_center %p %d %d", ANDROID_BATTERY_LANDSCAPE,
				fb_mode->xres, fb_mode->yres);
		}
		else
		{
			sprintf(cmd, "bmp display_center %p %d %d", ANDROID_BATTERY_PORTRAIT,
				fb_mode->xres, fb_mode->yres);
		}
	}
	else if(type == 2)
	{
		/* Show insert AC logo */
		if(landscape)
		{
			sprintf(cmd, "bmp display_center %p %d %d", INSERT_AC_LANDSCAPE,
				fb_mode->xres, fb_mode->yres);
		}
		else
		{
			sprintf(cmd, "bmp display_center %p %d %d", INSERT_AC_PORTRAIT,
				fb_mode->xres, fb_mode->yres);
		}
	}
	else if(type == 3)
	{
		/* Show insert recovery logo */
		if(landscape)
		{
			sprintf(cmd, "bmp display_center %p %d %d", RECOVERY_BMP_LANDSCAPE,
				fb_mode->xres, fb_mode->yres);
		}
		else
		{
			sprintf(cmd, "bmp display_center %p %d %d", RECOVERY_BMP_PORTRAIT,
				fb_mode->xres, fb_mode->yres);
		}
	}

	run_command(cmd, 0);
	flush_cache(g_disp_start_addr, g_disp_buf_size);

	cmdline = malloc(COMMAND_LINE_SIZE);
	strncpy(cmdline, getenv("bootargs"), COMMAND_LINE_SIZE);

	setenv("bootargs", cmdline);
	free(cmdline);
}

#else  /* !CONFIG_MMP_DISP */
static void show_logo(void) {}
#endif /* !CONFIG_MMP_DISP */

/*******************************************************************************
 *
 * Early Initialization
 * - MFP(Multi Function Pin) Configuration
 *
 * init_sequence_f[]
 *     --> board_early_init_f()
 *
 ******************************************************************************/
int board_early_init_f(void)
{
#ifdef CONFIG_CMD_MFP
	u32 mfp_cfg[] = {
		/* UART3: Debug UART */
		UART3_RXD_MMC2_DAT7_MFP33,
		UART3_TXD_MMC2_DAT6_MFP34,

		/* PWR_I2C */
		PWR_SCL_MFP67,
		PWR_SDA_MFP68,

		/* TWSI6: NFC */
		TWSI6_SCL_MMC2_DAT5_MFP35,
		TWSI6_SDA_MMC2_DAT4_MFP36,

		/* MMC3: eMMC */
		MMC3_DAT0_ND_IO8_MFP87,
		MMC3_DAT1_ND_IO9_MFP86,
		MMC3_DAT2_ND_IO10_MFP85,
		MMC3_DAT3_ND_IO11_MFP84,
		MMC3_DAT4_ND_IO12_MFP83,
		MMC3_DAT5_ND_IO13_MFP82,
		MMC3_DAT6_ND_IO14_MFP81,
		MMC3_DAT7_ND_IO15_MFP80,
		MMC3_CLK_SM_ADVMUX_MFP88,
		MMC3_CMD_SM_RDY_MFP89,
		MMC3_RST_ND_CLE_MFP90,

		/* MMC1: microSD */
		MMC1_DAT0_MFP62,
		MMC1_DAT1_MFP61,
		MMC1_DAT2_MFP60,
		MMC1_DAT3_MFP59,
		MMC1_DAT4_MFP58,
		MMC1_DAT5_MFP57,
		MMC1_DAT6_MFP56,
		MMC1_DAT7_MFP55,
		MMC1_CLK_MFP64,
		MMC1_CMD_MFP63,
		MMC1_WP_MFP66,
		MMC1_CD_ND_NCS1_MFP100,
		GPIO65_MFP65,		/* MMC1_CD is not used */

		/* DVC (Dynamic Voltage Control) pins */
		DVC_PIN0_MFP107,
		DVC_PIN1_MFP108,
		DVC_PIN2_MFP99,
		DVC_PIN3_MFP103,

		/* End of configureation */
		MFP_EOC
	};
	u32 mfp_cfg_discrete[] = {
		/* TWSI2: G-sensor / 9-axis inertial sensor */
		TWSI2_SCL_MFP178,
		TWSI2_SDA_MFP179,

		/* TWSI3: Camera */
		TWSI3_SCL_MFP176,
		TWSI3_SDA_MFP177,

		/* TWSI4: Ambient light sensor */
		TWSI4_SCL_MFP181,
		TWSI4_SDA_MFP180,

		/* TWSI5: Touch panel / Digitizer */
		TWSI5_SCL_MFP174,
		TWSI5_SDA_MFP175,

		/* LCD */
		LCD_RESET_MFP121,	/* VX5_RESET_N */
		LCD_BACKLIGHT_EN_MFP151,
		MFP_REG(0x0FC) | MFP_AF0 | MFP_DRIVE_MEDIUM | MFP_PULL_HIGH,	/* CLK_REQ */

		/* Keys */
		GPIO162_MFP162,		/* KEY_VOL_DOWN */
		GPIO160_MFP160,		/* KEY_VOL_UP */

		/* GPS */
		MFP_REG(0x104) | MFP_AF0 | MFP_DRIVE_MEDIUM | MFP_PULL_HIGH,	/* VCXO_REQ */

		/* Camera */
		MFP_REG(0x280) | MFP_AF3 | MFP_DRIVE_MEDIUM,	/* CAM_MCLK_R */
		MFP_REG(0x1F0) | MFP_AF2 | MFP_DRIVE_MEDIUM,	/* CAM_MCLK_F */

//		/* NFC */
//		GPIO112_MFP112,

		/*Version PIN*/
		(MFP_REG(0x1D8) | MFP_AF0 | MFP_PULL_HIGH), /* GPIO114_MFP114 */
		(MFP_REG(0x1DC) | MFP_AF0 | MFP_PULL_HIGH), /* GPIO115_MFP115 */
		(MFP_REG(0x1E0) | MFP_AF0 | MFP_PULL_HIGH), /* GPIO116_MFP116 */
		(MFP_REG(0x1E4) | MFP_AF0 | MFP_PULL_HIGH), /* GPIO117_MFP117 */

		/*LCD PIN*/
		(MFP_REG(0x30C) | MFP_AF0 | MFP_PULL_HIGH), /* GPIO195_MFP195 */
		(MFP_REG(0x310) | MFP_AF0 | MFP_PULL_HIGH), /* GPIO196_MFP196 */
		(MFP_REG(0x314) | MFP_AF0 | MFP_PULL_HIGH), /* GPIO197_MFP197 */
		(MFP_REG(0x218) | MFP_AF0 | MFP_DRIVE_MEDIUM), /* GPIO135_MFP135 */

		/* End of configureation */
		MFP_EOC
	};
	mfp_config(mfp_cfg);
	check_discrete();
	if (1 == pxa1928_discrete) {
		mfp_config(mfp_cfg_discrete);
	}
	else {
		/* !!! POP package is not supported !!! */
		while (1) {}
	}
#endif
	pxa_amp_init();
	return 0;
}

/*******************************************************************************
 *
 * Board Initialization
 *
 * init_sequence_r[]
 *     --> board_init()
 *
 ******************************************************************************/
int board_init(void)
{
	int ret;
	emmd_page = (struct emmd_page *)CRASH_BASE_ADDR;
	if (emmd_page->indicator == 0x454d4d44/*"EMMD"*/) {
		/* clear this indicator for prevent the re-entry */
		printf("Detect RAMDUMP signature!!\n");
		emmd_page->indicator = 0;
		flush_cache((unsigned long)(&emmd_page->indicator),
			    (unsigned long)sizeof(emmd_page->indicator));

		adtf_dump_all_trace_buffer();
		/* Register etb ddr buffer as RDI object if ramdump indicator is set*/
		ret = ramdump_attach_pbuffer("etbadtf", CONFIG_MV_ETB_DMP_ADDR, 0x10000);
		printf("add etbadtf to ramdump - %s\n", ret ? "fail" : "success");
		ret = ramdump_attach_pbuffer("powerud", (uintptr_t)emmd_page->pmic_power_reason,
						sizeof(emmd_page->pmic_power_reason));
		printf("add powerud to ramdump - %s\n", ret ? "fail" : "success");

		fastboot = 1;
	}
	check_chip_type();

#ifdef CONFIG_REVISION_TAG
	/* TODO: set board revision */
	board_rev = 0;
#endif /* CONFIG_REVISION_TAG */

	gd->bd->bi_arch_number = MACH_TYPE_PXA1928;
	gd->bd->bi_boot_params = 0x9a00000;

#ifdef CONFIG_CMD_GPIO
	turn_off_backlight();
	gpio_direction_input(recovery_key);
	gpio_direction_input(sdboot_key);
#endif /* CONFIG_CMD_GPIO */

	printf("run board_init\n");
	return 0;
}

#ifdef CONFIG_GENERIC_MMC
#ifdef CONFIG_POWER_88PM860
static unsigned char pxa1928_recovery_reg_read(void)
{
	u32 data = 0;
	struct pmic *p_base;

	p_base = pmic_get(board_pmic_chip->base_name);
	if (!p_base || pmic_probe(p_base)) {
		printf("access pmic failed...\n");
		return -1;
	}
	/* Get the magic number from RTC register */
	pmic_reg_read(p_base, 0xef, &data);
	return (unsigned char)data;
}
static unsigned char pxa1928_recovery_reg_write(unsigned char value)
{
	struct pmic *p_base;

	p_base = pmic_get(board_pmic_chip->base_name);
	if (!p_base || pmic_probe(p_base)) {
		printf("access pmic failed...\n");
		return -1;
	}
	/* Set the magic number from RTC register */
	pmic_reg_write(p_base, 0xef, value);
	return 0;
}
#else /* !CONFIG_POWER_88PM860 */
static unsigned char pxa1928_recovery_reg_read(void)
{
	return 0;
}
static unsigned char pxa1928_recovery_reg_write(unsigned char value)
{
	return 0;
}
#endif /* !CONFIG_POWER_88PM860 */

static struct recovery_reg_funcs pxa1928_recovery_reg_funcs = {
	.recovery_reg_read = pxa1928_recovery_reg_read,
	.recovery_reg_write = pxa1928_recovery_reg_write,
};

int recovery_key_detect(void)
{
#define A03BS_RECOVERY_BTN_WAIT (5000000)
#define RECOVERY_BTN_HOLD_TIME  (5000000)
	int t;
	int button_pressed = 0;
	int recovery_mode  = 0;
	
	for(t=0;t<RECOVERY_BTN_HOLD_TIME;t+=25000){
		button_pressed = !gpio_get_value(recovery_key);
		if(button_pressed==0)
			break;
		udelay(25000);
	}
	
	if(RECOVERY_BTN_HOLD_TIME > t)
		return recovery_mode;
	
	video_clear();
	show_logo(3);
	
	for(t=0;((t<A03BS_RECOVERY_BTN_WAIT) && (button_pressed));t+=25000){
		button_pressed = !gpio_get_value(recovery_key);
		udelay(25000);
	}
	
	if(A03BS_RECOVERY_BTN_WAIT <= t)
		return recovery_mode;
	
	for(t=0;((t<A03BS_RECOVERY_BTN_WAIT) && (!button_pressed));t+=25000){
		button_pressed = !gpio_get_value(recovery_key);
		udelay(25000);
	}
	
	if(A03BS_RECOVERY_BTN_WAIT <= t)
		return recovery_mode;
	
	for(t=0;t<RECOVERY_BTN_HOLD_TIME;t+=25000){
		button_pressed =!gpio_get_value(recovery_key);
		if(button_pressed==0)
			break;
		udelay(25000);
	}
	
	if(RECOVERY_BTN_HOLD_TIME <= t)
		recovery_mode = 1;
	
	return recovery_mode;
}

int sdboot_key_detect(void)
{
	return !gpio_get_value(sdboot_key);
}

#endif /* CONFIG_GENERIC_MMC */

#ifdef CONFIG_OF_LIBFDT
unsigned int dtb_offset(void)
{
	return 0;
}

void handle_dtb(struct fdt_header *devtree)
{
	char cmd[128];

	/* set dtb addr */
	sprintf(cmd, "fdt addr 0x%p", devtree);
	run_command(cmd, 0);

#ifdef CONFIG_MMP_DISP
	if(get_lcd_landscape()) {
		if (handle_disp_rsvmem(&mmp_mipi_lcd_info))
			printf("Attention, framebuffer maybe not reserved correctly!\n");
	
	}
	else if(handle_disp_rsvmem(&mmp_mipi_lcd_info_p))
		printf("Attention, framebuffer maybe not reserved correctly!\n");
#endif /* CONFIG_MMP_DISP */

	/* pass profile number */
	sprintf(cmd, "fdt set /profile marvell,profile-number <%d>\n", mv_profile);
	run_command(cmd, 0);

	/* pass iddq@1.05v */
	sprintf(cmd, "fdt set /iddq marvell,iddq-1p05 <%d>\n", iddq_1p05);
	run_command(cmd, 0);

	/* pass iddq@1.30v */
	sprintf(cmd, "fdt set /iddq marvell,iddq-1p30 <%d>\n", iddq_1p30);
	run_command(cmd, 0);
	/*
	 * we use 1926 pp table by default, if if sethighperf cmd is set,
	 * use pxa1928 pp instead.
	 */
	if (chip_type != PXA1926_2L_DISCRETE)
		run_command("fdt set /pp_version version pxa1928", 0);

	if (cpu_is_pxa1928_a0()) {
		run_command("fdt mknode / chip_version", 0);
		run_command("fdt set /chip_version version a0", 0);
		run_command("fdt set /clock-controller/peri_clock/gc2d_clk lpm-qos <3>", 0);
		run_command("fdt rm /soc/apb@d4000000/map@c3000000/ marvell,b0_fix", 0);
		run_command("fdt set /soc/apb@d4000000/thermal@d403b000 marvell,version-flag <3>", 0);
		/*Set ISP lpm-qos to PM_QOS_CPUIDLE_BLOCK_AXI
		* Because A0 chips doesn't support D0CG mode
		*/
		run_command("fdt set /soc/axi@f0400000/b52isp@0xF0B00000 lpm-qos <0x00000003>", 0);

		/* overwrite emmc rx/tx setting for A0 */
		run_command("fdt set /soc/axi@d4200000/sdh@d4217000 marvell,sdh-dtr-data "
			"<0 26000000 104000000 0 0 0 0 0 2 52000000 104000000 0 0 0 0 1 "
			"7 52000000 104000000 0 150 3 1 1 9 0xffffffff 104000000 0 0 0 0 0>", 0);
		/* overwrite sd card rx/tx setting for A0 */
		run_command("fdt set /soc/axi@d4200000/sdh@d4280000 marvell,sdh-dtr-data "
			"<0 26000000 104000000 0 0 0 0 0 1 26000000 104000000 0 0 0 0 1 "
			"3 52000000 104000000 0 0 0 0 1 4 52000000 104000000 0 0 0 0 1 "
			"5 52000000 104000000 0 0 0 0 1 7 52000000 104000000 0 0 0 0 1 "
			"9 0xffffffff 104000000 0 0 0 0 0>", 0);
		/* overwrite sdio rx/tx setting for A0 */
		run_command("fdt set /soc/axi@d4200000/sdh@d4280800 marvell,sdh-dtr-data "
			"<0 26000000 104000000 0 0 0 0 0 1 26000000 104000000 0 0 0 0 1 "
			"3 52000000 104000000 0 0 0 0 1 4 52000000 104000000 0 0 0 0 1 "
			"5 52000000 104000000 0 0 0 0 1 7 52000000 104000000 0 0 0 0 1 "
			"9 0xffffffff 104000000 0 0 0 0 0>", 0);
		/* disable emmc hs200 mode */
		run_command("fdt set /soc/axi@d4200000/sdh@d4217000 marvell,sdh-host-caps2-disable <0x20>", 0);
		/* disable sd card sdr104 mode */
		run_command("fdt set /soc/axi@d4200000/sdh@d4280000 marvell,sdh-host-caps-disable <0x40000>", 0);
		/* disable sdio sdr104 mode */
		run_command("fdt set /soc/axi@d4200000/sdh@d4280800 marvell,sdh-host-caps-disable <0x40000>", 0);
	} else {
		switch (chip_type) {
			case PXA1926_2L_DISCRETE:
				run_command("fdt set /chip_type type <0>", 0);
				break;
			case PXA1928_POP:
				run_command("fdt set /chip_type type <1>", 0);
				break;
			case PXA1928_4L:
				run_command("fdt set /chip_type type <2>", 0);
				break;
			default:
				run_command("fdt set /chip_type type <0>", 0);
				break;
		}

		/* update dtb so as not to enable ICU for B0 stepping */
		run_command("fdt set /pxa1928_apmu_ver version bx", 0);
		run_command("fdt rm /soc/axi/wakeupgen@d4284000", 0);
		/* set patch flag of uart break for B0 stepping */
		run_command("fdt set /soc/apb@d4000000/uart@d4018000 break-abnormal <1>", 0);
	}
}
#endif /* CONFIG_OF_LIBFDT */

static const int svt_profile_map_table[] = {
	999, 445, 433, 421, 408, 395, 383, 371,
	358, 358, 333, 333, 309, 309, 1,
};
void get_partinfo(void)
{
	int i;
	u32 val0, val1;
	struct fuse_reg_info arg;

	smc_get_fuse_info(LC_GET_FUSE_INFO_CMD, (void *)&arg);
	val0 = arg.arg3;
	val1 = arg.arg4;

	lvt = val0 & 0x7FF;
	nlvt = (val0 & 0x3FF800) >> 11;
	nsvt = ((val1 & 0x1) << 10) | ((val0 & 0xFFC00000) >> 22);
	svt = (val1 & 0xFFE) >> 1;
	iddq_1p05 = (val1 >>  12) & 0x3ff;
	iddq_1p30 = (val1 >>  22) & 0x3ff;

	for (i = 1; i < 15; i++) {
		if (svt >= svt_profile_map_table[i] &&
			svt < svt_profile_map_table[i - 1])
			break;
	}
	mv_profile = i + 1;
	if (mv_profile >= 15 || mv_profile < 0)
		mv_profile = 0;
}
void show_partinfo(void)
{
	printf("---------show partinfo---------\n");
	printf("LVT NUMBER: %u\n", lvt);
	printf("NLVT NUMBER: %u\n", nlvt);
	printf("NSVT NUMBER: %u\n", nsvt);
	printf("SVT NUMBER: %u\n", svt);
	printf("IDDQ @ 1.05v: %u\n", iddq_1p05);
	printf("IDDQ @ 1.30v: %u\n", iddq_1p30);
	printf("SoC Profile Number: %u\n", mv_profile);
	printf("-------------------------------\n");
}

int do_partinfo_read(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	show_partinfo();
	return 0;
}

U_BOOT_CMD(
	partinfo_read,	1,	0,	do_partinfo_read,
	"partinfo_read", ""
);

static int do_sethighperf(cmd_tbl_t *cmdtp, int flag, int argc, char * const argv[])
{
	char *cmdline;
	char *ep;
	unsigned int new_set;
	static unsigned int old_set;
	u32 cpu_max = CPU_MAX_FREQ_DEFAULT;
	u32 ddr_max = DDR_MAX_FREQ_DEFAULT;
	u32 gc3d_max = GC3D_MAX_FREQ_DEFAULT;
	u32 gc2d_max = GC2D_MAX_FREQ_DEFAULT;

	if (argc != 2) {
		printf("usage: sethighperf 0 or 1 to enable low or high performance\n");
		return -1;
	}
	new_set = simple_strtoul((const char *)argv[1], &ep, 10);
	if (new_set != 0 && new_set != 1) {
		printf("usage: sethighperf 0 or 1 to enable low or high performance\n");
		return -1;
	}
	if (((cpu_is_pxa1928_a0() && (1 != pxa1928_discrete)) || (chip_type != PXA1926_2L_DISCRETE)) &&
		old_set != new_set) {
		if (new_set == 1)
			highperf = 1;
		else if (new_set == 0)
			highperf = 0;

		old_set = new_set;
		cmdline = malloc(COMMAND_LINE_SIZE);
		strncpy(cmdline, getenv("bootargs"), COMMAND_LINE_SIZE);
		remove_cmdline_param(cmdline, "cpu_max=");
		remove_cmdline_param(cmdline, "ddr_max=");
		remove_cmdline_param(cmdline, "gc3d_max=");
		remove_cmdline_param(cmdline, "gc2d_max=");
		if (1 == new_set) {
			cpu_max = CPU_MAX_FREQ;
			gc3d_max = GC3D_MAX_FREQ;
			gc2d_max = GC2D_MAX_FREQ;
			if (ddr_speed != 0)
				ddr_max = ddr_speed;
		} else {
			if (ddr_speed != 0 && ddr_speed < DDR_MAX_FREQ_DEFAULT)
				ddr_max = ddr_speed;
		}
		sprintf(cmdline + strlen(cmdline), " cpu_max=%u000", cpu_max);
		sprintf(cmdline + strlen(cmdline), " ddr_max=%u000", ddr_max);
		sprintf(cmdline + strlen(cmdline), " gc3d_max=%u000", gc3d_max);
		sprintf(cmdline + strlen(cmdline), " gc2d_max=%u000", gc2d_max);
		setenv("bootargs", cmdline);
		free(cmdline);
	}
	printf("sethighperf success\n");
	return 0;
}

U_BOOT_CMD(
	sethighperf, 3, 0, do_sethighperf,
	"Setting cpu, ddr frequence for high performance or low performance",
	""
);

#ifdef CONFIG_PXA1928_CONCORD_BRINGUP
static void set_limit_max_frequency(unsigned int type, u32 max)
{
	return;
}
#else
static void set_limit_max_frequency(unsigned int type, u32 max)
{
	char *cmdline;
	char *rem;
	const char *add;
	u32 max_preferred;

	switch (type) {
	case SKU_CPU_MAX_PREFER:
		rem = "cpu_max=";
		add = " cpu_max=%u000";
		break;

	case SKU_DDR_MAX_PREFER:
		rem = "ddr_max=";
		add = " ddr_max=%u000";
		break;

	case SKU_GC3D_MAX_PREFER:
		rem = "gc3d_max=";
		add = " gc3d_max=%u000";
		break;

	case SKU_GC2D_MAX_PREFER:
		rem = "gc2d_max=";
		add = " gc2d_max=%u000";
		break;

	default:
		return;
	}

	max_preferred = get_sku_max_setting(type);
	if (0 == max_preferred)
		max_preferred = max;
	else
		max_preferred = min(max_preferred, max);

	cmdline = malloc(COMMAND_LINE_SIZE);
	strncpy(cmdline, getenv("bootargs"), COMMAND_LINE_SIZE);
	remove_cmdline_param(cmdline, rem);
	sprintf(cmdline + strlen(cmdline), add, max_preferred);
	setenv("bootargs", cmdline);
	free(cmdline);
}
#endif
#if defined(CONFIG_PXA1928_DFC)
/* copy from "drivers/power/pxa1928_freq.c" */
enum ddr_type {
	LPDDR2_533M = 0,
	LPDDR3_600M,
	LPDDR3_HYN2G,
	LPDDR3_ELP2G,
	LPDDR3_HYN2G_DIS,
};
#define REG_BASE_MCU		0xD0000000
#define REG_BASE_CIU		0xD4282C00
#define REG_CH0_MMAP0		(REG_BASE_MCU + 0x200)
#define REG_CH0_PMAP0		(REG_BASE_MCU + 0x210)
#define REG_CH0_DRAM_CONFIG_1	(REG_BASE_MCU + 0x300)
#define REG_MCK_CTRL		(REG_BASE_CIU + 0x98)
static u32 get_ddr_type(void)
{
	if (0x0A == ((readl(REG_CH0_DRAM_CONFIG_1) >> 4) & 0x0F))
	{	/* LPDDR3 */
		if (0x0D == ((readl(REG_CH0_MMAP0) >> 16) & 0x1F))
		{	/* 1GB(512MBx2) */
			return LPDDR3_600M;
		}
		else if (0x05 == ((readl(REG_CH0_PMAP0) >> 8) & 0x0F))
		{	/* Hynix 2GB */
			if (1 == (readl(REG_MCK_CTRL) & 0x01))
			{	/* Discrete package */
				return LPDDR3_HYN2G_DIS;
			}
			else
			{	/* PoP package */
				return LPDDR3_HYN2G;
			}
		}
		else
		{	/* Elpida 2GB */
			return LPDDR3_ELP2G;
		}
	}
	else
	{	/* LPDDR2(0x09) or DDR3(0x02) */
		return LPDDR2_533M;
	}
}
#endif /* CONFIG_PXA1928_DFC */

#define LOW_RAM_DDR_SIZE 0x30000000

static int ma01_get_powerup_mode(void);
static bool add_charger_cmdline(void);
static void wait_battery_charge(void);

#if 0
static int get_emmd_trigger_reason(void)
{
	unsigned char trigger_reason = (unsigned char)(emmd_page->trigger_reason);

	switch (trigger_reason) {
	case EMMD_READ_SIGNATURE:
		printf("EMMD: signature detected!\n");
		break;
	case EMMD_SOC_RSR_TSR:
		printf("EMMD: SOC RSR TSR!\n");
		break;
	case EMMD_POWER_DOWN:
		printf("EMMD: pmic power down!\n");
		break;
	case EMMD_KEY_COMBINATION:
		printf("EMMD: key combination!\n");
		break;
	default:
		printf("EMMD: unknown reason!\n");
	}
	return 0;
}
#endif

static int get_board_version(void)
{
	int ret,version1,version2,version3,version4,tmp;

	gpio_direction_input(MA01_VERSION1);
	gpio_direction_input(MA01_VERSION2);
	gpio_direction_input(MA01_VERSION3);
	gpio_direction_input(MA01_VERSION4);

	version1 = gpio_get_value(MA01_VERSION1);
	version2 = gpio_get_value(MA01_VERSION2);
	tmp = gpio_get_value(MA01_VERSION3);
	if(tmp)
		version3 = 0;
	else
		version3 = 1;

	tmp = gpio_get_value(MA01_VERSION4);
	if(tmp)
		version4 = 0;
	else
		version4 = 1;

	ret = version1 + (version2 << 1) + (version3 << 2) + + (version4 << 3);

	printf("Board Version %d\n",ret);

	gpio_direction_output(MA01_VERSION1, 0);
	gpio_direction_output(MA01_VERSION2, 0);
	gpio_direction_output(MA01_VERSION3, 0);
	gpio_direction_output(MA01_VERSION4, 0);

	return ret;
}

/*******************************************************************************
 *
 * Miscellaneous Platform-dependent Initialization
 *
 * init_sequence_r[]
 *     --> misc_init_r()
 *
 ******************************************************************************/
int misc_init_r(void)
{
	unsigned long i, ddr_size = 0;
	char *cmdline;
	ulong base, size;
	unsigned int rtc_mode;
	int powermode;
	__attribute__((unused)) enum reset_reason rr;
#if defined(CONFIG_PXA1928_DFC)
	u32 ddr_type;
	ddr_type = get_ddr_type();
#endif /* CONFIG_PXA1928_DFC */
	u32 cpu_max, ddr_max, gc3d_max, gc2d_max;

	enum sys_boot_up_reason reason;
	struct mmp_disp_plat_info *mi;

	mi = &mmp_mipi_lcd_info;

	if (!fastboot)
	{
		if(battery_fail)
		{
			wait_battery_charge();
		}

		reason = get_boot_up_reason(&emmd_page->pmic_power_status);
#if 1 /*AC‚âUSB‚ð‘}‚µ‚Ä‚à‹N“®‚µ‚È‚¢‚æ‚¤‚É‚·‚é‚É‚Í‚±‚±‚ð—LŒø‚É*/
		if(reason == SYS_BR_CHARGE)
		{
			printf("power off\n");
			pmic_enter_powerdown(board_pmic_chip);/*power off*/
			while(1);
		}
#endif
		turn_on_backlight();
		show_logo(0);
	}

	if (pxa_is_warm_reset())
		setenv("bootdelay", "-1");

	cmdline = malloc(COMMAND_LINE_SIZE);
	if (!cmdline) {
		printf("misc_init_r: can not allocate memory for cmdline\n");
		return -1;
	}
	for (i = 0; i < CONFIG_NR_DRAM_BANKS; i++) {
		if (gd->bd->bi_dram[i].size == 0)
			break;
		ddr_size += gd->bd->bi_dram[i].size;
	}

	/* set ddr size in bootargs */
	strncpy(cmdline, getenv("bootargs"), COMMAND_LINE_SIZE);
	remove_cmdline_param(cmdline, "mem=");
	sprintf(cmdline + strlen(cmdline), " mem=%ldM", ddr_size>>20);

	remove_cmdline_param(cmdline, "cgroup_disable=");
	sprintf(cmdline + strlen(cmdline), " cgroup_disable=memory");

	/* set cma size to 25MB */
	remove_cmdline_param(cmdline, "cma=");
	sprintf(cmdline + strlen(cmdline), " cma=51M");

	/* Set ioncarv on the command line */
	remove_cmdline_param(cmdline, "ioncarv=");
	base = CONFIG_SYS_TEXT_BASE;
	size = 0x9000000;
	if (!size || (base + size) > ddr_size)
		printf("ERROR: wrong ION setting!!");
	sprintf(cmdline + strlen(cmdline), " ioncarv=%ldM@0x%08lx",
		size>>20, base);
#ifdef CONFIG_RAMDUMP
	remove_cmdline_param(cmdline, "RDCA=");
	sprintf(cmdline + strlen(cmdline), " RDCA=%08lx",
		ramdump_desc.rdc_addr);
	/*
	 * Exclude the 2MB OBM area for now.
	 * Crash seem to set the phys_offset to the
	 * first p-header physical address.
	 */
	rd_mems[0].start = CONFIG_TZ_HYPERVISOR_SIZE;
	rd_mems[0].end = base;
	/*
	 * Dump the ION area which contains important data, e.g. ETB.
	 * Only skip the u-boot area to prevent gzip errors.
	 */
	rd_mems[1].start = CONFIG_SYS_RELOC_END;
	rd_mems[1].end = ddr_size;
#endif /* CONFIG_RAMDUMP */

	setenv("bootargs", cmdline);

	powermode = get_powermode();
	printf("flag of powermode is %d\n", powermode);

#ifdef CONFIG_OBM_PARAM_ADDR
	struct OBM2OSL *params = 0;
	int keypress = 0;

	strncpy(cmdline, getenv("bootargs"), COMMAND_LINE_SIZE);
	params = (struct OBM2OSL *)(uintptr_t)(*(u32 *)CONFIG_OBM_PARAM_ADDR);
	if (params && params->signature == OBM2OSL_IDENTIFIER)
		keypress = params->booting_mode;

	/* enable RTC CLK & read reboot mode */
	rtc_mode = *(unsigned int *)(RTC_CLK);
	rtc_mode = (rtc_mode | APBC_APBCLK) & (~APBC_RST);
	writel(rtc_mode, RTC_CLK);
	udelay(10);
	rtc_mode = *(unsigned int *)(RTC_BRN0);

#if 0
	/* adb reboot product */
	if (rtc_mode == PD_MAGIC_NUM) {
		*(volatile unsigned int *)(RTC_BRN0) = 0;
		keypress = params->booting_mode = PRODUCT_USB_MODE;
		printf("[reboot product] product usb mode %d\n", keypress);
	}

	if (keypress == PRODUCT_UART_MODE)
		powermode = POWER_MODE_DIAG_OVER_UART;
	else if (keypress == PRODUCT_USB_MODE)
		powermode = POWER_MODE_DIAG_OVER_USB;

	switch (powermode) {
	case POWER_MODE_DIAG_OVER_UART:
		remove_cmdline_param(cmdline, "androidboot.console=");
		remove_cmdline_param(cmdline, "androidboot.bsp=");
		remove_cmdline_param(cmdline, "console=");
		remove_cmdline_param(cmdline, "earlyprintk=");
		sprintf(cmdline + strlen(cmdline), " androidboot.bsp=2");
		show_debugmode_logo(mi, PRODUCT_MODE_LOGO_UART);
		break;

	case POWER_MODE_DIAG_OVER_USB:
		remove_cmdline_param(cmdline, "androidboot.console=");
		remove_cmdline_param(cmdline, "androidboot.bsp=");
		sprintf(cmdline + strlen(cmdline),
			" androidboot.console=ttyS0 androidboot.bsp=1");
		show_debugmode_logo(mi, PRODUCT_MODE_LOGO_USB);
		break;

	default:
		break;
	}
#endif

	setenv("bootargs", cmdline);
#endif /* CONFIG_OBM_PARAM_ADDR */

	/* set cpu, gc3d, gc2d  max frequency */
	if (chip_type == PXA1926_2L_DISCRETE) {
		cpu_max = CPU_MAX_FREQ_DEFAULT;
		ddr_max = DDR_MAX_FREQ_DEFAULT;
		gc3d_max = GC3D_MAX_FREQ_DEFAULT;
		gc2d_max = GC2D_MAX_FREQ_DEFAULT;
	} else {
		cpu_max = CPU_MAX_FREQ;
		ddr_max = DDR_MAX_FREQ;
		gc3d_max = GC3D_MAX_FREQ;
		gc2d_max = GC2D_MAX_FREQ;
	}

	if (chip_type == PXA1928_POP)
		ddr_max = DDR_MAX_FREQ_POP;

	set_limit_max_frequency(SKU_CPU_MAX_PREFER, cpu_max);
	set_limit_max_frequency(SKU_DDR_MAX_PREFER, ddr_max);
	set_limit_max_frequency(SKU_GC3D_MAX_PREFER, gc3d_max);
	set_limit_max_frequency(SKU_GC2D_MAX_PREFER, gc2d_max);
#ifdef CONFIG_REVISION_TAG
	ddr_speed = 0;
    serialno_read(board_sn);
	if (*board_sn != 0) {
		strncpy(cmdline, getenv("bootargs"), COMMAND_LINE_SIZE);
		remove_cmdline_param(cmdline, "androidboot.serialno=");
		sprintf(cmdline + strlen(cmdline), " androidboot.serialno=%s", board_sn);
		setenv("bootargs", cmdline);

		/* this sn is also used as fastboot serial no. */
		setenv("fb_serial", board_sn);
	}
#endif /* CONFIG_REVISION_TAG */

#ifdef CONFIG_CMD_FASTBOOT
	setenv("fbenv", "mmc0");
#endif /* CONFIG_CMD_FASTBOOT */

	if (ddr_size <= LOW_RAM_DDR_SIZE)
		sprintf(cmdline + strlen(cmdline), " androidboot.low_ram=true");

	get_partinfo();
	show_partinfo();
	/*
	 * bus 0 is used by pmic, set here for debug with
	 * "i2c probe", this should be the called just before exit,
	 * in case the default bus number is changed
	 */
	i2c_set_bus_num(0);

#if defined(CONFIG_PXA1928_DFC)
	if (!pxa_is_warm_reset()) {
		pxa1928_fc_init(ddr_type);

		run_command("setvolt vcc_main dvc0011 1300",0);
		if (chip_type == PXA1926_2L_DISCRETE)
			run_command("setcpurate 1057", 0);
		else
			run_command("setcpurate 1386", 0);
		run_command("setddrrate hwdfc 312", 0);
		run_command("setaxirate 156", 0);
	}
#endif

#ifdef CONFIG_RAMDUMP
	/* query MPMU reset reason (also saves the info to DDR for RAMDUMP) */
	rr = get_reset_reason();
	if (fastboot) {
#if 0	/* FIXME */
		const char *s;
		int ramdump;
		const char *dump_mode_var = "ramdump";

		get_emmd_trigger_reason();
		setenv("fbenv", "mmc0");
		s = getenv(dump_mode_var);
		if (!s) {
			setenv(dump_mode_var, "0");
			s = getenv(dump_mode_var);
		}
		ramdump = simple_strtol(s, NULL, 10);
		if (!ramdump && (0x1 != emmd_page->dump_style)) {
			show_debugmode_logo(mi, RAMDUMP_SD_LOGO);
			/*
			 * Use force=1: EMMD signature has been detected, so
			 * need to dump even if RDC signature is missing,
			 * e.g. because RAMDUMP is not enabled in kernel.
			 */
			if (run_command("ramdump 0 0 1 2", 0)) {
				printf("SD dump fail, enter fastboot mode\n");
				show_debugmode_logo(mi, RAMDUMP_USB_LOGO);
				run_command("fb", 0);
			} else {
				run_command("reset", 0);
			}
		} else {
			printf("ready to enter fastboot mode\n");
			show_debugmode_logo(mi, RAMDUMP_USB_LOGO);
			run_command("fb", 0);
		}
#else	/* FIXME */
		run_command("reset", 0);
#endif	/* FIXME */
	}
	/*
	 * Zero the entire DDR excluding the lower area already populated.
	 * Do this on power-on only. The reason is to eliminate the random
	 * contents DDR powers up with: this significantly improves the
	 * ramdump compression rate and reduces the dump size and time.
	 */
#ifndef BUILD_VARIANT_USER
	if (rr == rr_poweron)
		ddr_zero(0x10000000, ddr_size);
#endif
#endif
	/* adb reboot fastboot */
	if (rtc_mode == FB_MAGIC_NUM) {
		*(volatile unsigned int *)(RTC_BRN0) = 0;
		printf("[reboot fastboot] enter fastboot mode\n");
		run_command("fb",0);
	}

	*(u32 *)(CONFIG_CORE_BUSY_ADDR) = 0x0;

	/* get CP ddr range */
//	parse_cp_ddr_range();

	/* put cp/msa into LPM */
	printf("run cp_d2 to put cp/msa into LPM\n");
	run_command("cp_d2", 0);
	
	strncpy(cmdline, getenv("bootargs"), COMMAND_LINE_SIZE);
	if(get_lcd_landscape())
		strncat(cmdline, " androidboot.hwrotation=0", COMMAND_LINE_SIZE);
	else
		strncat(cmdline, " androidboot.hwrotation=90", COMMAND_LINE_SIZE);
		
	setenv("bootargs", cmdline);

	strncpy(cmdline, getenv("bootargs"), COMMAND_LINE_SIZE);
	remove_cmdline_param(cmdline, "androidboot.lcd=");
	if(get_lcd_landscape())
		strncat(cmdline, " androidboot.lcd=WXGA", COMMAND_LINE_SIZE);
	else
		strncat(cmdline, " androidboot.lcd=WXGA_P", COMMAND_LINE_SIZE);
	setenv("bootargs", cmdline);

	strncpy(cmdline, getenv("bootargs"), COMMAND_LINE_SIZE);
	remove_cmdline_param(cmdline, "androidboot.revision=");
	sprintf(cmdline + strlen(cmdline), " androidboot.revision=%d", revision);
	setenv("bootargs", cmdline);

	free(cmdline);
	return 0;
}

static bool add_charger_cmdline(void)
{
	char *cmdline = malloc(COMMAND_LINE_SIZE);
	if (!cmdline) {
		printf("%s: alloc for cmdline fails.\n", __func__);
		return false;
	}
	/*
	 * we need to access before the env is relocated,
	 * gd->env_buf is too small, don't use getenv();
	 */
	if (gd->flags & GD_FLG_ENV_READY) {
		strncpy(cmdline, getenv("bootargs"), COMMAND_LINE_SIZE);
		printf("%s: getenv()\n", __func__);
	} else {
		getenv_f("bootargs", cmdline, COMMAND_LINE_SIZE);
		printf("%s: getenv_f()\n", __func__);
	}
	debug("%s: bootargs = %s\n", __func__, getenv("bootargs"));
	sprintf(cmdline + strlen(cmdline), " androidboot.mode=charger");
	if (setenv("bootargs", cmdline)) {
		printf("%s: set bootargs fails\n", __func__);
		free(cmdline);
		return false;
	}
	debug("%s: bootargs = %s\n", __func__, getenv("bootargs"));

	free(cmdline);

	printf("run cp_d2 to put cp/msa into LPM\n");
	run_command("cp_d2", 0);

	return true;
}

#ifdef CONFIG_MV_SDHCI
/*******************************************************************************
 *
 * MMC Initialization
 *
 * init_sequence_r[]
 *     --> initr_mmc()
 *         --> mmc_initialize()
 *             --> board_mmc_init()
 *
 * eMMC:
 *   BUCK2/VCC_IO_NAND(1.8v)->eMMC(vmmcq)
 *   LDO4/V3P3_NAND(2.8v)->eMMC(vmmc) (default on)
 *
 * SD:
 *   LDO6/VCC_IO_MMC1(3.3v)->SD(vmmcq)
 *   LDO10/V_MMC_CARD(3.3v)->SD(vmmc) (default off)
 *
 ******************************************************************************/
int board_mmc_init(bd_t *bd)
{
	ulong mmc_base_address[CONFIG_SYS_MMC_NUM] = CONFIG_SYS_MMC_BASE;
	u32 mfp_mmc1_cd_config_legacy[] = { MMC1_CD_N_MFP65, ND_NCS1_MFP100, MFP_EOC };
	u8 i;
	u32 val;

	for (i = 0; i < CONFIG_SYS_MMC_NUM; i++) {
		if (mv_sdh_init(mmc_base_address[i], 104000000, 0,
				SDHCI_QUIRK_32BIT_DMA_ADDR))
			return 1;
		/*
		 * use default hardware clock gating
		 * by default, SD_FIFO_PARM = 0x70005
		 */
		if (i == 0) {
			/*
			 * emmc need to tune RX/TX under HS50
			 * RX need to set proper delays cycles.
			 * TX can work just invert the internal clock (TX_CFG_REG[30])
			 * but also set the delay cycles here for safety.
			 */
			writel(TX_MUX_DLL | TX_HOLD_DELAY0(0x16A),
						mmc_base_address[i] + TX_CFG_REG);
			writel(SDCLK_DELAY(0xA0) | SDCLK_SEL1(0x1),
						mmc_base_address[i] + RX_CFG_REG);
		} else {
			if ((board_rev == 1) || (board_rev == 2))
				mfp_config(mfp_mmc1_cd_config_legacy);
			/*
			 * sd card can work under HS50 by default.
			 * but also invert TX internal clock (TX_CFG_REG[30]) here for safety.
			 */
			val = readl(mmc_base_address[i] + TX_CFG_REG);
			val |= TX_INT_CLK_INV;
			writel(val, mmc_base_address[i] + TX_CFG_REG);
		}
	}

	p_recovery_reg_funcs = &pxa1928_recovery_reg_funcs;
	return 0;
}
#endif /* CONFIG_MV_SDHCI */

int get_mmc_env_dev(void)
{
	return 0;
}

const char * get_mmc_boot_dev(void)
{
	return "mmc dev 0 0";
}

/*******************************************************************************
 *
 * Reset CPU
 *
 ******************************************************************************/
void reset_cpu(ulong ignored)
{
#ifdef CONFIG_POWER_88PM860
	printf("pxa1928 board rebooting...\n");
	pmic_reset_bd(board_pmic_chip);
#endif /* CONFIG_POWER_88PM860 */
}

#ifdef CONFIG_DISPLAY_BOARDINFO
/*******************************************************************************
 *
 * Display Board Information
 *
 * init_sequence[]
 *     --> checkboard()
 *
 ******************************************************************************/
int checkboard(void)
{
	u32 val;

	val = smp_hw_cpuid();
	if (val >= 0 && val <= 3)
		printf("Boot Core: Core %d\n", val + 1);
	else
		printf("Boot Core is unknown.");

	val = smp_config();
	printf("Available Cores: %s %s %s %s\n"
			, (val & 0x1) ? ("Core 1") : ("")
			, (val & 0x2) ? ("Core 2") : ("")
			, (val & 0x4) ? ("Core 3") : ("")
			, (val & 0x8) ? ("Core 4") : ("")
	);

	return 0;
}
#endif /* CONFIG_DISPLAY_BOARDINFO */

#ifdef CONFIG_POWER_88PM860
/*******************************************************************************
 *
 * PMIC Configuration
 *
 * (call from power_init_common())
 *
 ******************************************************************************/
void board_pmic_power_fixup(struct pmic *p_power)
{
	u32 val;
	unsigned int mask, dvc_ctrl;

	/* enable buck1 dual phase mode */
	pmic_reg_read(p_power, 0x8e, &val);
	val |= (1 << 2);
	pmic_reg_write(p_power, 0x8e, val);

	/* set buck1 all the DVC register 16 levels all at 1.2v
	 * dvc_ctrl is the value of the two dvc control bits
	 */
	for (dvc_ctrl = 0; dvc_ctrl < DVC_CTRL_LVL; dvc_ctrl++) {
		pmic_reg_read(p_power, DVC_CONTROL_REG, &val);
		mask = (DVC_SET_ADDR1 | DVC_SET_ADDR2);
		val &= ~mask;
		val |= dvc_ctrl & mask;
		pmic_reg_write(p_power, DVC_CONTROL_REG, val);

		val = 0x30;
		pmic_reg_write(p_power, 0x3c, val);
		pmic_reg_write(p_power, 0x3d, val);
		pmic_reg_write(p_power, 0x3e, val);
		pmic_reg_write(p_power, 0x3f, val);

		pmic_reg_write(p_power, 0x4b, val);
		pmic_reg_write(p_power, 0x4c, val);
		pmic_reg_write(p_power, 0x4d, val);
		pmic_reg_write(p_power, 0x4e, val);

		/* set buck3 all the DVC register at 1.2v */
		val = 0x30;
		pmic_reg_write(p_power, 0x41, val);
		pmic_reg_write(p_power, 0x42, val);
		pmic_reg_write(p_power, 0x43, val);
		pmic_reg_write(p_power, 0x44, val);

		/* set buck5 all the DVC register at 3.3v for WIB_SYS */
		val = 0x72;
		pmic_reg_write(p_power, 0x46, val);
		pmic_reg_write(p_power, 0x47, val);
		pmic_reg_write(p_power, 0x48, val);
		pmic_reg_write(p_power, 0x49, val);
	}
	/* set ldo5 at 2.8v for emmc */
	marvell88pm_set_ldo_vol(p_power, 5, 2800000);

#ifdef CONFIG_MMP_DISP
#ifdef CONFIG_REVISION_TAG
	/* set ldo3(avdd_dsi) to 1.2v and enable it */
	marvell88pm_set_ldo_vol(p_power, 3, 1200000);

	/* set buck2(vcc_io_gpio1) for 5v_en to 1.8v and enable it */
	marvell88pm_set_buck_vol(p_power, 2, 1800000);

	/* set ldo12 to 3.3v and enable it */
	if(get_lcd_power())
	{
		marvell88pm_set_ldo_vol(p_power, 12, 2500000);
		marvell88pm_set_ldo_vol(p_power, 8, 1800000);
	}
	else
	{
		marvell88pm_set_ldo_vol(p_power, 12, 3300000);
	}
	/* set buck6 3.3v and enable it */
	marvell88pm_set_buck_vol(p_power, 6, 3300000);
#endif /* CONFIG_REVISION_TAG */
#endif /* CONFIG_MMP_DISP */

	/* set ldo17 at 2.8v */
	marvell88pm_set_ldo_vol(p_power, 17, 2800000);

#ifdef CONFIG_VIDEO_VX5B1D
	/* set ldo9 at 3.3v */
	marvell88pm_set_ldo_vol(p_power,  9, 3300000);	/* VX5_LVDS */

	/* set ldo10/ldo13 at 1.2v */
	marvell88pm_set_ldo_vol(p_power, 10, 1200000);	/* VX5_CORE */
	marvell88pm_set_ldo_vol(p_power, 13, 1200000);	/* VX5_MIPI */

	/* set ldo14/ldo15 at 1.8v */
	marvell88pm_set_ldo_vol(p_power, 14, 1800000);	/* VX5_DVDD0 */
	marvell88pm_set_ldo_vol(p_power, 15, 1800000);	/* VX5_DVDD1-3 */
#endif /* CONFIG_VIDEO_VX5B1D */

	marvell88pm_set_ldo_vol(p_power, 11, 1800000);	/* -> TP+1.8V */
	marvell88pm_set_ldo_vol(p_power, 18, 3300000);	/* -> PD+3.3V */

	marvell88pm_set_ldo_vol(p_power,  6, 1800000);	/* -> FCAM+1.8V, RCAM+1.8V */
	marvell88pm_set_ldo_vol(p_power,  7, 2800000);	/* -> FCAM+2.8V, RCAM+2.8V */
	marvell88pm_set_ldo_vol(p_power, 19, 1200000);	/* -> RCAM+1.2V */

	if(get_lcd_power()) {
		marvell88pm_set_buck_vol(p_power, 6, 3000000);
		pmic_reg_read(p_power, 0x50, &val);
		val |= 0x20;
		pmic_reg_write(p_power, 0x50, val);
	}
}
#endif /* CONFIG_POWER_88PM860 */

__weak struct pmic_chip_desc *get_marvell_pmic(void)
{
	return board_pmic_chip;
}

static int ma01_get_powerup_mode(void)
{
	struct pmic *p_chrg;
	int ret = 0;

	p_chrg = pmic_get(BQ24160_CHRG_NAME);

	ret = p_chrg->chrg->chrg_bat_present(p_chrg);
	if(ret == 1)
		return BATTERY_MODE;
	else
		return EXTERNAL_POWER_MODE;
}

// Based on
// - drivers/power/pmic/pmic_mrvl_common.c: batt_marvell_init()
extern int ma01_bat_alloc(unsigned char bus, struct pmic_chip_desc *chip);
static int ma01_batt_init(
	struct pmic_chip_desc *chip,
	u8 pmic_i2c_bus, u8 chg_i2c_bus, u8 fg_i2c_bus)
{
	int ret;
	struct pmic *p_fg, *p_chrg, *p_bat;

	if (!chip) {
		printf("--- %s: chip description is empty.\n", __func__);
		return -EINVAL;
	}
	
	ret = bq24160_chrg_alloc(chg_i2c_bus, chip);
	if (ret) {
		printf("init charger fails.\n");
		return -1;
	}

	ret = lc709203f_fg_alloc(fg_i2c_bus, chip);
	if (ret) {
		printf("init fuelgauge fails.\n");
		return -1;
	}

	ret = ma01_bat_alloc(pmic_i2c_bus, chip);
	if (ret) {
		printf("init charger/fuelgauge parent fails.\n");
		return -1;
	}

	p_chrg = pmic_get(chip->charger_name);
	if (!p_chrg) {
		printf("%s: access charger fails\n", chip->charger_name);
		return -1;
	}

	p_fg = pmic_get(chip->fuelgauge_name);
	if (!p_fg) {
		printf("%s: access fuelgauge fails\n", chip->fuelgauge_name);
		return -1;
	}

	p_bat = pmic_get(chip->battery_name);
	if (!p_bat) {
		printf("%s: access charger/fuelgauge parent fails\n",
		       chip->battery_name);
		return -1;
	}

	chip->get_powerup_mode = ma01_get_powerup_mode;

	p_fg->parent = p_bat;
	p_chrg->parent = p_bat;

	return p_bat->pbat->battery_init(p_bat, p_fg, p_chrg, NULL);
}

/*******************************************************************************
 *
 * PMIC Initialization
 *
 * init_sequence_r[]
 *     --> power_init_board()
 *
 ******************************************************************************/
int power_init_board(void)
{
	int ret;
	struct pmic *p_bat;
	u8 data;
	int reason;

	board_pmic_chip = calloc(sizeof(struct pmic_chip_desc), 1);
	if (!board_pmic_chip) {
		printf("%s: No available memory for allocation!\n", __func__);
		return -ENOMEM;
	}

	i2c_set_bus_num(PMIC_I2C_BUS);//initialize
	ret = pmic_marvell_init(PMIC_I2C_BUS, board_pmic_chip);
	if (ret < 0) {
		printf("PMIC init failed.\n");
		return ret;
	}

	i2c_read(0x30, 0x01, 1, &data, 1);
	reason = get_boot_up_reason(&emmd_page->pmic_power_status);
	if((reason != SYS_BR_CHARGE) && (reason != SYS_BR_FAULT_WAKEUP) && (reason != SYS_BR_REBOOT))
	{
printf("boot reason %d\n",reason);
		if((data & 0x01) == 0x00) //ONkey not pressing
		{
#define PMIC_WAKE_UP	(0xd)
#define PMIC_SW_PDOWN	(1 << 5)
			data = PMIC_SW_PDOWN;
			i2c_write(0x30, PMIC_WAKE_UP, 1, &data, 1);
			while(1);
		}
	}

	ret = ma01_batt_init(board_pmic_chip,
				PMIC_I2C_BUS,
				CHG_I2C_BUS,
				FG_I2C_BUS);
	if (ret < 0) {
		printf("Battery init failed.\n");
		battery_fail = 1;
		return ret;
	}

	p_bat = pmic_get(board_pmic_chip->battery_name);

	p_bat->fg->fg_battery_update(p_bat->pbat->fg, p_bat);

	printf("soc %d%%\n",p_bat->pbat->bat->state_of_chrg);

	if((short)p_bat->pbat->bat->state_of_chrg <= CONFIG_BATTERY_GUARD_SOC)
	{
		battery_fail = 1;
	}
	return 0;
}

static void wait_battery_charge(void)
{
	struct pmic *p_base;
	struct pmic *p_bat;
	int chrg_type;
	int count;
	int lcd_off_count;
	u32 val;
	int ret;
	enum sys_boot_up_reason reason;
	int fastboot = 0;

	printf("wait_battery_charge\n");

	p_base = pmic_get(board_pmic_chip->base_name);
	p_bat = pmic_get(board_pmic_chip->battery_name);

	while(1)
	{
		chrg_type = p_bat->chrg->chrg_type(p_bat->pbat->chrg);
//		printf("type %d\n",chrg_type);

#ifdef CONFIG_CMD_FASTBOOT
		if (tstc()) {	/* we got a key press	*/
			(void) getc();  /* consume input	*/
			fastboot = 1;
			break;
		}
#endif
		if(chrg_type > 0)
		{
			break;
		}
		else
		{
			turn_on_backlight();
			show_logo(2);
			mdelay(3000);
			chrg_type = p_bat->chrg->chrg_type(p_bat->pbat->chrg);
			if(chrg_type == 0)
			{
				turn_off_backlight();
				printf("power off\n");
				pmic_enter_powerdown(board_pmic_chip);/*power off*/
			}
			break;
		}
		mdelay(1000);
	}
	video_clear();

#ifdef CONFIG_CMD_FASTBOOT
	if(!fastboot)
#endif
	{
		pmic_probe(p_base);
		p_bat->fg->fg_battery_update(p_bat->pbat->fg, p_bat);
		if((short)p_bat->pbat->bat->state_of_chrg <= CONFIG_BATTERY_GUARD_SOC)
		{
			reason = get_boot_up_reason(&emmd_page->pmic_power_status);
			if(reason != SYS_BR_CHARGE) /*AC‚âUSB‚ð‘}‚µ‚Ä‚à‹N“®‚µ‚È‚¢‚æ‚¤‚É‚·‚é‚É‚Í‚±‚±‚ð—LŒø‚É*/
			{
				turn_on_backlight();
				show_logo(2);
				mdelay(3000);
				turn_off_backlight();
			}
			printf("power off\n");
			pmic_enter_powerdown(board_pmic_chip);/*power off*/
			while(1);
		}
	}
}

#ifdef CONFIG_CMD_NET
int board_eth_init(bd_t *bis)
{
	int res = -1;

#if defined(CONFIG_MV_UDC)
	if (usb_eth_initialize(bis) >= 0)
		res = 0;
#endif /* CONFIG_MV_UDC */
	return res;
}
#endif /* CONFIG_CMD_NET */

int serialno_read(char* serialno)
{
	unsigned char buff[512];
	int  i,sum,checkvalue;

	const int serialno_offset    = 0x00;
	const int serialno_ck_offset = 0x11;

	if(serialno == NULL)
		return 1;

	memset(buff,0x00,sizeof(buff));

#if defined(CONFIG_ENV_IS_IN_MMC)
	read_serialmac(buff);
#endif

	memcpy(&checkvalue,buff+serialno_ck_offset,sizeof(checkvalue));
	for(i=0,sum=0;i<15;i++){
		sum += buff[i];
	}
	
	if(sum==checkvalue && sum!=0)
		memcpy(serialno,buff+serialno_offset,15);
	else
		memcpy(serialno,"000000000000000",15);

	return 0;
}



