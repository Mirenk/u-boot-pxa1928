/*
 * Copyright (C) 2014
 * SANYO Techno Solutions Tottori Co., Ltd.
 *
 * SPDX-License-Identifier:	GPL-2.0+
 */

#ifndef __VX5B1D_H__
#define __VX5B1D_H__

#ifndef CONFIG_VIDEO_VX5B1D
#error CONFIG_VIDEO_VX5B1D is not defined.
#endif /* !CONFIG_VIDEO_VX5B1D */

extern void vx5b1d_reset(void);
extern int vx5b1d_init(int landscape, int power,int spi,int micom);
extern int vx5b1d_pwm_ctrl(int enable, uint32_t freq, uint16_t h_time);
extern int vx5b1d_vee_ctrl(int enable, u16 vee);

#endif /* __VX5B1D_H__ */
