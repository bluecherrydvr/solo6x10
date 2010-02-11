/*
 * Copyright (C) 2010 Ben Collins <bcollins@bluecherry.net>
 *
 * This program is free software; you can redistribute it and/or modify
 * it under the terms of the GNU General Public License as published by
 * the Free Software Foundation; either version 2 of the License, or
 * (at your option) any later version.
 *
 * This program is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License
 * along with this program; if not, write to the Free Software
 * Foundation, Inc., 59 Temple Place - Suite 330, Boston, MA 02111-1307, USA.
 */

#ifndef __SOLO6010_TW28_H
#define __SOLO6010_TW28_H

#include "solo6010.h"

#define TW_NUM_CHIP				4
#define TW_NUMCHANNEL_PERCHIP			4
#define TW_BASE_ADDR				0x28
#define TW_CHIP_OFFSET_ADDR(n)			(TW_BASE_ADDR + (n))

#define TW_AV_STAT_ADDR				0x5a
#define TW_HUE_ADDR(n)				(0x07 | ((n) << 4))
#define TW_SATURATION_ADDR(n)			(0x08 | ((n) << 4))
#define TW_CONTRAST_ADDR(n)			(0x09 | ((n) << 4))
#define TW_BRIGHTNESS_ADDR(n)			(0x0a | ((n) << 4))

/*
 * The top 2 bits of DEVID1 make up the the most significant bits of the
 * TW2815 device ID, followed by the top 5 bits of DEVID2.  The bottom
 * 3 bits of DEVID2 indicate the device revision.
 */
#define TW_REG_DEVID1				0x58
#define TW_REG_DEVID2				0x59

#define TW_REGVALUE_DEVID(id1,id2)		(((id1) >> 1) | ((id2) >> 3))
#define TW_REGVALUE_REVID(id2)			((id2) & 0x07)

#define TW_AUDIO_OUTPUT_VOL_ADDR		0x70
#define TW_AUDIO_INPUT_GAIN_ADDR(n)		(0x60+((n==4)|(n==1)))

#define TW_COLOR_HUE				1
#define TW_COLOR_SATURATION			2
#define TW_COLOR_CONTRAST			3
#define TW_COLOR_BRIGHTNESS			4

/* TW286x */
#define TW286X_AV_STAT_ADDR			0xfd
#define TW286x_HUE_ADDR(n)			(0x06|(n<<4))
#define TW286x_SATURATIONU_ADDR(n)		(0x04|(n<<4))
#define TW286x_SATURATIONV_ADDR(n)		(0x05|(n<<4))
#define TW286x_CONTRAST_ADDR(n)			(0x02|(n<<4))
#define TW286x_BRIGHTNESS_ADDR(n)		(0x01|(n<<4))

#define TW286x_AUDIO_OUTPUT_VOL_ADDR		0xdf
#define TW286x_AUDIO_INPUT_GAIN_ADDR(n)		(0xD0+((n==3)|(n==4)))

int solo_tw28_init(struct solo6010_dev *solo_dev);

#if 0
unsigned int tw2815_get_video_status(struct SOLO6010 *solo6010);
unsigned int tw2815_get_audio_status(struct SOLO6010 *solo6010);
extern void tw2815_Set_ColorComponentValue(struct SOLO6010 *solo6010,
					   unsigned int color_comp,
					   unsigned int channel,
					   unsigned int u_val);
extern int tw2815_Get_ColorComponentValue(struct SOLO6010 *solo6010,
					  unsigned int color_comp,
					  unsigned int channel);

extern void tw2815_Set_AudioOutVol(struct SOLO6010 *solo6010,
				   unsigned int u_val);
extern void tw2815_Set_AudioInGain(struct SOLO6010 *solo6010,
				   unsigned int channel,
				   unsigned int u_val);
#endif

#endif /* __SOLO6010_TW28_H */
