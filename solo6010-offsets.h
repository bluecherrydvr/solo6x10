/*
 * Copyright (C) 2011 Bluecherry, LLC www.bluecherrydvr.com
 * 
 * Original author: 
 * Ben Collins <bcollins@ubuntu.com>
 *
 * Additional work by: 
 * John Brooks <john.brooks@bluecherry.net>
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

#ifndef __SOLO6010_OFFSETS_H
#define __SOLO6010_OFFSETS_H

/* Offsets and sizes of the external address */
#define SOLO_DISP_EXT_ADDR(__solo)		0x00000000
#define SOLO_DISP_EXT_SIZE			0x00480000

/* This is the encoder on-screen display. Not sure why it needs double
 * area for each encoder (extended?) */
#define SOLO_EOSD_EXT_ADDR(__solo) \
		(SOLO_DISP_EXT_ADDR(__solo) + SOLO_DISP_EXT_SIZE)
#define SOLO_EOSD_EXT_SIZE(__solo) \
		((__solo->type == SOLO_DEV_6110) ? 0x00020000 : 0x00010000)
#define SOLO_EOSD_EXT_SIZE_MAX 0x00020000
#define SOLO_EOSD_EXT_AREA(__solo) \
		(SOLO_EOSD_EXT_SIZE(__solo) * __solo->nr_chans * 2)

#define SOLO_MOTION_EXT_ADDR(__solo) \
		(SOLO_EOSD_EXT_ADDR(__solo) + SOLO_EOSD_EXT_AREA(__solo))
#define SOLO_MOTION_EXT_SIZE			0x00080000

#define SOLO_G723_EXT_ADDR(__solo) \
		(SOLO_MOTION_EXT_ADDR(__solo) + SOLO_MOTION_EXT_SIZE)
#define SOLO_G723_EXT_SIZE			0x00010000

/* Capture is for storing frames that need to be encoded. These are raw,
 * which means PAL-D1 is the largest frame (hence the 18). Not sure what
 * the extra location is for. */
#define SOLO_CAP_EXT_ADDR(__solo) \
		(SOLO_G723_EXT_ADDR(__solo) + SOLO_G723_EXT_SIZE)
#define SOLO_CAP_EXT_MAX_PAGE			(18 + 15)
#define SOLO_CAP_EXT_AREA(__solo) \
		((SOLO_CAP_EXT_MAX_PAGE << 16) * (__solo->nr_chans + 1))

#define SOLO_MP4E_EXT_ADDR(__solo) \
		(SOLO_CAP_EXT_ADDR(__solo) + SOLO_CAP_EXT_AREA(__solo))
#define SOLO_MP4E_EXT_SIZE(__solo)		(0x00080000 * __solo->nr_chans)

#define SOLO_JPEG_EXT_ADDR(__solo) \
		(SOLO_MP4E_EXT_ADDR(__solo) + SOLO_MP4E_EXT_SIZE(__solo))
#define SOLO_JPEG_EXT_SIZE(__solo)		(0x00080000 * __solo->nr_chans)

/* This is to save full frames for P-Frame reference. Each input needs
 * two -- one for regular encoder, and one for extended encoder. */
#define SOLO_EREF_EXT_ADDR(__solo) \
		(SOLO_JPEG_EXT_ADDR(__solo) + SOLO_JPEG_EXT_SIZE(__solo))
#define SOLO_EREF_EXT_SIZE			0x00140000
#define SOLO_EREF_EXT_AREA(__solo) \
		(SOLO_EREF_EXT_SIZE * __solo->nr_chans * 2)

#define SOLO_SDRAM_END(__solo) \
		(SOLO_EREF_EXT_ADDR(__solo) + SOLO_EREF_EXT_AREA(__solo))

#endif /* __SOLO6010_OFFSETS_H */
