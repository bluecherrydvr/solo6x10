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

#ifndef __SOLO6010_P2M_H
#define __SOLO6010_P2M_H

#include "solo6010.h"

void solo_p2m_isr(struct solo6010_dev *solo_dev, int id);
void solo_p2m_error_isr(struct solo6010_dev *solo_dev, u32 status);

int solo_p2m_dma(struct solo6010_dev *solo_dev, int id, int wr, void *sys_addr,
		 u32 ext_addr, u32 size);

int solo_p2m_init(struct solo6010_dev *solo_dev);
void solo_p2m_exit(struct solo6010_dev *solo_dev);

#endif /* __SOLO6010_P2M_H */
