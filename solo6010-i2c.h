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

#ifndef __SOLO6010_I2C_H
#define __SOLO6010_I2C_H

#include "solo6010.h"

int solo_i2c_init(struct solo6010_dev *solo_dev);
void solo_i2c_exit(struct solo6010_dev *solo_dev);

int solo_i2c_isr(struct solo6010_dev *solo_dev);

u8 solo_i2c_readbyte(struct solo6010_dev *solo_dev, int id, u8 addr, u8 off);
void solo_i2c_writebyte(struct solo6010_dev *solo_dev, int id, u8 addr, u8 off,
			u8 data);

#endif /* __SOLO6010_I2C_H */
