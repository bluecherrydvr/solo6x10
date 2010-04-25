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

#ifndef __SOLO6010_G723_DEC_H
#define	__SOLO6010_G723_DEC_H

struct g723_state {
	int yl;

	short yu;
	short dms;
	short dml;
	short ap;

	short a[2];
	short b[6];
	short pk[2];
	short dq[6];
	short sr[2];

	char td;

	/* Used to track between calls to g723_decode */
	int bits;
	unsigned short buf;
};

void g723_init(struct g723_state *);

/* Decode a block of codes. Out buf must be at least
 * ((in_size * 8) / 3) * 2) bytes in length (or half that in shorts). */
int g723_decode(struct g723_state *state, const unsigned char *in_buf,
		int in_bytes, short *out_buf);

#endif /* __SOLO6010_G723_DEC_H */
