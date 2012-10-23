/*
 * Copyright (C) 2011 Bluecherry, LLC www.bluecherrydvr.com
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

#include <stdlib.h>
#include <stdarg.h>
#include <sys/ioctl.h>
#include <stdio.h>
#include <sys/types.h>
#include <sys/stat.h>
#include <fcntl.h>
#include <string.h>

#include <linux/videodev2.h>


int main(int argc, char **argv)
{
	if (argc != 3) {
		fprintf(stderr, "Usage: %s <viddev> <input>\n", argv[0]);
		return 1;
	}

	int input = atoi(argv[2]);

	int vfd = open(argv[1], O_RDWR);
	if (vfd < 0) {
		perror(argv[1]);
		return 1;
	}

	/* Verify this is the correct type */
	int ret = ioctl(vfd, VIDIOC_S_INPUT, &input);
	if (ret < 0) {
		perror("Error setting video input");
		return 1;
	}

	return 0;
}
