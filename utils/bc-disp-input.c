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

#define err_out(__msg, args...) do {			\
	fprintf(stderr, __msg ": %m\n", ## args);	\
	exit(1);					\
} while(0)

extern char *__progname;

static void usage(void)
{
	fprintf(stderr, "Usage: %s <viddev> <input>\n", __progname);
	exit(1);
}

int main(int argc, char **argv)
{
	const char *dev;
	int input, vfd;

	if (argc != 3)
		usage();

	dev = argv[1];
	input = atoi(argv[2]);

	if ((vfd = open(dev, O_RDWR)) < 0)
		err_out("Opening video device");

	/* Verify this is the correct type */
	if (ioctl(vfd, VIDIOC_S_INPUT, &input) < 0)
		err_out("Error setting video input");

	exit(0);
}
