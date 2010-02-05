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

#include <linux/kernel.h>
#include <linux/version.h>
#include <linux/module.h>
#include <linux/videodev2.h>
#include <media/v4l2-device.h>
#include <media/v4l2-ioctl.h>

#include "solo6010-v4l2.h"

static unsigned video_nr = -1;
module_param(video_nr, uint, 0644);
MODULE_PARM_DESC(video_nr, "videoX start number, -1 is autodetect");

static int vidioc_querycap(struct file *file, void  *priv,
			   struct v4l2_capability *cap)
{
	struct solo6010_dev *solo_dev = priv;

	strcpy(cap->driver, SOLO6010_NAME);
	strcpy(cap->card, SOLO6010_NAME);
	strlcpy(cap->bus_info, solo_dev->v4l2_dev.name, sizeof(cap->bus_info));
	cap->version = SOLO6010_VER_NUM;
	cap->capabilities = V4L2_CAP_VIDEO_CAPTURE |
			    V4L2_CAP_STREAMING     |
			    V4L2_CAP_READWRITE;
        return 0;
}

static int solo_disp_open(struct file *file)
{
	struct solo6010_dev *solo_dev = video_drvdata(file);

	return -EINVAL;
}

static const struct v4l2_file_operations solo_disp_fops = {
	.owner		= THIS_MODULE,
	.open		= solo_disp_open,
	.ioctl		= video_ioctl2,
};

static const struct v4l2_ioctl_ops solo_disp_ioctl_ops = {
	.vidioc_querycap	= vidioc_querycap,
};

static struct video_device solo_disp_template = {
	.name		= SOLO6010_NAME,
	.fops		= &solo_disp_fops,
	.ioctl_ops	= &solo_disp_ioctl_ops,
	.minor		= -1,
	.release	= video_device_release,

	.tvnorms	= V4L2_STD_NTSC,
	.current_norm	= V4L2_STD_NTSC_M,
};

static void solo_v4l2_remove(struct solo6010_dev *solo_dev)
{
	if (solo_dev->vfd) {
		video_unregister_device(solo_dev->vfd);
		solo_dev->vfd = NULL;
	}
	v4l2_device_unregister(&solo_dev->v4l2_dev);
}

int solo_v4l2_init(struct solo6010_dev *solo_dev)
{
	struct v4l2_device *v4l2_dev = &solo_dev->v4l2_dev;
	int ret;

	snprintf(v4l2_dev->name, sizeof(v4l2_dev->name),
		 "%s %s", SOLO6010_NAME, pci_name(solo_dev->pdev));
	ret = v4l2_device_register(NULL, v4l2_dev);
	if (ret)
		return ret;

	solo_dev->vfd = video_device_alloc();
	if (!solo_dev->vfd) {
		solo_v4l2_remove(solo_dev);
		return -ENOMEM;
	}

        *solo_dev->vfd = solo_disp_template;

	ret = video_register_device(solo_dev->vfd, VFL_TYPE_GRABBER, video_nr);
	if (ret < 0) {
		solo_v4l2_remove(solo_dev);
		return ret;
	}

	video_set_drvdata(solo_dev->vfd, solo_dev);

	snprintf(solo_dev->vfd->name, sizeof(solo_dev->vfd->name), "%s (%i)",
		 solo_disp_template.name, solo_dev->vfd->num);

	if (video_nr >= 0)
		video_nr++;

	v4l2_info(&solo_dev->v4l2_dev, "Registered as /dev/video%d\n",
		  solo_dev->vfd->num);

	return 0;
}

void solo_v4l2_exit(struct solo6010_dev *solo_dev)
{
	solo_v4l2_remove(solo_dev);
}
