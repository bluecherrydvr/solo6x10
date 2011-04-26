/*
 * Copyright (C) 2010 Bluecherry, LLC www.bluecherrydvr.com
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
#include <linux/module.h>
#include <linux/pci.h>
#include <linux/interrupt.h>
#include <linux/videodev2.h>

#include "solo6010.h"
#include "solo6010-tw28.h"

MODULE_DESCRIPTION("Softlogic 6x10 MPEG4/H.264/G.723 Encoder/Decoder V4L2/ALSA Driver");
MODULE_AUTHOR("Ben Collins <bcollins@bluecherry.net>");
MODULE_VERSION(SOLO6010_VERSION);
MODULE_LICENSE("GPL");

static int full_eeprom;
module_param(full_eeprom, uint, 0644);
MODULE_PARM_DESC(full_eeprom, "Allow access to full 128B EEPROM (dangerous, default is only top 64B)");

void solo6010_irq_on(struct solo6010_dev *solo_dev, u32 mask)
{
	solo_dev->irq_mask |= mask;
	solo_reg_write(solo_dev, SOLO_IRQ_ENABLE, solo_dev->irq_mask);
}

void solo6010_irq_off(struct solo6010_dev *solo_dev, u32 mask)
{
	solo_dev->irq_mask &= ~mask;
	solo_reg_write(solo_dev, SOLO_IRQ_ENABLE, solo_dev->irq_mask);
}

static void solo_set_time(struct solo6010_dev *solo_dev)
{
	struct timeval tv;

	do_gettimeofday(&tv);
	solo_reg_write(solo_dev, SOLO_TIMER_SEC, tv.tv_sec);
	solo_reg_write(solo_dev, SOLO_TIMER_USEC, tv.tv_usec);
}

static void solo_timer_sync(struct solo6010_dev *solo_dev)
{
	u32 sec, usec;
	struct timeval tv;
	long diff;

	if (solo_dev->type != SOLO_DEV_6110)
		return;

	solo_dev->time_sync++;

	if (solo_dev->time_sync % 60)
		return;

	sec = solo_reg_read(solo_dev, SOLO_TIMER_SEC);
	usec = solo_reg_read(solo_dev, SOLO_TIMER_USEC);
	do_gettimeofday(&tv);

	diff = (long)tv.tv_sec - (long)sec;
	diff = (diff * 1000000) + ((long)tv.tv_usec - (long)usec);

	if (diff > 1000 || diff < -1000) {
		solo_set_time(solo_dev);
	} else if (diff) {
		long usec_lsb = solo_dev->usec_lsb;

		usec_lsb -= diff / 4;
		if (usec_lsb < 0)
			usec_lsb = 0;
		else if (usec_lsb > 255)
			usec_lsb = 255;

		solo_dev->usec_lsb = usec_lsb;
		solo_reg_write(solo_dev, SOLO_TIMER_USEC_LSB, solo_dev->usec_lsb);
	}
}

static irqreturn_t solo6010_isr(int irq, void *data)
{
	struct solo6010_dev *solo_dev = data;
	u32 status;
	int i;

	status = solo_reg_read(solo_dev, SOLO_IRQ_STAT);
	if (!status)
		return IRQ_NONE;

	if (status & ~solo_dev->irq_mask) {
		solo_reg_write(solo_dev, SOLO_IRQ_STAT,
			       status & ~solo_dev->irq_mask);
		status &= solo_dev->irq_mask;
	}

	if (status & SOLO_IRQ_PCI_ERR)
		solo_p2m_error_isr(solo_dev);

	for (i = 0; i < SOLO_NR_P2M; i++)
		if (status & SOLO_IRQ_P2M(i))
			solo_p2m_isr(solo_dev, i);

	if (status & SOLO_IRQ_IIC)
		solo_i2c_isr(solo_dev);

	if (status & SOLO_IRQ_VIDEO_IN) {
		solo_video_in_isr(solo_dev);
		solo_timer_sync(solo_dev);
	}

	if (status & SOLO_IRQ_ENCODER)
		solo_enc_v4l2_isr(solo_dev);

	if (status & SOLO_IRQ_G723)
		solo_g723_isr(solo_dev);

	/* Clear all interrupts handled */
	solo_reg_write(solo_dev, SOLO_IRQ_STAT, status);

	return IRQ_HANDLED;
}

static void free_solo_dev(struct solo6010_dev *solo_dev)
{
	struct pci_dev *pdev;

	if (!solo_dev)
		return;

	if (solo_dev->dev.parent)
		device_unregister(&solo_dev->dev);

	pdev = solo_dev->pdev;

	/* If we never initialized the PCI device, then nothing else
	 * below here needs cleanup */
	if (!pdev) {
		kfree(solo_dev);
		return;
	}

	/* Bring down the sub-devices first */
	solo_g723_exit(solo_dev);
	solo_enc_v4l2_exit(solo_dev);
	solo_enc_exit(solo_dev);
	solo_v4l2_exit(solo_dev);
	solo_disp_exit(solo_dev);
	solo_gpio_exit(solo_dev);
	solo_p2m_exit(solo_dev);
	solo_i2c_exit(solo_dev);

	/* Now cleanup the PCI device */
	if (solo_dev->reg_base) {
		solo6010_irq_off(solo_dev, ~0);
		pci_iounmap(pdev, solo_dev->reg_base);
		if (pdev->irq)
			free_irq(pdev->irq, solo_dev);
	}

	pci_release_regions(pdev);
	pci_disable_device(pdev);
	pci_set_drvdata(pdev, NULL);

	kfree(solo_dev);
}

static ssize_t solo_set_eeprom(struct device *dev, struct device_attribute *attr,
			       const char *buf, size_t count)
{
	struct solo6010_dev *solo_dev =
		container_of(dev, struct solo6010_dev, dev);
	unsigned short *p = (unsigned short *)buf;
	int i;

	if (count & 0x1) {
		dev_warn(dev, "EEPROM Write is not 2 byte aligned "
			 "(truncating 1 byte)\n");
	}

	if (!full_eeprom && count > 64) {
		dev_warn(dev, "EEPROM Write truncated to 64 bytes\n");
		count = 64;
	} else if (full_eeprom && count > 128) {
		dev_warn(dev, "EEPROM Write truncated to 128 bytes\n");
		count = 128;
	}

	solo_eeprom_ewen(solo_dev, 1);

	for (i = full_eeprom ? 0 : 32; i < min((int)(full_eeprom ? 64 : 32),
					       (int)(count / 2)); i++)
		solo_eeprom_write(solo_dev, i, cpu_to_be16(p[i]));

	solo_eeprom_ewen(solo_dev, 0);

        return count;
}
static ssize_t solo_get_eeprom(struct device *dev, struct device_attribute *attr,
			       char *buf)
{
	struct solo6010_dev *solo_dev =
		container_of(dev, struct solo6010_dev, dev);
	unsigned short *p = (unsigned short *)buf;
	int count = (full_eeprom ? 128 : 64);
	int i;

	for (i = (full_eeprom ? 0 : 32); i < (count / 2); i++)
		p[i] = be16_to_cpu(solo_eeprom_read(solo_dev, i));

	return count;
}
static DEVICE_ATTR(eeprom, S_IWUSR | S_IRUGO, solo_get_eeprom, solo_set_eeprom);

static ssize_t solo_set_vid_type(struct device *dev,
				 struct device_attribute *attr,
				 const char *buf, size_t count)
{
	return -EPERM;
}

static ssize_t solo_get_vid_type(struct device *dev,
				 struct device_attribute *attr,
				 char *buf)
{
	struct solo6010_dev *solo_dev =
		container_of(dev, struct solo6010_dev, dev);

	return sprintf(buf, "%s", solo_dev->video_type ==
		       SOLO_VO_FMT_TYPE_NTSC ? "NTSC" : "PAL");
}
static DEVICE_ATTR(video_type, S_IWUSR | S_IRUGO, solo_get_vid_type,
		   solo_set_vid_type);

static struct device_attribute *const solo_dev_attrs[] = {
	&dev_attr_eeprom,
	&dev_attr_video_type,
};

static void solo_device_release(struct device *dev)
{
	/* Do nothing */
}

static int __devinit solo_sysfs_init(struct solo6010_dev *solo_dev)
{
	struct device *dev = &solo_dev->dev;
	const char *driver;
	int i;

	if (solo_dev->type == SOLO_DEV_6110)
		driver = "solo6110";
	else
		driver = "solo6010";

	dev->release = solo_device_release;
	dev->parent = &solo_dev->pdev->dev;
	set_dev_node(dev, dev_to_node(&solo_dev->pdev->dev));
	dev_set_name(dev, "%s-%d-%d", driver, solo_dev->vfd->num,
		     solo_dev->nr_chans);

	if (device_register(dev)) {
		dev->parent = NULL;
		return -ENOMEM;
	}

	for (i = 0; i < ARRAY_SIZE(solo_dev_attrs); i++) {
		if (device_create_file(dev, solo_dev_attrs[i])) {
			device_unregister(dev);
			return -ENOMEM;
		}
	}

	return 0;
}

static int __devinit solo6010_pci_probe(struct pci_dev *pdev,
					const struct pci_device_id *id)
{
	struct solo6010_dev *solo_dev;
	int ret;
	u8 chip_id;

	solo_dev = kzalloc(sizeof(*solo_dev), GFP_KERNEL);
	if (solo_dev == NULL)
		return -ENOMEM;

	if (id->driver_data == SOLO_DEV_6010)
		dev_info(&pdev->dev, "Probing Softlogic 6010\n");
	else
		dev_info(&pdev->dev, "Probing Softlogic 6110\n");

	solo_dev->type = id->driver_data;
	solo_dev->pdev = pdev;
	spin_lock_init(&solo_dev->reg_io_lock);
	pci_set_drvdata(pdev, solo_dev);
	solo_dev->p2m_msecs = 100; /* Only for during init */

	if ((ret = pci_enable_device(pdev)))
		goto fail_probe;

	pci_set_master(pdev);

	/* RETRY/TRDY Timeout disabled */
	pci_write_config_byte(pdev, 0x40, 0x00);
	pci_write_config_byte(pdev, 0x41, 0x00);

	if ((ret = pci_request_regions(pdev, SOLO6010_NAME)))
		goto fail_probe;

	if ((solo_dev->reg_base = pci_ioremap_bar(pdev, 0)) == NULL) {
		ret = -ENOMEM;
		goto fail_probe;
	}

	chip_id = solo_reg_read(solo_dev, SOLO_CHIP_OPTION) &
				SOLO_CHIP_ID_MASK;
	switch (chip_id) {
		case 7:
			solo_dev->nr_chans = 16;
			solo_dev->nr_ext = 5;
			break;
		case 6:
			solo_dev->nr_chans = 8;
			solo_dev->nr_ext = 2;
			break;
		default:
			dev_warn(&pdev->dev, "Invalid chip_id 0x%02x, "
				 "defaulting to 4 channels\n",
				 chip_id);
		case 5:
			solo_dev->nr_chans = 4;
			solo_dev->nr_ext = 1;
	}

	/* Disable all interrupts to start */
	solo6010_irq_off(solo_dev, ~0);

	/* Initial global settings */
	if (solo_dev->type == SOLO_DEV_6010) {
		solo_dev->clock_mhz = 108;
		solo_reg_write(solo_dev, SOLO_SYS_CFG, SOLO_SYS_CFG_SDRAM64BIT |
			       SOLO_SYS_CFG_INPUTDIV(25) |
			       SOLO_SYS_CFG_FEEDBACKDIV((solo_dev->clock_mhz * 2) - 2) |
			       SOLO_SYS_CFG_OUTDIV(3));
	} else {
		u32 divq, divf;

		solo_dev->clock_mhz = 135;

		if (solo_dev->clock_mhz < 125) {
			divq = 3;
			divf = (solo_dev->clock_mhz * 4) / 3 - 1;
		} else {
			divq = 2;
			divf = (solo_dev->clock_mhz * 2) / 3 - 1;
		}

		solo_reg_write(solo_dev, SOLO_PLL_CONFIG,
			       (1 << 20) | /* PLL_RANGE */
			       (8 << 15) | /* PLL_DIVR  */
			       (divq << 12 ) |
			       (divf <<  4 ) |
			       (1 <<  1)   /* PLL_FSEN */ );

		solo_reg_write(solo_dev, SOLO_SYS_CFG, SOLO_SYS_CFG_SDRAM64BIT);
	}

	solo_reg_write(solo_dev, SOLO_TIMER_CLOCK_NUM, solo_dev->clock_mhz - 1);

	/* PLL locking time of 1ms */
	mdelay(1);

	ret = request_irq(pdev->irq, solo6010_isr, IRQF_SHARED, SOLO6010_NAME,
			  solo_dev);
	if (ret)
		goto fail_probe;

	/* Handle this from the start */
	solo6010_irq_on(solo_dev, SOLO_IRQ_PCI_ERR);

	if ((ret = solo_i2c_init(solo_dev)))
		goto fail_probe;

	/* Setup the DMA engine */
	solo_reg_write(solo_dev, SOLO_DMA_CTRL,
		       SOLO_DMA_CTRL_REFRESH_CYCLE(1) |
		       SOLO_DMA_CTRL_SDRAM_SIZE(2) |
		       SOLO_DMA_CTRL_SDRAM_CLK_INVERT |
		       SOLO_DMA_CTRL_READ_CLK_SELECT |
		       SOLO_DMA_CTRL_LATENCY(1));

	/* Undocumented crap */
	if (solo_dev->type == SOLO_DEV_6010) {
		solo_reg_write(solo_dev, SOLO_DMA_CTRL1, 1 << 8);
	} else {
		solo_reg_write(solo_dev, SOLO_DMA_CTRL1, 3 << 8);
		solo_dev->usec_lsb = 0x3f;
		solo_set_time(solo_dev);
	}

	/* Disable watchdog */
	solo_reg_write(solo_dev, SOLO_TIMER_WATCHDOG, 0xff);

	/* Initialize sub components */

	if ((ret = solo_p2m_init(solo_dev)))
		goto fail_probe;

	if ((ret = solo_disp_init(solo_dev)))
		goto fail_probe;

	if ((ret = solo_gpio_init(solo_dev)))
		goto fail_probe;

	if ((ret = solo_tw28_init(solo_dev)))
		goto fail_probe;

	if ((ret = solo_v4l2_init(solo_dev)))
		goto fail_probe;

	if ((ret = solo_enc_init(solo_dev)))
		goto fail_probe;

	if ((ret = solo_enc_v4l2_init(solo_dev)))
		goto fail_probe;

	if ((ret = solo_g723_init(solo_dev)))
		goto fail_probe;

	if ((ret = solo_sysfs_init(solo_dev)))
		goto fail_probe;

	/* Now that init is over, set this lower */
	solo_dev->p2m_msecs = 10;

	return 0;

fail_probe:
	free_solo_dev(solo_dev);
	return ret;
}

static void __devexit solo6010_pci_remove(struct pci_dev *pdev)
{
	struct solo6010_dev *solo_dev = pci_get_drvdata(pdev);

	free_solo_dev(solo_dev);
}

static DEFINE_PCI_DEVICE_TABLE(solo6010_id_table) = {
	/* 6010 based cards */
	{ PCI_DEVICE(PCI_VENDOR_ID_SOFTLOGIC, PCI_DEVICE_ID_SOLO6010),
	  .driver_data = SOLO_DEV_6010 },
	{ PCI_DEVICE(PCI_VENDOR_ID_BLUECHERRY, PCI_DEVICE_ID_NEUSOLO_4),
	  .driver_data = SOLO_DEV_6010 },
	{ PCI_DEVICE(PCI_VENDOR_ID_BLUECHERRY, PCI_DEVICE_ID_NEUSOLO_9),
	  .driver_data = SOLO_DEV_6010 },
	{ PCI_DEVICE(PCI_VENDOR_ID_BLUECHERRY, PCI_DEVICE_ID_NEUSOLO_16),
	  .driver_data = SOLO_DEV_6010 },
	{ PCI_DEVICE(PCI_VENDOR_ID_BLUECHERRY, PCI_DEVICE_ID_BC_SOLO_4),
	  .driver_data = SOLO_DEV_6010 },
	{ PCI_DEVICE(PCI_VENDOR_ID_BLUECHERRY, PCI_DEVICE_ID_BC_SOLO_9),
	  .driver_data = SOLO_DEV_6010 },
	{ PCI_DEVICE(PCI_VENDOR_ID_BLUECHERRY, PCI_DEVICE_ID_BC_SOLO_16),
	  .driver_data = SOLO_DEV_6010 },
	/* 6110 based cards */
	{ PCI_DEVICE(PCI_VENDOR_ID_BLUECHERRY, PCI_DEVICE_ID_BC_6110_4),
	  .driver_data = SOLO_DEV_6110 },
	{ PCI_DEVICE(PCI_VENDOR_ID_BLUECHERRY, PCI_DEVICE_ID_BC_6110_8),
	  .driver_data = SOLO_DEV_6110 },
	{ PCI_DEVICE(PCI_VENDOR_ID_BLUECHERRY, PCI_DEVICE_ID_BC_6110_16),
	  .driver_data = SOLO_DEV_6110 },
	{0,}
};

MODULE_DEVICE_TABLE(pci, solo6010_id_table);

static struct pci_driver solo6010_pci_driver = {
	.name = SOLO6010_NAME,
	.id_table = solo6010_id_table,
	.probe = solo6010_pci_probe,
	.remove = solo6010_pci_remove,
};

static int __init solo6010_module_init(void)
{
	printk(KERN_INFO "Enabling Softlogic 6x10 Driver v%s\n",
	       SOLO6010_VERSION);
	return pci_register_driver(&solo6010_pci_driver);
}

static void __exit solo6010_module_exit(void)
{
	pci_unregister_driver(&solo6010_pci_driver);
}

module_init(solo6010_module_init);
module_exit(solo6010_module_exit);
