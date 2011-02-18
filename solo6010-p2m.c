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

#include "solo6010.h"

int solo_p2m_dma(struct solo6010_dev *solo_dev, int wr,
		 void *sys_addr, u32 ext_addr, u32 size,
		 int repeat, u32 ext_size)
{
	dma_addr_t dma_addr;
	int ret;

	if (WARN_ON_ONCE((unsigned long)sys_addr & 0x03))
		return -EINVAL;

	dma_addr = pci_map_single(solo_dev->pdev, sys_addr, size,
				  wr ? PCI_DMA_TODEVICE : PCI_DMA_FROMDEVICE);

	ret = solo_p2m_dma_t(solo_dev, wr, dma_addr, ext_addr, size,
			     repeat, ext_size);

	pci_unmap_single(solo_dev->pdev, dma_addr, size,
			 wr ? PCI_DMA_TODEVICE : PCI_DMA_FROMDEVICE);

	return ret;
}

int solo_p2m_dma_t(struct solo6010_dev *solo_dev, int wr,
		   dma_addr_t dma_addr, u32 ext_addr, u32 size,
		   int repeat, u32 ext_size)
{
	struct solo_p2m_dev *p2m_dev;
	unsigned int timeout = 0;
	u32 cfg, ctrl;
	int ret = 0;
	int id;

	if (WARN_ON_ONCE(dma_addr & 0x03))
		return -EINVAL;
	if (WARN_ON_ONCE(!size))
		return -EINVAL;

	/* Get next ID */
	spin_lock(&solo_dev->p2m_lock);
	id = solo_dev->p2m_next;
	solo_dev->p2m_next = (id + 1) % SOLO_NR_P2M;
	spin_unlock(&solo_dev->p2m_lock);

	p2m_dev = &solo_dev->p2m_dev[id];

	if (down_interruptible(&p2m_dev->mutex))
		return -EINTR;

	cfg = SOLO_P2M_COPY_SIZE(size >> 2);
	ctrl = SOLO_P2M_BURST_SIZE(SOLO_P2M_BURST_256) |
		(wr ? SOLO_P2M_WRITE : 0) | SOLO_P2M_TRANS_ON;

	if (repeat) {
		cfg |= SOLO_P2M_EXT_INC(ext_size >> 2);
		ctrl |=  SOLO_P2M_PCI_INC(size >> 2) |
			 SOLO_P2M_REPEAT(repeat);
	}

	INIT_COMPLETION(p2m_dev->completion);
	p2m_dev->error = 0;
	solo_reg_write(solo_dev, SOLO_P2M_TAR_ADR(id), dma_addr);
	solo_reg_write(solo_dev, SOLO_P2M_EXT_ADR(id), ext_addr);
	solo_reg_write(solo_dev, SOLO_P2M_EXT_CFG(id), cfg);
	solo_reg_write(solo_dev, SOLO_P2M_CONTROL(id), ctrl);

	timeout = wait_for_completion_timeout(&p2m_dev->completion,
					      msecs_to_jiffies(solo_dev->p2m_msecs));

	solo_reg_write(solo_dev, SOLO_P2M_CONTROL(id), 0);

	if (WARN_ON_ONCE(p2m_dev->error))
		ret = -EIO;
	else if (WARN_ON_ONCE(timeout == 0))
		ret = -EAGAIN;

	up(&p2m_dev->mutex);

	return ret;
}

void solo_p2m_isr(struct solo6010_dev *solo_dev, int id)
{
	complete(&solo_dev->p2m_dev[id].completion);
}

void solo_p2m_error_isr(struct solo6010_dev *solo_dev, u32 status)
{
	struct solo_p2m_dev *p2m_dev;
	int i;

	if (!(status & SOLO_PCI_ERR_P2M))
		return;

	for (i = 0; i < SOLO_NR_P2M; i++) {
		p2m_dev = &solo_dev->p2m_dev[i];
		p2m_dev->error = 1;
		solo_reg_write(solo_dev, SOLO_P2M_CONTROL(i), 0);
		complete(&p2m_dev->completion);
	}
}

void solo_p2m_exit(struct solo6010_dev *solo_dev)
{
	int i;

	for (i = 0; i < SOLO_NR_P2M; i++)
		solo6010_irq_off(solo_dev, SOLO_IRQ_P2M(i));
}

int solo_p2m_init(struct solo6010_dev *solo_dev)
{
	struct solo_p2m_dev *p2m_dev;
	int i;

	spin_lock_init(&solo_dev->p2m_lock);

	for (i = 0; i < SOLO_NR_P2M; i++) {
		p2m_dev = &solo_dev->p2m_dev[i];

		init_MUTEX(&p2m_dev->mutex);
		init_completion(&p2m_dev->completion);

		solo_reg_write(solo_dev, SOLO_P2M_DES_ADR(i),
			       __pa(p2m_dev->desc));

		solo_reg_write(solo_dev, SOLO_P2M_CONTROL(i), 0);
		solo_reg_write(solo_dev, SOLO_P2M_CONFIG(i),
			       SOLO_P2M_CSC_16BIT_565 |
			       SOLO_P2M_DMA_INTERVAL(0) |
			       SOLO_P2M_PCI_MASTER_MODE);
		solo6010_irq_on(solo_dev, SOLO_IRQ_P2M(i));
	}

	return 0;
}
