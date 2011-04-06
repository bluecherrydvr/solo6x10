/*
 * Copyright (C) 2010 Bluecherry, LLC www.bluecherrydvr.com
 * Copyright (C) 2010,2011 Ben Collins <bcollins@bluecherry.net>
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
	if (WARN_ON_ONCE(!size))
		return -EINVAL;

	dma_addr = pci_map_single(solo_dev->pdev, sys_addr, size,
				  wr ? PCI_DMA_TODEVICE : PCI_DMA_FROMDEVICE);

	ret = solo_p2m_dma_t(solo_dev, wr, dma_addr, ext_addr, size,
			     repeat, ext_size);

	pci_unmap_single(solo_dev->pdev, dma_addr, size,
			 wr ? PCI_DMA_TODEVICE : PCI_DMA_FROMDEVICE);

	return ret;
}

/* Mutex must be held for p2m_id before calling this!! */
int solo_p2m_dma_desc(struct solo6010_dev *solo_dev,
		      struct solo_p2m_desc *desc, dma_addr_t desc_dma,
		      int desc_cnt)
{
	struct solo_p2m_dev *p2m_dev;
	unsigned int timeout = 0;
	unsigned int config = 0;
	int ret = 0;
        int p2m_id;

	/* Get next ID */
	p2m_id = atomic_inc_return(&solo_dev->p2m_count) % SOLO_NR_P2M;
	if (p2m_id < 0)
		p2m_id = 0 - p2m_id;

	p2m_dev = &solo_dev->p2m_dev[p2m_id];

	if (mutex_lock_interruptible(&p2m_dev->mutex))
		return -EINTR;

	INIT_COMPLETION(p2m_dev->completion);
	p2m_dev->error = 0;

	if (desc_cnt > 1) {
		/* We only need to do this when we have more than one
		 * descriptor. */
		config = solo_reg_read(solo_dev, SOLO_P2M_CONFIG(p2m_id));

		solo_reg_write(solo_dev, SOLO_P2M_DES_ADR(p2m_id), desc_dma);
		solo_reg_write(solo_dev, SOLO_P2M_DESC_ID(p2m_id), desc_cnt);
		solo_reg_write(solo_dev, SOLO_P2M_CONFIG(p2m_id), config |
			       SOLO_P2M_DESC_MODE);
	} else {
		solo_reg_write(solo_dev, SOLO_P2M_TAR_ADR(p2m_id), desc[1].dma_addr);
		solo_reg_write(solo_dev, SOLO_P2M_EXT_ADR(p2m_id), desc[1].ext_addr);
		solo_reg_write(solo_dev, SOLO_P2M_EXT_CFG(p2m_id), desc[1].cfg);
		solo_reg_write(solo_dev, SOLO_P2M_CONTROL(p2m_id), desc[1].ctrl);
	}

	timeout = wait_for_completion_timeout(&p2m_dev->completion,
					      msecs_to_jiffies(solo_dev->p2m_msecs));

	solo_reg_write(solo_dev, SOLO_P2M_CONTROL(p2m_id), 0);

	if (desc_cnt > 1)
		solo_reg_write(solo_dev, SOLO_P2M_CONFIG(p2m_id), config);

	if (WARN_ON_ONCE(p2m_dev->error))
		ret = -EIO;
	else if (timeout == 0)
		ret = -EAGAIN;

	mutex_unlock(&p2m_dev->mutex);

	return ret;
}

void solo_p2m_fill_desc(struct solo_p2m_desc *desc, int wr,
			dma_addr_t dma_addr, u32 ext_addr, u32 size,
			int repeat, u32 ext_size)
{
	desc->cfg = SOLO_P2M_COPY_SIZE(size >> 2);
	desc->ctrl = SOLO_P2M_BURST_SIZE(SOLO_P2M_BURST_256) |
		(wr ? SOLO_P2M_WRITE : 0) | SOLO_P2M_TRANS_ON;

	if (repeat) {
		desc->cfg |= SOLO_P2M_EXT_INC(ext_size >> 2);
		desc->ctrl |=  SOLO_P2M_PCI_INC(size >> 2) |
			 SOLO_P2M_REPEAT(repeat);
	}

	desc->dma_addr = dma_addr;
	desc->ext_addr = ext_addr;
}

int solo_p2m_dma_t(struct solo6010_dev *solo_dev, int wr,
		   dma_addr_t dma_addr, u32 ext_addr, u32 size,
		   int repeat, u32 ext_size)
{
	struct solo_p2m_desc desc[2];

	solo_p2m_fill_desc(&desc[1], wr, dma_addr, ext_addr, size, repeat,
			   ext_size);

	/* No need for desc_dma since we know it is a single-shot */
	return solo_p2m_dma_desc(solo_dev, desc, 0, 1);
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

	for (i = 0; i < SOLO_NR_P2M; i++) {
		p2m_dev = &solo_dev->p2m_dev[i];

		mutex_init(&p2m_dev->mutex);
		init_completion(&p2m_dev->completion);

		solo_reg_write(solo_dev, SOLO_P2M_CONTROL(i), 0);
		solo_reg_write(solo_dev, SOLO_P2M_CONFIG(i),
			       SOLO_P2M_CSC_16BIT_565 |
			       SOLO_P2M_DESC_INTR_OPT |
			       SOLO_P2M_DMA_INTERVAL(0) |
			       SOLO_P2M_PCI_MASTER_MODE);
		solo6010_irq_on(solo_dev, SOLO_IRQ_P2M(i));
	}

	return 0;
}
