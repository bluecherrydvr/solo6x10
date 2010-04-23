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
#include <linux/mempool.h>
#include <linux/poll.h>
#include <linux/kthread.h>
#include <linux/freezer.h>
#include <linux/version.h>

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>

#include "solo6010.h"

#define G723_INTR_ORDER		0
#define G723_FDMA_PAGES		32
#define G723_MAX_ITEM		120
#define G723_DEC_CHANNEL	17

/* Sets up channels 16-19 for decoding and 0-15 for encoding */
#define OUTMODE_MASK		0x300

#define SAMPLERATE		8000
#define BITRATE			25

#define PERIOD_BYTES		256
#define PERIOD_MAX		64
#define MAX_BUFFER		(PERIOD_BYTES * PERIOD_MAX)

struct solo_snd_pcm {
	int				on;
	spinlock_t			lock;
	struct solo6010_dev		*solo_dev;
	struct snd_pcm_substream	*ss;
	unsigned char			buf[MAX_BUFFER];
};

static void solo_g723_config(struct solo6010_dev *solo_dev)
{
	int clk_div;

	clk_div = SOLO_CLOCK_MHZ / (SAMPLERATE * (BITRATE * 2) * 2);

	solo_reg_write(solo_dev, SOLO_AUDIO_SAMPLE,
		       SOLO_AUDIO_BITRATE(BITRATE) |
		       SOLO_AUDIO_CLK_DIV(clk_div));

	solo_reg_write(solo_dev, SOLO_AUDIO_FDMA_INTR,
		      SOLO_AUDIO_FDMA_INTERVAL(1) |
		      SOLO_AUDIO_INTR_ORDER(G723_INTR_ORDER) |
		      SOLO_AUDIO_FDMA_BASE(SOLO_G723_EXT_ADDR(solo_dev) >> 16));

	solo_reg_write(solo_dev, SOLO_AUDIO_CONTROL,
		       SOLO_AUDIO_ENABLE | SOLO_AUDIO_I2S_MODE |
		       SOLO_AUDIO_I2S_MULTI(3) | SOLO_AUDIO_MODE(OUTMODE_MASK));
}

void solo_g723_isr(struct solo6010_dev *solo_dev)
{
	solo_reg_write(solo_dev, SOLO_IRQ_STAT, SOLO_IRQ_G723);
}

static int snd_solo_hw_params(struct snd_pcm_substream *ss,
			      struct snd_pcm_hw_params *hw_params)
{
printk("hw_params: traced\n");
	return snd_pcm_lib_malloc_pages(ss, params_buffer_bytes(hw_params));
}

static int snd_solo_hw_free(struct snd_pcm_substream *ss)
{
printk("hw_free: traced\n");
	return snd_pcm_lib_free_pages(ss);
}

static struct snd_pcm_hardware snd_solo_pcm_hw = {
	.info			= (SNDRV_PCM_INFO_MMAP |
				   SNDRV_PCM_INFO_INTERLEAVED |
				   SNDRV_PCM_INFO_BLOCK_TRANSFER |
				   SNDRV_PCM_INFO_MMAP_VALID),
	.formats		= SNDRV_PCM_FMTBIT_S16_LE,
	.rates			= (SNDRV_PCM_RATE_CONTINUOUS |
				   SNDRV_PCM_RATE_8000),
	.rate_min		= 8000,
	.rate_max		= 8000,
	.channels_min		= 1,
	.channels_max		= 1,
	.buffer_bytes_max	= MAX_BUFFER,
	.period_bytes_min	= PERIOD_BYTES,
	.period_bytes_max	= PERIOD_BYTES,
	.periods_min		= 1,
	.periods_max		= PERIOD_MAX,
};

static int snd_solo_pcm_open(struct snd_pcm_substream *ss)
{
	struct solo6010_dev *solo_dev = snd_pcm_substream_chip(ss);
	struct solo_snd_pcm *solo_pcm;
printk("open: traced\n");
	solo_pcm = kzalloc(sizeof(*solo_pcm), GFP_KERNEL);
	if (solo_pcm == NULL)
		return -ENOMEM;

	spin_lock_init(&solo_pcm->lock);
	solo_pcm->solo_dev = solo_dev;
	solo_pcm->ss = ss;
	ss->runtime->hw = snd_solo_pcm_hw;

	snd_pcm_substream_chip(ss) = solo_pcm;

	return 0;
}

static int snd_solo_pcm_close(struct snd_pcm_substream *ss)
{
	struct solo_snd_pcm *solo_pcm = snd_pcm_substream_chip(ss);
printk("close: traced\n");
	snd_pcm_substream_chip(ss) = solo_pcm->solo_dev;
	kfree(solo_pcm);

        return 0;
}

static int snd_solo_pcm_trigger(struct snd_pcm_substream *ss, int cmd)
{
	struct solo_snd_pcm *solo_pcm = snd_pcm_substream_chip(ss);
	struct solo6010_dev *solo_dev = solo_pcm->solo_dev;
	int ret = -EINVAL;
	unsigned long flags;

printk("trigger: traced: %d\n", cmd);
	spin_lock_irqsave(&solo_pcm->lock, flags);

	switch (cmd) {
	case SNDRV_PCM_TRIGGER_START:
		if (solo_pcm->on == 0) {
			if (atomic_inc_return(&solo_dev->snd_users) == 1)
				solo6010_irq_on(solo_dev, SOLO_IRQ_G723);
			solo_pcm->on = 1;
		}
		ret = 0;
		break;
	case SNDRV_PCM_TRIGGER_STOP:
		if (solo_pcm->on) {
			if (atomic_dec_return(&solo_dev->snd_users) == 0)
				solo6010_irq_off(solo_dev, SOLO_IRQ_G723);
			solo_pcm->on = 0;
		}
		ret = 0;
		break;
	}

	spin_unlock_irqrestore(&solo_pcm->lock, flags);

	return ret;
}

static int snd_solo_pcm_prepare(struct snd_pcm_substream *ss)
{
//	struct solo_snd_pcm *solo_pcm = snd_pcm_substream_chip(ss);
//	struct solo6010_dev *solo_dev = solo_pcm->solo_dev;
printk("prepare: traced\n");
        return 0;
}

static snd_pcm_uframes_t snd_solo_pcm_pointer(struct snd_pcm_substream *ss)
{
//	struct solo_snd_pcm *solo_pcm = snd_pcm_substream_chip(ss);
//	struct solo6010_dev *solo_dev = solo_pcm->solo_dev;
printk("pointer: traced\n");
	return -EINVAL;
}

static struct snd_pcm_ops snd_solo_pcm_ops = {
	.open = snd_solo_pcm_open,
	.close = snd_solo_pcm_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_solo_hw_params,
	.hw_free = snd_solo_hw_free,
	.prepare = snd_solo_pcm_prepare,
	.trigger = snd_solo_pcm_trigger,
	.pointer = snd_solo_pcm_pointer,
};

static int solo_snd_pcm_init(struct solo6010_dev *solo_dev)
{
	struct snd_card *card = solo_dev->snd_card;
	struct snd_pcm *pcm;
	struct snd_pcm_substream *ss;
	int ret;
	int i;

	ret = snd_pcm_new(card, card->driver, 0, 0, solo_dev->nr_chans,
			  &pcm);
	if (ret < 0)
		return ret;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_solo_pcm_ops);

	snd_pcm_chip(pcm) = solo_dev;
	pcm->info_flags = 0;
	strcpy(pcm->name, card->shortname);

	for (i = 0, ss = pcm->streams[1].substream; ss; ss = ss->next, i++)
		sprintf(ss->name, "Camera #%d Audio", i);

	ret = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
					snd_dma_pci_data(solo_dev->pdev),
					PERIOD_BYTES, MAX_BUFFER);
	if (ret < 0)
		return ret;

	solo_dev->snd_pcm = pcm;

	return 0;
}

int solo_g723_init(struct solo6010_dev *solo_dev)
{
	static struct snd_device_ops ops = { NULL };
	struct snd_card *card;
	int ret;

	atomic_set(&solo_dev->snd_users, 0);

#if LINUX_VERSION_CODE > KERNEL_VERSION(2,6,29)
	ret = snd_card_create(SNDRV_DEFAULT_IDX1, "Softlogic",
			      THIS_MODULE, 0, &solo_dev->snd_card);
	if (ret < 0)
		return ret;
#else
	solo_dev->snd_card = snd_card_new(SNDRV_DEFAULT_IDX1, "Softlogic",
					  THIS_MODULE, 0);
	if (solo_dev->snd_card == NULL)
		return -ENOMEM;
#endif

	card = solo_dev->snd_card;

	strcpy(card->driver, SOLO6010_NAME);
	strcpy(card->shortname, "SOLO-6010 Audio");
	sprintf(card->longname, "%s on %s IRQ %d", card->shortname,
		pci_name(solo_dev->pdev), solo_dev->pdev->irq);
	snd_card_set_dev(card, &solo_dev->pdev->dev);

	ret = snd_device_new(card, SNDRV_DEV_LOWLEVEL, solo_dev, &ops);
	if (ret < 0)
		goto snd_error;

	if ((ret = solo_snd_pcm_init(solo_dev)) < 0)
		goto snd_error;

	if ((ret = snd_card_register(card)) < 0)
		goto snd_error;

	solo_g723_config(solo_dev);

	return 0;

snd_error:
	snd_card_free(card);
	return ret;
}

void solo_g723_exit(struct solo6010_dev *solo_dev)
{
	solo_reg_write(solo_dev, SOLO_AUDIO_CONTROL, 0);
	solo6010_irq_off(solo_dev, SOLO_IRQ_G723);

	snd_card_free(solo_dev->snd_card);
}
