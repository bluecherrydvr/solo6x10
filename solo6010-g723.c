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

#include <sound/core.h>
#include <sound/initval.h>
#include <sound/pcm.h>

#include "solo6010.h"

#define G723_INTR_ORDER		0
#define G723_FDMA_PAGES		32
#define G723_MAX_ITEM		120
#define G723_DEC_CHANNEL	17

/* XXX: Why the descrepancy of outmode mask? */
//#define OUTMODE_MASK		0x300
#define OUTMODE_MASK		0x000
//#define OUTMODE_MASK		0x3FF

#define SAMPLERATE		8000
#define BITRATE			25

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
		       SOLO_AUDIO_FDMA_BASE((SOLO_G723_EXT_ADDR(solo_dev) >> 16)
		       & 0xffff));

	solo_reg_write(solo_dev, SOLO_AUDIO_CONTROL,
		       SOLO_AUDIO_ENABLE | SOLO_AUDIO_I2S_MODE |
		       SOLO_AUDIO_I2S_MULTI(3) | SOLO_AUDIO_MODE(OUTMODE_MASK));
}

void solo_g723_interrupt(struct solo6010_dev *solo_dev)
{
	solo_reg_write(solo_dev, SOLO_IRQ_STAT, SOLO_IRQ_G723);
}

static int snd_solo_hw_params(struct snd_pcm_substream *substream,
			      struct snd_pcm_hw_params *hw_params)
{
	return snd_pcm_lib_malloc_pages(substream,
					params_buffer_bytes(hw_params));
}

static int snd_solo_hw_free(struct snd_pcm_substream *substream)
{
	return snd_pcm_lib_free_pages(substream);
}

static struct snd_pcm_hardware snd_solo_playback_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_CONTINUOUS | SNDRV_PCM_RATE_8000,
	.rate_min = 8000,
	.rate_max = 8000,
	.channels_min = 1,
	.channels_max = 1,
	.buffer_bytes_max = 48,
	.period_bytes_min = 48,
	.period_bytes_max = 48,
	.periods_min = 1,
	.periods_max = 1,
};

static struct snd_pcm_hardware snd_solo_capture_hw = {
	.info = SNDRV_PCM_INFO_MMAP | SNDRV_PCM_INFO_INTERLEAVED |
		SNDRV_PCM_INFO_MMAP_VALID | SNDRV_PCM_INFO_BLOCK_TRANSFER,
	.formats = SNDRV_PCM_FMTBIT_S16_LE,
	.rates = SNDRV_PCM_RATE_8000,
	.rate_min = 8000,
	.rate_max = 8000,
	.channels_min = 1,
	.channels_max = 20,
	.buffer_bytes_max = 960,
	.period_bytes_min = 48,
	.period_bytes_max = 960,
	.periods_min = 1,
	.periods_max = 1,
};

static int snd_solo_playback_open(struct snd_pcm_substream *ss)
{
	struct solo6010_dev *solo_dev = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *rt = ss->runtime;

	solo_dev->psubs = ss;
	rt->hw = snd_solo_playback_hw;

	return 0;
}

static int snd_solo_capture_open(struct snd_pcm_substream *ss)
{
	struct solo6010_dev *solo_dev = snd_pcm_substream_chip(ss);
	struct snd_pcm_runtime *rt = ss->runtime;

	solo_dev->csubs = ss;
	rt->hw = snd_solo_capture_hw;

	return 0;
}

static int snd_solo_playback_close(struct snd_pcm_substream *ss)
{
	struct solo6010_dev *solo_dev = snd_pcm_substream_chip(ss);
        solo_dev->psubs = NULL;
        return 0;
}

static int snd_solo_capture_close(struct snd_pcm_substream *ss)
{
	struct solo6010_dev *solo_dev = snd_pcm_substream_chip(ss);
        solo_dev->csubs = NULL;
        return 0;
}

static struct snd_pcm_ops snd_solo_playback_ops = {
	.open = snd_solo_playback_open,
	.close = snd_solo_playback_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_solo_hw_params,
	.hw_free = snd_solo_hw_free,
//	.prepare = snd_ad1889_playback_prepare,
//	.trigger = snd_ad1889_playback_trigger,
//	.pointer = snd_ad1889_playback_pointer,
};

static struct snd_pcm_ops snd_solo_capture_ops = {
	.open = snd_solo_capture_open,
	.close = snd_solo_capture_close,
	.ioctl = snd_pcm_lib_ioctl,
	.hw_params = snd_solo_hw_params,
	.hw_free = snd_solo_hw_free,
//	.prepare = snd_ad1889_capture_prepare,
//	.trigger = snd_ad1889_capture_trigger,
//	.pointer = snd_ad1889_capture_pointer,
};

static int solo_snd_pcm_init(struct solo6010_dev *solo_dev)
{
	struct snd_card *card = solo_dev->snd_card;
	struct snd_pcm *pcm;
	int ret;

	ret = snd_pcm_new(card, card->driver, 0, 1, 20, &pcm);
	if (ret < 0)
		return ret;

	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_PLAYBACK,
			&snd_solo_playback_ops);
	snd_pcm_set_ops(pcm, SNDRV_PCM_STREAM_CAPTURE,
			&snd_solo_capture_ops);

	pcm->private_data = solo_dev;
	pcm->info_flags = 0;
	strcpy(pcm->name, card->shortname);

	ret = snd_pcm_lib_preallocate_pages_for_all(pcm, SNDRV_DMA_TYPE_DEV,
					snd_dma_pci_data(solo_dev->pdev),
					960, 960);
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

	ret = snd_card_create(SNDRV_DEFAULT_IDX1, "Softlogic",
			      THIS_MODULE, 0, &solo_dev->snd_card);
	if (ret < 0)
		return ret;

	card = solo_dev->snd_card;

	strcpy(card->driver, SOLO6010_NAME);
	strcpy(card->shortname, "SOLO-6010 Audio");
	sprintf(card->longname, "%s on %s IRQ %d", card->shortname,
		pci_name(solo_dev->pdev), solo_dev->pdev->irq);
	snd_card_set_dev(card, &solo_dev->pdev->dev);

	ret = snd_device_new(card, SNDRV_DEV_LOWLEVEL, solo_dev, &ops);
	if (ret < 0) {
		snd_card_free(card);
		return ret;
	}

	ret = snd_card_register(card);
	if (ret < 0) {
		snd_card_free(card);
		return ret;
	}

	ret = solo_snd_pcm_init(solo_dev);
	if (ret < 0) {
		snd_card_free(card);
		return ret;
	}

	solo_g723_config(solo_dev);
	solo6010_irq_on(solo_dev, SOLO_IRQ_G723);

	return 0;
}

void solo_g723_exit(struct solo6010_dev *solo_dev)
{
	if (solo_dev->snd_card) {
		snd_card_free(solo_dev->snd_card);
		solo_dev->snd_card = NULL;
	}
	solo_reg_write(solo_dev, SOLO_AUDIO_CONTROL, 0);
	solo6010_irq_off(solo_dev, SOLO_IRQ_G723);
}
